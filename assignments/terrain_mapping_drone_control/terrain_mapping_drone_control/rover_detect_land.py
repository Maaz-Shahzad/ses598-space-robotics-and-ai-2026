# HARDCODED VALUES: takeoff altitude, search yaw rate, rover approach altitude, circling parameters,  

#!/usr/bin/env python3

import math
import time
import statistics

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import Image, CameraInfo
from px4_msgs.msg import VehicleOdometry, OffboardControlMode, VehicleCommand, TrajectorySetpoint, BatteryStatus 
from std_msgs.msg import String

from cv_bridge import CvBridge
import cv2
import numpy as np

# For synchronized subscription of RGB + Depth
from message_filters import ApproximateTimeSynchronizer, Subscriber


class CylinderMission(Node):
    def __init__(self):
        super().__init__('cylinder_mission_node')

        # ---------------------------------------------
        # PX4 / Offboard QoS
        # ---------------------------------------------
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ---------------------------------------------
        # Publishers
        # ---------------------------------------------
        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile
        )
        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile
        )
        self.vehicle_cmd_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile
        )

        # ---------------------------------------------
        # Subscribers
        # ---------------------------------------------
        # Drone odometry
        self.vehicle_odometry_sub = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry',
            self.odom_cb, qos_profile
        )

        # Camera info (for intrinsics)
        self.caminfo_sub = self.create_subscription(
            CameraInfo, '/drone/front_depth/camera_info',
            self.caminfo_callback, 10
        )

        # Approx time sync for RGB + Depth
        self.rgb_sub = Subscriber(self, Image, '/drone/front_rgb')
        self.depth_sub = Subscriber(self, Image, '/drone/front_depth')
        self.sync = ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub], queue_size=10, slop=0.1)
        self.sync.registerCallback(self.image_callback)

        # ---------------------------------------------
        # Internal State Machine
        # ---------------------------------------------
        # WAIT_INTRINSICS -> ARM_TAKEOFF -> CIRCLE -> SERVO -> HOVER
        # -> LAND -> DISARM -> COMPLETE -> DONE
#        self.takeoff_stage = 0  # 0 = vertical, 1 = move to circle start

        self.state = "WAIT_INTRINSICS"
        self.offboard_setpoint_counter = 0

        # Timer for controlling flight logic
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Current drone position: [x, y, z]
        self.position = [0.0, 0.0, 0.0]
        self.altitude = -5.0
        self.rover_width = 1.0
        self.bridge = CvBridge()

        # ---------------------------------------------
        # Camera intrinsics
        # ---------------------------------------------
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        # ---------------------------------------------
        # Circle flight parameters
        # ---------------------------------------------
        self.rover_center = [0.0,0.0]
        self.circle_radius = 4.0
        self.circle_speed = -0.01  # radians step per iteration
        self.theta = 0.0
        self.start_theta = 0.0
        self.theta_traversed = 0.0

        # ---------------------------------------------
        # SEARCH_YAW Vars and Params            
        # ---------------------------------------------
        
        self.yaw_angle_rad = 0.0
        self.yaw_vel_rad_s = 5.0 / 57.3 # 5 deg / s
        self.last_timer_time = self.get_clock().now().nanoseconds / 1e9
        self.search_target_x = 0.0

        # ---------------------------------------------
        # Cylinder detection and measurement
        # ---------------------------------------------
        self.measured_cylinders = []
        self.points_buffer = []
        self.sample_threshold = 10  # frames to accumulate for stable measurement
        self.desired_distance = 4.0
        self.distance_tolerance = 0.3
        self.hover_start_time = None
        self.servo_start_time = None
#        self.min_pixel_area = 5000  # adjust as needed
        self.min_pixel_area = 500  # adjust as needed

        # ---------------------------------------------
        # Detection cooldown control
        # ---------------------------------------------
        # We skip detection for 10 seconds after measuring a new cylinder
        self.detection_cooldown_until = 0.0

        # ---------------------------------------------
        # Land on the Rover
        # ---------------------------------------------
        # For ArUco logic
        self.markers = {}
        self.land_target = None

        # ArUco marker pose subscriber (string topic)
        self.marker_pose_sub = self.create_subscription(
            String, '/aruco/marker_pose', self.aruco_cb, 10
        )
        self.aruco_hover_start_time = None
        self.aruco_hover_height = -4.0

        # ---------------------------------------------
        # Logging mission details
        # ---------------------------------------------
        # Mission timing and energy tracking
        self.start_time = None
        self.battery_percent = None
        self.initial_battery = None

        # For mission battery tracking
        self.battery_at_mission_start = None
        self.battery_at_mission_end = None

        #self.battery_sub = self.create_subscription(
        #    BatteryStatus,
        #    '/fmu/out/battery_status',
        #    self.battery_cb,
        #    qos_profile
        #)
        self.battery_percent = 0.98

    # ---------------------------------------------
    # Battery logging
    # ---------------------------------------------
    #def battery_cb(self, msg):
    #    if not math.isnan(msg.volt_based_soc_estimate):
    #        # Keep an up-to-date snapshot of battery percentage
    #        self.battery_percent = msg.volt_based_soc_estimate

    # ---------------------------------------------
    # Callback: Vehicle Odometry
    # ---------------------------------------------
    def odom_cb(self, msg):
        self.position = [msg.position[0], msg.position[1], msg.position[2]]

#YET TO CHECK THE YAW ANGLE CALCULATION IMPLEMENTATION
        q = msg.q

        # 3. Calculate Yaw (Rotation around the Z-axis)
        siny_cosp = 2.0 * (q[0] * q[3] + q[1] * q[2])
        cosy_cosp = 1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3])
        
        # This stores the yaw in radians (-pi to pi)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    # ---------------------------------------------
    # Callback: Camera Info (intrinsics)
    # ---------------------------------------------
    def caminfo_callback(self, msg):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

        self.get_logger().info("Camera intrinsics received.")
        # Unsubscribe after receiving once
        if self.caminfo_sub is not None:
            self.destroy_subscription(self.caminfo_sub)
            self.caminfo_sub = None

    # ---------------------------------------------
    # Callback: Synchronized Image + Depth
    # ---------------------------------------------
    def image_callback(self, rgb_msg, depth_msg):
        # Skip detection during cooldown
        if time.time() < self.detection_cooldown_until:
            return

        # If intrinsics are not known yet, skip
        if self.fx is None or self.fy is None:
            return

        # Convert ROS → OpenCV
        rgb = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
        depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough').astype(np.float32)
        depth[depth == 0] = np.nan

        # Simple color-based segmentation (placeholder logic)
        hsv = cv2.cvtColor(rgb, cv2.COLOR_BGR2HSV)

#        lower_hsv = np.array([0, 0, 110])  # tune to your cylinder color
#        upper_hsv = np.array([180, 40, 180])
#
        lower_hsv = np.array([2, 70, 50])  # tune to your ROVER color
        upper_hsv = np.array([18, 255, 255])
        color_mask = cv2.inRange(hsv, lower_hsv, upper_hsv) > 0

        # Depth threshold
        depth_mask = np.logical_and(depth > 1.0, depth < 30.0)
        object_mask = np.logical_and(depth_mask, color_mask)

        # Morphological close to reduce noise
        object_mask = cv2.morphologyEx(
            object_mask.astype(np.uint8),
            cv2.MORPH_CLOSE,
            np.ones((5, 5), np.uint8)
        )

        # Contour detection
        contours, _ = cv2.findContours(object_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        filtered = [c for c in contours if cv2.contourArea(c) > self.min_pixel_area]


        # Visualization overlay
        overlay = rgb.copy()

        if len(filtered) > 0:
            # Sort by largest area
            filtered.sort(key=cv2.contourArea, reverse=True)
            contour = filtered[0]
            x, y, w, h = cv2.boundingRect(contour)
            # Inside image_callback, after you find the rover bounding box (x, y, w, h)
            self.rover_center_px = x + (w / 2)
            self.camera_center_px = rgb.shape[1] / 2  # Middle of the image width
            roi = depth[y:y + h, x:x + w]
            roi = roi[np.isfinite(roi)]

            if roi.size > 0:
                # Median depth in bounding box
                Z = float(np.median(roi))
                width_m = (w * Z) / self.fx
                height_m = (h * Z) / self.fy

                self.points_buffer.append((width_m, height_m, Z))

                # Debug bounding box
                cv2.rectangle(overlay, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(
                    overlay,
                    f"{width_m:.2f}m x {height_m:.2f}m",
                    (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    1
                )

                # Transition to SERVO if drone is in search modes
                if self.state in ["SEARCH_YAW", "CHANGE_POS"]:
                    self.get_logger().info("Detected potential rover. Switching to SERVO state.")
                    self.state = "SERVO"

        # Show debug windows
        cv2.imshow("RGB Detection", overlay)
        cv2.imshow("Mask", object_mask.astype(np.uint8) * 255)
        cv2.waitKey(1)
    # ---------------------------------------------
    # Aruco detection and transformation to drone coordinates
    # ---------------------------------------------
    def aruco_cb(self, msg):
        import re
        match = re.match(r"Marker (\d+) detected at x:([-\d.]+)m, y:([-\d.]+)m, z:([-\d.]+)m", msg.data)
        if match:
            marker_id = int(match.group(1))
            x = float(match.group(2))
            y = float(match.group(3))
            z = float(match.group(4))
            # Transform to drone frame: x,y,z => y,x,z
            drone_x = y
            drone_y = x
            drone_z = z
            self.markers[marker_id] = (drone_x, drone_y, drone_z)
            self.get_logger().info(f"Updated Marker {marker_id}: x={drone_x}, y={drone_y}, z={drone_z}")

    # ---------------------------------------------
    # Timer Callback: Main State Machine
    # ---------------------------------------------
    def timer_callback(self):
        # Publish offboard control mode each cycle
        self.publish_offboard_control_mode()

        # After ~1s, engage offboard + arm (if intrinsics are loaded)
        if self.offboard_setpoint_counter == 5:
            if self.state != "WAIT_INTRINSICS":
                self.engage_offboard_mode()
                self.arm()

        # State machine

        elif self.state == "WAIT_INTRINSICS":
            if (self.fx is not None) and (self.fy is not None) and (self.battery_percent is not None):
                # store the battery now, one time only
                if self.battery_at_mission_start is None:
                    self.battery_at_mission_start = self.battery_percent
                    self.get_logger().info(f"Locked battery_at_mission_start: {self.battery_at_mission_start:.4f}")

                self.get_logger().info("Intrinsics and battery OK. Moving to ARM_TAKEOFF.")
                self.state = "ARM_TAKEOFF"
                self.start_time = time.time()

        elif self.state == "ARM_TAKEOFF":
#            if self.takeoff_stage == 0:
                # Stage 1: Vertical takeoff to (0, 0, -5)
            target = [0.0, 0.0, -5.0]
#            self.publish_trajectory_setpoint(*target)
            self.publish_trajectory_setpoint(0.0, 0.0, -5.0, 0.0)

            dx = self.position[0] - target[0]
            dy = self.position[1] - target[1]
            dz = self.position[2] - target[2]
            dist = math.sqrt(dx**2 + dy**2 + dz**2)

            if dist < 0.5:
#               self.get_logger().info("Vertical takeoff complete. Proceeding to circle entry point.")
#               self.takeoff_stage = 1
                self.get_logger().info("Vertical takeoff complete. Proceeding to yaw to search for the rover.")
####################################################################
# COMMENTING THIS TO KEEP THE SIMULATION RUNNING FOR TROUBLESHOOTING
                self.state = "SEARCH_YAW" 
####################################################################

#            elif self.takeoff_stage == 1:
#                # Stage 2: Move to (15, 0, -5)
#                target = [15.0, 0.0, -5.0]
#                self.publish_trajectory_setpoint(*target)
#
#                dx = self.position[0] - target[0]
#                dy = self.position[1] - target[1]
#                dz = self.position[2] - target[2]
#                dist = math.sqrt(dx**2 + dy**2 + dz**2)
#
#                if dist < 0.5:
#                    # Set theta based on actual position
#                    self.theta = math.atan2(self.position[1], self.position[0])
#                    self.get_logger().info("Reached circle entry point. Switching to CIRCLE.")
#                    self.state = "CIRCLE"

#        elif self.state == "CIRCLE":
#            # Circle flight
#            x = self.circle_radius * math.cos(self.theta)
#            y = self.circle_radius * math.sin(self.theta)
#            z = self.altitude
#            self.theta += self.circle_speed
#            self.publish_trajectory_setpoint(x, y, z)

        elif self.state == "SEARCH_YAW":
            current_time = self.get_clock().now().nanoseconds / 1e9
            self.delta_t = current_time - self.last_timer_time 
            x = self.position[0]
            y = self.position[1]
            z = self.altitude
            self.yaw_angle_rad += self.yaw_vel_rad_s * self.delta_t;
            self.last_timer_time = current_time
            self.publish_trajectory_setpoint(x, y, z, self.yaw_angle_rad)
            if self.yaw_angle_rad > 2*math.pi:
                self.get_logger().info("360 Scan complete. Advancing 5m.")
                self.yaw_angle_rad = 0.0                
                self.search_target_x = self.position[0] + 5.0
                self.state = "CHANGE_POS"
                
        
        elif self.state == "CHANGE_POS":
            x = self.search_target_x
            y = self.position[1]
            z = self.altitude
            self.publish_trajectory_setpoint(x, y, z, self.yaw_angle_rad)
            if abs(self.position[0] - self.search_target_x) < 0.25:
                self.get_logger().info("Reached new search position. Starting yaw scan.")
                self.state = "SEARCH_YAW"

        # NEW SERVO STATE. MATCH WITH PREVIOUS AND CHECK.
        elif self.state == "SERVO":
            if self.servo_start_time is None:
                self.servo_start_time = time.time()

            current_distance = None
            if len(self.points_buffer) > 0:
                _, _, Z = self.points_buffer[-1]
                current_distance = Z

            if current_distance is None:
                if time.time() - self.servo_start_time > 5.0:
                    self.get_logger().warn("Object lost. Returning to SEARCH_YAW.")
                    self.points_buffer.clear()
                    self.servo_start_time = None
                    self.rover_center_px = None
                    self.state = "SEARCH_YAW"
                else:
                    self.publish_trajectory_setpoint(self.position[0], self.position[1], self.altitude, self.yaw)
            else:
                # --- 1. Calculate Errors with Deadzone ---
                pixel_error = self.camera_center_px - self.rover_center_px
                
                # If error is small, don't update yaw (prevents jitter)
                if abs(pixel_error) < 12:
                    yaw_correction = 0.0
                else:
                    # Lower gain (0.0006) for smoother tracking
                    yaw_correction = pixel_error * -0.001
                
                # Normalize target_yaw to stay within -pi to pi
                target_yaw = self.yaw + yaw_correction
                target_yaw = math.atan2(math.sin(target_yaw), math.cos(target_yaw))
#                self.get_logger().info("############################################", throttle_duration_sec=1.0)
#                self.get_logger().info("self.yaw: [%.2f] deg" % (self.yaw*57.3), throttle_duration_sec=1.0)
#                self.get_logger().info("pixel_error: [%.2f]" % (pixel_error), throttle_duration_sec=1.0)
#                self.get_logger().info("yaw_correction: [%.2f] deg" % (yaw_correction*57.3), throttle_duration_sec=1.0)
#                self.get_logger().info("target_yaw: [%.2f] deg" % (target_yaw*57.3), throttle_duration_sec=1.0)                

                distance_error = current_distance - self.desired_distance
                gain_pos = 0.4 # Slightly damped position gain

                # --- 2. Phase Control ---
                # PHASE 1: Aligning (Only Yaw)
                if abs(pixel_error) > 50: 
                    self.get_logger().info("Phase 1: Aligning Yaw.", throttle_duration_sec=1.0)
                    target_x = self.position[0]
                    target_y = self.position[1]
                    
                # PHASE 2: Approaching (Yaw + Position)
                else:
                    self.get_logger().info("Phase 2: Approaching Rover", throttle_duration_sec=1.0)
                    move_dist = distance_error * gain_pos
                    # Use current yaw for direction to keep travel linear
                    target_x = self.position[0] + (move_dist * math.cos(self.yaw))
                    target_y = self.position[1] + (move_dist * math.sin(self.yaw))
                    self.altitude = -2.0
#                    self.get_logger().info("############################################################")
#                    self.get_logger().info("current_distance: [%.2f]" % (current_distance))
#                    self.get_logger().info("current position: [%.2f, %.2f]" % (self.position[0], self.position[1]))
#                    self.get_logger().info("distance_error: [%.2f]" % (distance_error))
#                    self.get_logger().info("target [x,y,z]: [%.2f, %.2f, %.2f]" % (target_x, target_y, self.altitude))

                # --- 3. Execution ---
                self.publish_trajectory_setpoint(target_x, target_y, self.altitude, target_yaw)

                # --- 4. Final Transition ---
                if abs(distance_error) < self.distance_tolerance and abs(pixel_error) < 15:
                    self.get_logger().info("Target Locked. Moving to MAP_CIRCLE.")
#                    self.hover_start_time = time.time() # Ensure this is set for HOVER state
                    self.rover_center[0] = self.position[0] + (current_distance+self.rover_width/2.0) * math.cos(self.yaw) 
                    self.rover_center[1] = self.position[1] + (current_distance+self.rover_width/2.0) * math.sin(self.yaw)
                    self.theta = math.atan2(self.position[1] - self.rover_center[1], self.position[0] - self.rover_center[0])
                    self.start_theta = self.theta
                    self.theta_traversed = 0.0
                    self.state = "MAP_CIRCLE"

        elif self.state == "MAP_CIRCLE":
                    
                    # 1. Calculate the new position
                    dx = self.circle_radius * math.cos(self.theta)
                    dy = self.circle_radius * math.sin(self.theta)
                    
                    x = self.rover_center[0] + dx
                    y = self.rover_center[1] + dy
                    z = self.altitude

                    # 2. Calculate Yaw to face the center
                    # Since (dx, dy) is the vector from center to drone, 
                    # the vector from drone to center is (-dx, -dy)

                    target_yaw = math.atan2(-dy, -dx) + 20.0/57.3
                    target_yaw = math.atan2(math.sin(target_yaw), math.cos(target_yaw))
#                    # To rotate 90 degrees (pi/2 radians)
#                    target_yaw = math.atan2(-dy, -dx) + (math.pi / 2.0)
#                    # Normalize to stay within -pi to pi
#                    target_yaw = math.atan2(math.sin(target_yaw), math.cos(target_yaw))
#
                    # 3. Update angle for next iteration
                    self.theta += self.circle_speed
                    self.theta_traversed += abs(self.circle_speed)
                    
                    # 4. Publish with the target_yaw
                    self.publish_trajectory_setpoint(x, y, z, target_yaw)

                    # 5. Check for completion (one full circle)
                    if abs(self.theta_traversed) >= (2 * math.pi):
                        self.get_logger().info("Circle complete. Moving to ARUCO_HOVER.")
                        self.state = "ARUCO_HOVER"

#        elif self.state == "HOVER":
#            # Maintain current position at the hover altitude
#            self.publish_trajectory_setpoint(self.position[0], self.position[1], self.altitude)
#
#            # Check if 5 seconds have passed since entering HOVER
#            if time.time() - self.hover_start_time >= 7.0:
#                self.get_logger().info("7s hover done. Checking measurement.")
#
#                # Check if we collected bounding-box data
#                if len(self.points_buffer) > 0:
#                    widths, heights, depths = zip(*self.points_buffer)
#                    median_w = statistics.median(widths)
#                    median_h = statistics.median(heights)
#                    self.get_logger().info(
#                        f"[Cylinder Dimensions] Width={median_w:.2f} m, Height={median_h:.2f} m"
#                    )
#
#                    # Clear buffer for next object
#                    self.points_buffer.clear()
#
#                    # Compare with previously measured cylinders
#                    dimension_matched = False
#                    tolerance = 0.3  # example tolerance
#                    for (w_old, h_old) in self.measured_cylinders:
#                        if (abs(w_old - median_w) < tolerance) and (abs(h_old - median_h) < tolerance):
#                            dimension_matched = True
#                            break
#
#                    if dimension_matched:
#                        self.get_logger().info(
#                            "This cylinder matches a previously seen one. Mission done, landing."
#                        )
#                        # Go to the LAND state
#                        self.state = "ARUCO_HOVER"
#                    else:
#                        self.get_logger().info(
#                            "New cylinder dimension recorded. Resuming circle flight."
#                        )
#                        self.measured_cylinders.append((median_w, median_h))
#
#                        # OPTIONAL: Set a detection cooldown so the drone
#                        # won't detect the same cylinder immediately again
#                        self.detection_cooldown_until = time.time() + 6.0
#
#                        # Recalculate theta based on current position
#                        drone_x, drone_y, _ = self.position
#                        self.theta = math.atan2(drone_y, drone_x)
#                        self.get_logger().info(f"Rejoining circle from theta = {self.theta:.2f} rad")
#
#                        # Return to circle state
#                        self.state = "CIRCLE"
#                else:
#                    self.get_logger().warn("No data in points_buffer. Resuming circle anyway.")
#                    # Return to circle state
#                    self.state = "CIRCLE"

        elif self.state == "ARUCO_HOVER":
            self.publish_trajectory_setpoint(self.rover_center[0], self.rover_center[1], z=self.aruco_hover_height, yaw=0.0)

            # Start timer once
            if self.aruco_hover_start_time is None:
                if abs(self.position[2] - (self.aruco_hover_height)) < 0.3:
                    self.aruco_hover_start_time = time.time()
                    self.get_logger().info("Reached hover height. Holding for 5 seconds...")

            # After 5 seconds, transition to next state
            elif time.time() - self.aruco_hover_start_time >= 5.0:
                self.get_logger().info("5s ArUco hover complete. Selecting marker...")
                self.state = "ARUCO_SELECT"

#        elif self.state == "ARUCO_SELECT":
#            if len(self.markers) >= 2:
#                best_marker_id = None
#                min_z = float('inf')
#                for mid, (mx, my, mz) in self.markers.items():
#                    if mz < min_z:
#                        min_z = mz
#                        best_marker_id = mid
#                if best_marker_id is not None:
#                    dx, dy, dz = self.markers[best_marker_id]
#                    self.land_target = [dx, dy, -abs(20.0 - dz)]
#                    self.get_logger().info(
#                        f"Selected Marker {best_marker_id} for landing at x={dx:.2f}, y={dy:.2f}, z={-abs(20.0 - dz):.2f}"
#                    )
#                    self.state = "ARUCO_MOVE"
        elif self.state == "ARUCO_SELECT":
            # Change: Check if there is at least one marker present
            if self.markers:
                best_marker_id = None
                min_z = float('inf')
                
                # Logic remains the same: pick the closest marker 
                # (Even if there is only one, this loop will pick it)
                for mid, (mx, my, mz) in self.markers.items():
                    if mz < min_z:
                        min_z = mz
                        best_marker_id = mid
                
                if best_marker_id is not None:
                    dx, dy, dz = self.markers[best_marker_id]
                    # Store target and calculate descent altitude
                    # Ensure dz logic matches your specific environment heights
#                    self.land_target = [dx, dy, -abs(20.0 - dz)]
                    self.land_target = [self.position[0]+dx, self.position[1]+dy, self.position[2] + dz]

#                    dx, dy, dz = self.markers[best_marker_id]
#                                
#                    # ROTATION MATH: Convert Body Frame to World Frame
#                    # This rotates the camera offsets by the drone's current Yaw
#                    world_dx = dx * math.cos(self.yaw) - dy * math.sin(self.yaw)
#                    world_dy = dx * math.sin(self.yaw) + dy * math.cos(self.yaw)
#                    
#                    # Set the target relative to where the drone is NOW
#                    self.land_target = [
#                        self.position[0] + world_dx, 
#                        self.position[1] + world_dy, 
#                        self.position[2] + (dz - 0.2) # Target 20cm above marker surface
#                    ]
                    
                    self.get_logger().info(
                        f"Target Acquired: Marker {best_marker_id} "
                        f"at x={dx:.2f}, y={dy:.2f}, z={self.land_target[2]:.2f}"
                    )
                    self.state = "ARUCO_MOVE"

            else:
                # Optional: log that we are still searching
                self.get_logger().info("Waiting for ArUco detection...", throttle_duration_sec=2.0)



        elif self.state == "ARUCO_MOVE":
            x, y, z = self.land_target
            self.publish_trajectory_setpoint(x=x, y=y, z=z)
            dist = math.sqrt(
                (self.position[0] - x)**2 +
                (self.position[1] - y)**2 +
                (self.position[2] - z)**2)
            if dist < 0.5:
                self.get_logger().info("Reached marker position. Initiating LAND.")
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                self.state = "ARUCO_LAND"

        elif self.state == "ARUCO_LAND":

            self.get_logger().info("Landed successfully. Disarming...")
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0
            )
            self.state = "COMPLETE"

        elif self.state == "COMPLETE":
            self.get_logger().info("Mission complete.")

            if self.battery_at_mission_end is None and self.battery_percent is not None:
                self.battery_at_mission_end = self.battery_percent
                self.get_logger().info(f"Captured battery_at_mission_end: {self.battery_at_mission_end:.4f}")

            if self.start_time is not None:
                mission_duration = time.time() - self.start_time
                self.get_logger().info(f"Mission Duration: {mission_duration:.2f} seconds")

                if self.battery_at_mission_start is not None and self.battery_at_mission_end is not None:
                    used = (self.battery_at_mission_start - self.battery_at_mission_end) * 100.0
                    self.get_logger().info(f"Battery Used: {used:.3f}%")
                else:
                    self.get_logger().warn("Missing start/end battery data!")

            self.state = "DONE"

        elif self.state == "DONE":
            rclpy.shutdown()
            pass

        self.offboard_setpoint_counter += 1

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.offboard_control_mode_pub.publish(msg)

    def publish_trajectory_setpoint(self, x=0.0, y=0.0, z=0.0, yaw=0.0):
        msg = TrajectorySetpoint()
        msg.position = [float(x), float(y), float(z)]
        msg.yaw = float(yaw)
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.trajectory_pub.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.vehicle_cmd_pub.publish(msg)

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info("Arm command sent")

    def engage_offboard_mode(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,
            param2=6.0
        )
        self.get_logger().info("Offboard mode command sent")


def main(args=None):
    rclpy.init(args=args)
    node = CylinderMission()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Interrupted, shutting down.")
    finally:
        if rclpy.ok():  # Ensure it's not already shut down
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
