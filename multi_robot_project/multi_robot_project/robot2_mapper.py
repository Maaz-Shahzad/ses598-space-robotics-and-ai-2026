import sys
import os

# Manually add the venv site-packages to the path
# Adjust the path below if your venv is named differently or located elsewhere
venv_path = os.path.expanduser('~/multi_robot_ws/venv/lib/python3.12/site-packages')
if venv_path not in sys.path:
    sys.path.append(venv_path)

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from message_filters import Subscriber, ApproximateTimeSynchronizer
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import math
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R

class MultiRobotMapper(Node):
    def __init__(self):
        super().__init__('multi_robot_mapper')

        self.model = YOLO('yolov8n.pt')
        # Bridge to convert ROS images to OpenCV (NumPy) arrays
        self.bridge = CvBridge()

#        # --- ROBOT 1 SUBSCRIPTIONS ---
#        self.r1_rgb_sub = Subscriber(self, Image, '/robot1/camera/rgb')
#        self.r1_depth_sub = Subscriber(self, Image, '/robot1/camera/depth')
#        
#        # Synchronize Robot 1's RGB and Depth
#        self.r1_sync = ApproximateTimeSynchronizer(
#            [self.r1_rgb_sub, self.r1_depth_sub],
#            queue_size=10,
#            slop=0.1  # Max time difference (seconds) between messages to be paired
#        )
#        self.r1_sync.registerCallback(self.robot1_callback)

        # --- ROBOT 2 SUBSCRIPTIONS ---
        self.r2_rgb_sub = Subscriber(self, Image, '/robot2/camera/rgb')
        self.r2_depth_sub = Subscriber(self, Image, '/robot2/camera/depth')
        self.r2_odom_sub = Subscriber(self, Odometry, '/robot1/odom')
        
        # Synchronize Robot 2's RGB and Depth
        self.r2_sync = ApproximateTimeSynchronizer(
            [self.r2_rgb_sub, self.r2_depth_sub,self.r2_odom_sub],
            queue_size=3,
            slop=0.1
        )
        self.r2_sync.registerCallback(self.robot2_callback)

        self.get_logger().info("Multi-Robot Mapper initialized. Waiting for synchronized feeds...")

        # Storage for intrinsic parameters
#        self.r1_intrinsics = None
        self.r2_intrinsics = None
        # Subscribe to Camera Info (only need to grab these once)
#        self.create_subscription(CameraInfo, '/robot1/camera/camera_info',self.r1_info_cb, 10)
        self.create_subscription(CameraInfo, '/robot2/camera/camera_info',self.r2_info_cb, 10)

        self.frame_count = 0
        self.frame_skip = 2

        self.log_file = 'robot2_map_data.csv'
        with open(self.log_file, 'w') as f:
            f.write("label,cam_x,cam_y,cam_z,robot_x,robot_y,robot_z,robot_yaw\n")

#    def r1_info_cb(self, msg):
#        self.r1_intrinsics = msg.k # The 3x3 intrinsic matrix

    def r2_info_cb(self, msg):
        self.r2_intrinsics = msg.k

    def get_3d_camera_coords(self, u, v, depth, K):
        """Converts pixel (u,v) and depth to (x,y,z) in camera frame"""
        if depth <= 0: return None
        
        # K is a flattened 3x3 matrix: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        fx, cx = K[0], K[2]
        fy, cy = K[4], K[5]

#        x = (u - cx) * depth / fx
#        y = (v - cy) * depth / fy
#        z = depth

        x = depth
        y = -1*(u - cx) * depth / fx
        z = -1*(v - cy) * depth / fy

        return (x, y, z)

#    def robot1_callback(self, rgb_msg, depth_msg):
#        """Processes synchronized data from Robot 1"""
#        try:
#            # Convert to OpenCV formats
#            cv_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
#            cv_depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')
#            
#            # This is where YOLO will eventually go
#            self.process_logic("Robot 1", cv_image, cv_depth)
#            
#        except Exception as e:
#            self.get_logger().error(f"Error in Robot 1 callback: {e}")

    def robot2_callback(self, rgb_msg, depth_msg, odom_msg):
        # Only allow the AI to run once every 2 seconds (assuming 10Hz camera)
        # self.frame_count must be initialized in __init__
        self.frame_count += 1
        if self.frame_count % self.frame_skip != 0:
            return 

        try:
            # We only do the expensive CV2 conversion when we are actually going to use YOLO
            cv_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
            cv_depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')
            
            # Lower imgsz=320 makes the AI 4x faster than default 640
            results = self.model.predict(cv_image, conf=0.7, imgsz=320, verbose=False)
            
            # Logic for drawing and logging coordinates goes here
            self.process_detections(results, cv_depth, cv_image, odom_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error: {e}")

    def process_detections(self, results, depth_img, color_img, odom_msg):
        rx = odom_msg.pose.pose.position.x
        ry = odom_msg.pose.pose.position.y
        rz = odom_msg.pose.pose.position.z
        
        # Get Yaw
        q = odom_msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        r_yaw = math.atan2(siny_cosp, cosy_cosp)

        # K is the 3x3 intrinsic matrix from CameraInfo
        K = self.r2_intrinsics
        robot_name = "robot2"
        if K is None:
            self.get_logger().warn(f"No CameraInfo for {robot_name}, skipping 3D math.")
            return

        # YOLO returns a list of results (usually one per frame)
        for result in results:
            # result.boxes contains the bounding boxes and confidences
            for box in result.boxes:
                # 1. Get Coordinates and ID
                # box.xyxy is [xmin, ymin, xmax, ymax]
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                conf = float(box.conf[0])
                cls = int(box.cls[0])
                label = self.model.names[cls]

                # 2. Filter hallucinations (The "Airplane" fix)
                # Increase this if you still see fake detections
                if conf < 0.7:
                    continue

                # 3. Get the Depth at the center of the box
                u_center = int((x1 + x2) / 2)
                v_center = int((y1 + y2) / 2)
                
                # Use a small window (e.g., 5x5) to average depth 
                # This is more stable than a single pixel which might be noise
                depth_roi = depth_img[v_center-2:v_center+2, u_center-2:u_center+2]
                z_depth = np.nanmean(depth_roi)

                # 4. Convert to 3D Camera Coordinates
                coords_3d = self.get_3d_camera_coords(u_center, v_center, z_depth, K)

                if coords_3d:
                    x_c, y_c, z_c = coords_3d
                    
                    # Check for "Sanity" - ignore things too far or too close
                    if 0.1 < z_c < 8.0:
                        output_msg = f"{robot_name} detected {label} at Cam Coords: X={x_c:.2f}, Y={y_c:.2f}, Z={z_c:.2f}"
                        self.get_logger().info(output_msg)

                        # 5. Draw the results on the debug image
                        color = (0, 255, 0) # Green
                        cv2.rectangle(color_img, (x1, y1), (x2, y2), color, 2)
                        text = f"{label} {x_c:.1f}m"
                        cv2.putText(color_img, text, (x1, y1 - 10),cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                        with open(self.log_file, 'a') as f:
                            f.write(f"{label},{x_c:.4f},{y_c:.4f},{z_c:.4f},{rx:.4f},{ry:.4f},{rz:.4f},{r_yaw:.4f}\n")

        # Show the debug window (optional, but keep it for now)
        cv2.imshow(f"{robot_name} Color", color_img)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = MultiRobotMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
