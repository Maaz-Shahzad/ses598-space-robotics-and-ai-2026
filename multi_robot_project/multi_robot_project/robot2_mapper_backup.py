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
        
        # Synchronize Robot 2's RGB and Depth
        self.r2_sync = ApproximateTimeSynchronizer(
            [self.r2_rgb_sub, self.r2_depth_sub],
            queue_size=10,
            slop=0.3
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
        self.frame_skip = 10

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

        x = (u - cx) * depth / fx
        y = (v - cy) * depth / fy
        z = depth
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

    def robot2_callback(self, rgb_msg, depth_msg):
        """Processes synchronized data from Robot 2"""
        self.frame_count += 1
        if self.frame_count % self.frame_skip != 0:
            return
        try:
            cv_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
            cv_depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')
            
            self.process_logic("Robot 2", cv_image, cv_depth)
            
        except Exception as e:
            self.get_logger().error(f"Error in Robot 2 callback: {e}")

    def process_logic(self, robot_name, color_img, depth_img):
        # Determine which intrinsics to use
#        K = self.r1_intrinsics if robot_name == "Robot 1" else self.r2_intrinsics
        K = self.r2_intrinsics
        if K is None:
            self.get_logger().warn(f"Waiting for {robot_name} CameraInfo...")
            return

#        results = self.model(color_img, conf=0.5, verbose=False)
#        
#        # persist=True keeps the same ID on the cone as the robot moves
#        # results = self.model.track(color_img, conf=0.7, persist=True, verbose=False)
#
#        for result in results:
#            for box in result.boxes:
#                x1, y1, x2, y2 = map(int, box.xyxy[0])
#                u, v = int((x1 + x2) / 2), int((y1 + y2) / 2)
#                
#                dist = depth_img[v, u]
#                
#                # Convert to 3D!
#                coords_3d = self.get_3d_camera_coords(u, v, dist, K)
#                
#                if coords_3d:
#                    x, y, z = coords_3d
#                    label = f"{self.model.names[int(box.cls[0])]}: [{x:.1f}, {y:.1f}, {z:.1f}]m"
#                    self.get_logger().info(f"{robot_name} sees object at CAM coords: {label}")
#                    
#                    # Draw on screen
#                    cv2.rectangle(color_img, (x1, y1), (x2, y2), (255, 0, 0), 2)
#                    cv2.putText(color_img, label, (x1, y1 - 10), 
#                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

        cv2.imshow(f"{robot_name} Mapping", color_img)
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
