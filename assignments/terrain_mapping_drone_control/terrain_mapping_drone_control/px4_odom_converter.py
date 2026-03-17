#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from px4_msgs.msg import VehicleOdometry
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import TransformStamped
import tf2_ros

class PX4OdomConverter(Node):
    """
    Converts PX4 VehicleOdometry (NED, px4_msgs) to nav_msgs/Odometry (ENU, ROS standard)
    and publishes the odom -> base_link TF required by rtabmap.
    Uses camera info timestamp to stay in simulation time domain.
    """
    def __init__(self):
        super().__init__('px4_odom_converter')

        px4_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribe to PX4 odometry
        self.sub = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.odom_callback,
            px4_qos
        )

        # Subscribe to camera info to get sim timestamps
        self.camera_sub = self.create_subscription(
            CameraInfo,
            '/drone/front_rgb/camera_info',
            self.camera_info_callback,
            10
        )

        self.odom_pub = self.create_publisher(Odometry, '/drone/odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.last_sim_stamp = None
        self.get_logger().info('PX4 odometry converter started')

    def camera_info_callback(self, msg):
        """Store latest sim timestamp from camera info."""
        self.last_sim_stamp = msg.header.stamp

    def odom_callback(self, msg):
        """
        Convert PX4 NED frame to ROS ENU frame and publish.

        PX4 NED:  X=North, Y=East,  Z=Down
        ROS ENU:  X=East,  Y=North, Z=Up

        Position:  ENU_x =  NED_y
                   ENU_y =  NED_x
                   ENU_z = -NED_z

        Quaternion (NED->ENU):
                   ENU_w =  NED_w
                   ENU_x =  NED_y
                   ENU_y =  NED_x
                   ENU_z = -NED_z
        """
        # Wait until we have a valid sim timestamp from the camera
        if self.last_sim_stamp is None:
            return

        now = self.last_sim_stamp

        # NED -> ENU position conversion
        enu_x =  float(msg.position[1])
        enu_y =  float(msg.position[0])
        enu_z = -float(msg.position[2])

        # NED -> ENU quaternion conversion
        enu_qw =  float(msg.q[0])
        enu_qx =  float(msg.q[2])
        enu_qy =  float(msg.q[1])
        enu_qz = -float(msg.q[3])

        # Publish nav_msgs/Odometry
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = enu_x
        odom.pose.pose.position.y = enu_y
        odom.pose.pose.position.z = enu_z
        odom.pose.pose.orientation.w = enu_qw
        odom.pose.pose.orientation.x = enu_qx
        odom.pose.pose.orientation.y = enu_qy
        odom.pose.pose.orientation.z = enu_qz
        odom.pose.covariance[0]  = 0.01
        odom.pose.covariance[7]  = 0.01
        odom.pose.covariance[14] = 0.01
        odom.pose.covariance[21] = 0.001
        odom.pose.covariance[28] = 0.001
        odom.pose.covariance[35] = 0.001
        self.odom_pub.publish(odom)

        # Publish odom -> base_link TF
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = enu_x
        t.transform.translation.y = enu_y
        t.transform.translation.z = enu_z
        t.transform.rotation.w = enu_qw
        t.transform.rotation.x = enu_qx
        t.transform.rotation.y = enu_qy
        t.transform.rotation.z = enu_qz
        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = PX4OdomConverter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
