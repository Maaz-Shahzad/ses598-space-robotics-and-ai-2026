import os
import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R
import numpy as np

class Robot1Controller(Node):
    def __init__(self):
        super().__init__('robot1_controller')
        self.get_logger().info("robot1_controller started")
        self.pub = self.create_publisher(Twist, '/robot1/cmd_vel', 10)
        self.create_subscription(Odometry,'/robot1/odom',self.odom_callback,10)
        self.timer = self.create_timer(0.1, self.control_loop)

        self.state = "forward"
        self.state_start = self.get_clock().now().nanoseconds * 1e-9

        self.forward_time = 3.0
        self.turn_time = 1.5

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.dt_info_print = 2

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = msg.pose.pose.position.z

        q = msg.pose.pose.orientation
#        r = R.from_quat([q.x, q.y, q.z, q.w])
#        roll, pitch, yaw = r.as_euler('xyz', degrees=False)

        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

#        self.yaw = yaw

#        self.get_logger().info(f"Robot1 position: x={x}, y={y}, z={z}")    

#    def control_loop(self):
#
#        msg = Twist()
#        now = self.get_clock().now().nanoseconds * 1e-9
#        dt = now - self.state_start
#
#        if self.state == "forward":
#            msg.linear.x = 0.3
#            msg.angular.z = 0.0
#            if int(now * 10) % 10 == 0:
#                self.get_logger().info("state = forward")
#
#            if self.x > 6.0:
#                self.state = "turn"
#                self.state_start = now
#
#        elif self.state == "turn":
#            msg.linear.x = 0.0
#            msg.angular.z = 0.8
#            if int(now * 10) % 10 == 0:
#                self.get_logger().info("state = turn")
#
#            target_yaw = math.pi
#            turn_error = 5.0 * math.pi/180
#            diff = target_yaw - self.yaw
#            # Normalize to [-pi, pi] to handle the jump from 3.14 to -3.14
#            diff = (diff + math.pi) % (2 * math.pi) - math.pi
#            if abs(diff) < turn_error:
#                self.state = "back"
#                self.state_start = now
#
#        elif self.state == "back":
#            msg.linear.x = 0.3
#            msg.angular.z = 0.0
#            if int(now * 10) % 10 == 0:
#                self.get_logger().info("state = back")
#            if self.x < -0.1:
#                self.state = "stop"
#                self.state_start = now
#
#        elif self.state == "stop":
#            msg.linear.x = 0.0
#            msg.angular.z = 0.0
#
#        self.pub.publish(msg)
#
#        if int(now * 10) % 10 == 0:
#            self.get_logger().info("############## Publishing cmd_vel ##############")
#            self.get_logger().info(f"x = {self.x}")
#            yaw_deg = self.yaw * 180/math.pi
#            self.get_logger().info(f"yaw_deg = {yaw_deg}")
#            self.get_logger().info(f"yaw = {self.yaw}\n")

    def wrap_angle(self, a):
        return math.atan2(math.sin(a), math.cos(a))

    def angle_error(self, target, current):
        return np.arctan2(np.sin(target - current), np.cos(target - current))

    def control_loop(self):

        msg = Twist()
        now = self.get_clock().now().nanoseconds * 1e-9
        dt = now - self.state_start

        # ---------------- PARAMETERS (tune these) ----------------
        k_rho = 2.0        # forward speed gain
        k_alpha = 1.0      # heading gain
        k_turn = 2.0       # pure rotation gain

        goal_x = 5.0
        goal_y = 0.0

        return_x = 0.0
        return_y = 0.0

        dist_thresh = 0.1
        yaw_thresh = 1.0 * math.pi / 180.0

        # ---------------- CURRENT STATE ----------------
        if self.state == "forward":

            dx = goal_x - self.x
            dy = goal_y - self.y

            rho = math.sqrt(dx*dx + dy*dy)
            target_yaw = math.atan2(dy, dx)

            alpha = target_yaw - self.yaw
            alpha = self.wrap_angle(target_yaw - self.yaw)

            # PD-style control
            msg.linear.x = k_rho * rho
            msg.angular.z = k_alpha * alpha

            # limit speed
            msg.linear.x = min(msg.linear.x, 0.5)

#            if int(self.get_clock().now().nanoseconds * 1e-9 * 10) % self.dt_info_print == 0:
#                self.get_logger().info("state = forward")
#                self.get_logger().info(f"rho = {rho:.3f}, alpha = {alpha:.3f}")
#
            if rho < dist_thresh:
                self.state = "turn"
                self.yaw_initial = self.yaw
                self.state_start = now
#                self.get_logger().info("Reached goal → turning 180°")

        # ---------------- TURN 180 ----------------
        elif self.state == "turn":
#  1. 
            target_yaw = self.wrap_angle(self.yaw_initial + math.pi)
            error = self.wrap_angle(target_yaw - self.yaw)
    
            msg.linear.x = 0.0
            msg.angular.z = k_turn * self.angle_error(target_yaw, self.yaw) # Yet to check this
            #msg.angular.z = k_turn * error
            msg.angular.z = max(min(k_turn * error, 0.8), -0.8)

#            if int(self.get_clock().now().nanoseconds * 1e-9 * 10) % self.dt_info_print == 0:
#                self.get_logger().info("state = turn")
#                self.get_logger().info(f"yaw_error = {error:.3f}")
#                self.get_logger().info(f"yaw_error_deg = {error * 57.3:.3f}")
            if abs(error) < yaw_thresh:
                msg.angular.z = 0.0
                self.state = "back"
#                self.get_logger().info("Turn complete → returning")
#   2.
#            msg.angular.z = 1.0
#
#            if int(self.get_clock().now().nanoseconds * 1e-9 * 10) % self.dt_info_print == 0:
#                self.get_logger().info("state = turn")
#                self.get_logger().info(f"dt = {dt}")
#            if dt > 5.0:
#                msg.angular.z = 0.2
#                self.state = "back"
#                self.get_logger().info(f"EXIT TURN yaw = {self.yaw}")
#                self.get_logger().info("Turn complete → returning")

        # ---------------- RETURN ----------------
        elif self.state == "back":

#            dx = return_x - self.x
#            dy = return_y - self.y
#
#            rho = math.sqrt(dx*dx + dy*dy)
#            target_yaw = math.atan2(dy, dx)
#
#            alpha1 = target_yaw - self.yaw
#            alpha2 = (alpha1 + math.pi) % (2 * math.pi) - math.pi
#
#            msg.linear.x = k_rho * rho
#            msg.angular.z = k_alpha * alpha2
#
#            msg.linear.x = min(msg.linear.x, 0.5)

            dx = return_x - self.x
            dy = return_y - self.y

            rho = math.sqrt(dx*dx + dy*dy)
            target_yaw = math.atan2(dy, dx)

            alpha = target_yaw - self.yaw
            alpha = self.wrap_angle(target_yaw - self.yaw)

            # STEP 1: rotate until aligned
            if abs(alpha) > yaw_thresh:
                msg.linear.x = 0.0
                msg.angular.z = k_alpha * alpha
#                msg.angular.z = 0.8  #Rotate for camera testing

            # STEP 2: move forward only when aligned
            else:
                msg.linear.x = min(k_rho * rho, 0.5)
                msg.angular.z = k_alpha * alpha  # small correction

#                msg.linear.x = 0.0  #Stop for camera testing
#                msg.angular.z = 0.8  #Rotate for camera testing


#            if int(self.get_clock().now().nanoseconds * 1e-9 * 10) % self.dt_info_print == 0:
#                self.get_logger().info("state = back")
#                self.get_logger().info(f"rho = {rho:.3f}, alpha1 = {alpha1:.3f}, alpha2 = {alpha2:.3f}")
#                self.get_logger().info(f"rho = {rho:.3f}, alpha = {alpha:.3f}")
#                self.get_logger().info(f"target_yaw_deg = {target_yaw * 180/math.pi:.2f}")
#                self.get_logger().info(f"msg.angular.z = {msg.angular.z}")

            if rho < dist_thresh:
                self.state = "stop"
                self.state_start = now
#                self.get_logger().info("Returned to origin")

        # ---------------- STOP ----------------
        elif self.state == "stop":
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            if dt > 3:
                raise SystemExit
            

        self.pub.publish(msg)

        # debug print
#        if int(self.get_clock().now().nanoseconds * 1e-9 * 10) % self.dt_info_print == 0:
#            self.get_logger().info("Publishing cmd_vel")
#            self.get_logger().info(f"x = {self.x:.3f}, y = {self.y:.3f}")
#            self.get_logger().info(f"yaw_deg = {self.yaw * 180/math.pi:.2f}\n")
#            cos_yaw = math.cos(self.yaw)
#            sin_yaw = math.sin(self.yaw)
#            self.get_logger().info(f"x_dot direction ≈ {cos_yaw}, {sin_yaw}")



def main():
    rclpy.init()
    node = Robot1Controller()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

