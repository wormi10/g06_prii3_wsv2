#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion


class GoToXY(Node):
    def __init__(self):
        super().__init__('goto_xy')

        # Declare parameters (can be set from terminal)
        self.declare_parameter('target_x', 0.0)
        self.declare_parameter('target_y', 0.0)

        self.target_x = self.get_parameter('target_x').value
        self.target_y = self.get_parameter('target_y').value

        self.get_logger().info(
            f"Target set to x={self.target_x}, y={self.target_y}"
        )

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)

        self.timer = self.create_timer(0.05, self.control_loop)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.odom_received = False

    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        _, _, self.yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        self.odom_received = True

    def control_loop(self):
        if not self.odom_received:
            return

        dx = self.target_x - self.x
        dy = self.target_y - self.y
        distance = math.hypot(dx, dy)

        target_yaw = math.atan2(dy, dx)
        yaw_error = self.normalize_angle(target_yaw - self.yaw)

        cmd = Twist()

        # Stop if reached
        if distance < 0.05:
            self.cmd_pub.publish(Twist())
            self.get_logger().info("Target reached")
            return

        # Rotate first
        if abs(yaw_error) > 0.2:
            cmd.angular.z = 1.2 * yaw_error
            cmd.linear.x = 0.0
        else:
            cmd.linear.x = min(0.4, distance)
            cmd.angular.z = 0.6 * yaw_error

        self.cmd_pub.publish(cmd)

    @staticmethod
    def normalize_angle(a):
        while a > math.pi:
            a -= 2 * math.pi
        while a < -math.pi:
            a += 2 * math.pi
        return a


def main():
    rclpy.init()
    node = GoToXY()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
