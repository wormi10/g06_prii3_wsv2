#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import tf2_ros
from tf_transformations import euler_from_quaternion


class VisionWorldNavigator(Node):
    def __init__(self):
        super().__init__('vision_world_navigator')

        # -------- PARAMETERS --------
        self.declare_parameter('robot_marker_id', 3)
        self.declare_parameter('target_marker_id', 20)

        self.robot_id = self.get_parameter('robot_marker_id').value
        self.target_id = self.get_parameter('target_marker_id').value

        self.world_frame = 'world'
        self.robot_frame = f'aruco_{self.robot_id}'
        self.target_frame = f'aruco_{self.target_id}'

        # Control gains
        self.k_linear = 0.6
        self.k_angular = 2.0
        self.pos_tol = 0.05      # meters
        self.angle_tol = 0.10    # radians

        # ROS interfaces
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info("Vision WORLD navigation node started")
        self.get_logger().info(f"Robot frame: {self.robot_frame}")
        self.get_logger().info(f"Target frame: {self.target_frame}")

    def control_loop(self):
        try:
            tf_robot = self.tf_buffer.lookup_transform(
                self.world_frame,
                self.robot_frame,
                rclpy.time.Time()
            )

            tf_target = self.tf_buffer.lookup_transform(
                self.world_frame,
                self.target_frame,
                rclpy.time.Time()
            )

        except Exception:
            self.cmd_pub.publish(Twist())
            return

        # --- Robot pose ---
        rx = tf_robot.transform.translation.x
        ry = tf_robot.transform.translation.y

        q = tf_robot.transform.rotation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        # --- Target pose ---
        tx = tf_target.transform.translation.x
        ty = tf_target.transform.translation.y

        dx = tx - rx
        dy = ty - ry

        distance = math.hypot(dx, dy)
        target_angle = math.atan2(dy, dx)
        angle_error = self.normalize_angle(target_angle - yaw)

        cmd = Twist()

        # --- Arrival ---
        if distance < self.pos_tol:
            self.cmd_pub.publish(Twist())
            self.get_logger().info(
                f"Reached ArUco {self.target_id} at ({tx:.2f}, {ty:.2f})",
                throttle_duration_sec=2.0
            )
            return

        # --- Control ---
        if abs(angle_error) > self.angle_tol:
            cmd.angular.z = self.k_angular * angle_error
            cmd.linear.x = 0.0
        else:
            cmd.linear.x = self.k_linear * distance
            cmd.angular.z = self.k_angular * angle_error

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
    node = VisionWorldNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
