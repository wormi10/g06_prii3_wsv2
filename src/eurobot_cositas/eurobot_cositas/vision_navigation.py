#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener
import math


class VisionGuidedNavigation(Node):
    def __init__(self):
        super().__init__('vision_guided_navigation')

        # ===================== PARAMETERS =====================
        self.declare_parameter('reference_frame', 'aruco_2')   # world origin
        self.declare_parameter('robot_frame', 'aruco_3')       # robot marker
        self.declare_parameter('target_aruco_id', 20)          # target marker

        self.declare_parameter('linear_speed', 0.12)
        self.declare_parameter('angular_gain', 1.5)
        self.declare_parameter('distance_tolerance', 0.20)

        self.reference_frame = self.get_parameter('reference_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.target_id = self.get_parameter('target_aruco_id').value
        self.target_frame = f'aruco_{self.target_id}'

        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_gain = self.get_parameter('angular_gain').value
        self.distance_tolerance = self.get_parameter('distance_tolerance').value

        # ===================== ROS INTERFACES =====================
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Control loop at 10 Hz
        self.timer = self.create_timer(0.1, self.control_loop)

        self.reached = False

        self.get_logger().info(
            f"Vision-guided navigation ACTIVE → Target ArUco {self.target_id}"
        )

    # ==========================================================
    def control_loop(self):
        if self.reached:
            self.publish_stop()
            return

        try:
            robot_tf = self.tf_buffer.lookup_transform(
                self.reference_frame,
                self.robot_frame,
                rclpy.time.Time()
            )

            target_tf = self.tf_buffer.lookup_transform(
                self.reference_frame,
                self.target_frame,
                rclpy.time.Time()
            )

        except Exception:
            # TF not ready yet
            self.publish_stop()
            return

        # ---------------- Relative position ----------------
        dx = target_tf.transform.translation.x - robot_tf.transform.translation.x
        dy = target_tf.transform.translation.y - robot_tf.transform.translation.y

        distance = math.sqrt(dx * dx + dy * dy)
        target_angle = math.atan2(dy, dx)

        # ---------------- Robot yaw ----------------
        q = robot_tf.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

        angle_error = math.atan2(
            math.sin(target_angle - yaw),
            math.cos(target_angle - yaw)
        )

        cmd = Twist()

        # ---------------- Control law ----------------
        if distance > self.distance_tolerance:
            cmd.linear.x = self.linear_speed
            cmd.angular.z = self.angular_gain * angle_error
        else:
            self.get_logger().info(
                f"Target ArUco {self.target_id} reached"
            )
            self.reached = True
            self.publish_stop()
            return

        self.cmd_vel_pub.publish(cmd)

    # ==========================================================
    def publish_stop(self):
        self.cmd_vel_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = VisionGuidedNavigation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
