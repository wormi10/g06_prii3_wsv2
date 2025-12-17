#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion


class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')

        # -------- USER-DEFINED WAYPOINTS (meters) --------
        # Order: A → B → C → D → E → back to start
        self.waypoints = [
            ("20",  -0.622,  -0.902),
            ("21",  -0.625,  0.891),
            ("22",  0.181,  -0.894),
            ("23", 0.187,  0.898),
            ("02", -1.084,  -1.240),
        ]

        # Control parameters
        self.pos_tolerance = 0.05      # meters
        self.angle_tolerance = 0.10    # radians
        self.k_linear = 0.6
        self.k_angular = 1.5

        # State
        self.current_pose = None
        self.start_pose = None
        self.current_index = 0
        self.returning_home = False

        # ROS interfaces
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )

        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info("Waypoint navigation READY")

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        yaw = euler_from_quaternion([
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        ])[2]

        self.current_pose = (position.x, position.y, yaw)

        if self.start_pose is None:
            self.start_pose = self.current_pose
            self.get_logger().info(
                f"Start pose saved: x={position.x:.2f}, y={position.y:.2f}"
            )

    def control_loop(self):
        if self.current_pose is None:
            return

        if self.returning_home:
            target_x, target_y = self.start_pose[0], self.start_pose[1]
            target_name = "HOME"
        else:
            if self.current_index >= len(self.waypoints):
                self.get_logger().info("All waypoints reached → returning home")
                self.returning_home = True
                return

            target_name, target_x, target_y = self.waypoints[self.current_index]

        x, y, yaw = self.current_pose

        dx = target_x - x
        dy = target_y - y

        distance = math.hypot(dx, dy)
        target_angle = math.atan2(dy, dx)
        angle_error = self.normalize_angle(target_angle - yaw)

        cmd = Twist()

        # --- Arrival check ---
        if distance < self.pos_tolerance:
            self.cmd_pub.publish(Twist())

            if self.returning_home:
                self.get_logger().info("Returned to start. DONE.")
                rclpy.shutdown()
                return

            self.get_logger().info(f"Reached waypoint {target_name}")
            self.current_index += 1
            return

        # --- Control ---
        if abs(angle_error) > self.angle_tolerance:
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
    node = WaypointNavigator()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
