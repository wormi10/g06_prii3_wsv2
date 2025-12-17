#!/usr/bin/env python3

import math
import rclpy
import threading
import random
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion


class InteractiveWaypointNavigator(Node):
    def __init__(self):
        super().__init__('interactive_waypoint_navigation')

        # ---------------- ARUCO POSITIONS (FIXED MAP COORDS) ----------------
        self.aruco_positions = {
            "20": (-0.622, -0.902),
            "21": (-0.625,  0.891),
            "22": ( 0.181, -0.894),
            "23": ( 0.187,  0.898),
            "02": (-1.084, -1.240),
        }

        # ---------------- CONTROL PARAMS ----------------
        self.pos_tol = 0.05
        self.ang_tol = 0.10
        self.k_lin = 0.6
        self.k_ang = 1.5

        # ---------------- STATE ----------------
        self.current_pose = None
        self.start_pose = None
        self.goal = None
        self.paused = False
        self.sequence = []

        # ---------------- ROS ----------------
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_cb, 10
        )

        self.timer = self.create_timer(0.05, self.control_loop)

        # ---------------- MENU THREAD ----------------
        threading.Thread(target=self.menu_loop, daemon=True).start()

        self.get_logger().info("Interactive waypoint navigator READY")

    # ============================================================
    # ODOM
    # ============================================================
    def odom_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        self.current_pose = (p.x, p.y, yaw)

        if self.start_pose is None:
            self.start_pose = self.current_pose
            self.get_logger().info(
                f"Start pose saved: x={p.x:.2f}, y={p.y:.2f}"
            )

    # ============================================================
    # CONTROL LOOP
    # ============================================================
    def control_loop(self):
        if self.paused or self.goal is None or self.current_pose is None:
            return

        x, y, yaw = self.current_pose
        gx, gy = self.goal

        dx = gx - x
        dy = gy - y

        dist = math.hypot(dx, dy)
        target_yaw = math.atan2(dy, dx)
        yaw_err = self.norm_angle(target_yaw - yaw)

        cmd = Twist()

        if dist < self.pos_tol:
            self.get_logger().info("Goal reached")
            self.goal = None

            if self.sequence:
                self.goal = self.sequence.pop(0)
                self.get_logger().info(f"Next goal: {self.goal}")
            else:
                self.cmd_pub.publish(Twist())
            return

        if abs(yaw_err) > self.ang_tol:
            cmd.angular.z = self.k_ang * yaw_err
        else:
            cmd.linear.x = self.k_lin * dist
            cmd.angular.z = self.k_ang * yaw_err

        self.cmd_pub.publish(cmd)

    # ============================================================
    # MENU
    # ============================================================
    def menu_loop(self):
        while rclpy.ok():
            print("\n========== MENU ==========")
            print("1) Rotate toward ArUco")
            print("2) Go to ArUco")
            print("3) Visit ALL ArUcos (ordered)")
            print("4) Visit ALL ArUcos (random)")
            print("5) STOP")
            print("6) RESUME")
            print("7) Return HOME")
            print("q) Quit")
            print("==========================")

            choice = input("Select: ").strip()

            if choice == "1":
                self.rotate_toward()

            elif choice == "2":
                self.go_to_single()

            elif choice == "3":
                self.sequence = list(self.aruco_positions.values())
                self.goal = self.sequence.pop(0)
                self.paused = False
                self.get_logger().info("Visiting all ArUcos (ordered)")

            elif choice == "4":
                self.sequence = list(self.aruco_positions.values())
                random.shuffle(self.sequence)
                self.goal = self.sequence.pop(0)
                self.paused = False
                self.get_logger().info("Visiting all ArUcos (random)")

            elif choice == "5":
                self.paused = True
                self.cmd_pub.publish(Twist())
                self.get_logger().info("STOPPED")

            elif choice == "6":
                self.paused = False
                self.get_logger().info("RESUMED")

            elif choice == "7":
                if self.start_pose:
                    self.goal = (self.start_pose[0], self.start_pose[1])
                    self.paused = False
                    self.get_logger().info("Returning HOME")

            elif choice == "q":
                self.cmd_pub.publish(Twist())
                rclpy.shutdown()
                return

    # ============================================================
    # ACTIONS
    # ============================================================
    def rotate_toward(self):
        aid = input("ArUco ID: ").strip()
        if aid not in self.aruco_positions or self.current_pose is None:
            return

        gx, gy = self.aruco_positions[aid]
        x, y, yaw = self.current_pose
        angle = math.atan2(gy - y, gx - x)
        err = self.norm_angle(angle - yaw)

        cmd = Twist()
        cmd.angular.z = self.k_ang * err
        self.cmd_pub.publish(cmd)

        self.get_logger().info(f"Rotating toward ArUco {aid}")

    def go_to_single(self):
        aid = input("ArUco ID: ").strip()
        if aid not in self.aruco_positions:
            return

        self.goal = self.aruco_positions[aid]
        self.sequence = []
        self.paused = False
        self.get_logger().info(f"Going to ArUco {aid}")

    @staticmethod
    def norm_angle(a):
        while a > math.pi:
            a -= 2 * math.pi
        while a < -math.pi:
            a += 2 * math.pi
        return a


def main():
    rclpy.init()
    node = InteractiveWaypointNavigator()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
