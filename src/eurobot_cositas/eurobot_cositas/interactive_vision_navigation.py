#!/usr/bin/env python3
import math
import sys
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import tf2_ros


class InteractiveVisionNavigation(Node):
    def __init__(self):
        super().__init__('interactive_vision_navigation')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.state = 'IDLE'
        self.target_id = None

        self.timer = self.create_timer(0.05, self.control_loop)

        # Start input thread
        threading.Thread(target=self.user_interface, daemon=True).start()

        self.get_logger().info("Interactive overhead-camera navigation READY")

    # ---------------- USER INTERFACE ----------------

    def user_interface(self):
        while rclpy.ok():
            cmd = input(
                "\nCommands:\n"
                "  list               → list visible ArUcos\n"
                "  select <id>        → select target (e.g. select 20)\n"
                "  rotate             → rotate toward target\n"
                "  go                 → move straight to target\n"
                "  stop               → stop robot\n"
                "  quit               → exit\n"
                "> "
            ).strip()

            if cmd == 'list':
                self.list_arucos()

            elif cmd.startswith('select'):
                try:
                    self.target_id = f"aruco_{cmd.split()[1]}"
                    self.get_logger().info(f"Selected target: {self.target_id}")
                except Exception:
                    print("Invalid select command")

            elif cmd == 'rotate':
                if self.target_id:
                    self.state = 'ROTATE'
                else:
                    print("No target selected")

            elif cmd == 'go':
                if self.target_id:
                    self.state = 'MOVE'
                else:
                    print("No target selected")

            elif cmd == 'stop':
                self.state = 'IDLE'
                self.publish_stop()

            elif cmd == 'quit':
                rclpy.shutdown()
                sys.exit(0)

    # ---------------- TF HELPERS ----------------

    def get_relative_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                'aruco_robot',
                self.target_id,
                rclpy.time.Time()
            )
            x = tf.transform.translation.x
            y = tf.transform.translation.y
            dist = math.hypot(x, y)
            angle = math.atan2(y, x)
            return x, y, dist, angle
        except Exception:
            return None

    def list_arucos(self):
        print("\nVisible ArUcos:")
        for i in range(0, 50):
            frame = f"aruco_{i}"
            try:
                self.tf_buffer.lookup_transform(
                    'aruco_robot',
                    frame,
                    rclpy.time.Time()
                )
                print(f"  {frame}")
            except Exception:
                pass

    # ---------------- CONTROL LOOP ----------------

    def control_loop(self):
        if self.state == 'IDLE' or not self.target_id:
            return

        pose = self.get_relative_pose()
        if pose is None:
            self.publish_stop()
            return

        x, y, dist, angle = pose

        if self.state == 'ROTATE':
            if abs(angle) < 0.05:
                self.publish_stop()
                self.state = 'IDLE'
                self.get_logger().info("Rotation complete")
            else:
                cmd = Twist()
                cmd.angular.z = 1.5 * angle
                self.cmd_pub.publish(cmd)

        elif self.state == 'MOVE':
            if dist < 0.05:
                self.publish_stop()
                self.state = 'IDLE'
                self.get_logger().info("Target reached")
            else:
                cmd = Twist()
                cmd.linear.x = 0.2
                self.cmd_pub.publish(cmd)

    def publish_stop(self):
        self.cmd_pub.publish(Twist())


def main():
    rclpy.init()
    node = InteractiveVisionNavigation()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
