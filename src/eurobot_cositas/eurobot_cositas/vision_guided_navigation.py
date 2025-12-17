#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import tf2_ros

TARGET_ARUCO = "aruco_20"
WORLD = "overhead_camera_link"

class VisionGuidedNavigation(Node):
    def __init__(self):
        super().__init__('vision_guided_navigation')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.1, self.loop)

        self.state = "ROTATE"

        self.get_logger().info("DISCRETE VISION NAVIGATION ACTIVE")

    def get_xy(self, frame):
        tf = self.tf_buffer.lookup_transform(WORLD, frame, rclpy.time.Time())
        return tf.transform.translation.x, tf.transform.translation.y

    def loop(self):
        try:
            xr, yr = self.get_xy("aruco_robot")
            xt, yt = self.get_xy(TARGET_ARUCO)
        except Exception:
            self.cmd_pub.publish(Twist())
            return

        ex = xt - xr
        ey = yt - yr
        distance = math.hypot(ex, ey)

        cmd = Twist()

        # STOP
        if distance < 0.06:
            self.cmd_pub.publish(Twist())
            self.get_logger().info("TARGET REACHED")
            return

        # --------- STATE MACHINE ---------

        if self.state == "ROTATE":
            # Rotate until target roughly ahead (|ey| small)
            if abs(ey) < 0.02:
                self.state = "FORWARD"
                return

            cmd.angular.z = 0.4 if ey > 0 else -0.4
            self.cmd_pub.publish(cmd)
            return

        if self.state == "FORWARD":
            # Drive straight only
            cmd.linear.x = 0.2
            self.cmd_pub.publish(cmd)

            # If drifted sideways, stop and re-rotate
            if abs(ey) > 0.05:
                self.state = "ROTATE"
            return

def main():
    rclpy.init()
    node = VisionGuidedNavigation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
