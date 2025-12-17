#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import tf2_ros


ARUCO_IDS = [2, 20, 21, 22, 23]
REFERENCE_FRAME = "overhead_camera_link"


class ArucoMapViewer(Node):
    def __init__(self):
        super().__init__('aruco_map_viewer')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(1.0, self.timer_cb)

        self.get_logger().info("Aruco map viewer started")

    def timer_cb(self):
        self.get_logger().info("---- ArUco positions ----")

        for aruco_id in ARUCO_IDS:
            frame = f"aruco_{aruco_id}"

            try:
                tf = self.tf_buffer.lookup_transform(
                    REFERENCE_FRAME,
                    frame,
                    rclpy.time.Time()
                )

                x = tf.transform.translation.x
                y = tf.transform.translation.y
                z = tf.transform.translation.z

                self.get_logger().info(
                    f"{frame}: x={x:.3f}, y={y:.3f}, z={z:.3f}"
                )

            except Exception:
                self.get_logger().warn(f"{frame}: not visible")


def main():
    rclpy.init()
    node = ArucoMapViewer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
