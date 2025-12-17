#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import tf2_ros
import numpy as np
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R

# Known board coordinates (meters)
BOARD_POINTS = {
    "aruco_20": np.array([ 1.5,  1.0]),
    "aruco_21": np.array([-1.5,  1.0]),
    "aruco_22": np.array([ 1.5, -1.0]),
    "aruco_23": np.array([-1.5, -1.0]),
}

class BoardFramePublisher(Node):
    def __init__(self):
        super().__init__('board_frame_publisher')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.timer = self.create_timer(0.5, self.update_board_frame)
        self.get_logger().info("Board frame publisher started")

    def update_board_frame(self):
        camera_pts = []
        board_pts = []

        for aruco, board_xy in BOARD_POINTS.items():
            try:
                tf = self.tf_buffer.lookup_transform(
                    "overhead_camera_link",
                    aruco,
                    rclpy.time.Time()
                )
                camera_pts.append([
                    tf.transform.translation.x,
                    tf.transform.translation.y
                ])
                board_pts.append(board_xy)
            except Exception:
                return

        camera_pts = np.array(camera_pts)
        board_pts = np.array(board_pts)

        # Compute 2D rigid transform (no scale)
        H = self.compute_rigid_transform(camera_pts, board_pts)

        tx, ty = H[:2, 2]
        yaw = np.arctan2(H[1,0], H[0,0])

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "overhead_camera_link"
        t.child_frame_id = "board_frame"

        t.transform.translation.x = tx
        t.transform.translation.y = ty
        t.transform.translation.z = 0.0

        q = R.from_euler('z', yaw).as_quat()
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)

    def compute_rigid_transform(self, A, B):
        centroid_A = A.mean(axis=0)
        centroid_B = B.mean(axis=0)

        AA = A - centroid_A
        BB = B - centroid_B

        H = AA.T @ BB
        U, _, Vt = np.linalg.svd(H)
        Rm = Vt.T @ U.T

        if np.linalg.det(Rm) < 0:
            Vt[1,:] *= -1
            Rm = Vt.T @ U.T

        t = centroid_B - Rm @ centroid_A

        T = np.eye(3)
        T[:2,:2] = Rm
        T[:2,2] = t
        return T


def main():
    rclpy.init()
    rclpy.spin(BoardFramePublisher())
