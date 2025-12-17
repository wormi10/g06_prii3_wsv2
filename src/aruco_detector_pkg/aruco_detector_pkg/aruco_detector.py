#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import cv2
import cv2.aruco as aruco
import numpy as np

from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class ArucoTableDetector(Node):

    def __init__(self):
        super().__init__('aruco_table_detector')

        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/overhead_camera/image_raw',
            self.image_callback,
            10
        )

        self.info_sub = self.create_subscription(
            CameraInfo,
            '/overhead_camera/camera_info',
            self.camera_info_callback,
            10
        )

        self.bridge = CvBridge()
        self.tf_broadcaster = TransformBroadcaster(self)

        # ArUco (OpenCV 4.7 API)
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.aruco_params = aruco.DetectorParameters()

        # Camera parameters (filled from CameraInfo)
        self.camera_matrix = None
        self.dist_coeffs = None

        self.marker_size = 0.10  # meters (Eurobot table)

        self.get_logger().info("ArUco TABLE detector started (PBI 4.2).")

    # ---------------- Camera Info ----------------
    def camera_info_callback(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info("Camera calibration received.")

    # ---------------- Image ----------------
    def image_callback(self, msg):
        if self.camera_matrix is None:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = aruco.detectMarkers(
            gray,
            self.aruco_dict,
            parameters=self.aruco_params
        )

        if ids is None:
            return

        for i, marker_id in enumerate(ids.flatten()):
            self.process_marker(marker_id, corners[i], msg.header.stamp)

    # ---------------- Marker processing ----------------
    def process_marker(self, marker_id, corners, stamp):
        half = self.marker_size / 2.0

        obj_points = np.array([
            [-half,  half, 0.0],
            [ half,  half, 0.0],
            [ half, -half, 0.0],
            [-half, -half, 0.0]
        ], dtype=np.float32)

        img_points = corners.reshape(4, 2)

        success, rvec, tvec = cv2.solvePnP(
            obj_points,
            img_points,
            self.camera_matrix,
            self.dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE
        )

        if not success or np.isnan(rvec).any() or np.isnan(tvec).any():
            return

        R, _ = cv2.Rodrigues(rvec)
        quat = self.rotation_matrix_to_quaternion(R)

        if np.isnan(quat).any():
            return

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'overhead_camera_link'
        t.child_frame_id = f'aruco_{marker_id}'

        t.transform.translation.x = float(tvec[0])
        t.transform.translation.y = float(tvec[1])
        t.transform.translation.z = float(tvec[2])

        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.tf_broadcaster.sendTransform(t)

    # ---------------- Quaternion helper ----------------
    def rotation_matrix_to_quaternion(self, R):
        q = np.empty(4)
        trace = np.trace(R)

        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            q[3] = 0.25 / s
            q[0] = (R[2, 1] - R[1, 2]) * s
            q[1] = (R[0, 2] - R[2, 0]) * s
            q[2] = (R[1, 0] - R[0, 1]) * s
        else:
            i = np.argmax(np.diag(R))
            j = (i + 1) % 3
            k = (i + 2) % 3
            s = np.sqrt(1.0 + R[i, i] - R[j, j] - R[k, k]) * 2
            q[i] = 0.25 * s
            q[3] = (R[k, j] - R[j, k]) / s
            q[j] = (R[j, i] + R[i, j]) / s
            q[k] = (R[k, i] + R[i, k]) / s

        return q


def main(args=None):
    rclpy.init(args=args)
    node = ArucoTableDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
