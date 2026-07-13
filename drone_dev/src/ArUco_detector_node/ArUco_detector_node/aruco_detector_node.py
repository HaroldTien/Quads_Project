#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import PoseStamped

from .aruco_detector import ArucoDetector


def rvec_to_quaternion(rvec: np.ndarray):
    """Rodrigues rotation vector -> (x, y, z, w) quaternion (camera optical frame)."""
    rot_matrix, _ = cv2.Rodrigues(rvec)
    trace = np.trace(rot_matrix)
    if trace > 0.0:
        s = np.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (rot_matrix[2, 1] - rot_matrix[1, 2]) / s
        y = (rot_matrix[0, 2] - rot_matrix[2, 0]) / s
        z = (rot_matrix[1, 0] - rot_matrix[0, 1]) / s
    else:
        # Pick the largest diagonal element for numerical stability.
        i = int(np.argmax(np.diag(rot_matrix)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = np.sqrt(rot_matrix[i, i] - rot_matrix[j, j] - rot_matrix[k, k] + 1.0) * 2.0
        q = np.zeros(3)
        q[i] = 0.25 * s
        q[j] = (rot_matrix[j, i] + rot_matrix[i, j]) / s
        q[k] = (rot_matrix[k, i] + rot_matrix[i, k]) / s
        w = (rot_matrix[k, j] - rot_matrix[j, k]) / s
        x, y, z = q
    return float(x), float(y), float(z), float(w)


class ArucoDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__("aruco_detector_node")
        self.get_logger().info("ArUco detector node initialized")

        # Marker settings for your current landing pad marker.
        self.declare_parameter("marker_length_m", 0.20)
        self.declare_parameter("dictionary_name", "DICT_5X5_250")
        self.declare_parameter("target_marker_id", 0)
        marker_length_m = float(self.get_parameter("marker_length_m").value)
        dictionary_name = str(self.get_parameter("dictionary_name").value)
        self.target_marker_id = int(self.get_parameter("target_marker_id").value)

        # ROS <-> OpenCV image conversion helper.
        self.bridge = CvBridge()

        # Detector module (pure vision logic).
        self.detector = ArucoDetector(
            marker_length_m=marker_length_m,
            dictionary_name=dictionary_name,
        )
        self.get_logger().info(
            "ArUco settings: dictionary=%s marker_length_m=%.3f target_marker_id=%d"
            % (dictionary_name, marker_length_m, self.target_marker_id)
        )

        # Camera intrinsics from /camera/camera_info.
        self.camera_matrix = None
        self.dist_coeffs = None
        self.has_logged_missing_calib = False

        # Pose of the target marker in the CAMERA OPTICAL frame (z = depth along
        # the view axis = altitude above the pad). The landing controller
        # subscribes here and applies the camera->body->ENU transform itself, so
        # we publish the raw camera-frame tvec/rvec and do NO frame conversion.
        self.pose_pub = self.create_publisher(PoseStamped, "/aruco/pose", 10)

        # Subscribe to raw camera image stream.
        self.image_sub = self.create_subscription(
            Image,
            "/camera/image_raw",
            self.image_callback,
            10,
        )

        # Subscribe to camera calibration information.
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            "/camera/camera_info",
            self.camera_info_callback,
            10,
        )

    def camera_info_callback(self, msg: CameraInfo) -> None:
        # K is 3x3 flattened in row-major order.
        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d, dtype=np.float64).reshape(-1, 1)
        if not self.has_logged_missing_calib:
            self.get_logger().info("Camera calibration received from /camera/camera_info")
            self.has_logged_missing_calib = True

    def image_callback(self, msg: Image) -> None:
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # If calibration is not ready yet, still run marker detection only.
        if self.camera_matrix is None or self.dist_coeffs is None:
            corners, ids = self.detector.detect(frame)
            if ids is not None:
                self.get_logger().info(f"Detected IDs (no pose yet): {ids.flatten().tolist()}")
            return

        result = self.detector.detect_and_estimate(
            frame,
            self.camera_matrix,
            self.dist_coeffs,
            target_ids=[self.target_marker_id],
        )

        if result["ids"] is None:
            return

        ids_list = result["ids"].flatten().tolist()
        first_tvec = result["tvecs"][0].flatten().tolist()

        # Publish the target marker pose so the landing controller can act on it.
        # target_ids filtered to a single ID upstream, so index 0 is our pad.
        tvec = np.asarray(result["tvecs"][0], dtype=float).flatten()
        rvec = np.asarray(result["rvecs"][0], dtype=float).flatten()
        qx, qy, qz, qw = rvec_to_quaternion(rvec)

        pose = PoseStamped()
        # Stamp with the IMAGE acquisition time (not now()) so the controller's
        # freshness check measures true sensor age, not ROS receipt latency.
        pose.header.stamp = msg.header.stamp
        pose.header.frame_id = msg.header.frame_id or "down_camera_optical"
        pose.pose.position.x = float(tvec[0])
        pose.pose.position.y = float(tvec[1])
        pose.pose.position.z = float(tvec[2])
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        self.pose_pub.publish(pose)

        self.get_logger().info(
            f"Detected IDs: {ids_list} | published /aruco/pose tvec(m): {first_tvec}",
            throttle_duration_sec=1.0,
        )








def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down ArUco detector node")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
