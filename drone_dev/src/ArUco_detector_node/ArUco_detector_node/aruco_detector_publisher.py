#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import PoseStamped

from .aruco_detector import ArucoDetector, rvec_to_quaternion


class ArucoDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__("aruco_detector_node")
        self.get_logger().info("ArUco detector node initialized")

        # Marker settings for your current landing pad marker.
        self.declare_parameter("marker_length_m", 0.20)
        self.declare_parameter("dictionary_name", "DICT_5X5_250")
        self.declare_parameter("target_marker_id", 0)
        self.declare_parameter("camera_frame_id", "camera_frame")
        marker_length_m = float(self.get_parameter("marker_length_m").value)
        dictionary_name = str(self.get_parameter("dictionary_name").value)
        self.target_marker_id = int(self.get_parameter("target_marker_id").value)
        self.camera_frame_id = str(self.get_parameter("camera_frame_id").value)

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

        # Publish the detected marker pose (position + orientation) in the camera frame.
        self.pose_pub = self.create_publisher(PoseStamped, "/aruco/pose", 10)

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
        self.get_logger().info(f"Detected IDs: {ids_list} | first marker tvec(m): {first_tvec}")

        # Build and publish the pose of the first (target) marker.
        self.publish_pose(
            msg.header.stamp,
            result["rvecs"][0],
            result["tvecs"][0],
        )

    def publish_pose(self, stamp, rvec: np.ndarray, tvec: np.ndarray) -> None:
        # tvec is the marker position in the camera frame, in meters:
        #   x = right, y = down, z = depth (distance in front of the camera).
        tx, ty, tz = np.asarray(tvec, dtype=np.float64).flatten()
        qx, qy, qz, qw = rvec_to_quaternion(rvec)

        pose_msg = PoseStamped()
        # Reuse the image timestamp so downstream consumers can sync with the frame.
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = self.camera_frame_id

        pose_msg.pose.position.x = tx
        pose_msg.pose.position.y = ty
        pose_msg.pose.position.z = tz

        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw

        self.pose_pub.publish(pose_msg)



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
