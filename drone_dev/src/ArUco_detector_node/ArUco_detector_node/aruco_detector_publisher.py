#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import PoseStamped

from .aruco_detector import ArucoDetector, rvec_to_quaternion
from .pose_filter import PoseFilter


class ArucoDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__("aruco_detector_node")
        self.get_logger().info("ArUco detector node initialized")

        # Marker settings for your current landing pad marker.
        self.declare_parameter("marker_length_m", 0.20)
        # NOTE: must match your PRINTED marker family. The low-light pipeline was
        # tuned/tested on DICT_5X5_50; change this if your markers differ.
        self.declare_parameter("dictionary_name", "DICT_5X5_50")
        self.declare_parameter("target_marker_id", 0)

        # Low-light preprocessing tuning (see ArucoDetector).
        self.declare_parameter("enable_clahe", True)
        self.declare_parameter("clahe_clip_limit", 1.5)
        self.declare_parameter("clahe_tile_size", 8)
        self.declare_parameter("enable_denoise", True)
        self.declare_parameter("denoise_diameter", 5)
        self.declare_parameter("denoise_sigma_color", 50.0)
        self.declare_parameter("denoise_sigma_space", 50.0)

        # Temporal pose smoothing / outlier rejection (feeds landing_controller).
        self.declare_parameter("enable_pose_filter", True)
        self.declare_parameter("pose_smooth_alpha", 0.3)
        self.declare_parameter("pose_outlier_threshold_pos", 0.15)
        self.declare_parameter("pose_outlier_threshold_rot", 0.5)
        self.declare_parameter("pose_max_consecutive_rejects", 5)
        # Frame the pose is published in (must match camera_info; landing_controller
        # treats it as the camera optical frame).
        self.declare_parameter("pose_frame_id", "camera_optical")

        # Debug view: republish the frame with detected markers + pose axes drawn,
        # so you can watch the stream in rqt_image_view. Turn off to save CPU.
        self.declare_parameter("publish_debug_image", True)

        marker_length_m = float(self.get_parameter("marker_length_m").value)
        dictionary_name = str(self.get_parameter("dictionary_name").value)
        self.target_marker_id = int(self.get_parameter("target_marker_id").value)

        # ROS <-> OpenCV image conversion helper.
        self.bridge = CvBridge()

        # Detector module (pure vision logic).
        self.detector = ArucoDetector(
            marker_length_m=marker_length_m,
            dictionary_name=dictionary_name,
            enable_clahe=bool(self.get_parameter("enable_clahe").value),
            clahe_clip_limit=float(self.get_parameter("clahe_clip_limit").value),
            clahe_tile_size=int(self.get_parameter("clahe_tile_size").value),
            enable_denoise=bool(self.get_parameter("enable_denoise").value),
            denoise_diameter=int(self.get_parameter("denoise_diameter").value),
            denoise_sigma_color=float(self.get_parameter("denoise_sigma_color").value),
            denoise_sigma_space=float(self.get_parameter("denoise_sigma_space").value),
        )
        self.get_logger().info(
            "ArUco settings: dictionary=%s marker_length_m=%.3f target_marker_id=%d"
            % (dictionary_name, marker_length_m, self.target_marker_id)
        )

        # Pose publisher — THIS is the link landing_controller_node subscribes to.
        # Without it the controller never receives a target and nothing flies.
        self.pose_pub = self.create_publisher(PoseStamped, "/aruco/pose", 10)
        self.pose_frame_id = str(self.get_parameter("pose_frame_id").value)

        # Temporal smoothing so the controller gets a stable target instead of
        # frame-to-frame jitter. One filter per marker id (we only track one here).
        self.enable_pose_filter = bool(self.get_parameter("enable_pose_filter").value)
        self._filter_kwargs = dict(
            alpha=float(self.get_parameter("pose_smooth_alpha").value),
            outlier_threshold_pos=float(self.get_parameter("pose_outlier_threshold_pos").value),
            outlier_threshold_rot=float(self.get_parameter("pose_outlier_threshold_rot").value),
            max_consecutive_rejects=int(self.get_parameter("pose_max_consecutive_rejects").value),
        )
        self.pose_filters = {}

        # Debug image publisher — view with: ros2 run rqt_image_view rqt_image_view
        # and pick /aruco/debug_image.
        self.publish_debug_image = bool(self.get_parameter("publish_debug_image").value)
        self.debug_pub = (
            self.create_publisher(Image, "/aruco/debug_image", 10)
            if self.publish_debug_image else None
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
            # Still stream the raw frame so the view works before calibration.
            self._publish_debug(msg.header, frame)
            return

        result = self.detector.detect_and_estimate(
            frame,
            self.camera_matrix,
            self.dist_coeffs,
            target_ids=[self.target_marker_id],
        )

        if result["ids"] is None:
            # No marker this frame — still publish the live frame so the stream
            # never freezes when the pad drifts out of view.
            self._publish_debug(msg.header, frame)
            return

        # Overlay detected marker outline + pose axes for the debug view.
        debug = self.detector.draw_result(
            frame, result, self.camera_matrix, self.dist_coeffs)

        # detect_and_estimate was filtered to target_marker_id, so index 0 is our
        # landing-pad marker. tvec/rvec come back shaped (1,3); make them (3,1).
        tvec = np.asarray(result["tvecs"][0], dtype=np.float64).reshape(3, 1)
        rvec = np.asarray(result["rvecs"][0], dtype=np.float64).reshape(3, 1)

        # Validity gate: drop physically-impossible solves (marker behind camera
        # or non-finite) before they reach the filter / the controller.
        z = float(tvec[2, 0])
        if z <= 0.0 or not np.all(np.isfinite(tvec)):
            self._publish_debug(msg.header, debug)
            return

        is_valid = True
        if self.enable_pose_filter:
            flt = self.pose_filters.get(self.target_marker_id)
            if flt is None:
                flt = self.pose_filters[self.target_marker_id] = PoseFilter(**self._filter_kwargs)
            tvec, rvec, is_valid = flt.update(tvec, rvec)

        self._publish_pose(msg.header.stamp, tvec, rvec)

        # Draw the live distance on the debug frame too.
        cv2.putText(
            debug,
            "id=%d X:%+.2f Y:%+.2f Z:%.2fm%s" % (
                self.target_marker_id, float(tvec[0, 0]), float(tvec[1, 0]),
                float(tvec[2, 0]), "" if is_valid else " [rej]"),
            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
            (0, 255, 0) if is_valid else (0, 0, 255), 2, cv2.LINE_AA,
        )
        self._publish_debug(msg.header, debug)

        self.get_logger().info(
            "aruco id=%d X:%+.3f Y:%+.3f Z:%.3f m%s"
            % (self.target_marker_id, float(tvec[0, 0]), float(tvec[1, 0]),
               float(tvec[2, 0]), "" if is_valid else " [rejected]"),
            throttle_duration_sec=1.0,
        )

    def _publish_debug(self, header, image) -> None:
        """Republish an annotated frame on /aruco/debug_image for rqt_image_view."""
        if self.debug_pub is None:
            return
        debug_msg = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
        debug_msg.header = header
        self.debug_pub.publish(debug_msg)

    def _publish_pose(self, stamp, tvec, rvec) -> None:
        """Publish the marker pose on /aruco/pose in the camera optical frame."""
        qx, qy, qz, qw = rvec_to_quaternion(rvec)   # geometry order (x, y, z, w)
        pose = PoseStamped()
        pose.header.stamp = stamp                    # image time — for EKF/timeout logic
        pose.header.frame_id = self.pose_frame_id
        pose.pose.position.x = float(tvec[0, 0])
        pose.pose.position.y = float(tvec[1, 0])
        pose.pose.position.z = float(tvec[2, 0])
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        self.pose_pub.publish(pose)








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
