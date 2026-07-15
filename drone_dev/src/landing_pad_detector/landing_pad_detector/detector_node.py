#!/usr/bin/env python3
"""
Landing-pad detector node.

Subscribes to the camera stream published by csi_camera_publisher
(sensor_msgs/Image + sensor_msgs/CameraInfo), runs the low-light ArUco pipeline
(detector_lib) with temporal smoothing (PoseFilter), transforms the resulting
pose into the drone body frame via tf2, and publishes it for PX4 precision
landing (as geometry_msgs/PoseStamped, ready for the MAVROS landing_target
plugin — and mavros_msgs/LandingTarget too when mavros_msgs is installed).

It opens NO camera of its own. The camera driver is the single owner of the
hardware; this node is a pure consumer of frames. That is the whole point of the
split — see README.md.
"""
from __future__ import annotations

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from cv_bridge import CvBridge

import tf2_ros
import tf2_geometry_msgs  # noqa: F401  (registers PoseStamped transform support)

from .detector_lib import (
    DetectorConfig,
    build_clahe,
    build_detector,
    detect_scaled,
    estimate_pose_markers,
    preprocess_frame,
)
from .pose_filter import PoseFilter
from .rotation_utils import rvec_to_quat

# mavros_msgs is an optional runtime dependency. If it's not on the system the
# node still runs and publishes PoseStamped — you just don't get the raw
# LandingTarget message. This keeps the package buildable without MAVROS.
try:
    from mavros_msgs.msg import LandingTarget
    _HAS_MAVROS = True
except ImportError:  # pragma: no cover
    LandingTarget = None
    _HAS_MAVROS = False


# Sensor-data QoS: best-effort + keep-last-1 so we always process the freshest
# frame and never queue stale ones. This is the ROS equivalent of the standalone
# script's FrameGrabber "latest frame wins" behaviour.
SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    durability=DurabilityPolicy.VOLATILE,
    depth=1,
)


class LandingPadDetector(Node):
    def __init__(self, **kwargs):
        super().__init__('landing_pad_detector', **kwargs)

        # ---- Parameters (all detection tunables + framing/plumbing) ----
        self.declare_parameter('image_topic', 'camera/image_raw')
        self.declare_parameter('camera_info_topic', 'camera/camera_info')
        self.declare_parameter('landing_target_topic', '/mavros/landing_target/pose')
        self.declare_parameter('landing_pad_id', 0)
        self.declare_parameter('target_frame', 'base_link')   # drone body frame
        self.declare_parameter('transform_to_body', True)
        self.declare_parameter('print_pose_every_n', 30)

        # detector_lib config, mirrored as ROS params
        self.declare_parameter('marker_size_meters', 0.20)
        self.declare_parameter('aruco_dict_name', 'DICT_5X5_50')
        self.declare_parameter('clahe_clip_limit', 1.5)
        self.declare_parameter('detect_scale', 0.5)
        self.declare_parameter('fallback_every_n', 3)
        self.declare_parameter('min_marker_perimeter_rate', 0.05)
        self.declare_parameter('error_correction_rate', 0.5)
        self.declare_parameter('max_reproj_error_px', 4.0)

        # PoseFilter (temporal smoothing / outlier rejection)
        self.declare_parameter('pose_smooth_alpha', 0.3)
        self.declare_parameter('pose_outlier_threshold_pos', 0.15)
        self.declare_parameter('pose_outlier_threshold_rot', 0.5)
        self.declare_parameter('pose_max_consecutive_rejects', 5)

        gp = self.get_parameter
        image_topic = gp('image_topic').value
        info_topic = gp('camera_info_topic').value
        target_topic = gp('landing_target_topic').value

        self.landing_pad_id = int(gp('landing_pad_id').value)
        self.target_frame = gp('target_frame').value
        self.transform_to_body = bool(gp('transform_to_body').value)
        self.print_every_n = int(gp('print_pose_every_n').value)

        self.cfg = DetectorConfig(
            marker_size_meters=float(gp('marker_size_meters').value),
            aruco_dict_name=gp('aruco_dict_name').value,
            clahe_clip_limit=float(gp('clahe_clip_limit').value),
            detect_scale=float(gp('detect_scale').value),
            fallback_every_n=int(gp('fallback_every_n').value),
            min_marker_perimeter_rate=float(gp('min_marker_perimeter_rate').value),
            error_correction_rate=float(gp('error_correction_rate').value),
            max_reproj_error_px=float(gp('max_reproj_error_px').value),
        )

        self._filter_kwargs = dict(
            alpha=float(gp('pose_smooth_alpha').value),
            outlier_threshold_pos=float(gp('pose_outlier_threshold_pos').value),
            outlier_threshold_rot=float(gp('pose_outlier_threshold_rot').value),
            max_consecutive_rejects=int(gp('pose_max_consecutive_rejects').value),
        )

        # ---- Pipeline state ----
        self.bridge = CvBridge()
        self.detector = build_detector(self.cfg)
        self.clahe = build_clahe(self.cfg)
        self.pose_filters: dict[int, PoseFilter] = {}
        self.camera_matrix: np.ndarray | None = None
        self.dist_coeffs: np.ndarray | None = None
        self.optical_frame = 'camera_optical'
        self.misses = 0
        self.frame_count = 0

        # ---- tf2 (optical -> body) ----
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---- Publishers ----
        self.pose_pub = self.create_publisher(PoseStamped, target_topic, 10)
        # Also expose a plain debug pose in the optical frame for RViz / rosbag.
        self.debug_pub = self.create_publisher(PoseStamped, 'landing_pad/pose', 10)
        self.raw_pub = None
        if _HAS_MAVROS:
            self.raw_pub = self.create_publisher(
                LandingTarget, '/mavros/landing_target/raw', 10)

        # ---- Subscribers ----
        self.create_subscription(CameraInfo, info_topic, self.on_camera_info, SENSOR_QOS)
        self.create_subscription(Image, image_topic, self.on_image, SENSOR_QOS)

        self.get_logger().info(
            f"landing_pad_detector up. image='{image_topic}' info='{info_topic}' "
            f"target='{target_topic}' pad_id={self.landing_pad_id} "
            f"dict={self.cfg.aruco_dict_name} marker={self.cfg.marker_size_meters*1000:.0f}mm "
            f"mavros_msgs={'yes' if _HAS_MAVROS else 'no'}")

    # ------------------------------------------------------------------ #
    def on_camera_info(self, msg: CameraInfo):
        # Intrinsics come from the camera node — no local calibration files.
        self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        d = np.array(msg.d, dtype=np.float64)
        self.dist_coeffs = d.reshape(1, -1) if d.size else np.zeros((1, 5))
        if msg.header.frame_id:
            self.optical_frame = msg.header.frame_id

    def on_image(self, msg: Image):
        if self.camera_matrix is None:
            # Wait for the first CameraInfo so solvePnP has intrinsics.
            return

        # OV9281 is monochrome; ask cv_bridge for mono8 regardless of the wire
        # encoding (bgr8 or mono8) so this node works before/after the camera
        # node is switched to mono8. See README "mono8" note.
        gray = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        gray_clahe = preprocess_frame(gray, self.clahe, self.cfg)

        self.frame_count += 1
        run_fallback = (self.misses % self.cfg.fallback_every_n == 0)
        corners, ids, detect_mode = detect_scaled(
            self.detector, gray_clahe, run_fallback, self.cfg)

        if ids is None or len(ids) == 0:
            self.misses += 1
            return
        self.misses = 0

        rvecs, tvecs, reproj_errors = estimate_pose_markers(
            corners, self.camera_matrix, self.dist_coeffs, self.cfg)

        for i, marker_id in enumerate(ids.flatten()):
            if int(marker_id) != self.landing_pad_id:
                continue

            rvec, tvec = rvecs[i], tvecs[i]
            reproj_error = float(reproj_errors[i])
            raw_z = float(tvec[2, 0])

            # Validity gate: drop physically-impossible / unreliable solves before
            # they ever reach the smoothing filter.
            if (raw_z <= 0 or not np.all(np.isfinite(tvec))
                    or reproj_error > self.cfg.max_reproj_error_px):
                continue

            flt = self.pose_filters.get(marker_id)
            if flt is None:
                flt = self.pose_filters[marker_id] = PoseFilter(**self._filter_kwargs)
            tvec, rvec, is_valid = flt.update(tvec, rvec)

            self.publish_pose(msg.header.stamp, tvec, rvec, reproj_error,
                              is_valid, detect_mode, raw_z)

    # ------------------------------------------------------------------ #
    def publish_pose(self, stamp, tvec, rvec, reproj_error, is_valid, detect_mode, raw_z):
        # Build the pose in the camera optical frame (REP-103: x right, y down,
        # z forward), stamped with the IMAGE time so PX4's EKF fuses it correctly.
        quat = rvec_to_quat(rvec)  # [w, x, y, z]
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.optical_frame
        pose.pose.position.x = float(tvec[0, 0])
        pose.pose.position.y = float(tvec[1, 0])
        pose.pose.position.z = float(tvec[2, 0])
        pose.pose.orientation.w = float(quat[0])
        pose.pose.orientation.x = float(quat[1])
        pose.pose.orientation.y = float(quat[2])
        pose.pose.orientation.z = float(quat[3])

        self.debug_pub.publish(pose)

        # Transform into the drone body frame before sending to the autopilot. The
        # static camera-mounting transform (base_link -> camera_optical) must be
        # published elsewhere (launch file / URDF). Without it, PX4 servos toward a
        # point offset by however the camera is mounted.
        out = pose
        if self.transform_to_body:
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.target_frame, self.optical_frame, rclpy.time.Time())
                out = self._do_transform(pose, tf)
            except tf2_ros.TransformException as exc:
                self.get_logger().warn(
                    f"No transform {self.optical_frame}->{self.target_frame} "
                    f"({exc}); publishing pose in optical frame.", throttle_duration_sec=5.0)

        self.pose_pub.publish(out)

        if self.raw_pub is not None:
            self.raw_pub.publish(self._make_landing_target(out))

        if self.print_every_n <= 1 or self.frame_count % self.print_every_n == 0:
            x, y, z = out.pose.position.x, out.pose.position.y, out.pose.position.z
            flag = '' if is_valid else ' [rejected]'
            self.get_logger().info(
                f"LANDING PAD id={self.landing_pad_id} frame={out.header.frame_id} "
                f"X:{x:+.3f} Y:{y:+.3f} Z:{z:.3f}m (raw {raw_z:.3f}) "
                f"err:{reproj_error:.2f}px [{detect_mode}]{flag}")

    @staticmethod
    def _do_transform(pose: PoseStamped, tf) -> PoseStamped:
        """Apply a TF to a PoseStamped, tolerant of the Foxy vs Humble+ tf2 API.

        Humble+ has do_transform_pose_stamped(PoseStamped, tf)->PoseStamped.
        Foxy only has do_transform_pose(Pose, tf)->Pose, so wrap/unwrap.
        """
        fn = getattr(tf2_geometry_msgs, 'do_transform_pose_stamped', None)
        if fn is not None:
            return fn(pose, tf)
        out = PoseStamped()
        out.header.stamp = pose.header.stamp
        out.header.frame_id = tf.header.frame_id
        out.pose = tf2_geometry_msgs.do_transform_pose(pose.pose, tf)
        return out

    def _make_landing_target(self, pose: PoseStamped):
        lt = LandingTarget()
        lt.header = pose.header
        lt.target_num = 0
        # FRAME_LOCAL_NED=1 / BODY_NED=8 per MAV_FRAME; MAVROS reads the pose field
        # for the vision-based path, so frame here is mostly informational.
        lt.frame = 8
        lt.type = LandingTarget.TYPE_VISION_FIDUCIAL if hasattr(
            LandingTarget, 'TYPE_VISION_FIDUCIAL') else 0
        lt.pose = pose.pose
        return lt


def main(args=None):
    rclpy.init(args=args)
    node = LandingPadDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down — Ctrl+C received')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
