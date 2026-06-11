"""
Pose estimation and temporal stabilization.

This module is pure geometry/math — no camera, no display, no printing — so the
standalone app and a future ROS 2 node can both import and reuse it unchanged.
"""

from dataclasses import dataclass

import cv2
import numpy as np

from config import MARKER_SIZE_METERS, POSE_MAX_CONSECUTIVE_REJECTS


def estimate_pose_markers(corners, camera_matrix, dist_coeffs):
    """
    Estimate per-marker pose, resolving the planar (mirror) ambiguity.

    SOLVEPNP_IPPE_SQUARE yields two valid solutions for a planar square; solvePnP
    silently returns one, which is why the pose sometimes flips to a "behind the
    camera" mirror (negative Z). solvePnPGeneric returns BOTH solutions plus their
    reprojection errors, so we can deliberately keep the physically valid one.

    Selection rule: prefer the solution with positive Z (marker in front of the
    camera); among valid candidates pick the lowest reprojection error. Falls back
    to lowest reprojection error if neither has positive Z.

    Returns (rvecs, tvecs, reproj_errors) — reproj_errors is per-marker pixels, used
    downstream to gate out unreliable solves.
    """
    marker_size = MARKER_SIZE_METERS
    rvecs = []
    tvecs = []
    reproj_errors = []

    # Corner order REQUIRED by SOLVEPNP_IPPE_SQUARE: top-left, top-right,
    # bottom-right, bottom-left with +Y up. This matches the order ArUco's
    # detectMarkers returns corners in, so object/image points correspond.
    # (The previous Y-down ordering was mismatched and yielded garbage poses.)
    object_points = np.array([
        [-marker_size / 2, marker_size / 2, 0],
        [marker_size / 2, marker_size / 2, 0],
        [marker_size / 2, -marker_size / 2, 0],
        [-marker_size / 2, -marker_size / 2, 0],
    ], dtype=np.float32)

    for corner in corners:
        image_points = corner[0].astype(np.float32)
        retval, rvs, tvs, errs = cv2.solvePnPGeneric(
            object_points,
            image_points,
            camera_matrix,
            dist_coeffs,
            useExtrinsicGuess=False,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )

        if retval < 1:
            rvecs.append(np.zeros((3, 1)))
            tvecs.append(np.zeros((3, 1)))
            reproj_errors.append(np.inf)
            continue

        # Candidates: (reproj_error, rvec, tvec) for each returned solution.
        candidates = [
            (float(errs[k].item()), rvs[k], tvs[k]) for k in range(retval)
        ]
        front = [c for c in candidates if c[2][2, 0] > 0]  # positive Z (in front)
        pool = front if front else candidates
        err, rvec, tvec = min(pool, key=lambda c: c[0])

        rvecs.append(rvec)
        tvecs.append(tvec)
        reproj_errors.append(err)

    return np.array(rvecs), np.array(tvecs), np.array(reproj_errors)


def _rvec_to_quat(rvec: np.ndarray) -> np.ndarray:
    """Convert a Rodrigues rotation vector to a unit quaternion [w, x, y, z]."""
    rotmat, _ = cv2.Rodrigues(rvec)
    trace = np.trace(rotmat)

    if trace > 0:
        s = np.sqrt(trace + 1.0) * 2
        qw = 0.25 * s
        qx = (rotmat[2, 1] - rotmat[1, 2]) / s
        qy = (rotmat[0, 2] - rotmat[2, 0]) / s
        qz = (rotmat[1, 0] - rotmat[0, 1]) / s
    elif rotmat[0, 0] > rotmat[1, 1] and rotmat[0, 0] > rotmat[2, 2]:
        s = np.sqrt(1.0 + rotmat[0, 0] - rotmat[1, 1] - rotmat[2, 2]) * 2
        qw = (rotmat[2, 1] - rotmat[1, 2]) / s
        qx = 0.25 * s
        qy = (rotmat[0, 1] + rotmat[1, 0]) / s
        qz = (rotmat[0, 2] + rotmat[2, 0]) / s
    elif rotmat[1, 1] > rotmat[2, 2]:
        s = np.sqrt(1.0 + rotmat[1, 1] - rotmat[0, 0] - rotmat[2, 2]) * 2
        qw = (rotmat[0, 2] - rotmat[2, 0]) / s
        qx = (rotmat[0, 1] + rotmat[1, 0]) / s
        qy = 0.25 * s
        qz = (rotmat[1, 2] + rotmat[2, 1]) / s
    else:
        s = np.sqrt(1.0 + rotmat[2, 2] - rotmat[0, 0] - rotmat[1, 1]) * 2
        qw = (rotmat[1, 0] - rotmat[0, 1]) / s
        qx = (rotmat[0, 2] + rotmat[2, 0]) / s
        qy = (rotmat[1, 2] + rotmat[2, 1]) / s
        qz = 0.25 * s

    return np.array([qw, qx, qy, qz])


def _quat_to_rvec(quat: np.ndarray) -> np.ndarray:
    """Convert a unit quaternion [w, x, y, z] back to a Rodrigues rotation vector."""
    qw, qx, qy, qz = quat
    rotmat = np.array([
        [1 - 2 * (qy ** 2 + qz ** 2), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx ** 2 + qz ** 2), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx ** 2 + qy ** 2)],
    ])
    rvec, _ = cv2.Rodrigues(rotmat)
    return rvec


def _quat_slerp(q0: np.ndarray, q1: np.ndarray, t: float) -> np.ndarray:
    """Spherical linear interpolation between two unit quaternions."""
    dot = np.dot(q0, q1)

    # Take the shorter path around the hypersphere
    if dot < 0:
        q1 = -q1
        dot = -dot

    if dot > 0.9995:
        result = q0 + t * (q1 - q0)
        return result / np.linalg.norm(result)

    theta_0 = np.arccos(np.clip(dot, -1.0, 1.0))
    theta = theta_0 * t

    q_perp = q1 - q0 * dot
    q_perp = q_perp / np.linalg.norm(q_perp)

    return q0 * np.cos(theta) + q_perp * np.sin(theta)


def _quat_angle_diff(q0: np.ndarray, q1: np.ndarray) -> float:
    """Angular distance (radians) between two unit quaternions."""
    dot = np.clip(np.abs(np.dot(q0, q1)), -1.0, 1.0)
    return 2 * np.arccos(dot)


class PoseFilter:
    def __init__(self, alpha=0.3, outlier_threshold_pos=0.15, outlier_threshold_rot=0.5,
                 max_consecutive_rejects=POSE_MAX_CONSECUTIVE_REJECTS):
        self.alpha = alpha
        self.outlier_threshold_pos = outlier_threshold_pos
        self.outlier_threshold_rot = outlier_threshold_rot
        self.max_consecutive_rejects = max_consecutive_rejects
        self.consecutive_rejects = 0
        self.smooth_tvec: np.ndarray | None = None
        self.smooth_quat: np.ndarray | None = None

    def _seed(self, tvec, rvec, quat):
        """(Re)initialize the filter state from a measurement and accept it."""
        self.smooth_tvec = tvec.copy()
        self.smooth_quat = quat
        self.consecutive_rejects = 0
        return tvec.copy(), rvec.copy(), True

    def update(self, tvec: np.ndarray, rvec: np.ndarray):
        quat = _rvec_to_quat(rvec)

        if self.smooth_tvec is None or self.smooth_quat is None:
            return self._seed(tvec, rvec, quat)

        pos_delta = np.linalg.norm(tvec - self.smooth_tvec)
        rot_delta = _quat_angle_diff(self.smooth_quat, quat)

        is_outlier = (
            pos_delta > self.outlier_threshold_pos
            or rot_delta > self.outlier_threshold_rot
        )

        if is_outlier:
            self.consecutive_rejects += 1
            # Sustained rejection means the marker genuinely moved (or we were stuck
            # on a stale estimate) — re-lock to the latest measurement.
            if self.consecutive_rejects >= self.max_consecutive_rejects:
                return self._seed(tvec, rvec, quat)
            return self.smooth_tvec.copy(), _quat_to_rvec(self.smooth_quat), False

        self.consecutive_rejects = 0
        self.smooth_tvec = (1 - self.alpha) * self.smooth_tvec + self.alpha * tvec
        self.smooth_quat = _quat_slerp(self.smooth_quat, quat, self.alpha)

        return self.smooth_tvec.copy(), _quat_to_rvec(self.smooth_quat), True


@dataclass
class LandingPadPose:
    """
    Smoothed landing-pad pose, expressed in the OpenCV CAMERA frame:
      +X = right, +Y = down, +Z = forward (out of the lens).
    tvec is meters from camera to marker center; rvec is the marker orientation
    as a Rodrigues vector. reproj_error is the solver's pixel reprojection error.

    NOTE for flight-controller integration (via MAVROS): this is the CAMERA frame,
    not the drone body/nav frame. The downstream MAVROS node must apply the
    camera→body mounting transform (rotation + lever-arm) — typically via a TF2
    frame lookup — before publishing. For a typical down-facing camera,
    body-forward ≈ camera +Y-ish and body-down ≈ camera +Z, but the exact mapping
    depends on how the camera is bolted on, so it lives in the integration layer.
    """
    marker_id: int
    tvec: np.ndarray          # (3,1) meters, camera frame
    rvec: np.ndarray          # (3,1) Rodrigues, camera frame
    reproj_error: float       # pixels
    is_valid: bool            # False if this frame's measurement was an outlier
    detect_mode: str          # "enhanced" or "raw"
