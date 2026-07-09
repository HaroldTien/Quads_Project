from __future__ import annotations  # allow "X | None" type hints on Python 3.9

import cv2
import cv2.aruco as aruco
import numpy as np
import sys
import time
from dataclasses import dataclass
from pathlib import Path


# -------- User settings --------
MARKER_SIZE_METERS = 0.20   # 20 cm marker side length
LANDING_PAD_ID = 0
# 5x5 family options: 50, 100, 250, 1000
ARUCO_DICT_NAME = "DICT_5X5_50"
# Index 0 = Arducam OV9281 (works on Jetson/V4L2, but NOT on macOS — its mono
# format won't negotiate with AVFoundation, so reads hang). For macOS bench
# testing use index 1 = built-in MacBook camera. Revert to 0 for the Jetson/OV9281.
# NOTE: built-in cam intrinsics differ from the OV9281 calibration files, so the
# metric distance (Z) will be wrong here — detection/jitter/flip behavior is still valid.
CAMERA_INDEX = 0
FRAME_WIDTH = 1280
FRAME_HEIGHT = 800
FPS = 30
# ------------------------------

# -------- Low-light settings --------
# CLAHE parameters — increase clipLimit for more aggressive contrast boost
CLAHE_CLIP_LIMIT = 1.5
CLAHE_TILE_SIZE = (8, 8)

# Edge-preserving denoise on the CLAHE output before detection. CLAHE amplifies
# sensor grain, which wobbles the detected corners frame-to-frame; bilateral
# filtering suppresses that noise while keeping marker edges crisp.
DENOISE = True
DENOISE_DIAMETER = 5        # bilateral neighborhood diameter (pixels)
DENOISE_SIGMA_COLOR = 50    # intensity sigma
DENOISE_SIGMA_SPACE = 50    # spatial sigma

# Camera exposure/gain — tweak if your camera supports manual control
# Set to -1 to leave at OS default (safe fallback)
MANUAL_EXPOSURE = -1       # e.g. 100 (ms*10 on V4L2). -1 = auto
MANUAL_GAIN = -1           # e.g. 200. -1 = auto
# ------------------------------------

# -------- Detector tuning --------
# Lower = detects smaller/farther markers, but tiny markers have noisy corners
# (jittery pose). Moderate value trades a little reach for stability.
MIN_MARKER_PERIMETER_RATE = 0.05
# Higher = tolerates more bit errors (helps in noise) but admits marginal/false
# reads. Moderate value keeps low-light tolerance without inviting noisy detections.
ERROR_CORRECTION_RATE = 0.5
# ---------------------------------

# -------- Pose stabilization settings --------
# Exponential smoothing factor (0.0-1.0): higher = more smoothing, slower response
POSE_SMOOTH_ALPHA = 0.3
# Max allowed change in position (meters) before rejecting as outlier
POSE_OUTLIER_THRESHOLD_POS = 0.15
# Max allowed change in orientation (radians) before rejecting as outlier
POSE_OUTLIER_THRESHOLD_ROT = 0.5
# After this many consecutive rejections, assume the marker genuinely moved and
# reset the filter to the latest measurement so it re-locks instead of sticking.
POSE_MAX_CONSECUTIVE_REJECTS = 5
# Reject any solved pose whose reprojection error (pixels) exceeds this — a high
# value means the corners and pose disagree, i.e. an unreliable solve.
MAX_REPROJ_ERROR_PX = 4.0
# ------------------------------------


def load_calibration(base_dir: Path):
    camera_matrix_path = base_dir / "camera_matrix.npy"
    dist_coeffs_path = base_dir / "dist_coeffs.npy"

    if not camera_matrix_path.exists() or not dist_coeffs_path.exists():
        raise FileNotFoundError(
            "Missing calibration files. Expected 'camera_matrix.npy' and "
            "'dist_coeffs.npy' in the script directory."
        )

    camera_matrix = np.load(str(camera_matrix_path))
    dist_coeffs = np.load(str(dist_coeffs_path))
    return camera_matrix, dist_coeffs


def open_camera():
    if sys.platform == "darwin":
        print("macOS detected: using default OpenCV camera backend.")
        cap = cv2.VideoCapture(CAMERA_INDEX)
    else:
        cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)

    if not cap.isOpened():
        return None

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, FPS)

    # Push sensor harder in low light if manual values provided
    if MANUAL_EXPOSURE != -1:
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)   # 1 = manual on most V4L2 cameras
        cap.set(cv2.CAP_PROP_EXPOSURE, MANUAL_EXPOSURE)
        print(f"Exposure set to {MANUAL_EXPOSURE}")

    if MANUAL_GAIN != -1:
        cap.set(cv2.CAP_PROP_GAIN, MANUAL_GAIN)
        print(f"Gain set to {MANUAL_GAIN}")

    actual_w = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    actual_h = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    actual_fps = cap.get(cv2.CAP_PROP_FPS)
    if (actual_w, actual_h) != (FRAME_WIDTH, FRAME_HEIGHT) or actual_fps != FPS:
        print(
            f"Warning: requested {FRAME_WIDTH}x{FRAME_HEIGHT}@{FPS}fps, "
            f"camera reports {int(actual_w)}x{int(actual_h)}@{actual_fps:.1f}fps."
        )

    # Retry until the first successful frame read, or give up after 40 attempts
    for _ in range(40):
        ret, _ = cap.read()
        if ret:
            return cap
        time.sleep(0.02)

    cap.release()
    return None


def build_detector():
    """
    Returns an ArucoDetector with parameters tuned for low-light/shadow conditions.
    Key changes vs defaults:
      - Wider adaptive threshold window range catches markers under uneven lighting
      - Slightly lower minMarkerPerimeterRate helps with small/partially lit markers
      - Higher errorCorrectionRate tolerates bit errors from noise in dark regions
    """
    aruco_dict_id = getattr(aruco, ARUCO_DICT_NAME, None)
    if aruco_dict_id is None:
        raise ValueError(
            f"Invalid ARUCO_DICT_NAME='{ARUCO_DICT_NAME}'. "
            "Use one of: DICT_5X5_50, DICT_5X5_100, DICT_5X5_250, DICT_5X5_1000."
        )

    aruco_dict = aruco.getPredefinedDictionary(aruco_dict_id)
    params = aruco.DetectorParameters()

    # Adaptive thresholding — wider range handles patchy shadows
    params.adaptiveThreshWinSizeMin = 3
    params.adaptiveThreshWinSizeMax = 53
    params.adaptiveThreshWinSizeStep = 10
    params.adaptiveThreshConstant = 7

    # Marker size floor — see MIN_MARKER_PERIMETER_RATE (stability vs reach trade-off)
    params.minMarkerPerimeterRate = MIN_MARKER_PERIMETER_RATE

    # Bit-error tolerance — see ERROR_CORRECTION_RATE
    params.errorCorrectionRate = ERROR_CORRECTION_RATE

    # Improve corner refinement accuracy
    params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
    params.cornerRefinementWinSize = 5

    return aruco.ArucoDetector(aruco_dict, params)


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


def preprocess_frame(gray: np.ndarray, clahe) -> np.ndarray:
    """
    Apply CLAHE to equalise contrast locally — core fix for shadows. Optionally
    follow with an edge-preserving bilateral filter to suppress the sensor grain
    CLAHE amplifies (a major source of corner/pose jitter in low light).
    """
    enhanced = clahe.apply(gray)
    if DENOISE:
        enhanced = cv2.bilateralFilter(
            enhanced, DENOISE_DIAMETER, DENOISE_SIGMA_COLOR, DENOISE_SIGMA_SPACE
        )
    return enhanced


def detect_with_fallback(detector, gray_clahe: np.ndarray):
    """
    Two-pass detection:
      Pass 1: CLAHE-enhanced grayscale (handles most low-light cases)
      Pass 2: Adaptive threshold on top of CLAHE (handles extreme shadows/hotspots)
    Returns corners, ids from whichever pass succeeds first.
    """
    corners, ids, rejected = detector.detectMarkers(gray_clahe)

    if ids is not None and len(ids) > 0:
        return corners, ids, "clahe"

    # Fallback: binarise with adaptive threshold to crush uneven shadows
    adaptive = cv2.adaptiveThreshold(
        gray_clahe,
        255,
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY,
        11,
        2,
    )
    corners, ids, _ = detector.detectMarkers(adaptive)

    return corners, ids, "adaptive"


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

    NOTE for flight-controller integration: this is the CAMERA frame, not the
    drone body/nav frame. The downstream MAVLink step must apply the camera→body
    mounting transform (rotation + lever-arm) before sending LANDING_TARGET. For a
    typical down-facing camera, body-forward ≈ camera +Y-ish and body-down ≈ camera
    +Z — but the exact mapping depends on how the camera is bolted on, so it lives
    in the integration layer, not here.
    """
    marker_id: int
    tvec: np.ndarray          # (3,1) meters, camera frame
    rvec: np.ndarray          # (3,1) Rodrigues, camera frame
    reproj_error: float       # pixels
    is_valid: bool            # False if this frame's measurement was an outlier
    detect_mode: str          # "clahe" or "adaptive"


def publish_pose(pose: LandingPadPose) -> None:
    """
    Hook for sending the landing-pad pose to the flight controller.

    TODO (follow-on): convert `pose` from the camera frame to the drone body/nav
    frame using the camera mounting transform, then send a MAVLink LANDING_TARGET
    message (e.g. via pymavlink) so the autopilot can servo the precision landing.
    Currently a no-op so the vision pipeline can run standalone on the bench.
    """
    # Intentionally left as a no-op until MAVLink transport is wired up.
    return


def main():
    base_dir = Path(__file__).resolve().parent
    camera_matrix, dist_coeffs = load_calibration(base_dir)

    cap = open_camera()
    if cap is None:
        print("ERROR: Cannot open camera or receive frames.")
        print("On macOS this uses the default OpenCV backend; on Jetson it uses /dev/video0 with V4L2.")
        print("Check that your camera is connected and not in use by another app.")
        raise SystemExit(1)

    detector = build_detector()

    # CLAHE instance — reused every frame (cheap)
    clahe = cv2.createCLAHE(clipLimit=CLAHE_CLIP_LIMIT, tileGridSize=CLAHE_TILE_SIZE)

    # Per-marker pose filters for temporal smoothing
    pose_filters = {}

    print("Camera ready. Low-light ArUco detection started.")
    print(
        f"Using {ARUCO_DICT_NAME}, landing pad ID={LANDING_PAD_ID}, "
        f"marker size={MARKER_SIZE_METERS * 1000:.0f} mm."
    )
    print("Press 'q' to quit.\n")

    try:
        _run_loop(cap, detector, clahe, pose_filters, camera_matrix, dist_coeffs)
    finally:
        cap.release()
        cv2.destroyAllWindows()


def _run_loop(cap, detector, clahe, pose_filters, camera_matrix, dist_coeffs):
    while True:
        ret, frame = cap.read()
        if not ret or frame is None:
            print("Warning: dropped frame.")
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray_clahe = preprocess_frame(gray, clahe)

        # Detection runs on the untouched 1-channel CLAHE image.
        corners, ids, detect_mode = detect_with_fallback(detector, gray_clahe)

        # Display copy: promote CLAHE feed to 3-channel BGR so we can draw
        # colored overlays on it. The detector never sees this copy.
        display = cv2.cvtColor(gray_clahe, cv2.COLOR_GRAY2BGR)

        if ids is not None and len(ids) > 0:
            aruco.drawDetectedMarkers(display, corners, ids)

            rvecs, tvecs, reproj_errors = estimate_pose_markers(
                corners, camera_matrix, dist_coeffs
            )

            for i, marker_id in enumerate(ids.flatten()):
                rvec = rvecs[i]
                tvec = tvecs[i]
                reproj_error = float(reproj_errors[i])

                # Validity gate: drop physically-impossible or unreliable solves
                # before they ever reach the smoothing filter, so one bad solve
                # can't corrupt the estimate.
                raw_z = float(tvec[2, 0])
                if (
                    raw_z <= 0
                    or not np.all(np.isfinite(tvec))
                    or reproj_error > MAX_REPROJ_ERROR_PX
                ):
                    p = corners[i][0][0].astype(int)
                    cv2.putText(
                        display,
                        f"ID:{marker_id} [bad pose] err:{reproj_error:.1f}px",
                        (p[0], max(25, p[1] - 10)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.55,
                        (0, 0, 255),
                        2,
                        cv2.LINE_AA,
                    )
                    continue

                # Initialize filter for this marker if first detection
                if marker_id not in pose_filters:
                    pose_filters[marker_id] = PoseFilter(
                        alpha=POSE_SMOOTH_ALPHA,
                        outlier_threshold_pos=POSE_OUTLIER_THRESHOLD_POS,
                        outlier_threshold_rot=POSE_OUTLIER_THRESHOLD_ROT,
                    )

                # Apply temporal smoothing and outlier rejection
                tvec, rvec, is_valid = pose_filters[marker_id].update(tvec, rvec)

                cv2.drawFrameAxes(display, camera_matrix, dist_coeffs, rvec, tvec, 0.08)

                x, y, z = tvec.flatten()
                validity_flag = "" if is_valid else " [rejected]"
                # raw_z is this frame's measurement; z is the smoothed output —
                # the gap between them is a direct readout of residual jitter.
                label = (
                    f"ID:{marker_id} X:{x:+.2f} Y:{y:+.2f} Z:{z:.2f}m "
                    f"(raw {raw_z:.2f}) err:{reproj_error:.1f}px "
                    f"[{detect_mode}]{validity_flag}"
                )

                p = corners[i][0][0].astype(int)
                cv2.putText(
                    display,
                    label,
                    (p[0], max(25, p[1] - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (0, 255, 0) if marker_id == LANDING_PAD_ID else (0, 200, 255),
                    2,
                    cv2.LINE_AA,
                )

                if marker_id == LANDING_PAD_ID:
                    print(
                        f"LANDING PAD ID:{marker_id} "
                        f"X:{x:+.3f}m Y:{y:+.3f}m Dist:{z:.3f}m (raw {raw_z:.3f}) "
                        f"err:{reproj_error:.2f}px [{detect_mode}]{validity_flag}"
                    )
                    publish_pose(LandingPadPose(
                        marker_id=int(marker_id),
                        tvec=tvec,
                        rvec=rvec,
                        reproj_error=reproj_error,
                        is_valid=is_valid,
                        detect_mode=detect_mode,
                    ))

        cv2.putText(
            display,
            "Press q to quit",
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.imshow("CLAHE ArUco Detection (low-light)", display)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break



if __name__ == "__main__":
    main()