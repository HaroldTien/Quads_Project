from __future__ import annotations  # allow "X | None" type hints on Python 3.9

import cv2
import cv2.aruco as aruco
import numpy as np
import sys
import threading
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
DENOISE = False
DENOISE_DIAMETER = 5        # bilateral neighborhood diameter (pixels)
DENOISE_SIGMA_COLOR = 50    # intensity sigma
DENOISE_SIGMA_SPACE = 50    # spatial sigma

# Camera exposure/gain — tweak if your camera supports manual control
# Set to -1 to leave at OS default (safe fallback)
MANUAL_EXPOSURE = -1       # e.g. 100 (ms*10 on V4L2). -1 = auto
MANUAL_GAIN = -1           # e.g. 200. -1 = auto
# ------------------------------------

# -------- Performance (Jetson Nano) --------
# The adaptive-threshold fallback pass roughly doubles detection cost, and on a
# marker-less frame the CLAHE pass fails and we pay for the fallback every frame
# for nothing. Instead, only run the fallback once every Nth marker-less frame,
# so the common no-marker case isn't paying double continuously. 1 = every frame
# (old behavior); 3 = at most once per 3 marker-less frames.
FALLBACK_EVERY_N = 3

# OV9281 is a monochrome sensor. Capture its raw 1-channel stream instead of
# letting V4L2/OpenCV expand it to BGR and then converting back to gray — that
# removes two full-frame conversions per frame on the Nano. Set False if your
# pipeline rejects the GREY format and capture returns empty frames.
NATIVE_GRAY = True

# Detection resolution scale. detectMarkers cost scales with pixel count, and the
# adaptive-threshold sweep dominates the frame budget on the Nano. Detecting on a
# downscaled copy (0.5 = quarter the pixels) is the single biggest FPS win. We map
# the corners back to full resolution and re-refine them with cornerSubPix on the
# full-res image (see refine_corners_fullres), so pose accuracy is preserved — only
# the coarse marker *search* runs at low res. 1.0 = detect at full res (old behavior).
DETECT_SCALE = 0.5

# V4L2 keeps a FIFO of driver buffers; at steady state the capture thread sits
# several frames behind the sensor, which is the visible latency. Requesting a
# 1-deep buffer makes the driver hand back the freshest frame instead of draining
# a backlog. Not every V4L2 device honors it, but on the OV9281/Nano it noticeably
# cuts glass-to-display delay. Set False to restore deep buffering (smoother FPS
# graph, worse latency).
LOW_LATENCY_BUFFER = True

# imshow of a 1280x800 frame is one of the most expensive per-iteration costs on
# a Jetson Nano and it adds display latency. Turn the preview off entirely for
# max throughput (headless/flight), or show only every Nth frame. Pose output
# and publish_pose() still run every frame regardless.
# Keep this False for the lowest-latency bench/flight runs; enable only when you
# explicitly want to inspect the preview window.
SHOW_WINDOW = False
SHOW_EVERY_N = 3
# Downscale the preview before imshow — the GTK blit of a full 1280x800 image is a
# top per-frame cost and adds display latency on the Nano. Detection/pose are
# unaffected (this only touches what's drawn). 1.0 = show full size.
DISPLAY_SCALE = 0.5

# Printing to the terminal on every frame is surprisingly expensive and can add
# visible lag to the real-time loop. Throttle pose output to every Nth frame.
PRINT_POSE_EVERY_N = 30

# Print per-block timings (preprocess/detect) every 30 frames to find the
# bottleneck. Off by default so the console stays clean during normal runs.
PROFILE = False
# -------------------------------------------

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

    # Latency: the V4L2 driver FIFO fills faster than the loop drains it, and the
    # FrameGrabber can only dequeue in order (one read per iteration), so it sits
    # a few frames behind the sensor at steady state. Requesting a 1-deep buffer
    # makes the driver hand back the freshest frame. See LOW_LATENCY_BUFFER for the
    # FPS-vs-latency trade-off note.
    if LOW_LATENCY_BUFFER:
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    # OV9281 is monochrome: grab the raw single-channel GREY stream and stop
    # OpenCV auto-expanding it to BGR. Saves a color conversion on capture (and
    # the BGR->GRAY convert in the loop). See NATIVE_GRAY.
    if NATIVE_GRAY:
        grey_fourcc = ord("G") | (ord("R") << 8) | (ord("E") << 16) | (ord("Y") << 24)
        cap.set(cv2.CAP_PROP_FOURCC, grey_fourcc)
        cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)

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


class FrameGrabber:
    """
    Background capture thread that keeps ONLY the most recent frame.

    Perceived lag comes from the driver frame queue filling up faster than the
    processing loop drains it (CAP_PROP_BUFFERSIZE=1 is ignored on macOS
    AVFoundation and often on V4L2). This thread reads flat-out and discards
    stale frames, so the main loop always processes the freshest frame and never
    falls behind — latency stays bounded to a single frame regardless of how
    slow detection is. A monotonically increasing `seq` lets the loop skip a
    frame it already processed instead of burning cycles re-detecting it.
    """

    def __init__(self, cap):
        self.cap = cap
        self._lock = threading.Lock()
        self._frame = None
        self._seq = 0
        self._running = True
        self._thread = threading.Thread(target=self._reader, daemon=True)
        self._thread.start()

    def _reader(self):
        while self._running:
            ret, frame = self.cap.read()
            if not ret or frame is None:
                continue
            # read() allocates a fresh buffer each call, so rebinding here never
            # clobbers a frame the main loop is still holding a reference to.
            with self._lock:
                self._frame = frame
                self._seq += 1

    def read(self):
        """Return (frame, seq) for the latest frame, or (None, seq) if none yet."""
        with self._lock:
            return self._frame, self._seq

    def stop(self):
        self._running = False
        self._thread.join(timeout=1.0)
        self.cap.release()


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

    # Adaptive thresholding — each window size in this range is a full-frame
    # threshold pass, so the count directly sets detection cost. 3..53 step 10 is
    # SIX passes; trimming to 3..23 step 10 (THREE passes) roughly halves the
    # threshold stage while still covering small-to-mid window sizes that catch
    # patchy shadows. Widen Max again if you lose markers under strong gradients.
    params.adaptiveThreshWinSizeMin = 3
    params.adaptiveThreshWinSizeMax = 23
    params.adaptiveThreshWinSizeStep = 10
    params.adaptiveThreshConstant = 7

    # Marker size floor — see MIN_MARKER_PERIMETER_RATE (stability vs reach trade-off)
    params.minMarkerPerimeterRate = MIN_MARKER_PERIMETER_RATE

    # Bit-error tolerance — see ERROR_CORRECTION_RATE
    params.errorCorrectionRate = ERROR_CORRECTION_RATE

    # Corner refinement is done OUTSIDE the detector (refine_corners_fullres) on the
    # full-resolution image, so the detector's own refinement — which would run on
    # the downscaled search image and be thrown away — is disabled. This also lets
    # the coarse search skip refinement entirely.
    params.cornerRefinementMethod = aruco.CORNER_REFINE_NONE

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


def detect_with_fallback(detector, gray_clahe: np.ndarray, run_fallback: bool = True):
    """
    Two-pass detection:
      Pass 1: CLAHE-enhanced grayscale (handles most low-light cases)
      Pass 2: Adaptive threshold on top of CLAHE (handles extreme shadows/hotspots)
    Returns corners, ids from whichever pass succeeds first.

    The second pass is expensive, so `run_fallback=False` skips it — the caller
    throttles it (see FALLBACK_EVERY_N) so marker-less frames don't pay double
    every single frame. detect_mode is "none" when pass 1 fails and the fallback
    is skipped.
    """
    corners, ids, rejected = detector.detectMarkers(gray_clahe)

    if ids is not None and len(ids) > 0:
        return corners, ids, "clahe"

    if not run_fallback:
        return corners, ids, "none"

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


_SUBPIX_CRITERIA = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01)


def refine_corners_fullres(gray: np.ndarray, corners):
    """
    Refine marker corners to sub-pixel accuracy on the FULL-resolution image.

    When DETECT_SCALE < 1 the marker search runs on a downscaled frame, so the
    returned corners (after being scaled back up) are only accurate to ~1/scale
    pixels — enough to jitter the pose. cornerSubPix snaps each corner to the true
    intensity edge in the full-res image, recovering the precision we skipped in the
    detector. Runs on the handful of detected corners only, so it's cheap.
    """
    refined = []
    for corner in corners:
        pts = corner.reshape(-1, 1, 2).astype(np.float32)
        cv2.cornerSubPix(gray, pts, (5, 5), (-1, -1), _SUBPIX_CRITERIA)
        refined.append(pts.reshape(1, 4, 2))
    return refined


def detect_scaled(detector, gray_clahe: np.ndarray, run_fallback: bool):
    """
    Run detection on a DETECT_SCALE copy of the CLAHE image, then map corners back
    to full resolution and re-refine them there. Falls back to plain full-res
    detection when DETECT_SCALE == 1.0.
    """
    if DETECT_SCALE == 1.0:
        corners, ids, mode = detect_with_fallback(detector, gray_clahe, run_fallback)
        if ids is not None and len(ids) > 0:
            corners = refine_corners_fullres(gray_clahe, corners)
        return corners, ids, mode

    small = cv2.resize(
        gray_clahe, None, fx=DETECT_SCALE, fy=DETECT_SCALE, interpolation=cv2.INTER_AREA
    )
    corners, ids, mode = detect_with_fallback(detector, small, run_fallback)

    if ids is not None and len(ids) > 0:
        inv = 1.0 / DETECT_SCALE
        corners = [c * inv for c in corners]                 # small -> full-res coords
        corners = refine_corners_fullres(gray_clahe, corners)  # recover sub-pixel edges

    return corners, ids, mode


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

    # Capture runs in its own thread so the loop always gets the freshest frame.
    grabber = FrameGrabber(cap)

    try:
        _run_loop(grabber, detector, clahe, pose_filters, camera_matrix, dist_coeffs)
    finally:
        grabber.stop()
        cv2.destroyAllWindows()


def _run_loop(grabber, detector, clahe, pose_filters, camera_matrix, dist_coeffs):
    # Counts consecutive marker-less frames so we only pay for the expensive
    # adaptive fallback once every FALLBACK_EVERY_N of them.
    misses = 0
    frame_count = 0
    last_seq = -1
    # Rolling FPS of the processing loop (exponentially smoothed so it doesn't
    # flicker frame-to-frame). Measured per processed frame, so it reflects real
    # throughput after skipping duplicate captures.
    fps = 0.0
    prev_t = time.perf_counter()
    while True:
        frame, seq = grabber.read()
        # No new frame since last iteration: don't re-process the same image
        # (wastes CPU). Service the GUI if shown, else back off briefly so we
        # don't spin a core hot waiting for the next capture.
        if frame is None or seq == last_seq:
            if SHOW_WINDOW:
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
            else:
                time.sleep(0.001)
            continue
        last_seq = seq

        frame_count += 1
        show = SHOW_WINDOW and (frame_count % SHOW_EVERY_N == 0)

        # Update rolling FPS from the gap since the last processed frame.
        now = time.perf_counter()
        dt = now - prev_t
        prev_t = now
        if dt > 0:
            inst_fps = 1.0 / dt
            fps = inst_fps if fps == 0.0 else 0.9 * fps + 0.1 * inst_fps

        t0 = time.perf_counter()
        # With NATIVE_GRAY the frame already arrives single-channel; otherwise
        # it's BGR and needs converting. Handle both so the flag is safe to flip.
        gray = frame if frame.ndim == 2 else cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray_clahe = preprocess_frame(gray, clahe)
        t1 = time.perf_counter()

        # Detection runs on the untouched 1-channel CLAHE image. Only spend the
        # adaptive fallback on the first marker-less frame and then every Nth.
        run_fallback = (misses % FALLBACK_EVERY_N == 0)
        corners, ids, detect_mode = detect_scaled(
            detector, gray_clahe, run_fallback
        )
        t2 = time.perf_counter()

        misses = 0 if (ids is not None and len(ids) > 0) else misses + 1

        if PROFILE and frame_count % 30 == 0:
            print(
                f"{fps:.1f} FPS  "
                f"preprocess {(t1 - t0) * 1000:.1f}ms  "
                f"detect {(t2 - t1) * 1000:.1f}ms  [{detect_mode}]"
            )

        # Display copy: promote CLAHE feed to 3-channel BGR so we can draw
        # colored overlays on it. Built only when we're actually showing this
        # frame — the GRAY2BGR + imshow is a top cost on the Nano.
        display = cv2.cvtColor(gray_clahe, cv2.COLOR_GRAY2BGR) if show else None

        if ids is not None and len(ids) > 0:
            if display is not None:
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
                    if display is not None:
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

                x, y, z = tvec.flatten()
                validity_flag = "" if is_valid else " [rejected]"

                if display is not None:
                    cv2.drawFrameAxes(display, camera_matrix, dist_coeffs, rvec, tvec, 0.08)

                    # raw_z is this frame's measurement; z is the smoothed output
                    # — the gap between them is a direct readout of residual jitter.
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
                    if PRINT_POSE_EVERY_N <= 1 or frame_count % PRINT_POSE_EVERY_N == 0:
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

        if display is not None:
            cv2.putText(
                display,
                f"{fps:.1f} FPS | q to quit",
                (20, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
            if DISPLAY_SCALE != 1.0:
                display = cv2.resize(
                    display, None, fx=DISPLAY_SCALE, fy=DISPLAY_SCALE,
                    interpolation=cv2.INTER_AREA,
                )
            cv2.imshow("CLAHE ArUco Detection (low-light)", display)

        if SHOW_WINDOW:
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break



if __name__ == "__main__":
    main()