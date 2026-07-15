"""
Pure ArUco low-light detection + pose estimation, lifted out of
landingPadDetection/Camera_calibration_OV9218/vision_detection_lowLight.py.

Everything here is frame-in / pose-out — no camera, no ROS, no module-level
globals. All tunables live on DetectorConfig so the ROS node can drive them from
parameters (and so the same code is unit-testable off a recorded image).
"""
from __future__ import annotations

from dataclasses import dataclass, field

import cv2
import cv2.aruco as aruco
import numpy as np


@dataclass
class DetectorConfig:
    # --- Marker / dictionary ---
    marker_size_meters: float = 0.20          # marker side length
    aruco_dict_name: str = "DICT_5X5_50"      # 5x5 family: 50/100/250/1000

    # --- Low-light preprocessing (CLAHE) ---
    clahe_clip_limit: float = 1.5
    clahe_tile_size: tuple = (8, 8)
    denoise: bool = False
    denoise_diameter: int = 5
    denoise_sigma_color: int = 50
    denoise_sigma_space: int = 50

    # --- Detection ---
    detect_scale: float = 0.5                 # detect at this fraction of full res
    fallback_every_n: int = 3                 # throttle the expensive adaptive pass
    min_marker_perimeter_rate: float = 0.05
    error_correction_rate: float = 0.5
    adaptive_win_min: int = 3
    adaptive_win_max: int = 23
    adaptive_win_step: int = 10
    adaptive_constant: int = 7

    # --- Pose validity gate ---
    max_reproj_error_px: float = 4.0


def build_clahe(cfg: DetectorConfig):
    """CLAHE instance — reused every frame (cheap)."""
    return cv2.createCLAHE(
        clipLimit=cfg.clahe_clip_limit, tileGridSize=tuple(cfg.clahe_tile_size)
    )


def build_detector(cfg: DetectorConfig):
    """
    Returns an ArucoDetector with parameters tuned for low-light/shadow conditions.
    Wider adaptive threshold window range catches markers under uneven lighting;
    lower minMarkerPerimeterRate helps with small/partially lit markers; higher
    errorCorrectionRate tolerates bit errors from noise in dark regions.
    """
    aruco_dict_id = getattr(aruco, cfg.aruco_dict_name, None)
    if aruco_dict_id is None:
        raise ValueError(
            f"Invalid aruco_dict_name='{cfg.aruco_dict_name}'. "
            "Use one of: DICT_5X5_50, DICT_5X5_100, DICT_5X5_250, DICT_5X5_1000."
        )

    aruco_dict = aruco.getPredefinedDictionary(aruco_dict_id)
    params = aruco.DetectorParameters()

    # Adaptive thresholding — each window size in this range is a full-frame
    # threshold pass, so the count directly sets detection cost.
    params.adaptiveThreshWinSizeMin = cfg.adaptive_win_min
    params.adaptiveThreshWinSizeMax = cfg.adaptive_win_max
    params.adaptiveThreshWinSizeStep = cfg.adaptive_win_step
    params.adaptiveThreshConstant = cfg.adaptive_constant

    params.minMarkerPerimeterRate = cfg.min_marker_perimeter_rate
    params.errorCorrectionRate = cfg.error_correction_rate
    params.cornerRefinementMethod = aruco.CORNER_REFINE_NONE

    return aruco.ArucoDetector(aruco_dict, params)


def preprocess_frame(gray: np.ndarray, clahe, cfg: DetectorConfig) -> np.ndarray:
    enhanced = clahe.apply(gray)
    if cfg.denoise:
        enhanced = cv2.bilateralFilter(
            enhanced, cfg.denoise_diameter, cfg.denoise_sigma_color, cfg.denoise_sigma_space
        )
    return enhanced


def detect_with_fallback(detector, gray_clahe: np.ndarray, run_fallback: bool = True):
    """
    Two-pass detection:
      Pass 1: CLAHE-enhanced grayscale (handles most low-light cases)
      Pass 2: Adaptive threshold on top of CLAHE (handles extreme shadows/hotspots)
    Returns (corners, ids, mode) from whichever pass succeeds first. The second pass
    is expensive, so run_fallback=False skips it (the caller throttles it).
    """
    corners, ids, _rejected = detector.detectMarkers(gray_clahe)

    if ids is not None and len(ids) > 0:
        return corners, ids, "clahe"

    if not run_fallback:
        return corners, ids, "none"

    adaptive = cv2.adaptiveThreshold(
        gray_clahe, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2
    )
    corners, ids, _ = detector.detectMarkers(adaptive)
    return corners, ids, "adaptive"


_SUBPIX_CRITERIA = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01)


def refine_corners_fullres(gray: np.ndarray, corners):
    refined = []
    for corner in corners:
        pts = corner.reshape(-1, 1, 2).astype(np.float32)
        cv2.cornerSubPix(gray, pts, (5, 5), (-1, -1), _SUBPIX_CRITERIA)
        refined.append(pts.reshape(1, 4, 2))
    return refined


def detect_scaled(detector, gray_clahe: np.ndarray, run_fallback: bool, cfg: DetectorConfig):
    """Detect at cfg.detect_scale, then map corners back to full-res and sub-pixel refine."""
    if cfg.detect_scale == 1.0:
        corners, ids, mode = detect_with_fallback(detector, gray_clahe, run_fallback)
        if ids is not None and len(ids) > 0:
            corners = refine_corners_fullres(gray_clahe, corners)
        return corners, ids, mode

    small = cv2.resize(
        gray_clahe, None, fx=cfg.detect_scale, fy=cfg.detect_scale,
        interpolation=cv2.INTER_AREA,
    )
    corners, ids, mode = detect_with_fallback(detector, small, run_fallback)

    if ids is not None and len(ids) > 0:
        inv = 1.0 / cfg.detect_scale
        corners = [c * inv for c in corners]                    # small -> full-res coords
        corners = refine_corners_fullres(gray_clahe, corners)   # recover sub-pixel edges

    return corners, ids, mode


def estimate_pose_markers(corners, camera_matrix, dist_coeffs, cfg: DetectorConfig):
    """Solve each marker's pose in the camera optical frame. Returns (rvecs, tvecs, errs)."""
    marker_size = cfg.marker_size_meters
    rvecs, tvecs, reproj_errors = [], [], []

    # Corner order REQUIRED by SOLVEPNP_IPPE_SQUARE: top-left, top-right,
    # bottom-right, bottom-left with +Y up — matches ArUco detectMarkers output.
    object_points = np.array([
        [-marker_size / 2, marker_size / 2, 0],
        [marker_size / 2, marker_size / 2, 0],
        [marker_size / 2, -marker_size / 2, 0],
        [-marker_size / 2, -marker_size / 2, 0],
    ], dtype=np.float32)

    for corner in corners:
        image_points = corner[0].astype(np.float32)
        retval, rvs, tvs, errs = cv2.solvePnPGeneric(
            object_points, image_points, camera_matrix, dist_coeffs,
            useExtrinsicGuess=False, flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )

        if retval < 1:
            rvecs.append(np.zeros((3, 1)))
            tvecs.append(np.zeros((3, 1)))
            reproj_errors.append(np.inf)
            continue

        candidates = [(float(errs[k].item()), rvs[k], tvs[k]) for k in range(retval)]
        front = [c for c in candidates if c[2][2, 0] > 0]   # positive Z (in front)
        pool = front if front else candidates
        err, rvec, tvec = min(pool, key=lambda c: c[0])

        rvecs.append(rvec)
        tvecs.append(tvec)
        reproj_errors.append(err)

    return np.array(rvecs), np.array(tvecs), np.array(reproj_errors)
