"""ArUco detector construction, frame preprocessing, and marker detection."""

import cv2
import cv2.aruco as aruco
import numpy as np

from config import (
    ARUCO_DICT_NAME,
    MIN_MARKER_PERIMETER_RATE,
    ERROR_CORRECTION_RATE,
    DENOISE,
    DENOISE_DIAMETER,
    DENOISE_SIGMA_COLOR,
    DENOISE_SIGMA_SPACE,
)


def build_detector():
    """
    Returns an ArucoDetector tuned for low-light/shadow conditions while keeping
    pose stable:
      - Wide adaptive-threshold window range handles markers under uneven lighting.
      - minMarkerPerimeterRate / errorCorrectionRate are kept MODERATE (see the
        constants) — permissive enough for dim markers, but tight enough to reject
        the tiny/noisy detections that drive pose jitter.
      - Subpixel corner refinement for accurate corner locations.
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


def detect_with_fallback(detector, gray: np.ndarray, gray_enhanced: np.ndarray):
    """
    Two-pass detection, returning corners/ids from whichever pass succeeds first
    plus a tag of which input worked:
      Pass 1 ("enhanced"): CLAHE (+ optional denoise) image — best for low light
                           and uneven shadows.
      Pass 2 ("raw"):      original grayscale — recovers markers that aggressive
                           CLAHE/denoise washed out or whose noise it amplified.

    We deliberately do NOT pre-binarise the image: ArUco's detectMarkers already
    runs its own multi-window adaptive thresholding (the adaptiveThreshWinSize*
    params), so feeding it a pre-thresholded binary image is redundant and tends to
    hurt detection rather than help.
    """
    corners, ids, _ = detector.detectMarkers(gray_enhanced)
    if ids is not None and len(ids) > 0:
        return corners, ids, "enhanced"

    corners, ids, _ = detector.detectMarkers(gray)
    return corners, ids, "raw"
