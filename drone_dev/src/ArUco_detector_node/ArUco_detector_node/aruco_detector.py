"""Reusable ArUco detection utilities for the ROS 2 node."""

from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np


def rvec_to_quaternion(rvec: np.ndarray) -> Tuple[float, float, float, float]:
    """Convert an OpenCV Rodrigues rotation vector to a (x, y, z, w) quaternion.

    The output ordering matches geometry_msgs/Quaternion (x, y, z, w).
    """
    rotation_matrix, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64).reshape(3, 1))
    r = rotation_matrix
    trace = r[0, 0] + r[1, 1] + r[2, 2]

    if trace > 0.0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (r[2, 1] - r[1, 2]) * s
        y = (r[0, 2] - r[2, 0]) * s
        z = (r[1, 0] - r[0, 1]) * s
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = 2.0 * np.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2])
        w = (r[2, 1] - r[1, 2]) / s
        x = 0.25 * s
        y = (r[0, 1] + r[1, 0]) / s
        z = (r[0, 2] + r[2, 0]) / s
    elif r[1, 1] > r[2, 2]:
        s = 2.0 * np.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2])
        w = (r[0, 2] - r[2, 0]) / s
        x = (r[0, 1] + r[1, 0]) / s
        y = 0.25 * s
        z = (r[1, 2] + r[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1])
        w = (r[1, 0] - r[0, 1]) / s
        x = (r[0, 2] + r[2, 0]) / s
        y = (r[1, 2] + r[2, 1]) / s
        z = 0.25 * s

    return (float(x), float(y), float(z), float(w))


class ArucoDetector:
    """Small helper class that wraps OpenCV ArUco detection + pose estimation."""

    def __init__(
        self,
        marker_length_m: float = 0.20,
        dictionary_name: str = "DICT_5X5_250",
        enable_clahe: bool = True,
        clahe_clip_limit: float = 1.5,
        clahe_tile_size: int = 8,
        enable_denoise: bool = True,
        denoise_diameter: int = 5,
        denoise_sigma_color: float = 50.0,
        denoise_sigma_space: float = 50.0,
    ) -> None:
        # Physical marker size in meters (used by pose estimation).
        self.marker_length_m = marker_length_m

        # Pick one marker family. Must match your printed marker dictionary.
        if not hasattr(cv2.aruco, dictionary_name):
            raise ValueError(f"Unsupported ArUco dictionary: {dictionary_name}")
        dictionary_id = getattr(cv2.aruco, dictionary_name)
        self.dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)

        # Configure detector behavior tuned for low-light / uneven lighting:
        #  - Wide adaptive-threshold window range handles patchy shadows & glare.
        #  - Moderate perimeter/error-correction rates stay permissive for dim
        #    markers without admitting noisy/false reads.
        #  - Subpixel corner refinement for accurate corner locations.
        self.parameters = cv2.aruco.DetectorParameters()
        self.parameters.adaptiveThreshWinSizeMin = 3
        self.parameters.adaptiveThreshWinSizeMax = 53
        self.parameters.adaptiveThreshWinSizeStep = 10
        self.parameters.adaptiveThreshConstant = 7
        self.parameters.minMarkerPerimeterRate = 0.05
        self.parameters.errorCorrectionRate = 0.5
        self.parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.parameters.cornerRefinementWinSize = 5

        # Low-light preprocessing config. CLAHE equalises local contrast (the core
        # low-light fix); bilateral denoise suppresses the sensor grain CLAHE
        # amplifies, which otherwise wobbles corners frame-to-frame.
        self.enable_clahe = enable_clahe
        self.clahe = (
            cv2.createCLAHE(
                clipLimit=clahe_clip_limit,
                tileGridSize=(clahe_tile_size, clahe_tile_size),
            )
            if enable_clahe
            else None
        )
        self.enable_denoise = enable_denoise
        self.denoise_diameter = denoise_diameter
        self.denoise_sigma_color = denoise_sigma_color
        self.denoise_sigma_space = denoise_sigma_space

        # OpenCV has two APIs depending on version. Keep both for compatibility.
        self.use_modern_api = hasattr(cv2.aruco, "ArucoDetector")
        if self.use_modern_api:
            self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)

    def _detect_markers(self, image: np.ndarray) -> Tuple[List[np.ndarray], np.ndarray]:
        """Single detection pass on a (grayscale) image via whichever API exists."""
        if self.use_modern_api:
            corners, ids, _ = self.detector.detectMarkers(image)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                image,
                self.dictionary,
                parameters=self.parameters,
            )
        return corners, ids

    def _preprocess(self, gray: np.ndarray) -> np.ndarray:
        """Apply CLAHE (+ optional bilateral denoise) for low-light contrast."""
        enhanced = self.clahe.apply(gray) if self.clahe is not None else gray
        if self.enable_denoise:
            enhanced = cv2.bilateralFilter(
                enhanced,
                self.denoise_diameter,
                self.denoise_sigma_color,
                self.denoise_sigma_space,
            )
        return enhanced

    def detect(self, frame_bgr: np.ndarray) -> Tuple[List[np.ndarray], np.ndarray]:
        """
        Detect markers and return (corners, ids).

        Two-pass strategy for robustness across lighting:
          Pass 1 ("enhanced"): CLAHE (+ denoise) image — best for low light/shadows.
          Pass 2 ("raw"):      original grayscale — recovers markers that aggressive
                               CLAHE/denoise washed out (e.g. strong/over-exposed light).

        We feed grayscale (not pre-binarised) images: detectMarkers runs its own
        multi-window adaptive thresholding, so pre-thresholding is redundant.
        """
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)

        if self.enable_clahe or self.enable_denoise:
            enhanced = self._preprocess(gray)
            corners, ids = self._detect_markers(enhanced)
            if ids is not None and len(ids) > 0:
                return corners, ids

        # Fallback (or the only pass when preprocessing is disabled): raw gray.
        corners, ids = self._detect_markers(gray)

        # Normalize "no detection" to an empty list + None for easier handling.
        if ids is None:
            return [], None
        return corners, ids

    def estimate_pose(
        self,
        corners: List[np.ndarray],
        camera_matrix: np.ndarray,
        dist_coeffs: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Estimate marker pose and return (rvecs, tvecs)."""
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners,
            self.marker_length_m,
            camera_matrix,
            dist_coeffs,
        )
        return rvecs, tvecs

    def detect_and_estimate(
        self,
        frame_bgr: np.ndarray,
        camera_matrix: np.ndarray,
        dist_coeffs: np.ndarray,
        target_ids: Optional[List[int]] = None,
    ) -> Dict[str, object]:
        """Detect markers and estimate pose in one call."""
        corners, ids = self.detect(frame_bgr)

        if ids is None:
            return {"ids": None, "corners": [], "rvecs": None, "tvecs": None}

        # Optional ID filter (e.g. keep only landing pad marker ID 0).
        if target_ids:
            keep_indices = [i for i, marker_id in enumerate(ids.flatten()) if int(marker_id) in target_ids]
            if not keep_indices:
                return {"ids": None, "corners": [], "rvecs": None, "tvecs": None}
            corners = [corners[i] for i in keep_indices]
            ids = ids[keep_indices]

        rvecs, tvecs = self.estimate_pose(corners, camera_matrix, dist_coeffs)
        return {"ids": ids, "corners": corners, "rvecs": rvecs, "tvecs": tvecs}

    def draw_result(
        self,
        frame_bgr: np.ndarray,
        result: Dict[str, object],
        camera_matrix: np.ndarray,
        dist_coeffs: np.ndarray,
    ) -> np.ndarray:
        """Return a copy of frame with detected markers and axes drawn."""
        output = frame_bgr.copy()
        ids = result["ids"]
        corners = result["corners"]

        if ids is None:
            return output

        cv2.aruco.drawDetectedMarkers(output, corners, ids)

        # Draw pose axis for each marker (modern API fallback included).
        for i in range(len(ids)):
            if hasattr(cv2, "drawFrameAxes"):
                cv2.drawFrameAxes(
                    output,
                    camera_matrix,
                    dist_coeffs,
                    result["rvecs"][i],
                    result["tvecs"][i],
                    self.marker_length_m * 0.5,
                )
        return output
