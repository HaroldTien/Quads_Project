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
    ) -> None:
        # Physical marker size in meters (used by pose estimation).
        self.marker_length_m = marker_length_m

        # Pick one marker family. Must match your printed marker dictionary.
        if not hasattr(cv2.aruco, dictionary_name):
            raise ValueError(f"Unsupported ArUco dictionary: {dictionary_name}")
        dictionary_id = getattr(cv2.aruco, dictionary_name)
        self.dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)

        # Configure detector behavior (thresholding, corner refinement, etc.).
        self.parameters = cv2.aruco.DetectorParameters()

        # OpenCV has two APIs depending on version. Keep both for compatibility.
        self.use_modern_api = hasattr(cv2.aruco, "ArucoDetector")
        if self.use_modern_api:
            self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)

    def detect(self, frame_bgr: np.ndarray) -> Tuple[List[np.ndarray], np.ndarray]:
        """Detect markers and return (corners, ids)."""
        if self.use_modern_api:
            corners, ids, _ = self.detector.detectMarkers(frame_bgr)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                frame_bgr,
                self.dictionary,
                parameters=self.parameters,
            )

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
