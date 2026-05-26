import cv2
import cv2.aruco as aruco
import numpy as np
import sys
import time
from pathlib import Path


# -------- User settings --------
MARKER_SIZE_METERS = 0.20   # 20 cm marker side length
LANDING_PAD_ID = 0
# 5x5 family options: 50, 100, 250, 1000
ARUCO_DICT_NAME = "DICT_5X5_50"
CAMERA_INDEX = 0
FRAME_WIDTH = 1280
FRAME_HEIGHT = 800
FPS = 30
# ------------------------------

# -------- Low-light settings --------
# CLAHE parameters — increase clipLimit for more aggressive contrast boost
CLAHE_CLIP_LIMIT = 3.0
CLAHE_TILE_SIZE = (8, 8)

# Camera exposure/gain — tweak if your camera supports manual control
# Set to -1 to leave at OS default (safe fallback)
MANUAL_EXPOSURE = -1       # e.g. 100 (ms*10 on V4L2). -1 = auto
MANUAL_GAIN = -1           # e.g. 200. -1 = auto
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

    # Warm-up frames for stability
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

    # More permissive marker size to catch dark/dim markers
    params.minMarkerPerimeterRate = 0.02

    # Tolerate more bit errors (noise from grain in dark images)
    params.errorCorrectionRate = 0.8

    # Improve corner refinement accuracy
    params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
    params.cornerRefinementWinSize = 5

    return aruco.ArucoDetector(aruco_dict, params)


def preprocess_frame(gray: np.ndarray, clahe) -> np.ndarray:
    """Apply CLAHE to equalise contrast locally — core fix for shadows."""
    return clahe.apply(gray)


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

    print("Camera ready. Low-light ArUco detection started.")
    print(
        f"Using {ARUCO_DICT_NAME}, landing pad ID={LANDING_PAD_ID}, "
        f"marker size={MARKER_SIZE_METERS * 1000:.0f} mm."
    )
    print("Press 'q' to quit.\n")

    while True:
        ret, frame = cap.read()
        if not ret or frame is None:
            print("Warning: dropped frame.")
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray_clahe = preprocess_frame(gray, clahe)

        corners, ids, detect_mode = detect_with_fallback(detector, gray_clahe)

        if ids is not None and len(ids) > 0:
            aruco.drawDetectedMarkers(frame, corners, ids)

            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners,
                MARKER_SIZE_METERS,
                camera_matrix,
                dist_coeffs,
            )

            for i, marker_id in enumerate(ids.flatten()):
                rvec = rvecs[i]
                tvec = tvecs[i]

                cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.08)

                x, y, z = tvec[0]
                label = f"ID:{marker_id} X:{x:+.2f}m Y:{y:+.2f}m Z:{z:.2f}m [{detect_mode}]"

                p = corners[i][0][0].astype(int)
                cv2.putText(
                    frame,
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
                        f"X:{x:+.3f}m Y:{y:+.3f}m Dist:{z:.3f}m [{detect_mode}]"
                    )

        # Show CLAHE-enhanced feed in a second window for debugging
        # Comment out the line below when not needed
        cv2.imshow("CLAHE preview (low-light debug)", gray_clahe)

        cv2.putText(
            frame,
            "Press q to quit",
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.imshow("OV9218 ArUco Detection", frame)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()