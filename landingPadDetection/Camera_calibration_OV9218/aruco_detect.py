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


def load_calibration(base_dir: Path):
    camera_matrix_path = base_dir / "camera_matrix.npy"
    dist_coeffs_path = base_dir / "dist_coeffs.npy"

    if not camera_matrix_path.exists() or not dist_coeffs_path.exists():
        raise FileNotFoundError(
            "Missing calibration files. Expected 'camera_matrix.npy' and "
            "'dist_coeffs.npy' in Camera_calibration_OV9218."
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

    # Warm-up frames for stability.
    for _ in range(40):
        ret, _ = cap.read()
        if ret:
            return cap
        time.sleep(0.02)

    cap.release()
    return None


def main():
    base_dir = Path(__file__).resolve().parent
    camera_matrix, dist_coeffs = load_calibration(base_dir)

    cap = open_camera()
    if cap is None:
        print("ERROR: Cannot open camera or receive frames.")
        print("On macOS this uses the default OpenCV backend; on Jetson it uses /dev/video0 with V4L2.")
        print("Check that your camera is connected and not in use by another app.")
        raise SystemExit(1)

    aruco_dict_id = getattr(aruco, ARUCO_DICT_NAME, None)
    if aruco_dict_id is None:
        raise ValueError(
            f"Invalid ARUCO_DICT_NAME='{ARUCO_DICT_NAME}'. "
            "Use one of: DICT_5X5_50, DICT_5X5_100, DICT_5X5_250, DICT_5X5_1000."
        )

    aruco_dict = aruco.getPredefinedDictionary(aruco_dict_id)
    params = aruco.DetectorParameters()
    detector = aruco.ArucoDetector(aruco_dict, params)

    print("Camera ready. Live ArUco detection started.")
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
        corners, ids, _ = detector.detectMarkers(gray)

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
                label = f"ID:{marker_id} X:{x:+.2f}m Y:{y:+.2f}m Z:{z:.2f}m"

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
                        f"X:{x:+.3f}m Y:{y:+.3f}m Dist:{z:.3f}m"
                    )

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
