import cv2
import cv2.aruco as aruco
import numpy as np
from pathlib import Path

# Load calibration files from common locations.
base_dir = Path(__file__).resolve().parent
search_dirs = [
    base_dir,
    base_dir / "Camera_calibration_OV9218",
    base_dir / "Camera_calibration_OV9218",
]

camera_matrix_path = None
dist_coeffs_path = None

for directory in search_dirs:
    cm_path = directory / "camera_matrix.npy"
    dc_path = directory / "dist_coeffs.npy"
    if cm_path.exists() and dc_path.exists():
        camera_matrix_path = cm_path
        dist_coeffs_path = dc_path
        break

if camera_matrix_path is None or dist_coeffs_path is None:
    raise FileNotFoundError(
        "Calibration files not found. Expected both 'camera_matrix.npy' and "
        "'dist_coeffs.npy' in project root, 'Camera_calibration', or "
        "'Camera_Calibration'."
    )

camera_matrix = np.load(str(camera_matrix_path))
dist_coeffs = np.load(str(dist_coeffs_path))

MARKER_SIZE = 0.2        # 200mm = 0.2 meters
LANDING_PAD_ID = 0

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_50)  # fixed
parameters = aruco.DetectorParameters()
detector = aruco.ArucoDetector(aruco_dict, parameters)

# CSI camera pipeline
def gstreamer_pipeline(width=1280, height=720, fps=30, flip=0):
    return (
        f"nvarguscamerasrc ! "
        f"video/x-raw(memory:NVMM), "
        f"width={width}, height={height}, "
        f"framerate={fps}/1 ! "
        f"nvvidconv flip-method={flip} ! "
        f"video/x-raw, format=BGRx ! "
        f"videoconvert ! "
        f"video/x-raw, format=BGR ! "
        f"appsink drop=1"
    )

cap = cv2.VideoCapture(gstreamer_pipeline(), cv2.CAP_GSTREAMER)

if not cap.isOpened():
    print("ERROR: Cannot open CSI camera!")
    exit()

print("Camera ready! Searching for landing pad...")
print("Press Ctrl+C to quit\n")

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame!")
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, rejected = detector.detectMarkers(gray)

    if ids is not None:
        for i, marker_id in enumerate(ids.flatten()):
            if marker_id == LANDING_PAD_ID:

                # Estimate position
                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(
                    corners[i:i+1],
                    MARKER_SIZE,
                    camera_matrix,
                    dist_coeffs
                )

                x = tvec[0][0][0]   # left/right  (negative=left, positive=right)
                y = tvec[0][0][1]   # up/down      (negative=up,   positive=down)
                z = tvec[0][0][2]   # distance from camera

                print(f"FOUND  ID:{marker_id}  "
                      f"X:{x:+.3f}m  "
                      f"Y:{y:+.3f}m  "
                      f"Dist:{z:.3f}m")
            else:
                print(f"Seen marker ID:{marker_id} (not landing pad)")
    else:
        print("Searching...")

cap.release()