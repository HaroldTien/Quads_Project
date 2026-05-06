import cv2
import numpy as np
import glob
import os

# Must match YOUR printed checkerboard inner corners
CHECKERBOARD = (9, 6)  # change this if your board is different

criteria = (cv2.TERM_CRITERIA_EPS + 
            cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 
                        0:CHECKERBOARD[1]].T.reshape(-1, 2)

objpoints = []
imgpoints = []

# Search from script directory so execution cwd does not matter.
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
search_patterns = [
    os.path.join(BASE_DIR, 'image_taken', 'calib_*.jpg'),   # current folder name
    os.path.join(BASE_DIR, 'images_taken', 'calib_*.jpg'),  # backward compatibility
    os.path.join(BASE_DIR, 'calib_*.jpg'),                  # flat layout fallback
]

images = []
for pattern in search_patterns:
    images.extend(glob.glob(pattern))

images = sorted(set(images))

print(f"Found {len(images)} images")

good = 0
bad = 0

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

    if ret:
        objpoints.append(objp)
        corners_refined = cv2.cornerSubPix(
            gray, corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners_refined)
        good += 1
        print(f"  OK: {fname}")
    else:
        bad += 1
        print(f"  SKIP: {fname} (corners not found)")

print(f"\n{good} good / {bad} skipped")

if good < 10:
    print("Need at least 10 good images — retake more photos!")
    exit()

print("\nRunning calibration...")
ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None)

print(f"\nCalibration error: {ret:.4f} (lower is better, <1.0 is good)")
print(f"\ncamera_matrix:\n{camera_matrix}")
print(f"\ndist_coeffs:\n{dist_coeffs}")

# Save results
np.save('camera_matrix.npy', camera_matrix)
np.save('dist_coeffs.npy', dist_coeffs)
print("\nSaved camera_matrix.npy and dist_coeffs.npy")