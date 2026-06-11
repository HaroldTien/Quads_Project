"""
Central configuration for the ArUco landing-pad pipeline.

All tunable constants live here so the other modules (camera, detection, pose,
app) — and, later, a ROS 2 node — import from a single source of truth instead
of each carrying its own copy.
"""

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
DENOISE_DIAMETER = 5        # bilateral ne  ighborhood diameter (pixels)
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
