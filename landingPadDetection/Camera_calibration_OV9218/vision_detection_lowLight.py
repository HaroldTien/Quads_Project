from __future__ import annotations  

import cv2
import cv2.aruco as aruco
import numpy as np
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path

from pose_filter import PoseFilter


# -------- User settings --------
MARKER_SIZE_METERS = 0.20   # 20 cm marker side length
LANDING_PAD_ID = 0
# 5x5 family options: 50, 100, 250, 1000
ARUCO_DICT_NAME = "DICT_5X5_50"

CAMERA_INDEX = 0
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
FPS = 30

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
FALLBACK_EVERY_N = 3
NATIVE_GRAY = False
DETECT_SCALE = 0.5
LOW_LATENCY_BUFFER = True
SHOW_WINDOW = False
SHOW_EVERY_N = 3
DISPLAY_SCALE = 0.5
PRINT_POSE_EVERY_N = 30
PROFILE = False
MIN_MARKER_PERIMETER_RATE = 0.05

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

    def __init__(self, cap):
        self.cap = cap
        self._lock = threading.Lock()
        self._frame = None
        self._seq = 0
        self._running = True
        
        self.capture_fps = 0.0
        self._thread = threading.Thread(target=self._reader, daemon=True)
        self._thread.start()

    def _reader(self):
        prev = time.perf_counter()
        while self._running:
            ret, frame = self.cap.read()
            if not ret or frame is None:
                continue
            now = time.perf_counter()
            gap = now - prev
            prev = now
            if gap > 0:
                inst = 1.0 / gap
                self.capture_fps = inst if self.capture_fps == 0.0 else 0.9 * self.capture_fps + 0.1 * inst
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

    params.adaptiveThreshWinSizeMin = 3
    params.adaptiveThreshWinSizeMax = 23
    params.adaptiveThreshWinSizeStep = 10
    params.adaptiveThreshConstant = 7

    # Marker size floor — see MIN_MARKER_PERIMETER_RATE (stability vs reach trade-off)
    params.minMarkerPerimeterRate = MIN_MARKER_PERIMETER_RATE

    # Bit-error tolerance — see ERROR_CORRECTION_RATE
    params.errorCorrectionRate = ERROR_CORRECTION_RATE
    params.cornerRefinementMethod = aruco.CORNER_REFINE_NONE

    return aruco.ArucoDetector(aruco_dict, params)


def estimate_pose_markers(corners, camera_matrix, dist_coeffs):
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
    refined = []
    for corner in corners:
        pts = corner.reshape(-1, 1, 2).astype(np.float32)
        cv2.cornerSubPix(gray, pts, (5, 5), (-1, -1), _SUBPIX_CRITERIA)
        refined.append(pts.reshape(1, 4, 2))
    return refined


def detect_scaled(detector, gray_clahe: np.ndarray, run_fallback: bool):
    
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


@dataclass
class LandingPadPose:
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
                f"loop {fps:.1f} FPS  capture {grabber.capture_fps:.1f} FPS  "
                f"preprocess {(t1 - t0) * 1000:.1f}ms  "
                f"detect {(t2 - t1) * 1000:.1f}ms  [{detect_mode}]"
            )

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
                        max_consecutive_rejects=POSE_MAX_CONSECUTIVE_REJECTS,
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