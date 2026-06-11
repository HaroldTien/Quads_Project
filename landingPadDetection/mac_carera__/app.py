"""
Standalone runner: opens the camera, runs the detection/pose/smoothing pipeline,
and shows an annotated debug window. This is the "app shell" — all the reusable
logic lives in camera.py, detection.py, and pose.py, so a future ROS 2 node can
replace this file's loop (and publish_pose) while importing the same core.

Run:  python3 app.py     (press 'q' to quit)
"""

from pathlib import Path

import cv2
import cv2.aruco as aruco
import numpy as np

from config import (
    ARUCO_DICT_NAME,
    LANDING_PAD_ID,
    MARKER_SIZE_METERS,
    CLAHE_CLIP_LIMIT,
    CLAHE_TILE_SIZE,
    POSE_SMOOTH_ALPHA,
    POSE_OUTLIER_THRESHOLD_POS,
    POSE_OUTLIER_THRESHOLD_ROT,
    MAX_REPROJ_ERROR_PX,
)
from camera import open_camera, load_calibration
from detection import build_detector, preprocess_frame, detect_with_fallback
from pose import estimate_pose_markers, PoseFilter, LandingPadPose


def publish_pose(pose: LandingPadPose) -> None:
    """
    Hook for handing the landing-pad pose to the flight controller via MAVROS.

    TODO (follow-on): run this pipeline inside a ROS 2 (rclpy) node, transform
    `pose` from the camera frame to the drone body/nav frame (TF2), and publish to
    the MAVROS landing-target plugin — i.e. a geometry_msgs/PoseStamped on
    /mavros/landing_target/pose, or a mavros_msgs/LandingTarget on
    /mavros/landing_target/raw. MAVROS relays it to the autopilot for precision
    landing. Currently a no-op so the vision pipeline can run standalone on the bench.
    """
    # Intentionally left as a no-op until the MAVROS publisher is wired up.
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

    try:
        _run_loop(cap, detector, clahe, pose_filters, camera_matrix, dist_coeffs)
    finally:
        cap.release()
        cv2.destroyAllWindows()


def _run_loop(cap, detector, clahe, pose_filters, camera_matrix, dist_coeffs):
    while True:
        ret, frame = cap.read()
        if not ret or frame is None:
            print("Warning: dropped frame.")
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray_clahe = preprocess_frame(gray, clahe)

        # Detection runs on the 1-channel images; "enhanced" first, then raw gray.
        corners, ids, detect_mode = detect_with_fallback(detector, gray, gray_clahe)

        # Display copy: promote CLAHE feed to 3-channel BGR so we can draw
        # colored overlays on it. The detector never sees this copy.
        display = cv2.cvtColor(gray_clahe, cv2.COLOR_GRAY2BGR)

        if ids is not None and len(ids) > 0:
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
                    )

                # Apply temporal smoothing and outlier rejection
                tvec, rvec, is_valid = pose_filters[marker_id].update(tvec, rvec)

                cv2.drawFrameAxes(display, camera_matrix, dist_coeffs, rvec, tvec, 0.08)

                x, y, z = tvec.flatten()
                validity_flag = "" if is_valid else " [rejected]"
                # raw_z is this frame's measurement; z is the smoothed output —
                # the gap between them is a direct readout of residual jitter.
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

        cv2.putText(
            display,
            "Press q to quit",
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.imshow("CLAHE ArUco Detection (low-light)", display)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break


if __name__ == "__main__":
    main()
