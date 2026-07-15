# landing_pad_detector

Low-light ArUco landing-pad detection for **OV9281 + Jetson Nano + Pixhawk 6C (PX4)**.

This package is the *perception* half of the precision-landing pipeline. It does
**not** open the camera. `csi_camera_publisher` (the camera driver) owns the
hardware and publishes frames; this node subscribes, finds the landing pad,
estimates its 3-D pose, and forwards it to PX4 via MAVROS.

```
OV9281 ──► csi_camera_node ──► /camera/image_raw ─┐
                              /camera/camera_info ─┤
                                                   ▼
                                     landing_pad_detector
                                                   │  /mavros/landing_target/pose
                                                   ▼
                                       MAVROS ──MAVLink──► PX4 (Pixhawk 6C)
```

## Why a separate package (not merged into the camera node)

- **One camera owner.** Only the driver touches `/dev/video0`; detection is a pure
  consumer, so there's no hardware contention.
- **Testable offline.** Record a rosbag of `/camera/image_raw` once and replay it
  to tune CLAHE/ArUco params on a laptop — no drone needed.
- **Same speed, when it matters.** For flight, `composed.py` co-locates both nodes
  in one process (see below), so the split costs nothing at runtime.

The heavy lifting is the algorithm code lifted verbatim from
`landingPadDetection/Camera_calibration_OV9218/vision_detection_lowLight.py`:
`detector_lib.py` (CLAHE + ArUco + solvePnP), `pose_filter.py`, `rotation_utils.py`.
Only the camera-open / threading / display code was dropped — ROS replaces it.

## Layout

| File | Role |
|------|------|
| `detector_lib.py` | Pure frame-in / pose-out pipeline (no ROS). All tunables on `DetectorConfig`. |
| `pose_filter.py`, `rotation_utils.py` | Ported temporal smoothing + rotation math. |
| `detector_node.py` | ROS node: subscribe → detect → tf2 to body frame → publish. |
| `composed.py` | Single-process camera + detector (flight runtime). |
| `config/detector_params.yaml` | All parameters. |
| `launch/precision_land.launch.py` | Two separate processes (bench/debug). |
| `launch/composed_precision_land.launch.py` | One process (flight). |

## Build

```bash
cd ~/Documents/Quads_Project/drone_dev
colcon build --packages-select landing_pad_detector
source install/setup.bash
```

`mavros_msgs` is an **optional** dependency — the node builds and runs without it
(publishing only `PoseStamped`). Install it to also emit `mavros_msgs/LandingTarget`.

## Run

**Bench / debugging (two processes):**
```bash
ros2 launch landing_pad_detector precision_land.launch.py
```
**Flight (single process, co-located):**
```bash
ros2 launch landing_pad_detector composed_precision_land.launch.py
```
Start your existing MAVROS bringup separately — it owns the Pixhawk serial link.

Replay a recording instead of the live camera:
```bash
ros2 run landing_pad_detector detector_node --ros-args --params-file config/detector_params.yaml
ros2 bag play my_recording          # publishes /camera/image_raw + /camera/camera_info
```

## Three things you MUST set for real flights

1. **Camera-mounting transform.** The detector outputs the pad in the
   `camera_optical` frame, then uses tf2 to convert to `base_link` (body). The
   launch files publish a **placeholder** `base_link -> camera_optical` static
   transform — edit the translation/rotation to your measured mounting, or the
   drone servos toward an offset point. Set `transform_to_body: false` to publish
   the raw optical-frame pose instead (e.g. while testing on the bench).

2. **Calibration resolution must match.** Intrinsics arrive live on
   `/camera/camera_info` from the camera node's `.npy` files. Those files must be
   calibrated at the resolution the node actually publishes (currently 1280×800),
   or every distance is wrong. The old bench script assumed 640×480 — do not reuse
   that calibration blindly.

3. **PX4 precision-landing params** (set in QGroundControl):
   - `PLD_ENABLED` = enabled
   - Land mode configured to use the vision target (`RTL`/`AUTO.LAND` precision
     landing). Confirm your PX4 version's exact `PLD_*` set.
   - Verify MAVROS is passing `LANDING_TARGET` through (`ros2 topic echo
     /mavros/landing_target/pose` while a marker is in view).

## Performance note: publish `mono8`

The OV9281 is **monochrome**, but `csi_camera_node` currently publishes `bgr8`
(3× the data for no information). This detector already requests `mono8` from
`cv_bridge`, so it works either way — but switching the camera node to publish
`mono8` cuts the per-frame copy cost on the Nano. Recommended follow-up:
change the GStreamer pipeline / `cv2_to_imgmsg(..., encoding='mono8')` in the
camera node once you're ready.

## Note on "zero-copy / composable"

True zero-copy intra-process transport in ROS 2 is a C++ (`rclcpp`) feature.
`composed.py` co-locates both Python nodes in one process on a shared executor —
which removes the inter-process hop and is the meaningful win on the Nano — but
does not pass the numpy image by raw pointer the way a C++ intra-process pipeline
would. If profiling later shows the frame copy is the bottleneck, port the camera
+ detector to C++ components. Until then, Python co-location is the right call.
