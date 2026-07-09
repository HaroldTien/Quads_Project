# Quads Project

End-to-end development environment for an autonomous quadcopter (Holybro
S500 class airframe) that finds an ArUco landing pad and lands on it. The
project glues together three layers:

1. **Flight stack** — PX4-Autopilot with a matching Gazebo SITL setup for a
   custom S500 airframe (`drone_dev_sim/`).
2. **Perception + control stack** — a ROS 2 workspace running on a Jetson
   Orin Nano companion computer: a CSI camera publisher, an ArUco marker
   detector tuned for low light, and a precision-landing controller that
   drives PX4 through MAVROS (`drone_dev/src/`).
3. **Camera tooling** — standalone Python scripts for intrinsic camera
   calibration (chessboard) and live ArUco prototyping, used to produce the
   `.npy` calibration files consumed by the ROS 2 nodes
   (`landingPadDetection/`).

The same airframe ID (`4052`) is used on real hardware and in simulation so
parameter tunes transfer cleanly between the two.

## Project Structure

```
Quads_Project/
├── PX4-Autopilot/              # PX4 Autopilot firmware (gitignored, cloned separately)
├── drone_dev/                  # ROS 2 perception + control workspace
│   ├── README.md               # Node startup quick-start + ROS 2 cheat sheet
│   ├── colcon.meta             # colcon: use ./src as the workspace base
│   └── src/
│       ├── csi_camera_node/        # ROS 2 pkg: csi_camera_publisher
│       │   ├── csi_camera_publisher/csi_camera_node.py
│       │   ├── launch/camera.launch.py
│       │   ├── config/camera_params.yaml
│       │   ├── data/                   # camera_matrix.npy / dist_coeffs.npy
│       │   ├── package.xml / setup.py / setup.cfg
│       │   └── test/                   # pep257 / flake8 / copyright linters
│       ├── ArUco_detector_node/    # ROS 2 pkg: aruco_detector_node
│       │   ├── ArUco_detector_node/aruco_detector_publisher.py
│       │   ├── ArUco_detector_node/aruco_detector.py
│       │   ├── tools/generate_aruco_marker.py
│       │   └── package.xml / setup.py / setup.cfg
│       └── landing_controller_node/ # ROS 2 pkg: landing_controller
│           ├── landing_controller/landing_controller_node.py
│           ├── landing_controller/controller.py
│           ├── landing_controller/takeoffcontroller.py
│           ├── launch/landing.launch.py
│           ├── launch/full_stack.launch.py
│           ├── config/landing_params.yaml
│           ├── package.xml / setup.py / setup.cfg
│           └── test/test_controller.py
├── drone_dev_sim/              # PX4 SITL (Gazebo) configuration
│   ├── CMakeLists.txt          # Copies model+airframe into PX4 and launches SITL
│   ├── airframes/4052_holybro_s500_sim.sh    # S500 airframe for Gazebo
│   └── model/                  # Custom Gazebo model (s500)
│       ├── model.sdf
│       ├── model.config
│       └── meshes/ thumbnails/
├── landingPadDetection/        # Standalone (non-ROS) calibration + ArUco scripts
│   ├── Camera_calibration_OV9218/  # Arducam OV9281 (global shutter, V4L2)
│   │   ├── collectImage.py
│   │   ├── Camera_Calibration.py
│   │   ├── stream.py
│   │   ├── aruco_detect.py
│   │   ├── vision_detection_lowLight.py
│   │   └── camera_matrix.npy / dist_coeffs.npy
│   ├── Camera_calibration_Im219/   # Raspberry Pi IMX219 (nvarguscamerasrc)
│   │   ├── collectImage.py
│   │   ├── Camera_Calibration.py
│   │   ├── detect.py
│   │   └── camera_matrix.npy / dist_coeffs.npy
│   └── mac_carera__/           # Modular detection/pose pipeline (macOS bench testing)
│       ├── app.py              # standalone runner (camera → detect → pose → overlay)
│       ├── camera.py / config.py / detection.py / pose.py
│       └── camera_matrix.npy / dist_coeffs.npy
├── build/ log/                 # colcon build artifacts
└── README.md                   # This file
```

> `PX4-Autopilot/` is **not** committed (see the root `.gitignore`). Clone it
> as a sibling of `drone_dev/` before running any of the SITL targets:
> `git clone https://github.com/PX4/PX4-Autopilot.git --recursive`.

## Components

### 1. PX4 SITL / Gazebo — `drone_dev_sim/`

Creates a custom Gazebo airframe `4052_gz_s500` that combines:

- **Hardware tuning** from the real S500 setup (PID gains, IMU filters,
  thrust model, rotor geometry, etc. — based on `4052_holybro_s500`)
- **Gazebo wiring** from `4006_gz_px4vision` (EKF2, optical flow, MAVLink
  rates, `SIM_GZ_EC_*` actuators)

A custom `s500` Gazebo model is provided. The CMake target chain
(`setup_airframe`, `setup_model`, `clean_px4_for_reconfig`, `setup`,
`build_px4`, `run_simulation`, `clean_setup`, `clean_px4`) copies these files
into the correct PX4 directories and forces a reconfigure when the airframe
list changes.

| Component | Source | Destination in PX4 |
|-----------|--------|--------------------|
| Airframe | `drone_dev_sim/airframes/4052_holybro_s500_sim.sh` | `PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/4052_gz_s500` |
| Model    | `drone_dev_sim/model/` | `PX4-Autopilot/Tools/simulation/gz/models/s500/` |

#### Naming convention

- Airframe filename `4052_gz_s500`:
  - `4052` → `SYS_AUTOSTART` ID (same as hardware)
  - `gz` → Gazebo simulator selector
  - `s500` → model name PX4 extracts from the filename
- PX4 target: `gz_s500` (used with `make px4_sitl gz_s500`)
- Gazebo model directory: `s500` (PX4 sets `PX4_SIM_MODEL=gz_s500`, and
  Gazebo searches for the directory **without** the `gz_` prefix)

### 2. ROS 2 Workspace — `drone_dev/src/`

A colcon workspace running on the Jetson Orin Nano companion computer.
`colcon.meta` pins the workspace base path to `src/`, so you can build from
the `drone_dev/` directory. `drone_dev/README.md` has a per-node startup
quick-start and a ROS 2 command cheat sheet.

#### `csi_camera_publisher` (`src/csi_camera_node/`)

Publishes the Arducam **OV9281** global-shutter CSI camera through the Jetson
Tegra V4L2 path using a GStreamer pipeline:

```
nvv4l2camerasrc → UYVY → nvvidconv → BGRx → BGR → appsink
```

| Topic | Type | Description |
|-------|------|-------------|
| `/camera/image_raw` | `sensor_msgs/Image` (`bgr8`) | Live camera frames |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | K, D, R, P built from the calibration `.npy` files |

Parameters (`config/camera_params.yaml`, override via launch file):

- `device` (default `/dev/video0`)
- `width` / `height` (default `1280 × 800`)
- `fps` (default `30`)
- `camera_matrix_path` / `dist_coeffs_path` — default to files installed from
  `src/csi_camera_node/data/` (drop `camera_matrix.npy` and `dist_coeffs.npy`
  produced by `landingPadDetection/` there before building; a calibration
  pair is already checked in).

Executable: `csi_camera_node`. Launch: `ros2 launch csi_camera_publisher
camera.launch.py`.

#### `aruco_detector_node` (`src/ArUco_detector_node/`)

Subscribes to the camera topics above and runs OpenCV ArUco detection +
`estimatePoseSingleMarkers`, with a low-light preprocessing pipeline.

- `aruco_detector.py` — pure-OpenCV helper (`ArucoDetector`) that supports
  both the modern `cv2.aruco.ArucoDetector` API and the legacy module-level
  functions. Runs a two-pass detection strategy: first on a CLAHE-enhanced
  (+ bilateral-denoised) frame for low light, then falls back to the raw
  grayscale frame if nothing is found. Also provides `rvec_to_quaternion`
  and a `draw_result` utility.
- `aruco_detector_publisher.py` — ROS 2 node (`aruco_detector_node`) that
  wires the helper to `/camera/image_raw` and `/camera/camera_info`, logs
  detected IDs, and logs the translation vector (`tvec`) of the first
  matching marker.

Parameters:

- `marker_length_m` (default `0.20` — 20 cm printed marker)
- `dictionary_name` (default `DICT_5X5_50` — must match the printed marker
  family)
- `target_marker_id` (default `0` — the landing pad)
- Low-light tuning: `enable_clahe`, `clahe_clip_limit`, `clahe_tile_size`,
  `enable_denoise`, `denoise_diameter`, `denoise_sigma_color`,
  `denoise_sigma_space`

Tool: `tools/generate_aruco_marker.py --id 0 --size 600 --out marker.png` to
produce a printable marker image (note: the tool generates from
`DICT_5X5_250`; IDs 0–49 also exist in `DICT_5X5_50`, but regenerate with the
matching dictionary if in doubt).

> ⚠️ The landing controller subscribes to `/aruco/pose`
> (`geometry_msgs/PoseStamped`), but the current revision of
> `aruco_detector_publisher.py` only logs detections — the pose-publishing
> code from the earlier revision was dropped when the low-light pipeline was
> ported in. Until it is restored, the full takeoff→land loop will sit in
> SEARCH.

#### `landing_controller` (`src/landing_controller_node/`)

Precision takeoff/landing controller: converts the marker pose into MAVROS
velocity setpoints. Split into ROS-free math modules (unit-testable on a
laptop) and a thin ROS node:

- `controller.py` — `camera_to_enu()` frame conversion plus a proportional
  controller with per-axis velocity clamps.
- `takeoffcontroller.py` — pure vertical-climb logic to a target altitude.
- `landing_controller_node.py` — the node. Subscribes to `/aruco/pose`,
  `/mavros/state`, and `/mavros/local_position/pose`; publishes
  `mavros_msgs/PositionTarget` velocity setpoints on
  `/mavros/setpoint_raw/local` at a fixed rate (default 20 Hz, must stay
  ≥ 2 Hz for OFFBOARD); calls the `/mavros/cmd/arming` and `/mavros/set_mode`
  services. State machine: `TAKEOFF → SEARCH → CENTER → DESCEND → LAND`
  (with `HOLD` when the marker is lost), handing off to PX4 `AUTO.LAND` for
  the final touchdown.

Parameters (`config/landing_params.yaml`): P gains (`kp_xy`, `kp_z`),
velocity clamps (`max_xy`, `max_z`), thresholds (`center_tol`, `land_alt`),
timing (`pose_timeout`, `rate_hz`), takeoff (`target_alt`, `climb_vel`,
`alt_tol`, `enable_takeoff`), and `auto_offboard` (auto-arm + switch to
OFFBOARD once setpoints stream — **leave `false` until bench-tested with
props off**).

Launch files:

- `landing.launch.py` — landing controller only.
- `full_stack.launch.py` — camera + ArUco detector + landing controller.

Unit tests: `test/test_controller.py` (pure-Python, runs with `pytest`).

### 3. Camera Calibration & ArUco Prototyping — `landingPadDetection/`

Standalone Python scripts (no ROS) used to produce `camera_matrix.npy` /
`dist_coeffs.npy` for the ROS 2 nodes, and to prototype detection before
moving it into ROS.

- **`Camera_calibration_OV9218/`** — Arducam OV9281 via V4L2
  (`cv2.VideoCapture(0, cv2.CAP_V4L2)`).
  - `collectImage.py` — interactive capture (ENTER/`c` to save, `q` to quit)
    targeting ~20 chessboard images at 1280×800.
  - `Camera_Calibration.py` — runs `cv2.calibrateCamera` on a 9×6 inner-corner
    chessboard and writes `camera_matrix.npy` / `dist_coeffs.npy`.
  - `stream.py` — quick 1280×800 @ 120 fps grayscale preview.
  - `aruco_detect.py` — live ArUco detection with axis overlay and per-marker
    distance read-out; uses `DICT_5X5_50` and a 20 cm marker.
  - `vision_detection_lowLight.py` — low-light detection prototype (CLAHE +
    bilateral denoise + tuned detector parameters); this pipeline is what was
    ported into the ROS 2 `aruco_detector_node`.
- **`Camera_calibration_Im219/`** — Raspberry Pi-style **IMX219** via the
  Jetson `nvarguscamerasrc` GStreamer pipeline (1280×720).
  - `collectImage.py` — terminal-driven capture (press ENTER to save).
  - `Camera_Calibration.py` — same calibration flow as the OV9281 version.
  - `detect.py` — live landing-pad ArUco detection (ID 0, 200 mm marker).
- **`mac_carera__/`** — the detection pipeline refactored into reusable
  modules (`camera.py`, `config.py`, `detection.py`, `pose.py`) with an
  `app.py` shell, so a ROS 2 node can import the same core. Adds pose
  stabilisation on top of detection: exponential smoothing, outlier
  rejection, and a reprojection-error gate. Set up for bench testing on a
  MacBook camera (see `config.py` — set `CAMERA_INDEX` back for the
  Jetson/OV9281).

Both calibration folders ship example `camera_matrix.npy` /
`dist_coeffs.npy` (the captured chessboard frames are written to
`image_taken/` / `images_taken/` locally but are not committed).

## Usage

### A. Run the Gazebo SITL Simulation

```bash
cd ~/Quads_Project/drone_dev_sim
cmake -B build
cd build
make run_simulation
```

Available targets:

| Target | Description |
|--------|-------------|
| `make setup` | Copy custom model + airframe into PX4 and force reconfigure |
| `make build_px4` | Build PX4 SITL firmware only |
| `make run_simulation` | Build and launch Gazebo with the S500 |
| `make clean_setup` | Remove the custom files from PX4 |
| `make clean_px4` | Remove the PX4 SITL build directory |

Equivalently, after `make setup` you can run PX4 directly:
`cd ../PX4-Autopilot && make px4_sitl gz_s500`.

On real hardware, set `SYS_AUTOSTART=4052` in QGroundControl so the same
tuning parameters load.

### B. Build & Run the ROS 2 Stack

> Tested on a Jetson Orin Nano running JetPack with ROS 2 Humble. Requires the
> Arducam Jetvariety driver loaded for the OV9281 (`lsmod | grep arducam`) and
> MAVROS installed for the landing controller.

```bash
cd ~/Quads_Project/drone_dev

# Optional: refresh the camera calibration shipped in the camera package
# (installed to share/csi_camera_publisher/data/)
cp ../landingPadDetection/Camera_calibration_OV9218/camera_matrix.npy \
   ../landingPadDetection/Camera_calibration_OV9218/dist_coeffs.npy \
   src/csi_camera_node/data/

# Build the workspace (uses colcon.meta to scope to ./src)
colcon build --symlink-install
source install/setup.bash
```

Then, in separate sourced terminals:

```bash
# Camera publisher
ros2 launch csi_camera_publisher camera.launch.py

# ArUco detector
ros2 run aruco_detector_node aruco_detector_node \
  --ros-args -p target_marker_id:=0 -p marker_length_m:=0.20

# MAVROS (Pixhawk over serial)
ros2 launch mavros px4.launch fcu_url:=/dev/ttyTHS1:921600

# Landing controller
ros2 launch landing_controller landing.launch.py
```

Or launch camera + detector + landing controller together:

```bash
ros2 launch landing_controller full_stack.launch.py
```

See `drone_dev/README.md` for a fuller cheat sheet (topic inspection,
`ros2 topic echo /aruco/pose`, etc.).

### C. Calibrate a Camera with `landingPadDetection/`

```bash
# OV9281 (Arducam global shutter, V4L2)
cd landingPadDetection/Camera_calibration_OV9218
python3 collectImage.py        # capture ~20 chessboard frames into image_taken/
python3 Camera_Calibration.py  # writes camera_matrix.npy and dist_coeffs.npy
python3 aruco_detect.py        # optional live ArUco sanity-check
python3 vision_detection_lowLight.py  # optional low-light pipeline check

# IMX219 (RPi-style CSI, nvarguscamerasrc)
cd landingPadDetection/Camera_calibration_Im219
python3 collectImage.py
python3 Camera_Calibration.py
python3 detect.py
```

Then copy the resulting `.npy` files into
`drone_dev/src/csi_camera_node/data/` (see step B) before rebuilding the
ROS 2 workspace.

### D. Generate a Printable ArUco Marker

```bash
python3 drone_dev/src/ArUco_detector_node/tools/generate_aruco_marker.py \
    --id 0 --size 600 --out aruco_id0.png
```

Print at the size that matches `marker_length_m` (default 20 cm).

## Notes

- `PX4-Autopilot/` is gitignored at the repo root; `drone_dev/.gitignore`
  keeps colcon artifacts (`build/`, `install/`, `log/`) out of the workspace.
  The top-level `build/` and `log/` directories contain colcon artifacts from
  building at the repo root and are currently checked in.
- The simulation airframe is **copied** into PX4 (not symlinked); the PX4
  build directory is wiped (`clean_px4_for_reconfig`) whenever the airframe
  list changes so PX4 rescans `init.d-posix/airframes/`.
- Hardware and simulation share the same `SYS_AUTOSTART=4052` ID so tuning
  parameters carry over between flights and SITL runs.
- The ROS 2 nodes assume ROS 2 Humble on a Jetson Orin Nano, but the pure
  Python helpers (`aruco_detector.py`, `controller.py`,
  `takeoffcontroller.py`, and the calibration/prototyping scripts) run on any
  host with OpenCV ≥ 4.7.
- `auto_offboard` auto-arms the vehicle and switches it into OFFBOARD mode.
  Keep it `false` (the default) until the stack has been bench-tested with
  propellers removed.
