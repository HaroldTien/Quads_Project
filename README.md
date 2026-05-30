# Quads Project

End-to-end development environment for an autonomous S500 quadcopter. The
project glues together three layers:

1. **Flight stack** — PX4-Autopilot firmware for the Pixhawk 6C plus a matching
   Gazebo SITL setup (`drone_dev/CMakeLists.txt`, `drone_dev_sim/`).
2. **Perception stack** — a ROS 2 workspace running on a Jetson Orin Nano
   companion computer that publishes images from a CSI camera and runs ArUco
   marker detection for landing-pad localisation (`drone_dev/src/`).
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
├── drone_dev/                  # Hardware flight + ROS 2 perception workspace
│   ├── CMakeLists.txt          # Builds/uploads PX4 firmware to Pixhawk 6C
│   ├── colcon.meta             # colcon: use ./src as the workspace base
│   └── src/
│       ├── csi_camera_node/        # ROS 2 pkg: csi_camera_publisher
│       │   ├── csi_camera_publisher/csi_camera_node.py
│       │   ├── launch/camera.launch.py
│       │   ├── config/camera_params.yaml
│       │   ├── package.xml / setup.py / setup.cfg
│       │   └── test/                   # pep257 / flake8 / copyright linters
│       └── ArUco_detector_node/    # ROS 2 pkg: aruco_detector_node
│           ├── ArUco_detector_node/aruco_detector_node.py
│           ├── ArUco_detector_node/aruco_detector.py
│           ├── tools/generate_aruco_marker.py
│           └── package.xml / setup.py / setup.cfg
├── drone_dev_sim/              # PX4 SITL (Gazebo) configuration
│   ├── CMakeLists.txt          # Copies model+airframe into PX4 and launches SITL
│   ├── airframes/4052_holybro_s500_sim.sh     # S500 airframe for Gazebo
│   └── model/                  # Custom Gazebo model (s500)
│       ├── model.sdf
│       └── model.config
├── landingPadDetection/        # Standalone (non-ROS) calibration + ArUco scripts
│   ├── Camera_calibration_OV9218/  # Arducam OV9281 (global shutter, V4L2)
│   │   ├── collectImage.py
│   │   ├── Camera_Calibration.py
│   │   ├── stream.py
│   │   ├── aruco_detect.py
│   │   ├── camera_matrix.npy / dist_coeffs.npy
│   │   └── image_taken/                 # captured chessboard frames
│   └── Camera_calibration_Im219/   # Raspberry Pi IMX219 (nvarguscamerasrc)
│       ├── collectImage.py
│       ├── Camera_Calibration.py
│       ├── detect.py
│       ├── camera_matrix.npy / dist_coeffs.npy
│       └── images_taken/                # captured chessboard frames
├── build/ install/ log/        # colcon build artifacts (gitignored)
└── README.md                   # This file
```

> `PX4-Autopilot/` is **not** committed (see `.gitignore`). Clone it as a
> sibling of `drone_dev/` before running any of the firmware/SITL targets:
> `git clone https://github.com/PX4/PX4-Autopilot.git --recursive`.

## Components

### 1. Hardware Flight Firmware — `drone_dev/CMakeLists.txt`

Wraps PX4's build system so you can flash the Pixhawk 6C without remembering
PX4 board target names.

- **Board target**: `px4_fmu-v6c_default` (Pixhawk 6C)
- **Airframe**: `4052_holybro_s500` — already shipped with PX4, no custom
  files are copied for hardware flight.
- **Airframe ID**: `4052` (set `SYS_AUTOSTART=4052` in QGroundControl)

### 2. PX4 SITL / Gazebo — `drone_dev_sim/`

Creates a custom Gazebo airframe `4052_gz_s500` that combines:

- **Hardware tuning** from the real `4052_holybro_s500` (PID gains, IMU
  filters, thrust model, rotor geometry, etc.)
- **Gazebo wiring** from `4006_gz_px4vision` (EKF2, optical flow, MAVLink
  rates, `SIM_GZ_EC_*` actuators)

A custom `s500` Gazebo model is provided because no S500 model ships with
PX4's SITL out of the box. The CMake target chain (`setup_airframe`,
`setup_model`, `clean_px4_for_reconfig`, `setup`, `build_px4`,
`run_simulation`, `clean_setup`, `clean_px4`) copies these files into the
correct PX4 directories and forces a reconfigure when the airframe list
changes.

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

### 3. ROS 2 Workspace — `drone_dev/src/`

A small colcon workspace running on the Jetson Orin Nano companion computer.
`colcon.meta` pins the workspace base path to `src/`, so you can build from
the `drone_dev/` directory.

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
- `camera_matrix_path` / `dist_coeffs_path` — default to files in the
  installed `share/csi_camera_publisher/data/` (drop `camera_matrix.npy` and
  `dist_coeffs.npy` produced by `landingPadDetection/` there before building).

Executable: `csi_camera_node`. Launch: `ros2 launch csi_camera_publisher
camera.launch.py`.

#### `aruco_detector_node` (`src/ArUco_detector_node/`)

Subscribes to the camera topics above and runs OpenCV ArUco detection +
`estimatePoseSingleMarkers`.

- `aruco_detector.py` — pure-OpenCV helper (`ArucoDetector`) that supports
  both the modern `cv2.aruco.ArucoDetector` API and the legacy module-level
  functions, plus a `draw_result` utility.
- `aruco_detector_node.py` — ROS 2 node that wires the helper to
  `/camera/image_raw` and `/camera/camera_info`, logs detected IDs, and prints
  the translation vector (`tvec`) of the first matching marker.

Parameters:

- `marker_length_m` (default `0.20` — 20 cm printed marker)
- `dictionary_name` (default `DICT_5X5_250`)
- `target_marker_id` (default `0` — the landing pad)

Tool: `tools/generate_aruco_marker.py --id 0 --size 600 --out marker.png` to
produce a printable marker image.

### 4. Camera Calibration & ArUco Prototyping — `landingPadDetection/`

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
- **`Camera_calibration_Im219/`** — Raspberry Pi-style **IMX219** via the
  Jetson `nvarguscamerasrc` GStreamer pipeline.
  - `collectImage.py` — terminal-driven capture (press ENTER to save).
  - `Camera_Calibration.py` — same calibration flow as the OV9281 version.
  - `detect.py` — live landing-pad ArUco detection (ID 0, 200 mm marker).

Both folders ship example `camera_matrix.npy` / `dist_coeffs.npy` and the
captured chessboard frames used to produce them (under `image_taken/` and
`images_taken/` respectively).

## Usage

### A. Build and Upload Firmware to the Pixhawk 6C

```bash
cd ~/Quads_Project/drone_dev
cmake -B build
cd build

# Make sure the Pixhawk 6C is connected via USB!
make uploadtofc
```

Available targets:

| Target | Description |
|--------|-------------|
| `make build_fw` | Build PX4 firmware for Pixhawk 6C (no upload) |
| `make uploadtofc` | Build **and** upload over USB ⭐ |
| `make force_upload` | Force upload (use if the normal upload fails) |
| `make clean_px4` | `make distclean` inside PX4-Autopilot |

After flashing, set `SYS_AUTOSTART=4052` in QGroundControl to select the
`4052_holybro_s500` airframe.

### B. Run the Gazebo SITL Simulation

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

### C. Build & Run the ROS 2 Perception Workspace

> Tested on a Jetson Orin Nano running JetPack with ROS 2 Humble. Requires the
> Arducam Jetvariety driver loaded for the OV9281 (`lsmod | grep arducam`).

```bash
cd ~/Quads_Project/drone_dev

# Optional: drop your camera calibration into the camera package data dir
# so it gets installed to share/csi_camera_publisher/data/
mkdir -p src/csi_camera_node/data
cp ../landingPadDetection/Camera_calibration_OV9218/camera_matrix.npy \
   ../landingPadDetection/Camera_calibration_OV9218/dist_coeffs.npy \
   src/csi_camera_node/data/

# Build the workspace (uses colcon.meta to scope to ./src)
colcon build --symlink-install
source install/setup.bash

# Terminal 1 — camera publisher
ros2 launch csi_camera_publisher camera.launch.py

# Terminal 2 — ArUco detector
ros2 run aruco_detector_node aruco_detector_node \
  --ros-args -p target_marker_id:=0 -p marker_length_m:=0.20
```

The detector logs the detected marker IDs and the first marker's translation
vector in metres (camera frame).



### D. Calibrate a Camera with `landingPadDetection/`

```bash
# OV9281 (Arducam global shutter, V4L2)
cd landingPadDetection/Camera_calibration_OV9218
python3 collectImage.py        # capture ~20 chessboard frames into image_taken/
python3 Camera_Calibration.py  # writes camera_matrix.npy and dist_coeffs.npy
python3 aruco_detect.py        # optional live ArUco sanity-check

# IMX219 (RPi-style CSI, nvarguscamerasrc)
cd landingPadDetection/Camera_calibration_Im219
python3 collectImage.py
python3 Camera_Calibration.py
python3 detect.py
```

Then copy the resulting `.npy` files into
`drone_dev/src/csi_camera_node/data/` (see step C) before rebuilding the ROS 2
workspace.

### E. Generate a Printable ArUco Marker

```bash
python3 drone_dev/src/ArUco_detector_node/tools/generate_aruco_marker.py \
    --id 0 --size 600 --out aruco_id0.png
```

Print at the size that matches `marker_length_m` (default 20 cm).

## Notes

- The `PX4-Autopilot/`, `build/`, `install/`, and `log/` directories are
  gitignored — see the root `.gitignore` and `drone_dev/.gitignore`.
- The simulation airframe is **copied** into PX4 (not symlinked); the PX4
  build directory is wiped (`clean_px4_for_reconfig`) whenever the airframe
  list changes so PX4 rescans `init.d-posix/airframes/`.
- Hardware and simulation share the same `SYS_AUTOSTART=4052` ID so tuning
  parameters carry over between flights and SITL runs.
- The ROS 2 nodes assume ROS 2 Humble on a Jetson Orin Nano, but the pure
  Python helpers (`aruco_detector.py`, calibration scripts) run on any host
  with OpenCV ≥ 4.7.
