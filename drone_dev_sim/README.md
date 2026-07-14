# PX4 + Gazebo Harmonic SITL — S500 Drone Simulation

This project integrates a custom **S500** drone model and airframe into
[PX4-Autopilot](https://github.com/PX4/PX4-Autopilot) and runs it in Gazebo Sim 8
(Harmonic) SITL. A CMake wrapper handles installing the custom files into PX4,
building the firmware, and launching the simulation — all from a single command.

---

## Layout

| Path | Purpose |
|------|---------|
| `model/` | Custom Gazebo model (installed into PX4 as `s500`); includes a down-facing camera |
| `airframes/4052_holybro_s500_sim.sh` | PX4 airframe config (installed as `4052_gz_s500`) |
| `aruco_landing_pad/` | ArUco landing-pad model (DICT_5X5_50, ID 0), included in the world at the origin |
| `tools/register_airframe.sh` | Registers the airframe in PX4's airframe list |
| `tools/register_landing_pad.sh` | Includes/removes the landing pad in PX4's world file |
| `tools/run_sim_stack.sh` | One-command launcher for the whole stack (tmux) |
| `CMakeLists.txt` | Build/install/run targets |

PX4-Autopilot is expected to live at `~/Projects/Quads_Project/PX4-Autopilot`
(a sibling of this directory).

---

## Prerequisites

These only need to be done once.

### 1. Install PX4 build dependencies

```bash
cd ~/Projects/Quads_Project/PX4-Autopilot
python -m pip install --user -r Tools/setup/requirements.txt
bash ./Tools/setup/ubuntu.sh
```

### 2. Disable conda auto-activation (fixes Python sqlite3 build error)

If `(base)` is active, Anaconda's broken sqlite3 will fail the PX4 build.

```bash
conda config --set auto_activate_base false
```

Open a new terminal afterward (or run `conda deactivate` before building).

---

## Running the Simulation

From this directory:

```bash
cd ~/Projects/Quads_Project/drone_dev_sim
cmake -B build                                   # configure (once, if build/ is missing)
cmake --build build --target run_simulation
```

The `run_simulation` target does everything in one shot:

1. **Installs** the custom model (`model/` → PX4's Gazebo models as `s500`) and
   airframe (`airframes/4052_holybro_s500_sim.sh` → PX4 as `4052_gz_s500`), and
   registers the airframe in PX4's airframe list.
2. **Cleans** PX4's build dir to force reconfiguration with the new airframe.
3. **Builds** PX4 SITL and **launches** Gazebo with the S500 drone
   (equivalent to `make px4_sitl gz_s500`).

> The first build takes ~10–30 minutes. Gazebo opens automatically and PX4 boots
> to a `pxh>` shell.

---

## Quick Start — full ArUco landing test

### Option A — one command (recommended)

Launches the entire stack (sim + MAVROS + camera bridge + detector +
controller) in a single tmux window with 5 labeled panes, waiting for the sim
to boot before starting the ROS side:

```bash
~/Projects/Quads_Project/drone_dev_sim/tools/run_sim_stack.sh
```

- Requires `tmux` (`sudo apt install tmux`). Without it, the script falls back
  to opening 5 separate `gnome-terminal` windows.
- Re-attach:  `tmux attach -t s500sim`
- Detach (leave running):  `Ctrl-b d`
- Stop everything:  `tmux kill-session -t s500sim`

The script also handles the common failure modes automatically: it strips
anaconda from `PATH` in every pane (anaconda's protoc breaks the PX4 gz build),
kills orphaned `gz`/`px4` processes from previous runs (a stale gz server stops
the Gazebo GUI from opening), and delays the controller until the MAVROS link
is up and the EKF has settled (an early one-shot arm request would be rejected).

The `pxh>` shell stays interactive in its pane. The controller runs with
`auto_offboard:=true`, so the drone takes off, centers on the pad, and lands on
its own.

### Option B — five terminals (manual, copy-paste)

The same sequence by hand, one terminal per block, **in this order** (the sim
must be up before MAVROS, or MAVROS spams `Time jump detected`). Leave the drone
on the ground — the controller does its own takeoff.

**Terminal 1 — simulation (PX4 + Gazebo):**

```bash
cd ~/Projects/Quads_Project/drone_dev_sim
cmake --build build --target run_simulation
```

Wait for `Ready for takeoff!` in the `pxh>` console before continuing.

**Terminal 2 — MAVROS:**

```bash
source /opt/ros/humble/setup.bash
ros2 launch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14580"
```

**Terminal 3 — camera bridge (Gazebo → ROS 2):**

```bash
source /opt/ros/humble/setup.bash
ros2 run ros_gz_bridge parameter_bridge \
  /down_camera@sensor_msgs/msg/Image@gz.msgs.Image \
  /camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo \
  --ros-args -r /down_camera:=/camera/image_raw -r /camera_info:=/camera/camera_info
```

**Terminal 4 — ArUco detector:**

```bash
source /opt/ros/humble/setup.bash
source ~/Projects/Quads_Project/drone_dev/install/setup.bash
ros2 run aruco_detector_node aruco_detector_node
```

(No detections while grounded — the camera is too close to the pad. Normal.)

**Terminal 5 — landing controller (starts the flight):**

```bash
source /opt/ros/humble/setup.bash
source ~/Projects/Quads_Project/drone_dev/install/setup.bash
ros2 run landing_controller landing_controller_node --ros-args \
  --params-file ~/Projects/Quads_Project/drone_dev/src/landing_controller_node/config/landing_params.yaml \
  -p auto_offboard:=true
```

`-p auto_offboard:=true` overrides the yaml's safe default (false) so the
controller switches PX4 to OFFBOARD and arms by itself — sim only; keep the
yaml default for the real drone. The state machine then runs
TAKEOFF → SEARCH → CENTER → DESCEND → AUTO.LAND on its own.

### If arming is rejected

- `Preflight Fail: No connection to the ground control station` → the airframe
  sets `NAV_DLL_ACT 0` to prevent this; if it reappears (e.g. param override),
  run `param set NAV_DLL_ACT 0` in the `pxh>` shell — a PX4 rebuild resets
  params to airframe defaults.
- The controller requests OFFBOARD + arm **once**; if arming was rejected,
  fix the cause then either restart the controller or run `commander arm`
  in `pxh>` (mode will already be OFFBOARD).
- Emergency: `commander mode auto:land` in `pxh>` brings the drone down;
  `commander disarm -f` kills motors.

### Handy checks while it flies

```bash
ros2 topic echo /mavros/state --once      # connected/armed/mode
ros2 topic hz /camera/image_raw           # camera feed ~25 Hz
ros2 topic echo /aruco/pose               # marker pose stream (once airborne)
ros2 run rqt_image_view rqt_image_view /camera/image_raw   # watch the camera
```

---

## Available Targets

| Target | Description |
|--------|-------------|
| `run_simulation` | Install custom files, build PX4, and launch Gazebo |
| `setup`          | Only copy custom model + airframe into PX4 |
| `build_px4`      | Install + build firmware (no simulation launch) |
| `clean_setup`    | Remove custom files from PX4 and unregister the airframe |
| `clean_px4`      | Delete PX4's `build/px4_sitl_default` directory |

Run any target with:

```bash
cmake --build build --target <target>
```

---

## Connecting the ROS 2 landing stack

The offboard ArUco-landing nodes live in the sibling workspace
`~/Projects/Quads_Project/drone_dev` (packages `aruco_detector_node` and
`landing_controller_node`; `csi_camera_node` is Jetson/CSI hardware-only and is
**not** used in simulation). Two stock ROS 2 tools connect them to this sim:

- **MAVROS** bridges PX4 SITL (MAVLink) ↔ ROS 2 (`/mavros/*`) — the sim
  stand-in for the companion-computer ↔ Pixhawk serial link on the real drone.
- **ros_gz_bridge** republishes the Gazebo down-camera as the topics the detector
  expects (`/camera/image_raw`, `/camera/camera_info`), replacing `csi_camera_node`.

Run each command in its own terminal, after `source /opt/ros/humble/setup.bash`.
The simulation (`run_simulation`) must already be running.

### One-time prerequisites

```bash
# ROS cv_bridge needs numpy < 2; remove the user-site numpy 2.x if present
pip3 uninstall numpy            # falls back to system numpy 1.x (used by cv_bridge)

# build the ROS 2 workspace
cd ~/Projects/Quads_Project/drone_dev
colcon build && source install/setup.bash
```

### 1. MAVROS — PX4 ↔ ROS 2

```bash
ros2 launch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14580"
# verify in another terminal: ros2 topic echo /mavros/state  -> connected: true
```

PX4 SITL listens on UDP `14580` and sends to `14540` (see `px4-rc.mavlink`);
adjust the ports if your PX4 config differs.

### 2. Camera bridge — Gazebo → ROS 2

```bash
ros2 run ros_gz_bridge parameter_bridge \
  /down_camera@sensor_msgs/msg/Image@gz.msgs.Image \
  /camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo \
  --ros-args -r /down_camera:=/camera/image_raw -r /camera_info:=/camera/camera_info
```

The Gazebo down-camera publishes image on `/down_camera` and info on
`/camera_info`; the remaps rename them to `/camera/image_raw` +
`/camera/camera_info`, exactly what `aruco_detector_node` subscribes to.

### 3. The landing nodes

```bash
ros2 run aruco_detector_node aruco_detector_node
ros2 run landing_controller landing_controller_node --ros-args \
  --params-file ~/Projects/Quads_Project/drone_dev/src/landing_controller_node/config/landing_params.yaml \
  -p auto_offboard:=true    # sim only: lets the controller switch to OFFBOARD and arm itself
```

See **Quick Start** above for the full terminal-by-terminal sequence and
troubleshooting.

### Just viewing the camera (no landing stack)

```bash
ros2 run ros_gz_image image_bridge /down_camera        # bridge image to ROS 2
ros2 run rqt_image_view rqt_image_view /down_camera    # view the feed
```

Or with no ROS at all: Gazebo GUI → **⋮** menu → **Image Display** → topic
`/down_camera`.

---

## Notes

- **Why the airframe registration step?** PX4 does *not* glob its airframes
  directory — every airframe must be listed explicitly in
  `ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt`. Copying the file
  in is not enough; without registration PX4 fails at startup with
  `Unknown model gz_s500`. `tools/register_airframe.sh` handles this idempotently
  (and reverses it on `clean_setup`).
- **Model name mapping:** the airframe filename `4052_gz_s500` tells PX4 to set
  `PX4_SIM_MODEL=gz_s500`, and Gazebo looks for the `s500` model directory
  (without the `gz_` prefix).
- **Rebuilding after model/airframe changes:** re-run `run_simulation` — the
  `setup` step cleans PX4's build dir so changes are picked up.
- **Build failures:** if it fails with `No module named 'future'`, run
  `pip3 install --user future`. If `gz-transport not found`, ensure
  `gz-transport13` is present in PX4's `gz_bridge/CMakeLists.txt`.
