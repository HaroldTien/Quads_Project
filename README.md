# Quads Project

This project contains PX4-Autopilot configurations for the QAV250 quadcopter, supporting both real flight hardware (Pixhawk 6C) and Gazebo simulation.

## Project Structure

```
Quads_Project/
├── PX4-Autopilot/          # PX4 Autopilot firmware (gitignored)
├── drone_dev/              # Hardware flight configuration
│   ├── airframe/           # Hardware airframe configuration
│   │   └── 4052_holybro_qav250  # QAV250 hardware airframe
│   └── CMakeLists.txt      # Build system for Pixhawk 6C
├── drone_dev_sim/          # Simulation configuration
│   ├── model/              # Custom Gazebo model for QAV250
│   │   ├── model.sdf       # Gazebo model definition
│   │   └── model.config    # Model metadata
│   ├── airframes/          # Simulation airframe configurations
│   │   └── 4052_holybro_qav250_sim.sh  # QAV250 simulation airframe
│   └── CMakeLists.txt      # Build system for Gazebo simulation
└── README.md               # This file
```

### 1. Hardware Flight Configuration (`drone_dev/`)

Configured for real flight with Pixhawk 6C flight controller:

- **Airframe**: `4052_holybro_qav250` (HolyBro QAV250 - already included in PX4)
- **Board target**: `px4_fmu-v6c_default` (Pixhawk 6C)
- **Note**: No custom files needed - uses standard PX4 airframe

The `4052_holybro_qav250` airframe is already part of PX4-Autopilot, so no copying is required.

### 2. Simulation Configuration (`drone_dev_sim/`)

Created a custom Gazebo simulation airframe (`4052_gz_qav250`) that combines:

- **Hardware tuning parameters** from `4052_holybro_qav250` (real QAV250 flight controller configuration)
- **Gazebo simulation settings** from `4006_gz_px4vision` (Gazebo simulation configuration)

The airframe file is located at:
- Source: `drone_dev_sim/airframes/4052_holybro_qav250_sim.sh`
- Destination: `PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/4052_gz_qav250`

### 3. Custom Gazebo Model (Simulation Only)

Created a custom Gazebo model for QAV250 based on the PX4Vision model:

- Model name: `qav250`
- Location: `drone_dev_sim/model/`
- Copied to: `PX4-Autopilot/Tools/simulation/gz/models/qav250/`

The model includes:
- Physical properties (mass, inertia) tuned for QAV250
- Visual representation using PX4Vision mesh
- Sensor configurations (air pressure, magnetometer, IMU, etc.)

### 4. Build System Integration

Created CMake build systems for both hardware and simulation:

#### Hardware (`drone_dev/CMakeLists.txt`):
- **Builds firmware** for Pixhawk 6C (`px4_fmu-v6c_default`)
- **Uploads firmware** to flight controller via USB
- Uses existing PX4 airframe (no file copying needed)

#### Simulation (`drone_dev_sim/CMakeLists.txt`):
- **Automatically copies** custom model and airframe to PX4 directories
- **Forces PX4 reconfiguration** when new airframes are added
- **Provides convenient targets** for building and running simulations
- Custom files needed because QAV250 doesn't exist in Gazebo sim

### 4. Key Fixes

#### Model Name Matching
- Fixed model directory name from `qav250_custom` to `qav250` to match PX4's naming convention
- PX4 extracts model name from airframe filename: `4052_gz_qav250` → `qav250`
- Gazebo looks for model directory without the `gz_` prefix

#### Airframe Target Generation
- Fixed target name from `4052_gz_qav250` to `gz_qav250`
- PX4's CMakeLists.txt generates targets as `gz_<model_name>` where model_name is extracted from airframe filename

## Usage

### Option A: Hardware Flight (Pixhawk 6C)

#### Initial Setup

```bash
cd /home/harold/Projects/Quads_Project/drone_dev
cmake -B build
cd build
```

#### Upload Firmware to Pixhawk 6C

```bash
# Make sure Pixhawk 6C is connected via USB first!
make uploadtofc
```

This will:
1. Build PX4 firmware for Pixhawk 6C
2. Upload firmware to flight controller

#### Available Targets (Hardware)

- `make build_fw` - Build firmware only (no upload)
- `make uploadtofc` - Build and upload to Pixhawk 6C ⭐
- `make force_upload` - Force upload (if normal upload fails)
- `make clean_px4` - Clean PX4 build

**Important Notes:**
- Airframe ID is **4052** (set `SYS_AUTOSTART=4052` in QGroundControl)
- The `4052_holybro_qav250` airframe is already included in PX4
- Connect Pixhawk 6C via USB before running `make uploadtofc`
- Use `force_upload` if normal upload fails

### Option B: Simulation (Gazebo)

#### Initial Setup

```bash
cd /home/harold/Projects/Quads_Project/drone_dev_sim
cmake -B build
```

#### Run Simulation

```bash
cd /home/harold/Projects/Quads_Project/drone_dev_sim/build
make run_simulation
```

This will:
1. Copy custom model and airframe to PX4 directories
2. Clean PX4 build to force reconfiguration
3. Build PX4 SITL firmware
4. Launch Gazebo simulation with the custom QAV250 model

#### Available Targets (Simulation)

- `make setup` - Copy custom files to PX4
- `make build_px4` - Build PX4 firmware only (don't run)
- `make run_simulation` - Build and run simulation
- `make clean_setup` - Remove custom files from PX4
- `make clean_px4` - Clean PX4 build directory

## Technical Details

### Airframe Naming Convention

- Airframe filename: `4052_gz_qav250`
  - `4052` = SYS_AUTOSTART ID (matches hardware QAV250)
  - `gz` = Gazebo simulator identifier
  - `qav250` = Model name extracted by PX4

- PX4 target name: `gz_qav250`
  - Generated by PX4's CMakeLists.txt from airframe filename
  - Used with: `make px4_sitl gz_qav250`

- Gazebo model directory: `qav250`
  - Located at: `PX4-Autopilot/Tools/simulation/gz/models/qav250/`
  - Gazebo searches for this directory when `PX4_SIM_MODEL=gz_qav250` is set

### File Locations

#### Hardware Flight Files

No custom files required - uses standard PX4 airframe `4052_holybro_qav250` already in PX4-Autopilot.

#### Simulation Files

| Component | Source | Destination |
|-----------|--------|-------------|
| Airframe | `drone_dev_sim/airframes/4052_holybro_qav250_sim.sh` | `PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/4052_gz_qav250` |
| Model | `drone_dev_sim/model/` | `PX4-Autopilot/Tools/simulation/gz/models/qav250/` |

## Notes

- The `PX4-Autopilot/` directory is gitignored (see `.gitignore`)
- Custom files are copied to PX4 directories during build, not symlinked
- PX4 build directory is cleaned when new airframes are added to force reconfiguration
