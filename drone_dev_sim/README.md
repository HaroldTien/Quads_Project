# PX4 + Gazebo Harmonic SITL — S500 Drone Simulation

This project integrates a custom **S500** drone model and airframe into
[PX4-Autopilot](https://github.com/PX4/PX4-Autopilot) and runs it in Gazebo Sim 8
(Harmonic) SITL. A CMake wrapper handles installing the custom files into PX4,
building the firmware, and launching the simulation — all from a single command.

---

## Layout

| Path | Purpose |
|------|---------|
| `model/` | Custom Gazebo model (installed into PX4 as `s500`) |
| `airframes/4052_holybro_s500_sim.sh` | PX4 airframe config (installed as `4052_gz_s500`) |
| `tools/register_airframe.sh` | Registers the airframe in PX4's airframe list |
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
