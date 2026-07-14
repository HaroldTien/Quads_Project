############################################################################
#
# CMakeLists.txt for building and integrating custom drone_dev
# model and airframes into PX4-Autopilot for Gazebo simulation
#
# This will:
#   1. Copy custom model to PX4's Gazebo models directory
#   2. Copy custom airframe to PX4's ROMFS airframes directory
#   3. Set up build targets for easy installation
#
# Usage:
#   cd /home/harold/Projects/Quads_Project/drone_dev_sim
#   cmake -B build
#   cmake --build build --target install_all
#   cd ../PX4-Autopilot
#   make px4_sitl 4052_gz_s500
#
############################################################################

cmake_minimum_required(VERSION 3.16)
project(drone_dev_sim)

# Get absolute paths
get_filename_component(DRONE_DEV_DIR ${CMAKE_CURRENT_SOURCE_DIR} ABSOLUTE)
get_filename_component(PX4_DIR "${DRONE_DEV_DIR}/../PX4-Autopilot" ABSOLUTE)

# Verify PX4 directory exists
if(NOT EXISTS "${PX4_DIR}")
    message(FATAL_ERROR "PX4-Autopilot directory not found at: ${PX4_DIR}")
endif()

# Custom paths
set(CUSTOM_MODEL_DIR "${DRONE_DEV_DIR}/model")
set(CUSTOM_AIRFRAME_DIR "${DRONE_DEV_DIR}/airframes")
# Model name must match what PX4 extracts from airframe filename (4052_gz_s500 -> s500)
# PX4 sets PX4_SIM_MODEL=gz_s500, and Gazebo looks for "s500" directory (without gz_ prefix)
set(CUSTOM_MODEL_NAME "s500")
set(CUSTOM_AIRFRAME_SOURCE "4052_holybro_s500_sim.sh")
set(CUSTOM_AIRFRAME_DEST "4052_gz_s500")

# ArUco landing pad: a separate Gazebo model included into the world at origin
set(LANDING_PAD_DIR "${DRONE_DEV_DIR}/aruco_landing_pad")
set(LANDING_PAD_NAME "aruco_landing_pad")

# PX4 paths
set(PX4_AIRFRAME_DIR "${PX4_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes")
set(PX4_AIRFRAME_LIST "${PX4_AIRFRAME_DIR}/CMakeLists.txt")
set(PX4_MODEL_DIR "${PX4_DIR}/Tools/simulation/gz/models")
set(PX4_BUILD_DIR "${PX4_DIR}/build/px4_sitl_default")
set(PX4_WORLD_FILE "${PX4_DIR}/Tools/simulation/gz/worlds/default.sdf")

# Helper that registers the airframe name in PX4's explicit airframe list
set(REGISTER_AIRFRAME_SCRIPT "${DRONE_DEV_DIR}/tools/register_airframe.sh")
# Helper that includes/removes the landing pad in PX4's world file
set(REGISTER_LANDING_PAD_SCRIPT "${DRONE_DEV_DIR}/tools/register_landing_pad.sh")

# Verify custom files exist
if(NOT EXISTS "${CUSTOM_MODEL_DIR}/model.sdf")
    message(WARNING "Model file not found: ${CUSTOM_MODEL_DIR}/model.sdf")
endif()

if(NOT EXISTS "${CUSTOM_AIRFRAME_DIR}/${CUSTOM_AIRFRAME_SOURCE}")
    message(WARNING "Airframe file not found: ${CUSTOM_AIRFRAME_DIR}/${CUSTOM_AIRFRAME_SOURCE}")
endif()


############################################################################
# Setup: Copy/Link custom files to PX4 directories
############################################################################

# Function to copy airframe file to PX4 and register it in PX4's airframe list.
# The copy alone is not enough: PX4 only builds airframes explicitly listed in
# ROMFS/.../init.d-posix/airframes/CMakeLists.txt (no globbing), so we also
# register the destination name there or PX4 aborts with "Unknown model".
add_custom_target(setup_airframe
    COMMAND ${CMAKE_COMMAND} -E make_directory "${PX4_AIRFRAME_DIR}"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CUSTOM_AIRFRAME_DIR}/${CUSTOM_AIRFRAME_SOURCE}"
        "${PX4_AIRFRAME_DIR}/${CUSTOM_AIRFRAME_DEST}"
    COMMAND chmod 755 "${PX4_AIRFRAME_DIR}/${CUSTOM_AIRFRAME_DEST}"
    COMMAND bash "${REGISTER_AIRFRAME_SCRIPT}"
        "${PX4_AIRFRAME_LIST}" "${CUSTOM_AIRFRAME_DEST}"
    COMMENT "Copying airframe file to PX4 and registering it in the airframe list"
    VERBATIM
)

# Function to copy model directory to PX4
add_custom_target(setup_model
    COMMAND ${CMAKE_COMMAND} -E make_directory "${PX4_MODEL_DIR}"
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${CUSTOM_MODEL_DIR}"
        "${PX4_MODEL_DIR}/${CUSTOM_MODEL_NAME}"
    COMMENT "Copying model directory to PX4"
    VERBATIM
)

# Install the ArUco landing pad model into PX4's Gazebo models directory and
# include it in the world at the origin. Reading a model/world is a runtime
# operation for Gazebo, so this does NOT require a PX4 firmware rebuild.
add_custom_target(setup_landing_pad
    COMMAND ${CMAKE_COMMAND} -E make_directory "${PX4_MODEL_DIR}"
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        "${LANDING_PAD_DIR}"
        "${PX4_MODEL_DIR}/${LANDING_PAD_NAME}"
    COMMAND bash "${REGISTER_LANDING_PAD_SCRIPT}" "${PX4_WORLD_FILE}"
    COMMENT "Installing ArUco landing pad model and including it in the world"
    VERBATIM
)

# Clean PX4 build directory if it exists (to force reconfiguration with new airframe)
# This ensures PX4's build system will rescan and recognize the new airframe
# Using rm -rf which won't fail if directory doesn't exist
add_custom_target(clean_px4_for_reconfig
    COMMAND rm -rf "${PX4_BUILD_DIR}"
    COMMENT "Cleaning PX4 build to force reconfiguration with new airframe"
    DEPENDS setup_airframe
)

# Combined setup target
add_custom_target(setup
    DEPENDS setup_airframe setup_model setup_landing_pad clean_px4_for_reconfig
    COMMENT "Setting up custom model and airframe in PX4"
)

############################################################################
# Build PX4 Firmware
############################################################################

# Find make command
find_program(MAKE_COMMAND make)
if(NOT MAKE_COMMAND)
    message(FATAL_ERROR "make command not found. Please install make.")
endif()

# Extract model name from airframe filename (e.g., 4052_gz_s500 -> s500)
# This matches PX4's CMakeLists.txt logic: string(REGEX REPLACE ".*_gz_" "" model_name ${gz_airframe})
set(AIRFRAME_MODEL_NAME "s500")

# Build PX4 SITL (only build, don't run)
add_custom_target(build_px4
    COMMAND ${MAKE_COMMAND} -C "${PX4_DIR}" px4_sitl gz_${AIRFRAME_MODEL_NAME}
    WORKING_DIRECTORY "${PX4_DIR}"
    DEPENDS setup
    COMMENT "Building PX4 SITL firmware (without running simulation)"
    USES_TERMINAL
)

############################################################################
# Run Simulation
############################################################################

# Run simulation with custom model
# Note: PX4 generates targets as gz_<model_name> where model_name is extracted
# from the airframe filename by removing everything before and including "_gz_"
add_custom_target(run_simulation
    COMMAND ${MAKE_COMMAND} -C "${PX4_DIR}" px4_sitl gz_${AIRFRAME_MODEL_NAME}
    WORKING_DIRECTORY "${PX4_DIR}"
    DEPENDS setup
    COMMENT "Installing files, building PX4, and starting Gazebo simulation"
    USES_TERMINAL
)


############################################################################
# Clean targets
############################################################################

add_custom_target(clean_setup
    COMMAND ${CMAKE_COMMAND} -E remove "${PX4_AIRFRAME_DIR}/${CUSTOM_AIRFRAME_DEST}"
    COMMAND ${CMAKE_COMMAND} -E remove_directory "${PX4_MODEL_DIR}/${CUSTOM_MODEL_NAME}"
    COMMAND ${CMAKE_COMMAND} -E remove_directory "${PX4_MODEL_DIR}/${LANDING_PAD_NAME}"
    COMMAND bash "${REGISTER_AIRFRAME_SCRIPT}"
        "${PX4_AIRFRAME_LIST}" "${CUSTOM_AIRFRAME_DEST}" --unregister
    COMMAND bash "${REGISTER_LANDING_PAD_SCRIPT}" "${PX4_WORLD_FILE}" --unregister
    COMMENT "Cleaning up custom files from PX4 and unregistering the airframe"
    VERBATIM
)

add_custom_target(clean_px4
    COMMAND ${CMAKE_COMMAND} -E remove_directory "${PX4_BUILD_DIR}"
    COMMENT "Cleaning PX4 build directory"
)

############################################################################
# Information
############################################################################

message(STATUS "==========================================")
message(STATUS "Drone Development Environment Setup")
message(STATUS "==========================================")
message(STATUS "DRONE_DEV_DIR: ${DRONE_DEV_DIR}")
message(STATUS "PX4_DIR: ${PX4_DIR}")
message(STATUS "Custom Model: ${CUSTOM_MODEL_DIR} -> ${PX4_MODEL_DIR}/${CUSTOM_MODEL_NAME}")
message(STATUS "Custom Airframe: ${CUSTOM_AIRFRAME_DIR}/${CUSTOM_AIRFRAME_SOURCE} -> ${PX4_AIRFRAME_DIR}/${CUSTOM_AIRFRAME_DEST}")
message(STATUS "")
message(STATUS "Available targets:")
message(STATUS "  make setup          - Copy custom files to PX4")
message(STATUS "  make build_px4      - Build PX4 firmware")
message(STATUS "  make run_simulation - Build and run simulation")
message(STATUS "  make clean_setup    - Remove custom files from PX4")
message(STATUS "  make clean_px4      - Clean PX4 build")
message(STATUS "==========================================")

