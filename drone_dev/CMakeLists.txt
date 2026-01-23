############################################################################
#
# CMakeLists.txt for building and uploading PX4 firmware to Pixhawk 6C
# Uses existing QAV250 airframe (4052_holybro_qav250) in PX4
#
# This will:
#   1. Build PX4 firmware for Pixhawk 6C (px4_fmu-v6c_default)
#   2. Upload firmware to flight controller via USB
#
# Usage:
#   cd /home/harold/Projects/Quads_Project/drone_dev
#   cmake -B build
#   cd build
#   make uploadtofc     # Build and upload to Pixhawk 6C
#
# Note: The QAV250 airframe (4052_holybro_qav250) already exists in PX4,
#       so we don't need to copy any custom files.
#
############################################################################

cmake_minimum_required(VERSION 3.16)
project(drone_dev_hardware)

# Get absolute paths
get_filename_component(DRONE_DEV_DIR ${CMAKE_CURRENT_SOURCE_DIR} ABSOLUTE)
get_filename_component(PX4_DIR "${DRONE_DEV_DIR}/../PX4-Autopilot" ABSOLUTE)

# Verify PX4 directory exists
if(NOT EXISTS "${PX4_DIR}")
    message(FATAL_ERROR "PX4-Autopilot directory not found at: ${PX4_DIR}")
endif()

# PX4 configuration
set(PX4_BOARD_TARGET "px4_fmu-v6c_default")

############################################################################
# Build and Upload Targets
############################################################################

# Find make command
find_program(MAKE_COMMAND make)
if(NOT MAKE_COMMAND)
    message(FATAL_ERROR "make command not found. Please install make.")
endif()

# Build PX4 firmware for Pixhawk 6C (no upload)
add_custom_target(build_fw
    COMMAND ${MAKE_COMMAND} -C "${PX4_DIR}" ${PX4_BOARD_TARGET}
    WORKING_DIRECTORY "${PX4_DIR}"
    COMMENT "Building PX4 firmware for Pixhawk 6C (${PX4_BOARD_TARGET})"
    USES_TERMINAL
)

# Upload firmware to Pixhawk 6C via USB
# Note: Make sure your Pixhawk 6C is connected via USB before running this
add_custom_target(uploadtofc
    COMMAND ${MAKE_COMMAND} -C "${PX4_DIR}" ${PX4_BOARD_TARGET} upload
    WORKING_DIRECTORY "${PX4_DIR}"
    COMMENT "Building and uploading PX4 firmware to Pixhawk 6C"
    USES_TERMINAL
)

# Force upload (bypasses some checks)
add_custom_target(force_upload
    COMMAND ${MAKE_COMMAND} -C "${PX4_DIR}" ${PX4_BOARD_TARGET} force-upload
    WORKING_DIRECTORY "${PX4_DIR}"
    COMMENT "Force uploading PX4 firmware to Pixhawk 6C"
    USES_TERMINAL
)

############################################################################
# Clean target
############################################################################

add_custom_target(clean_px4
    COMMAND ${MAKE_COMMAND} -C "${PX4_DIR}" distclean
    WORKING_DIRECTORY "${PX4_DIR}"
    COMMENT "Cleaning PX4 build directory"
    USES_TERMINAL
)

############################################################################
# Information
############################################################################

message(STATUS "==========================================")
message(STATUS "PX4 Hardware Build Setup (Pixhawk 6C)")
message(STATUS "==========================================")
message(STATUS "DRONE_DEV_DIR: ${DRONE_DEV_DIR}")
message(STATUS "PX4_DIR: ${PX4_DIR}")
message(STATUS "Board Target: ${PX4_BOARD_TARGET}")
message(STATUS "Airframe: 4052_holybro_qav250 (already in PX4)")
message(STATUS "")
message(STATUS "Available targets:")
message(STATUS "  make build_fw     - Build firmware (no upload)")
message(STATUS "  make uploadtofc   - Build and upload to Pixhawk 6C")
message(STATUS "  make force_upload - Force upload (if normal upload fails)")
message(STATUS "  make clean_px4    - Clean PX4 build")
message(STATUS "")
message(STATUS "IMPORTANT:")
message(STATUS "  1. Connect Pixhawk 6C via USB before running 'make uploadtofc'")
message(STATUS "  2. The airframe ID is 4052 (use SYS_AUTOSTART=4052 in QGC)")
message(STATUS "  3. Airframe 4052_holybro_qav250 is already included in PX4")
message(STATUS "==========================================")
