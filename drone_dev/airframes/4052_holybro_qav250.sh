#!/bin/sh
#
# @name HolyBro QAV250 SITL (Gazebo)
# Combined airframe: QAV250 tuning parameters with Gazebo simulation
# Based on: 4052_holybro_qav250 (hardware) + 4006_gz_px4vision (simulation)
#
# @url https://docs.px4.io/main/en/frames_multicopter/holybro_qav250_pixhawk4_mini.html
#
# @type Quadrotor x
# @class Copter
#

. ${R}etc/init.d/rc.mc_defaults

# Gazebo Simulation Configuration
PX4_SIMULATOR=${PX4_SIMULATOR:=gz}
PX4_GZ_WORLD=${PX4_GZ_WORLD:=default}
# Note: PX4's CMakeLists.txt will override this with PX4_SIM_MODEL=gz_qav250
# Gazebo will look for "qav250" directory (without gz_ prefix)
PX4_SIM_MODEL=${PX4_SIM_MODEL:=qav250}

param set-default SIM_GZ_EN 1

# Commander Parameters
param set-default COM_DISARM_LAND 0.5

# EKF2 parameters (from gz_px4vision - useful for simulation)
param set-default EKF2_DRAG_CTRL 1
param set-default EKF2_IMU_POS_X 0.02
param set-default EKF2_GPS_POS_X 0.055
param set-default EKF2_GPS_POS_Z -0.15
param set-default EKF2_MIN_RNG 0.03
param set-default EKF2_OF_CTRL 1
param set-default EKF2_OF_POS_X 0.055
param set-default EKF2_OF_POS_Y 0.02
param set-default EKF2_OF_POS_Z 0.065
param set-default EKF2_REQ_HDRIFT 0.3
param set-default EKF2_REQ_SACC 1
param set-default EKF2_REQ_VDRIFT 0.3
param set-default EKF2_RNG_A_HMAX 8
param set-default EKF2_RNG_A_VMAX 2
param set-default EKF2_RNG_POS_X 0.055
param set-default EKF2_RNG_POS_Y -0.01
param set-default EKF2_RNG_POS_Z 0.065
param set-default EKF2_PCOEF_XP -0.25
param set-default EKF2_PCOEF_YN -0.55
param set-default EKF2_PCOEF_YP -0.55

# MAVLink parameters
param set-default MAV_1_RATE 80000
param set-default MAV_1_MODE 9

# Vehicle attitude PID tuning (from QAV250 - optimized for real hardware)
param set-default MC_AIRMODE 1
param set-default MC_PITCHRATE_D 0.0012
param set-default MC_PITCHRATE_I 0.35
param set-default MC_PITCHRATE_MAX 1200
param set-default MC_PITCHRATE_P 0.082
param set-default MC_PITCH_P 8
param set-default MC_ROLLRATE_D 0.0012
param set-default MC_ROLLRATE_I 0.3
param set-default MC_ROLLRATE_MAX 1200
param set-default MC_ROLLRATE_P 0.076
param set-default MC_ROLL_P 8
param set-default MC_YAWRATE_I 0.3
param set-default MC_YAWRATE_MAX 600
param set-default MC_YAWRATE_P 0.25
param set-default MC_YAW_P 4

# Acro mode parameters (from gz_px4vision)
param set-default MC_ACRO_P_MAX 200
param set-default MC_ACRO_R_MAX 200
param set-default MC_ACRO_Y_MAX 150

# Position Control Tuning (mixed: QAV250 for hover, gz_px4vision for other)
param set-default CP_DIST 6
param set-default MPC_ACC_DOWN_MAX 5
param set-default MPC_ACC_HOR_MAX 10
param set-default MPC_MANTHR_MIN 0
param set-default MPC_MAN_TILT_MAX 60
param set-default MPC_MAN_Y_MAX 120
param set-default MPC_THR_CURVE 1
param set-default MPC_THR_HOVER 0.25
param set-default MPC_THR_MIN 0.05
param set-default MPC_VEL_MANUAL 5
param set-default MPC_XY_VEL_MAX 5
param set-default MPC_XY_VEL_P_ACC 1.58
param set-default MPC_XY_TRAJ_P 0.3
param set-default MPC_Z_VEL_P_ACC 5
param set-default MPC_Z_VEL_I_ACC 1.7
param set-default MPC_LAND_ALT1 3
param set-default MPC_LAND_ALT2 1
param set-default CP_GO_NO_DATA 1

# Navigator Parameters
param set-default NAV_ACC_RAD 2
param set-default NAV_DLL_ACT 2

# RTL Parameters
param set-default RTL_DESCEND_ALT 5
param set-default RTL_RETURN_ALT 5

# Logging Parameters
param set-default SDLOG_PROFILE 131

# Sensors Parameters (from gz_px4vision)
param set-default SENS_FLOW_MAXHGT 25
param set-default SENS_FLOW_MINHGT 0.5

# IMU Parameters (from QAV250 - optimized for real hardware)
param set-default IMU_GYRO_CUTOFF 120
param set-default IMU_DGYRO_CUTOFF 45

# Power Parameters
param set-default BAT1_N_CELLS 4

# Thrust Model
param set-default THR_MDL_FAC 0.3

# Square quadrotor X PX4 numbering (same in both)
param set-default CA_ROTOR_COUNT 4
param set-default CA_ROTOR0_PX 1
param set-default CA_ROTOR0_PY 1
param set-default CA_ROTOR1_PX -1
param set-default CA_ROTOR1_PY -1
param set-default CA_ROTOR2_PX 1
param set-default CA_ROTOR2_PY -1
param set-default CA_ROTOR2_KM -0.05
param set-default CA_ROTOR3_PX -1
param set-default CA_ROTOR3_PY 1
param set-default CA_ROTOR3_KM -0.05

# Gazebo Simulation Actuator Configuration
param set-default SIM_GZ_EC_FUNC1 101
param set-default SIM_GZ_EC_FUNC2 102
param set-default SIM_GZ_EC_FUNC3 103
param set-default SIM_GZ_EC_FUNC4 104

param set-default SIM_GZ_EC_MIN1 150
param set-default SIM_GZ_EC_MIN2 150
param set-default SIM_GZ_EC_MIN3 150
param set-default SIM_GZ_EC_MIN4 150

param set-default SIM_GZ_EC_MAX1 1100
param set-default SIM_GZ_EC_MAX2 1100
param set-default SIM_GZ_EC_MAX3 1100
param set-default SIM_GZ_EC_MAX4 1100
