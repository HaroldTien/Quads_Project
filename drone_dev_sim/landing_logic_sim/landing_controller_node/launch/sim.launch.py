"""
Tier-2 Gazebo simulation bring-up for ArUco precision landing.

This launch handles the ROS side ONLY. Start PX4 + Gazebo separately first:

    cd <PX4-Autopilot>
    make px4_sitl gz_qav250

(or use the drone_dev_sim `run_simulation` CMake target). PX4 starts the gz
server + the "default" world, which provides the render engine + sensors-system
plugin the model's down_camera needs. Once the world is up, run:

    ros2 launch landing_controller sim.launch.py

What this brings up:
  1. MAVROS         — bridges PX4 MAVLink <-> /mavros/* (fcu_url arg below).
  2. ros_gz_bridge  — gz /camera + /camera_info  ->  /camera/image_raw + /camera/camera_info.
  3. pad spawn      — drops the ArUco landing_pad model into the running world.
  4. aruco_detector — publishes /aruco/pose from the camera stream.
  5. landing_controller — the takeoff -> search -> center -> descend -> land FSM.

Nodes 2-5 are delayed a few seconds so the gz world and MAVROS link are alive
before we bridge topics and spawn the pad.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('landing_controller')
    models_dir = os.path.join(pkg_share, 'models')
    pad_sdf = os.path.join(models_dir, 'landing_pad', 'model.sdf')
    sim_params = os.path.join(pkg_share, 'config', 'landing_params_sim.yaml')

    # --- Launch args ---
    fcu_url = LaunchConfiguration('fcu_url')
    world = LaunchConfiguration('world')
    pad_x = LaunchConfiguration('pad_x')
    pad_y = LaunchConfiguration('pad_y')

    declare_args = [
        DeclareLaunchArgument(
            'fcu_url', default_value='udp://:14540@127.0.0.1:14557',
            description='MAVLink endpoint for MAVROS (PX4 SITL onboard link).'),
        DeclareLaunchArgument(
            'world', default_value='default',
            description='Name of the running gz world to spawn the pad into.'),
        DeclareLaunchArgument(
            'pad_x', default_value='0.0',
            description='Landing pad X in the world (m). Default: under takeoff spot.'),
        DeclareLaunchArgument(
            'pad_y', default_value='0.0',
            description='Landing pad Y in the world (m).'),
    ]

    # Let gz resolve the pad model (and its relative texture URIs) by path.
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=models_dir + os.pathsep + os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
    )

    # 1. MAVROS (run the node directly to stay distro-agnostic).
    mavros = Node(
        package='mavros',
        executable='mavros_node',
        output='screen',
        parameters=[{'fcu_url': fcu_url}],
    )

    # 2. gz -> ROS bridge for the camera. NOTE: with <topic>camera</topic> in the
    #    SDF, gz publishes image on /camera and info on /camera_info. If your gz
    #    version scopes/names them differently, check `gz topic -l` and adjust.
    #    '[' = gz->ros only (we never write these from ROS).
    cam_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[
            '/camera@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        remappings=[
            ('/camera', '/camera/image_raw'),
            ('/camera_info', '/camera/camera_info'),
        ],
    )

    # 3. Spawn the ArUco landing pad into the running world.
    spawn_pad = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-world', world,
            '-file', pad_sdf,
            '-name', 'landing_pad',
            '-x', pad_x, '-y', pad_y, '-z', '0.01',
        ],
    )

    # 4. ArUco detector -> /aruco/pose. Params MUST match the pad marker.
    aruco = Node(
        package='aruco_detector_node',
        executable='aruco_detector_node',
        name='aruco_detector_node',
        output='screen',
        parameters=[{
            'marker_length_m': 0.20,
            'dictionary_name': 'DICT_5X5_250',
            'target_marker_id': 0,
        }],
    )

    # 5. Landing controller FSM (sim params: auto_offboard=true).
    landing = Node(
        package='landing_controller',
        executable='landing_controller_node',
        name='landing_controller_node',
        output='screen',
        parameters=[sim_params],
    )

    # Give the gz world + MAVLink link a few seconds before bridging/spawning.
    delayed = TimerAction(period=5.0, actions=[cam_bridge, spawn_pad, aruco, landing])

    return LaunchDescription(
        declare_args + [set_gz_resource_path, mavros, delayed]
    )
