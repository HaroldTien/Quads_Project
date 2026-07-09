"""
Launch the offboard control foundation with parameters from offboard_params.yaml.

Run with:
    ros2 launch landing_controller offboard.launch.py

Assumes PX4 SITL and MAVROS are already up (see the plan's verification steps).
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('landing_controller'),
        'config',
        'offboard_params.yaml')

    return LaunchDescription([
        Node(
            package='landing_controller',
            executable='offboard_control_node',
            name='offboard_control_node',
            output='screen',
            parameters=[config],
        ),
    ])
