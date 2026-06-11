"""
Launch the landing controller with parameters from landing_params.yaml.

Run with:
    ros2 launch landing_controller landing.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('landing_controller'),
        'config',
        'landing_params.yaml')

    return LaunchDescription([
        Node(
            package='landing_controller',
            executable='landing_controller_node',
            name='landing_controller_node',
            output='screen',
            parameters=[config],
        ),
    ])