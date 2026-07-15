"""
Single-process co-location of camera + detector (see composed.py) plus the
static camera-mounting transform. Use this for flight once the pipeline is
tuned — one process, no inter-process image serialization.

Detection params are passed to the co-located detector node; the camera node
reads csi_camera_publisher's own camera_params.yaml. MAVROS runs separately.
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    detector_config = PathJoinSubstitution([
        FindPackageShare('landing_pad_detector'), 'config', 'detector_params.yaml'
    ])

    return LaunchDescription([
        Node(
            package='landing_pad_detector',
            executable='composed',
            name='landing_pad_detector',
            output='screen',
            parameters=[detector_config],
        ),
        # Same static mounting transform as precision_land.launch.py — keep the
        # two in sync. REPLACE with your measured camera mounting.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_optical_tf',
            output='screen',
            arguments=[
                '--x', '0.10', '--y', '0.0', '--z', '-0.05',
                '--qx', '0.0', '--qy', '0.70710678', '--qz', '-0.70710678', '--qw', '0.0',
                '--frame-id', 'base_link', '--child-frame-id', 'camera_optical',
            ],
        ),
    ])
