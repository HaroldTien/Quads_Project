from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare('csi_camera_publisher'),
        'config',
        'camera_params.yaml'
    ])

    return LaunchDescription([
        Node(
            package='csi_camera_publisher',
            executable='csi_camera_node',
            name='csi_camera_node',
            output='screen',
            parameters=[config],
        )
    ])