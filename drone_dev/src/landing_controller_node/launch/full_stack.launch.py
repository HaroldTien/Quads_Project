from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    landing_params = os.path.join(
        get_package_share_directory('landing_controller'),
        'config', 'landing_params.yaml'
    )
    camera_params = os.path.join(
        get_package_share_directory('csi_camera_publisher'),
        'config', 'camera_params.yaml'
    )

    return LaunchDescription([

        Node(
            package='csi_camera_publisher',
            executable='csi_camera_node',
            name='csi_camera_node',
            output='screen',
            parameters=[camera_params],
        ),

        Node(
            package='aruco_detector_node',
            executable='aruco_detector_node',
            name='aruco_detector_node',
            output='screen',
        ),

        Node(
            package='landing_controller',
            executable='landing_controller_node',
            name='landing_controller_node',
            parameters=[landing_params],
            output='screen',
        ),
    ])