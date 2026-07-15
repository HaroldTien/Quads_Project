"""
Bring up camera + detector as two SEPARATE processes (recommended while
bench-testing: each node can be restarted, rosbag'd, and viewed in RViz alone).

Also publishes the static base_link -> camera_optical transform the detector
needs to convert the pad pose into the drone body frame. EDIT THOSE NUMBERS to
match how the OV9281 is physically mounted — the placeholder below is a
belly-mounted, straight-down camera and is almost certainly not your exact rig.

MAVROS is intentionally not started here — run your existing mavros bringup
separately (it owns the Pixhawk serial link). The detector publishes to
/mavros/landing_target/pose, which the MAVROS landing_target plugin consumes.
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    camera_config = PathJoinSubstitution([
        FindPackageShare('csi_camera_publisher'), 'config', 'camera_params.yaml'
    ])
    detector_config = PathJoinSubstitution([
        FindPackageShare('landing_pad_detector'), 'config', 'detector_params.yaml'
    ])

    return LaunchDescription([
        Node(
            package='csi_camera_publisher',
            executable='csi_camera_node',
            name='csi_camera_node',
            output='screen',
            parameters=[camera_config],
        ),
        Node(
            package='landing_pad_detector',
            executable='detector_node',
            name='landing_pad_detector',
            output='screen',
            parameters=[detector_config],
        ),
        # Static camera-mounting transform: base_link -> camera_optical.
        # args: x y z  qx qy qz qw  parent child
        # Placeholder: camera 0.10 m forward, 0.05 m below the body origin, optical
        # axis pointing straight down (REP-103 optical z = "forward" = world-down).
        # REPLACE the translation and rotation with your measured mounting.
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
