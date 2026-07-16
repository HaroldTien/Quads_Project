"""One-terminal test launch for the ArUco detector integration.

Brings up the CSI camera publisher, the ArUco detector, and a pop-up viewer
window showing the annotated detection stream (/aruco/debug_image). Inspect
the pose from a second (sourced) terminal with:

    ros2 topic echo /aruco/pose

On a headless setup (SSH without X), disable the window with:
    ros2 launch aruco_detector_node test.launch.py open_viewer:=false

Run with:
    ros2 launch aruco_detector_node test.launch.py
Override marker settings, e.g.:
    ros2 launch aruco_detector_node test.launch.py target_marker_id:=3 marker_length_m:=0.15
Skip the camera (already running it elsewhere):
    ros2 launch aruco_detector_node test.launch.py start_camera:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    target_marker_id = LaunchConfiguration("target_marker_id")
    marker_length_m = LaunchConfiguration("marker_length_m")
    dictionary_name = LaunchConfiguration("dictionary_name")
    start_camera = LaunchConfiguration("start_camera")
    open_viewer = LaunchConfiguration("open_viewer")

    camera_launch = PathJoinSubstitution([
        FindPackageShare("csi_camera_publisher"),
        "launch",
        "camera.launch.py",
    ])

    return LaunchDescription([
        DeclareLaunchArgument("target_marker_id", default_value="0"),
        DeclareLaunchArgument("marker_length_m", default_value="0.20"),
        DeclareLaunchArgument("dictionary_name", default_value="DICT_5X5_50"),
        DeclareLaunchArgument(
            "start_camera", default_value="true",
            description="Set false if the camera publisher is already running."),
        DeclareLaunchArgument(
            "open_viewer", default_value="true",
            description="Pop up the ArUco viewer window on /aruco/debug_image. "
                        "Set false on headless setups (SSH without X)."),

        # Camera publisher (provides /camera/image_raw + /camera/camera_info).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(camera_launch),
            condition=IfCondition(start_camera),
        ),

        # ArUco detector under test.
        Node(
            package="aruco_detector_node",
            executable="aruco_detector_node",
            name="aruco_detector_node",
            output="screen",
            parameters=[{
                "target_marker_id": target_marker_id,
                "marker_length_m": marker_length_m,
                "dictionary_name": dictionary_name,
            }],
        ),

        # Pop-up viewer window showing the annotated detection stream.
        Node(
            package="aruco_detector_node",
            executable="aruco_viewer",
            name="aruco_debug_viewer",
            output="screen",
            condition=IfCondition(open_viewer),
        ),
    ])
