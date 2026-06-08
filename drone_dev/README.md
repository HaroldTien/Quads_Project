#### Quick Start (generic node startup)

Most ROS 2 nodes in this repo start the same way.

In every new terminal, run the shared setup first:

```bash
cd ~/Workspace/Quads_Project/drone_dev
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Then run the node-specific command:

| Node | Command |
|------|---------|
| CSI camera publisher | `ros2 launch csi_camera_publisher camera.launch.py` |
| ArUco detector | `ros2 run aruco_detector_node aruco_detector_node --ros-args -p target_marker_id:=0 -p marker_length_m:=0.20` |
| Landing controller | `ros2 launch landing_controller landing.launch.py` |
| MAVROS (Pixhawk over serial) | `ros2 launch mavros px4.launch fcu_url:=/dev/ttyTHS1:921600` |
| Full stack (camera + ArUco + landing) | `ros2 launch landing_controller full_stack.launch.py` |
| Pose check (optional) | `ros2 topic echo /aruco/pose` |

Notes:

- Build once after code/package changes: `colcon build --symlink-install`.
- If a command says `package ... not found`, your terminal is not sourced to this workspace yet.


#### Basic ROS 2 Commands (Cheat Sheet)

Use these to inspect nodes, topics, and messages quickly.

```bash
# Package / executable discovery
ros2 pkg list
ros2 pkg executables csi_camera_publisher
ros2 pkg executables aruco_detector_node

# Node status
ros2 node list
ros2 node info /csi_camera_node

# Topic status
ros2 topic list
ros2 topic info /camera/image_raw
ros2 topic info /camera/camera_info
ros2 topic info /aruco/pose

# Observe topic data
ros2 topic echo /camera/camera_info
ros2 topic echo /aruco/pose
ros2 topic hz /aruco/pose
ros2 topic echo /mavros/state --once

# Message type / interface
ros2 topic type /aruco/pose
ros2 interface show geometry_msgs/msg/PoseStamped

```

