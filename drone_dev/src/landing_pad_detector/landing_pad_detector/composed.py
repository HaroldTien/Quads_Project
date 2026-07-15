#!/usr/bin/env python3
"""
Single-process co-location of the camera driver and the detector.

This is the practical "composable / intra-process" runtime for rclpy: both nodes
live in ONE process and are driven by one MultiThreadedExecutor, so the camera
node hands frames to the detector without a second process or IPC hop.

Note on zero-copy: true zero-copy intra-process transport in ROS 2 is an rclcpp
(C++) feature. Python nodes co-located this way still share a process and avoid
inter-process serialization overhead, but the numpy image is not passed by raw
pointer the way a C++ intra-process pipeline would. For the Jetson Nano this
single-process layout is the meaningful win; if profiling later shows the image
copy is the bottleneck, port the two nodes to C++ components. See README.md.

Prefer the separate-process launch (precision_land.launch.py) while bench-testing
so you can restart, rosbag, and RViz each node independently.
"""
from __future__ import annotations

import rclpy
from rclpy.executors import MultiThreadedExecutor

from csi_camera_publisher.csi_camera_node import CsiCameraNode
from .detector_node import LandingPadDetector


def main(args=None):
    rclpy.init(args=args)
    camera = CsiCameraNode()
    detector = LandingPadDetector()

    executor = MultiThreadedExecutor()
    executor.add_node(camera)
    executor.add_node(detector)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        detector.destroy_node()
        camera.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
