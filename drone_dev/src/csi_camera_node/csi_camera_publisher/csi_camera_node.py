#!/usr/bin/env python3
"""CSI camera node..."""

import rclpy # The core ROS 2 Python library. Handles initialisation, shutdown, and the executor (the engine that keeps your node alive).
from rclpy.node import Node # The base class for all ROS 2 nodes.
from rclpy.qos import qos_profile_sensor_data # Best-effort QoS preset suited to high-rate sensor streams.
from sensor_msgs.msg import Image, CameraInfo # ROS 2 messages for image and camera info.
from cv_bridge import CvBridge # A library for converting images between ROS 2 and OpenCV.
import cv2 # A library for computer vision.
import numpy as np # A library for numerical computing with Python.
from ament_index_python.packages import get_package_share_directory
import os


# Gstreamer pipline for the CSI camera.

def gst_pipeline(device = '/dev/video0', width=1280, height=800, fps=30):
    """Build GStreamer pipeline string for the mono CSI sensor on Jetson.
    """
    return (
        f"v4l2src device={device} ! "
        f"video/x-raw, format=GRAY8, width={width}, height={height}, framerate={fps}/1 ! "
        f"appsink drop=1 max-buffers=1 sync=false"
        # Only keep 1 frame queued at a time. Prevents memory buildup if the node briefly lags.
        # Don't try to sync to a GStreamer clock. Just deliver frames as fast as the camera produces them.
    )


class CsiCameraNode(Node):
    def __init__(self):
        super().__init__('csi_camera_node')

        share_dir = get_package_share_directory('csi_camera_publisher')

        # define parameters 
        self.declare_parameter('device', '/dev/video0')
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 800)
        self.declare_parameter('fps', 30)
        self.declare_parameter(
            'camera_matrix_path',
            os.path.join(share_dir, 'data', 'camera_matrix.npy')
        )
        self.declare_parameter(
            'dist_coeffs_path',
            os.path.join(share_dir, 'data', 'dist_coeffs.npy')
        )

        # get parameters
        self.device = self.get_parameter('device').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.fps = self.get_parameter('fps').value

        self.camera_matrix_path = self.get_parameter('camera_matrix_path').value
        self.dist_coeffs_path = self.get_parameter('dist_coeffs_path').value

        # load camera calibration files
        self.camera_matrix = np.load(self.camera_matrix_path)
        self.dist_coeffs = np.load(self.dist_coeffs_path)
        self.get_logger().info(f'Camera matrix: {self.camera_matrix}')

        # create publisher

        #self.create_publisher(msg_type, topic, qos) 
        
        self.image_publisher = self.create_publisher(Image, 'camera/image_raw', qos_profile_sensor_data)
        self.camera_info_publisher = self.create_publisher(CameraInfo, 'camera/camera_info', qos_profile_sensor_data)

        # convert images between ROS 2 and OpenCV
        self.bridge = CvBridge()

        # Stall watchdog: the Jetson VI/CSI capture occasionally hits a
        # "corr_err: discarding frame" and the V4L2 stream hangs for good.
        # After this many consecutive failed reads, tear down and reopen the
        # GStreamer pipeline instead of warning forever on a dead stream.
        self.declare_parameter('reopen_after_failures', 3)
        self.reopen_after_failures = self.get_parameter('reopen_after_failures').value
        self.consecutive_failures = 0

        # open camera
        self.cap = None
        if not self._open_camera():
            self.get_logger().error('Failed to open camera -  check pipeline and /dev/video0')
            raise  RuntimeError('Failed to open camera')

        
        # build the camera info message
        self.camera_info = self.build_camera_info(self.width, self.height)

        # start the timer
        self.timer = self.create_timer(1.0/self.fps, self.timer_callback)
        self.get_logger().info('CSI camera node initialised')

    def _open_camera(self) -> bool:
        """(Re)open the GStreamer capture. Returns True when the camera is up."""
        if self.cap is not None:
            self.cap.release()

        pipline = gst_pipeline(device=self.device, width=self.width, height=self.height, fps=self.fps)
        self.cap = cv2.VideoCapture(pipline, cv2.CAP_GSTREAMER)

        # Keep OpenCV from promoting the single-channel GRAY8 buffer to 3-channel BGR.
        self.cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)

        # Fail a stalled read after 2s instead of OpenCV's default 30s, so the
        # watchdog can reopen the pipeline quickly (and the executor is not
        # blocked for 30s per dead read).
        self.cap.set(cv2.CAP_PROP_READ_TIMEOUT_MSEC, 2000)

        if not self.cap.isOpened():
            return False

        self.consecutive_failures = 0
        self.get_logger().info(f'Camera opened: {self.width}x{self.height} @ {self.fps}fps')
        return True

    def build_camera_info(self, width, height):
        # build camera info message from loaded calibration matrix
        msg = CameraInfo()
        msg.width = width
        msg.height = height
        msg.header.frame_id = 'camera_optical'


        # K: 3x3 camera matrix → flattened to 9 values
        msg.k = self.camera_matrix.flatten().tolist()

        # D: distortion coefficients → exactly 5 values
        D = self.dist_coeffs.flatten().tolist()
        while( len(D) < 5):
            D.append(0.0)
        msg.d = D[:5]

        msg.distortion_model = 'plumb_bob'

        # R: rectification matrix → identity for single camera
        msg.r = [1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0]

        # P: projection matrix → derived from K, 4th column zeros for mono                        
        K = self.camera_matrix
        msg.p =[
            K[0,0], 0.0, K[0,2], 0.0,
            0.0, K[1,1], K[1,2], 0.0,
            0.0, 0.0, 1.0, 0.0
        ]
         
        return msg
    
    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.consecutive_failures += 1
            self.get_logger().warn(
                f'Failed to grab frame ({self.consecutive_failures} in a row)')
            if self.consecutive_failures >= self.reopen_after_failures:
                self.get_logger().warn('Camera stream stalled - reopening pipeline')
                if not self._open_camera():
                    self.get_logger().error('Camera reopen failed, will retry')
            return

        self.consecutive_failures = 0

        #single timestamp shared by two messages
        now = self.get_clock().now().to_msg()

        image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='mono8')
        image_msg.header.stamp = now
        image_msg.header.frame_id = 'camera_optical'
        self.image_publisher.publish(image_msg)

        self.camera_info.header.stamp = now
        self.camera_info_publisher.publish(self.camera_info)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CsiCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down — Ctrl+C received')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
