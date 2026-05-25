#!/usr/bin/env python3
"""CSI camera node..."""

import rclpy # The core ROS 2 Python library. Handles initialisation, shutdown, and the executor (the engine that keeps your node alive).
from rclpy.node import Node # The base class for all ROS 2 nodes.
from sensor_msgs.msg import Image, CameraInfo # ROS 2 messages for image and camera info.
from cv_bridge import CvBridge # A library for converting images between ROS 2 and OpenCV.
import cv2 # A library for computer vision.
import numpy as np # A library for numerical computing with Python.
from ament_index_python.packages import get_package_share_directory
import os


# Gstreamer pipline for the CSI camera.

def gst_pipeline(device = '/dev/video0', width=1280, height=800):
    """Build GStreamer pipeline string for OV9281 on Jetson via Tegra VI."""
    return (
        f"nvv4l2camerasrc device={device} ! "
        f"video/x-raw(memory:NVMM), format=UYVY, width={width}, height={height} ! "
        f"nvvidconv ! "
        f"video/x-raw, format=BGRx ! "
        f"videoconvert ! "
        f"video/x-raw, format=BGR ! "
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
        # qos: Quality of Service depth
        self.image_publisher = self.create_publisher(Image, 'camera/image_raw', 10)
        self.camera_info_publisher = self.create_publisher(CameraInfo, 'camera/camera_info', 10)
        self.bridge = CvBridge() # convert images between ROS 2 and OpenCV --  Instantiated once here and reused every frame. Creating it once is efficient — it sets up internal buffers at construction time.

        # open camera
        pipline = gst_pipeline(device=self.device, width=self.width, height=self.height)
        self.cap = cv2.VideoCapture(pipline, cv2.CAP_GSTREAMER)

        if not self.cap.isOpened():
            self.get_logger().error('Failed to open camera -  check pipeline and /dev/video0')
            raise  RuntimeError('Failed to open camera')
        else:        
            self.get_logger().info(f'Camera opened: {self.width}x{self.height} @ {self.fps}fps')

        
        # build the camera info message
        self.camera_info = self.build_camera_info(self.width, self.height)

        # start the timer
        self.timer = self.create_timer(1.0/self.fps, self.timer_callback)
        self.get_logger().info('CSI camera node initialised')

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
            self.get_logger().warn('Failed to grab frame')
            return # skip this frame and try again so no error is raised
            
        #single timestamp shared by two messages
        now = self.get_clock().now().to_msg()

        image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
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
