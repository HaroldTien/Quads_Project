#!/usr/bin/env python3
"""Pop-up OpenCV window showing the ArUco detector's annotated debug stream.

Subscribes to /aruco/debug_image (published by aruco_detector_publisher) and
displays it with cv2.imshow. Needs a display — run on the desktop or with X
forwarding; launch with open_viewer:=false to skip it on headless setups.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class ArucoDebugViewer(Node):
    def __init__(self):
        super().__init__("aruco_debug_viewer")

        self.declare_parameter("image_topic", "/aruco/debug_image")
        self.declare_parameter("window_name", "ArUco Viewer")

        image_topic = str(self.get_parameter("image_topic").value)
        self.window_name = str(self.get_parameter("window_name").value)

        self.bridge = CvBridge()
        self.latest_frame = None
        self.window_created = False

        # Depth 1: only the newest frame matters for display; a deeper queue
        # just buffers stale frames when the GUI briefly stalls.
        self.image_sub = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            ),
        )

        # GUI work happens on this timer, not in the subscriber callback:
        # imshow/waitKey must be pumped regularly from one place, and a slow
        # display must never back-pressure the image subscription.
        self.display_timer = self.create_timer(1.0 / 30.0, self.display_callback)

        self.get_logger().info(f"ArUco viewer waiting for frames on {image_topic}")

    def image_callback(self, msg: Image) -> None:
        # Only keep the newest frame; the timer decides when to draw it.
        self.latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def display_callback(self) -> None:
        if self.latest_frame is None:
            return

        if not self.window_created:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            self.window_created = True

        cv2.imshow(self.window_name, self.latest_frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            self.get_logger().info("'q' pressed, shutting down viewer")
            raise SystemExit

        # Closing the window (X button) shuts the node down instead of leaving a
        # zombie subscriber running. Checked after imshow/waitKey have pumped the
        # GUI, since a window is not visible until then.
        #
        # Only 0 means "window exists but is no longer visible". Some builds --
        # including the GTK3 OpenCV on this Jetson -- never implement
        # WND_PROP_VISIBLE and return -1 forever, which would otherwise read as a
        # closed window on the very first frame. Treat -1 as "unknown" and let 'q'
        # or Ctrl-C handle shutdown there instead.
        if cv2.getWindowProperty(self.window_name, cv2.WND_PROP_VISIBLE) == 0:
            self.get_logger().info("Viewer window closed, shutting down")
            raise SystemExit


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArucoDebugViewer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
