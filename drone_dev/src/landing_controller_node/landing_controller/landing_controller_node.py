"""
Landing controller ROS 2 node.

This file is the "plumbing": it connects ROS topics/services to the pure
math in controller.py. It does four things:

  1. SUBSCRIBE to /aruco/pose  (where is the marker?)
  2. Run a STATE MACHINE        (search -> center -> descend -> land)
  3. PUBLISH velocity setpoints to MAVROS at a steady rate (>= 2 Hz)
  4. Handle the OFFBOARD handshake (stream first, THEN switch mode + arm)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

from .controller import LandingController, camera_to_enu


class LandingControllerNode(Node):
    def __init__(self):
        super().__init__('landing_controller_node')

        # --- Parameters (overridable from landing_params.yaml) ---
        # Declare tunable parameters for the landing controller node.
        # These can be overridden via landing_params.yaml at launch time.
        self.declare_parameter('kp_xy', 0.5)             # Proportional gain for horizontal control
        self.declare_parameter('kp_z', 0.3)              # Proportional gain for descent control
        self.declare_parameter('max_xy', 0.4)            # Maximum allowed horizontal velocity (m/s)
        self.declare_parameter('max_z', 0.3)             # Maximum allowed vertical velocity (m/s)
        self.declare_parameter('center_tol', 0.10)       # Tolerance for being "centered" above the marker (m)
        self.declare_parameter('land_alt', 0.20)         # Altitude to switch to AUTO.LAND mode (m)
        self.declare_parameter('pose_timeout', 0.5)      # Marker considered "lost" if no pose for this duration (s)
        self.declare_parameter('rate_hz', 20.0)          # Rate at which velocity setpoints are streamed (Hz)
        self.declare_parameter('auto_offboard', False)   # If true, auto-arm and switch to OFFBOARD mode (CAUTION)

        gp = self.get_parameter
        self.controller = LandingController(
            kp_xy=gp('kp_xy').value, kp_z=gp('kp_z').value,
            max_xy=gp('max_xy').value, max_z=gp('max_z').value,
            center_tol=gp('center_tol').value, land_alt=gp('land_alt').value,
        )
        self.pose_timeout = gp('pose_timeout').value
        self.rate_hz = gp('rate_hz').value
        self.auto_offboard = gp('auto_offboard').value

        # --- State ---
        self.state = 'SEARCH'
        self.latest_enu = None
        self.last_pose_time = None
        self.current_alt = None          # altitude above marker (cam z)
        self.mav_state = State()         # latest /mavros/state
        self._offboard_requested = False

        # --- QoS ---
        # MAVROS publishes its state topic (/mavros/state) with BEST_EFFORT reliability,
        # which means the subscriber must also use BEST_EFFORT to receive messages,
        # otherwise you will not receive any state updates—a common silent failure.
        # Setting 'history' to KEEP_LAST and 'depth' to 10 is standard and sufficient for this topic.
        state_qos = QoSProfile(depth=10)
        state_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        state_qos.history = HistoryPolicy.KEEP_LAST

        # --- Subscribers ---
        self.create_subscription(
            PoseStamped, '/aruco/pose', self.pose_callback, 10)
        self.create_subscription(
            State, '/mavros/state', self.state_callback, state_qos)

        # Publisher for velocity setpoints to MAVROS (/mavros/setpoint_velocity/cmd_vel)
        self.vel_pub = self.create_publisher(
            TwistStamped, '/mavros/setpoint_velocity/cmd_vel', 10)
     

        # Call MAVROS arming service (True=arm, False=disarm).
        self.arming_client = self.create_client(
            CommandBool, '/mavros/cmd/arming')
        # Call MAVROS mode service (e.g., OFFBOARD or AUTO.LAND).
        self.set_mode_client = self.create_client(
            SetMode, '/mavros/set_mode')

        # --- The control loop: fixed-rate timer (THE heartbeat) ---
        # Create a timer that calls the control_loop method at the specified rate.
        self.timer = self.create_timer(1.0 / self.rate_hz, self.control_loop)

        self.get_logger().info(
            f'Landing controller up. Streaming at {self.rate_hz} Hz. '
            f'auto_offboard={self.auto_offboard}')

    # ---------- Callbacks ----------

    def pose_callback(self, msg: PoseStamped):
        """Marker seen — convert to ENU and stash it with a timestamp."""
        cam_xyz = (msg.pose.position.x,
                   msg.pose.position.y,
                   msg.pose.position.z)
        self.current_alt = msg.pose.position.z
        self.latest_enu = camera_to_enu(cam_xyz)
        self.last_pose_time = self.get_clock().now()

    def state_callback(self, msg: State):
        self.mav_state = msg

    # ---------- Helpers ----------

    def _marker_fresh(self):
        """Have we seen the marker recently enough to trust it?"""
        if self.last_pose_time is None:
            return False
        age = (self.get_clock().now() - self.last_pose_time).nanoseconds * 1e-9
        return age < self.pose_timeout

    def _publish_velocity(self, vx, vy, vz):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        msg.twist.linear.z = vz
        self.vel_pub.publish(msg)

    def _request_offboard_and_arm(self):
        """Fire-and-forget async requests. Only call once setpoints flow."""
        if self.set_mode_client.service_is_ready():
            req = SetMode.Request()
            req.custom_mode = 'OFFBOARD'
            self.set_mode_client.call_async(req)
        if self.arming_client.service_is_ready():
            req = CommandBool.Request()
            req.value = True
            self.arming_client.call_async(req)

    # ---------- The heartbeat ----------

    def control_loop(self):
        """
        Runs at rate_hz no matter what. ALWAYS publishes a setpoint so the
        OFFBOARD stream never goes silent. Decides what to publish based on
        the state machine.
        """
        fresh = self._marker_fresh()

        # --- State transitions ---
        if not fresh:
            # Lost the marker (or never had it). Hold position safely.
            if self.state in ('CENTER', 'DESCEND'):
                self.state = 'HOLD'
            elif self.state != 'HOLD':
                self.state = 'SEARCH'
        else:
            # Check if the marker is centered within the tolerance.
            centered = self.controller.is_centered(self.latest_enu)
            # Check if the current altitude is below the landing altitude.
            low_enough = (self.current_alt is not None
                          and self.current_alt < self.controller.land_alt)

            if self.state in ('SEARCH', 'HOLD'):
                self.state = 'CENTER'
            if self.state == 'CENTER' and centered:
                self.state = 'DESCEND'
            if self.state == 'DESCEND' and not centered:
                self.state = 'CENTER'        # drifted — re-center first
            if self.state == 'DESCEND' and low_enough:
                self.state = 'LAND'

        # --- State actions: every branch publishes SOMETHING ---
        if self.state in ('SEARCH', 'HOLD'):
            self._publish_velocity(0.0, 0.0, 0.0)        # hover in place
        elif self.state == 'CENTER':
            vx, vy, vz = self.controller.compute_velocity(
                self.latest_enu, descend=False)
            self._publish_velocity(vx, vy, vz)
        elif self.state == 'DESCEND':
            vx, vy, vz = self.controller.compute_velocity(
                self.latest_enu, descend=True)
            self._publish_velocity(vx, vy, vz)
        elif self.state == 'LAND':
            self._publish_velocity(0.0, 0.0, 0.0)
            self._do_land()

        # --- OFFBOARD handshake: only after the stream is alive ---
        if (self.auto_offboard and not self._offboard_requested
                and self.mav_state.connected):
            self._request_offboard_and_arm()
            self._offboard_requested = True

        self.get_logger().info(f'state={self.state} fresh={fresh}',
                               throttle_duration_sec=1.0)

    def _do_land(self):
        """Hand off to PX4's AUTO.LAND, which does a gentle touchdown."""
        if self.set_mode_client.service_is_ready():
            req = SetMode.Request()
            req.custom_mode = 'AUTO.LAND'
            self.set_mode_client.call_async(req)


def main(args=None):
    rclpy.init(args=args)
    node = LandingControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()