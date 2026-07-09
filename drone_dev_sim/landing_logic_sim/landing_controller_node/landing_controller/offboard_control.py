"""
Offboard control foundation (sub-task 1).

Proves basic offboard control BEFORE any marker/landing logic exists. It is a
steady setpoint "pump": a fixed-rate timer that ALWAYS publishes one setpoint,
because PX4 refuses OFFBOARD unless setpoints already stream faster than 2 Hz and
drops OFFBOARD the instant the stream stalls. On top of that pump it runs a tiny
phase machine:

    INIT ── connected + warmed up + OFFBOARD + armed ──▶ TAKEOFF
    TAKEOFF ── altitude within alt_tol of target_alt ──▶ HOLD
    HOLD ── ~/land service called, or land_after_s elapsed ──▶ LAND
    LAND ── AUTO.LAND engaged once ──▶ (PX4 lands + auto-disarms)

This node is deliberately marker-agnostic. The SEARCH/ACQUIRE/CENTER/DESCENT/LAND
FSM is a later increment that reuses this same pump + handshake.

The pure helpers (reached_altitude, build_position_setpoint) are ROS-free so they
can be unit-tested on a laptop without a workspace.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode
from std_srvs.srv import Trigger

from .offboard_logic import reached_altitude, build_position_setpoint


# ---------- ROS node ----------

class OffboardControlNode(Node):
    def __init__(self):
        super().__init__('offboard_control_node')

        # --- Parameters (overridable via offboard_params.yaml) ---
        self.declare_parameter('rate_hz', 20.0)       # setpoint stream rate (>= 2 Hz)
        self.declare_parameter('target_alt', 2.0)     # hover altitude (m, ENU up)
        self.declare_parameter('alt_tol', 0.15)       # "reached altitude" band (m)
        # auto_arm: auto request OFFBOARD + arm. True is convenient for SITL; set
        # FALSE on real hardware until bench-tested with props off.
        self.declare_parameter('auto_arm', True)
        # land_after_s: 0 = wait for the ~/land service; >0 = auto-land after
        # holding that many seconds (hands-free SITL runs).
        self.declare_parameter('land_after_s', 0.0)
        # Seconds of setpoint streaming before we first request OFFBOARD.
        self.declare_parameter('warmup_s', 1.0)
        # How often to (re)request OFFBOARD + arm until they stick.
        self.declare_parameter('request_period_s', 2.0)

        gp = self.get_parameter
        self.rate_hz = float(gp('rate_hz').value)
        self.target_alt = float(gp('target_alt').value)
        self.alt_tol = float(gp('alt_tol').value)
        self.auto_arm = bool(gp('auto_arm').value)
        self.land_after_s = float(gp('land_after_s').value)
        self.warmup_s = float(gp('warmup_s').value)
        self.request_period_s = float(gp('request_period_s').value)

        # --- State ---
        self.phase = 'INIT'
        self.mav_state = State()
        self.current_alt = None       # /mavros/local_position/pose z (ENU up)
        self.home_xy = None           # latched (x, y) from first local pose
        self.ticks = 0                # timer ticks since start (for warmup)
        self.hold_start_time = None   # when we entered HOLD (for land_after_s)
        self.last_request_time = None # throttle for OFFBOARD/arm requests
        self.land_requested = False   # ~/land called or land_after_s elapsed
        self._land_mode_sent = False  # AUTO.LAND SetMode fired once

        # --- QoS: MAVROS publishes state/pose BEST_EFFORT and subscribes
        # setpoints BEST_EFFORT. Match it or silently receive/send nothing. ---
        be_qos = QoSProfile(depth=10)
        be_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        be_qos.history = HistoryPolicy.KEEP_LAST

        # --- Subscribers ---
        self.create_subscription(State, '/mavros/state',
                                 self._state_cb, be_qos)
        self.create_subscription(PoseStamped, '/mavros/local_position/pose',
                                 self._local_pose_cb, be_qos)

        # --- Publisher (the pump) ---
        self.sp_pub = self.create_publisher(
            PositionTarget, '/mavros/setpoint_raw/local', be_qos)

        # --- Service clients ---
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        # --- Service server: command landing ---
        self.create_service(Trigger, '~/land', self._land_srv_cb)

        # --- The heartbeat ---
        self.timer = self.create_timer(1.0 / self.rate_hz, self.control_loop)

        self.get_logger().info(
            f'Offboard control up. rate={self.rate_hz} Hz target_alt={self.target_alt} m '
            f'auto_arm={self.auto_arm} land_after_s={self.land_after_s}')

    # ---------- Callbacks ----------

    def _state_cb(self, msg: State):
        self.mav_state = msg

    def _local_pose_cb(self, msg: PoseStamped):
        self.current_alt = msg.pose.position.z
        if self.home_xy is None:
            self.home_xy = (msg.pose.position.x, msg.pose.position.y)

    def _land_srv_cb(self, request, response):
        self.land_requested = True
        response.success = True
        response.message = 'Landing requested — entering AUTO.LAND.'
        self.get_logger().info('~/land called — will hand off to AUTO.LAND.')
        return response

    # ---------- Helpers ----------

    def _now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _publish_setpoint(self):
        """ALWAYS publish one setpoint: hold (home_xy, target_alt)."""
        x, y = self.home_xy if self.home_xy is not None else (0.0, 0.0)
        msg = build_position_setpoint(PositionTarget, x, y, self.target_alt)
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        self.sp_pub.publish(msg)

    def _maybe_request_offboard_and_arm(self):
        """Throttled, fire-and-forget OFFBOARD + arm until both stick."""
        if not self.auto_arm or not self.mav_state.connected:
            return
        # Warm the stream up before the first request.
        if self.ticks < self.warmup_s * self.rate_hz:
            return
        now = self._now_s()
        if (self.last_request_time is not None
                and now - self.last_request_time < self.request_period_s):
            return
        self.last_request_time = now

        if self.mav_state.mode != 'OFFBOARD' and self.set_mode_client.service_is_ready():
            req = SetMode.Request()
            req.custom_mode = 'OFFBOARD'
            self.set_mode_client.call_async(req)
            self.get_logger().info('Requesting OFFBOARD ...', throttle_duration_sec=2.0)
        if not self.mav_state.armed and self.arming_client.service_is_ready():
            req = CommandBool.Request()
            req.value = True
            self.arming_client.call_async(req)
            self.get_logger().info('Requesting ARM ...', throttle_duration_sec=2.0)

    def _engage_auto_land(self):
        """Hand off to PX4 AUTO.LAND (gentle touchdown + auto-disarm)."""
        if self.mav_state.mode == 'AUTO.LAND':
            return
        if self.set_mode_client.service_is_ready():
            req = SetMode.Request()
            req.custom_mode = 'AUTO.LAND'
            self.set_mode_client.call_async(req)
            self.get_logger().info('Requesting AUTO.LAND ...', throttle_duration_sec=2.0)

    # ---------- The heartbeat ----------

    def control_loop(self):
        """Runs at rate_hz. ALWAYS publishes a setpoint, THEN steps the phase."""
        self.ticks += 1

        # The pump: never goes silent, in every phase (keeps OFFBOARD alive
        # right up until AUTO.LAND takes over).
        self._publish_setpoint()

        armed_offboard = (self.mav_state.armed
                          and self.mav_state.mode == 'OFFBOARD')

        # --- Phase transitions ---
        if self.phase == 'INIT':
            self._maybe_request_offboard_and_arm()
            if armed_offboard:
                self.phase = 'TAKEOFF'
                self.get_logger().info('Armed + OFFBOARD — climbing to target_alt.')

        elif self.phase == 'TAKEOFF':
            self._maybe_request_offboard_and_arm()  # keep re-requesting if it drops
            if reached_altitude(self.current_alt, self.target_alt, self.alt_tol):
                self.phase = 'HOLD'
                self.hold_start_time = self._now_s()
                self.get_logger().info(
                    f'Reached {self.current_alt:.2f} m — holding.')

        elif self.phase == 'HOLD':
            self._maybe_request_offboard_and_arm()
            timed_out = (self.land_after_s > 0.0
                         and self.hold_start_time is not None
                         and self._now_s() - self.hold_start_time >= self.land_after_s)
            if self.land_requested or timed_out:
                self.phase = 'LAND'
                self.get_logger().info('Entering LAND.')

        elif self.phase == 'LAND':
            # Keep pumping setpoints until AUTO.LAND is confirmed so the stream
            # never stalls mid-transition; then PX4 owns the descent.
            self._engage_auto_land()

        self.get_logger().info(
            f'phase={self.phase} armed={self.mav_state.armed} '
            f'mode={self.mav_state.mode} alt={self.current_alt}',
            throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = OffboardControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
