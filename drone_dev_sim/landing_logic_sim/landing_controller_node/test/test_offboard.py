# test/test_offboard.py — pure-logic tests for the offboard foundation.
# ROS-free: exercises landing_controller.offboard_logic with a stub PositionTarget,
# so it runs on a laptop without a sourced workspace.
from landing_controller.offboard_logic import (
    reached_altitude,
    build_position_setpoint,
)


class _Vec3:
    """Stand-in for geometry_msgs Point (msg.position.x/y/z)."""
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0


class FakePositionTarget:
    """Minimal stub mirroring the mavros_msgs/PositionTarget fields we set.

    Bit values match the real message so type_mask assertions are meaningful.
    """
    FRAME_LOCAL_NED = 1
    IGNORE_PX = 1
    IGNORE_PY = 2
    IGNORE_PZ = 4
    IGNORE_VX = 8
    IGNORE_VY = 16
    IGNORE_VZ = 32
    IGNORE_AFX = 64
    IGNORE_AFY = 128
    IGNORE_AFZ = 256
    IGNORE_YAW = 1024
    IGNORE_YAW_RATE = 2048

    def __init__(self):
        self.coordinate_frame = 0
        self.type_mask = 0
        self.position = _Vec3()
        self.yaw = 0.0


# ---- reached_altitude ----

def test_unknown_altitude_is_never_reached():
    assert reached_altitude(None, 2.0, 0.15) is False


def test_within_tolerance_is_reached():
    assert reached_altitude(1.90, 2.0, 0.15)     # 0.10 error < 0.15
    assert reached_altitude(2.10, 2.0, 0.15)     # symmetric above


def test_outside_tolerance_is_not_reached():
    assert not reached_altitude(1.5, 2.0, 0.15)  # still climbing


# ---- build_position_setpoint ----

def test_setpoint_carries_position_and_yaw():
    sp = build_position_setpoint(FakePositionTarget, 1.0, -2.0, 3.0, yaw=0.5)
    assert (sp.position.x, sp.position.y, sp.position.z) == (1.0, -2.0, 3.0)
    assert sp.yaw == 0.5
    assert sp.coordinate_frame == FakePositionTarget.FRAME_LOCAL_NED


def test_setpoint_mask_ignores_velocity_accel_yawrate_but_uses_position():
    sp = build_position_setpoint(FakePositionTarget, 0.0, 0.0, 2.0)
    m = sp.type_mask
    # velocity, accel, and yaw_rate must be ignored...
    for bit in (
        FakePositionTarget.IGNORE_VX, FakePositionTarget.IGNORE_VY,
        FakePositionTarget.IGNORE_VZ, FakePositionTarget.IGNORE_AFX,
        FakePositionTarget.IGNORE_AFY, FakePositionTarget.IGNORE_AFZ,
        FakePositionTarget.IGNORE_YAW_RATE,
    ):
        assert m & bit, f"expected ignore bit {bit} to be set"
    # ...while the position fields stay ACTIVE (their ignore bits are clear).
    for bit in (
        FakePositionTarget.IGNORE_PX, FakePositionTarget.IGNORE_PY,
        FakePositionTarget.IGNORE_PZ,
    ):
        assert not (m & bit), f"position ignore bit {bit} must be clear"
