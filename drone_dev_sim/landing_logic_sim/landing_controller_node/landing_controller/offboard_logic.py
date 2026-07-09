"""
Pure (ROS-free) helpers for the offboard control foundation.

Kept import-free of rclpy/mavros so they can be unit-tested on a laptop without a
sourced workspace — same split as controller.py vs landing_controller_node.py.
"""


def reached_altitude(current_alt, target_alt, tol):
    """True once altitude is within tol of target (and altitude is known).

    An unknown altitude (None) is never "reached".
    """
    if current_alt is None:
        return False
    return abs(current_alt - target_alt) <= tol


def build_position_setpoint(PositionTargetCls, x, y, z, yaw=0.0):
    """
    Build a PositionTarget that commands a position hold at (x, y, z), yaw.

    PositionTargetCls is injected (not imported) so this stays ROS-free and
    testable with a stub. ENU values with FRAME_LOCAL_NED — MAVROS applies the
    ENU->NED transform, so z is altitude (up), matching
    /mavros/local_position/pose. The type_mask ignores velocity + acceleration +
    yaw_rate so PX4 uses position + yaw only.
    """
    msg = PositionTargetCls()
    msg.coordinate_frame = PositionTargetCls.FRAME_LOCAL_NED
    msg.type_mask = (
        PositionTargetCls.IGNORE_VX
        | PositionTargetCls.IGNORE_VY
        | PositionTargetCls.IGNORE_VZ
        | PositionTargetCls.IGNORE_AFX
        | PositionTargetCls.IGNORE_AFY
        | PositionTargetCls.IGNORE_AFZ
        | PositionTargetCls.IGNORE_YAW_RATE
    )
    msg.position.x = float(x)
    msg.position.y = float(y)
    msg.position.z = float(z)
    msg.yaw = float(yaw)
    return msg
