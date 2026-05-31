"""
Pure control logic for precision landing — NO ROS code in here.

Keeping this ROS-free means you can unit-test it on your laptop:
    python3 -c "from controller import LandingController; ..."
without sourcing a workspace or having a drone connected.

Two responsibilities:
  1. camera_to_enu()  — rotate the marker offset from the camera's
                        optical frame into the drone's ENU world frame.
  2. compute_velocity() — a proportional (P) controller that turns
                          position error into a velocity command.
"""

import numpy as np


def camera_to_enu(cam_xyz, yaw=0.0):
    """
    Convert a position offset expressed in the downward camera's optical
    frame into ENU (East-North-Up), which is what MAVROS expects.

    Camera optical frame (your detector's output):
        x = right (in image)
        y = down  (in image)
        z = forward / depth = altitude above the marker

    For a camera pointing straight DOWN, mounted so the top of the image
    faces the drone's nose:
        camera +x (right)   ->  body +y? depends on mount. Start simple:
        We map the *horizontal* image axes onto the horizontal world plane
        and treat camera z (depth) as world Up (negated, see below).

    The mapping below is the common starting point. EXPECT to flip a sign
    or swap an axis once you bench-test — every airframe's mount differs.

        east  =  cam_x          (marker to the right -> move east)
        north = -cam_y          (marker "up" in image -> move north)
        up    = -cam_z          (marker is BELOW, so to close the gap we
                                 descend -> negative up)

    `yaw` lets you rotate the horizontal command if the camera's top edge
    is not aligned with the drone's nose. Leave at 0.0 until basics work.
    """
    cx, cy, cz = float(cam_xyz[0]), float(cam_xyz[1]), float(cam_xyz[2])

    east = cx
    north = -cy
    up = -cz

    # Rotate the horizontal (east/north) vector by yaw if the camera is
    # mounted rotated relative to the airframe nose.
    if yaw != 0.0:
        c, s = np.cos(yaw), np.sin(yaw)
        east, north = c * east - s * north, s * east + c * north

    return np.array([east, north, up], dtype=float)


class LandingController:
    """
    Proportional controller with safety clamps.

    The core idea of a P controller:  velocity = Kp * error
    The bigger the error (marker far from center), the faster we move.
    As we close in, error shrinks, so the command shrinks too — the drone
    naturally decelerates as it arrives. No overshoot logic needed for a
    first pass, though you may add I and D terms later (PID).
    """

    def __init__(self, kp_xy=0.5, kp_z=0.3,
                 max_xy=0.4, max_z=0.3,
                 center_tol=0.10, land_alt=0.20):
        # Gains: how aggressively we react to error, per axis.
        self.kp_xy = kp_xy        # horizontal gain
        self.kp_z = kp_z          # vertical (descent) gain

        # Safety clamps: never command faster than this (m/s).
        # A bad detection must NOT produce a violent lurch.
        self.max_xy = max_xy
        self.max_z = max_z

        # Thresholds for the state machine.
        self.center_tol = center_tol   # within this (m) = "centered"
        self.land_alt = land_alt       # below this altitude (m) = "land"

    def is_centered(self, enu):
        """Horizontal error small enough to begin/continue descending?"""
        horizontal_err = np.hypot(enu[0], enu[1])
        return horizontal_err < self.center_tol

    def compute_velocity(self, enu, descend=False):
        """
        Given the marker offset in ENU, return a (vx, vy, vz) command.

        When descend=False we hold altitude (vz=0) and only correct
        horizontally — this is the CENTER state.
        When descend=True we also command a downward velocity — DESCEND.
        """
        vx = self.kp_xy * enu[0]    # east error  -> east velocity
        vy = self.kp_xy * enu[1]    # north error -> north velocity

        if descend:
            # enu[2] is negative (marker below), so this gives negative vz
            # = downward, which is what we want.
            vz = self.kp_z * enu[2]
        else:
            vz = 0.0

        # Clamp every axis so no single command can exceed safe limits.
        vx = float(np.clip(vx, -self.max_xy, self.max_xy))
        vy = float(np.clip(vy, -self.max_xy, self.max_xy))
        vz = float(np.clip(vz, -self.max_z, self.max_z))

        return vx, vy, vz