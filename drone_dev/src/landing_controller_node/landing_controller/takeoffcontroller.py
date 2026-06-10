# takeoff.py — pure takeoff logic, no ROS imports

class TakeoffController:
    """Commands a vertical climb to a target altitude.

    ROS-free so it can be unit-tested on a laptop. Knows nothing
    about markers, MAVROS, or the landing controller.
    """

    def __init__(self, target_alt, climb_vel, alt_tol):
        self.target_alt = target_alt
        self.climb_vel = climb_vel
        self.alt_tol = alt_tol

    def is_complete(self, current_alt):
        """True once we're within tolerance of the target."""
        return current_alt >= (self.target_alt - self.alt_tol)

    def compute_velocity(self, current_alt):
        """Return (vx, vy, vz) for this tick.

        Pure vertical climb. Returns zero climb once complete so a
        late call can't overshoot.
        """
        if self.is_complete(current_alt):
            return (0.0, 0.0, 0.0)
        return (0.0, 0.0, self.climb_vel)