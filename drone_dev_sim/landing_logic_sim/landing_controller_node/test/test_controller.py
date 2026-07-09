# test/test_controller.py
from drone_dev_sim.landing_logic_sim.landing_controller_node.landing_controller.controller import LandingController, camera_to_enu
import numpy as np

def test_marker_to_the_right_moves_east():
    enu = camera_to_enu([0.3, 0.0, 0.9])   # 30cm right
    assert enu[0] > 0                       # east positive
    assert abs(enu[1]) < 1e-9               # no north
    assert enu[2] < 0                       # up negative (descend)

def test_centered_marker_is_detected():
    ctrl = LandingController(center_tol=0.10)
    assert ctrl.is_centered(camera_to_enu([0.02, 0.01, 0.5]))
    assert not ctrl.is_centered(camera_to_enu([0.3, 0.0, 0.5]))

def test_velocity_is_clamped():
    ctrl = LandingController(kp_xy=10.0, max_xy=0.4)   # huge gain
    vx, vy, vz = ctrl.compute_velocity(camera_to_enu([5.0, 0.0, 1.0]))
    assert abs(vx) <= 0.4                    # clamp held