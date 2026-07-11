"""
Rotation math utilities — pure, general-purpose conversions between Rodrigues
rotation vectors and unit quaternions, plus quaternion interpolation/distance.

No dependency on the vision pipeline; reusable anywhere pose/rotation math is
needed. Quaternions are ordered [w, x, y, z].
"""
from __future__ import annotations

import cv2
import numpy as np


def rvec_to_quat(rvec: np.ndarray) -> np.ndarray:
    """Convert a Rodrigues rotation vector to a unit quaternion [w, x, y, z]."""
    rotmat, _ = cv2.Rodrigues(rvec)
    trace = np.trace(rotmat)

    if trace > 0:
        s = np.sqrt(trace + 1.0) * 2
        qw = 0.25 * s
        qx = (rotmat[2, 1] - rotmat[1, 2]) / s
        qy = (rotmat[0, 2] - rotmat[2, 0]) / s
        qz = (rotmat[1, 0] - rotmat[0, 1]) / s
    elif rotmat[0, 0] > rotmat[1, 1] and rotmat[0, 0] > rotmat[2, 2]:
        s = np.sqrt(1.0 + rotmat[0, 0] - rotmat[1, 1] - rotmat[2, 2]) * 2
        qw = (rotmat[2, 1] - rotmat[1, 2]) / s
        qx = 0.25 * s
        qy = (rotmat[0, 1] + rotmat[1, 0]) / s
        qz = (rotmat[0, 2] + rotmat[2, 0]) / s
    elif rotmat[1, 1] > rotmat[2, 2]:
        s = np.sqrt(1.0 + rotmat[1, 1] - rotmat[0, 0] - rotmat[2, 2]) * 2
        qw = (rotmat[0, 2] - rotmat[2, 0]) / s
        qx = (rotmat[0, 1] + rotmat[1, 0]) / s
        qy = 0.25 * s
        qz = (rotmat[1, 2] + rotmat[2, 1]) / s
    else:
        s = np.sqrt(1.0 + rotmat[2, 2] - rotmat[0, 0] - rotmat[1, 1]) * 2
        qw = (rotmat[1, 0] - rotmat[0, 1]) / s
        qx = (rotmat[0, 2] + rotmat[2, 0]) / s
        qy = (rotmat[1, 2] + rotmat[2, 1]) / s
        qz = 0.25 * s

    return np.array([qw, qx, qy, qz])


def quat_to_rvec(quat: np.ndarray) -> np.ndarray:
    """Convert a unit quaternion [w, x, y, z] back to a Rodrigues rotation vector."""
    qw, qx, qy, qz = quat
    rotmat = np.array([
        [1 - 2 * (qy ** 2 + qz ** 2), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx ** 2 + qz ** 2), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx ** 2 + qy ** 2)],
    ])
    rvec, _ = cv2.Rodrigues(rotmat)
    return rvec


def quat_slerp(q0: np.ndarray, q1: np.ndarray, t: float) -> np.ndarray:
    """Spherical linear interpolation between two unit quaternions."""
    dot = np.dot(q0, q1)

    # Take the shorter path around the hypersphere
    if dot < 0:
        q1 = -q1
        dot = -dot

    if dot > 0.9995:
        result = q0 + t * (q1 - q0)
        return result / np.linalg.norm(result)

    theta_0 = np.arccos(np.clip(dot, -1.0, 1.0))
    theta = theta_0 * t

    q_perp = q1 - q0 * dot
    q_perp = q_perp / np.linalg.norm(q_perp)

    return q0 * np.cos(theta) + q_perp * np.sin(theta)


def quat_angle_diff(q0: np.ndarray, q1: np.ndarray) -> float:
    """Angular distance (radians) between two unit quaternions."""
    dot = np.clip(np.abs(np.dot(q0, q1)), -1.0, 1.0)
    return 2 * np.arccos(dot)
