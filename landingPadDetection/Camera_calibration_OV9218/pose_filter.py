"""
Temporal pose smoothing with outlier rejection.

PoseFilter exponentially smooths translation and slerps orientation, rejecting
measurements that jump too far as outliers — with a re-lock after sustained
rejection so it doesn't stick on a stale estimate when the marker truly moves.
"""
from __future__ import annotations

import numpy as np

from rotation_utils import quat_angle_diff, quat_slerp, quat_to_rvec, rvec_to_quat

# After this many consecutive rejections, assume the marker genuinely moved and
# reset the filter to the latest measurement so it re-locks instead of sticking.
DEFAULT_MAX_CONSECUTIVE_REJECTS = 5


class PoseFilter:
    def __init__(self, alpha=0.3, outlier_threshold_pos=0.15, outlier_threshold_rot=0.5,
                 max_consecutive_rejects=DEFAULT_MAX_CONSECUTIVE_REJECTS):
        self.alpha = alpha
        self.outlier_threshold_pos = outlier_threshold_pos
        self.outlier_threshold_rot = outlier_threshold_rot
        self.max_consecutive_rejects = max_consecutive_rejects
        self.consecutive_rejects = 0
        self.smooth_tvec: np.ndarray | None = None
        self.smooth_quat: np.ndarray | None = None

    def _seed(self, tvec, rvec, quat):
        """(Re)initialize the filter state from a measurement and accept it."""
        self.smooth_tvec = tvec.copy()
        self.smooth_quat = quat
        self.consecutive_rejects = 0
        return tvec.copy(), rvec.copy(), True

    def update(self, tvec: np.ndarray, rvec: np.ndarray):
        quat = rvec_to_quat(rvec)

        if self.smooth_tvec is None or self.smooth_quat is None:
            return self._seed(tvec, rvec, quat)

        pos_delta = np.linalg.norm(tvec - self.smooth_tvec)
        rot_delta = quat_angle_diff(self.smooth_quat, quat)

        is_outlier = (
            pos_delta > self.outlier_threshold_pos
            or rot_delta > self.outlier_threshold_rot
        )

        if is_outlier:
            self.consecutive_rejects += 1
            # Sustained rejection means the marker genuinely moved (or we were stuck
            # on a stale estimate) — re-lock to the latest measurement.
            if self.consecutive_rejects >= self.max_consecutive_rejects:
                return self._seed(tvec, rvec, quat)
            return self.smooth_tvec.copy(), quat_to_rvec(self.smooth_quat), False

        self.consecutive_rejects = 0
        self.smooth_tvec = (1 - self.alpha) * self.smooth_tvec + self.alpha * tvec
        self.smooth_quat = quat_slerp(self.smooth_quat, quat, self.alpha)

        return self.smooth_tvec.copy(), quat_to_rvec(self.smooth_quat), True
