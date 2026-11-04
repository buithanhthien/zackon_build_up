"""Odometry feedback following a fixed quintic path; no ROS or detection.

Input yaw is motion-facing (body yaw + pi for reverse). Output u>=0 and
physical yaw rate omega. The ROS adapter alone changes u to linear.x=-u.
The heading PID includes a cross-track heading correction and curvature
feed-forward. Spatial progress, not elapsed wall time, advances reference.
"""

from dataclasses import dataclass
from math import atan2, cos, sin

import numpy as np

from ..planning.quintic import QuinticPath


@dataclass(frozen=True)
class TrackingCommand:
    speed: float
    omega: float
    progress: float
    remaining: float
    cross_track: float
    heading_error: float


def terminal_yaw_rate(heading_error, max_omega=.30, gain=1.5):
    """Stationary terminal correction; translational speed is always zero."""
    return float(np.clip(-gain*heading_error, -max_omega, max_omega))


class TrajectoryTracker:
    def __init__(self, path: QuinticPath, *, heading_kp=2., heading_ki=.02,
                 heading_kd=.02, cross_track_gain=.7, max_cross_track=.12):
        self.path = path
        self.kp, self.ki, self.kd = heading_kp, heading_ki, heading_kd
        self.cross_gain, self.max_cross_track = cross_track_gain, max_cross_track
        self.index = 0
        self.integral = 0.
        self.previous_error = None

    def command(self, pose, dt, *, max_speed=.08, max_omega=.30, blind=False):
        if not np.isfinite([pose['x'], pose['y'], pose['yaw'], dt]).all() or dt <= 0:
            raise ValueError('Invalid odometry pose or control interval')
        # A bounded forward window cannot jump to the endpoint of a close curve.
        limit = int(np.searchsorted(self.path.distance, self.path.distance[self.index]+.20, side='right'))
        candidates = self.path.xy[self.index:max(self.index+1, limit)]
        self.index += int(np.argmin(np.linalg.norm(candidates-[pose['x'], pose['y']], axis=1)))
        point, heading = self.path.xy[self.index], self.path.yaw[self.index]
        dx, dy = pose['x']-point[0], pose['y']-point[1]
        cross = -sin(heading)*dx+cos(heading)*dy
        if abs(cross) > self.max_cross_track:
            raise ValueError(f'Cross-track deviation {cross:.3f}m exceeds {self.max_cross_track}m')
        progress = float(self.path.distance[self.index])
        remaining = max(0., self.path.length-progress)
        speed = min(max_speed, max(.012, .6*remaining))
        if blind:
            speed = min(speed, .025)
        correction = atan2(-self.cross_gain*cross, speed+.04)
        desired_heading = heading+correction
        error = atan2(sin(desired_heading-pose['yaw']), cos(desired_heading-pose['yaw']))
        derivative = 0. if self.previous_error is None else (error-self.previous_error)/dt
        self.integral = float(np.clip(self.integral+error*dt, -.3, .3))
        self.previous_error = error
        omega = (speed*self.path.curvature[self.index] + self.kp*error +
                 self.ki*self.integral + self.kd*derivative)
        # Keep the requested turning curvature when angular velocity saturates.
        scale = min(1., max_omega/max(abs(omega), 1e-9))
        return TrackingCommand(speed*scale, omega*scale, progress, remaining, float(cross), error)
