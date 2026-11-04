"""Geometric quintic in the final approach frame (all distances in metres).

Contract: start/goal are motion-facing SE(2) poses in the same fixed frame.
For reverse driving their yaw is body yaw + pi. No ROS messages or sensor
state enter this module. Boundary conditions constrain y, dy/dx, d2y/dx2
at both ends; this is not two independent zero-velocity time polynomials.
"""

from dataclasses import dataclass
from math import atan2, ceil, cos, sin, tan

import numpy as np


class PlanningError(ValueError):
    """No safe monotone reverse approach in the configured planning envelope."""


@dataclass(frozen=True)
class QuinticPath:
    xy: np.ndarray
    yaw: np.ndarray
    curvature: np.ndarray
    distance: np.ndarray
    coefficients: np.ndarray
    goal: dict

    @property
    def length(self):
        return float(self.distance[-1])


def plan_quintic(start, goal, *, resolution=.005, max_lateral=.65,
                 max_heading=.8, max_curvature=8., max_distance=2.5):
    """Join start to goal with zero curvature at each endpoint.

    x increases monotonically towards zero in the goal frame. Reversals,
    loops, too-short manoeuvres, excessive curvature and nonfinite inputs
    are rejected instead of silently returning an infeasible reference.
    """
    values = [pose[key] for pose in (start, goal) for key in ('x', 'y', 'yaw')]
    if not np.isfinite(values).all() or min(resolution, max_curvature) <= 0:
        raise PlanningError('Nonfinite pose or invalid planning limits')
    c, s = cos(goal['yaw']), sin(goal['yaw'])
    delta = np.array([start['x']-goal['x'], start['y']-goal['y']])
    x0, y0 = np.array([[c, s], [-s, c]]) @ delta
    heading = atan2(sin(start['yaw']-goal['yaw']), cos(start['yaw']-goal['yaw']))
    length_x = -float(x0)
    if not .04 <= length_x <= max_distance:
        raise PlanningError(f'Approach distance {length_x:.3f}m outside [0.04, {max_distance}]')
    if abs(y0) > max_lateral or abs(heading) > max_heading:
        raise PlanningError(f'Start outside planning envelope: y={y0:.3f}m, heading={heading:.3f}rad')
    # Normalized parameter q=(x-x0)/length_x. Each coefficient has units m.
    a0, a1, a2 = float(y0), length_x*tan(heading), 0.
    a3, a4, a5 = np.linalg.solve(
        [[1., 1., 1.], [3., 4., 5.], [6., 12., 20.]],
        [-a0-a1-a2, -a1-2*a2, -2*a2])
    coefficients = np.array([a0, a1, a2, a3, a4, a5])
    q = np.linspace(0., 1., max(81, ceil(length_x/resolution)+1))
    y = np.polynomial.polynomial.polyval(q, coefficients)
    slope = np.polynomial.polynomial.polyval(q, np.polynomial.polynomial.polyder(coefficients))/length_x
    second = np.polynomial.polynomial.polyval(q, np.polynomial.polynomial.polyder(coefficients, 2))/length_x**2
    curvature = second/(1+slope*slope)**1.5
    if np.max(np.abs(curvature)) > max_curvature:
        raise PlanningError(f'Curvature {np.max(np.abs(curvature)):.3f}/m exceeds {max_curvature}/m')
    local = np.column_stack((x0+length_x*q, y))
    xy = local @ np.array([[c, s], [-s, c]]) + [goal['x'], goal['y']]
    yaw = np.arctan(slope)+goal['yaw']
    distance = np.r_[0., np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))]
    return QuinticPath(xy, yaw, curvature, distance, coefficients, dict(goal))


def swept_collision(path, points, *, half_length=.30, half_width=.25,
                    margin=0., start_distance=0., details=None):
    """Check sampled rectangular footprints, not just the path centreline.

    points must be scan obstacle endpoints in the SAME fixed frame as path.
    Unknown/occluded space is not proven free by this 2-D check. Caller owns
    sensor freshness, ego-return filtering and emergency stopping.
    """
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    points = points[np.isfinite(points).all(axis=1)]
    if not len(points):
        return False
    for xy, yaw, along in zip(path.xy, path.yaw, path.distance):
        if along < start_distance-.01:
            continue
        relative = points-xy
        c, s = cos(yaw), sin(yaw)
        longitudinal = relative[:, 0]*c+relative[:, 1]*s
        lateral = -relative[:, 0]*s+relative[:, 1]*c
        inside = ((np.abs(longitudinal) < half_length+margin) &
                  (np.abs(lateral) < half_width+margin))
        if np.any(inside):
            if details is not None:
                indices = np.flatnonzero(inside)
                details.update(count=int(len(indices)),
                    points_odom=points[indices[:12]].tolist(),
                    points_at_collision_pose=np.column_stack((longitudinal[inside], lateral[inside]))[:12].tolist(),
                    collision_pose=dict(x=float(xy[0]), y=float(xy[1]), yaw=float(yaw)),
                    path_distance=float(along),
                    footprint=dict(half_length=half_length, half_width=half_width, margin=margin))
            return True
    return False


def rotation_collision(pose, delta_yaw, points, **limits):
    """Include intermediate body orientations during a final stationary turn."""
    count = max(3, ceil(abs(delta_yaw)/.02)+1)
    angles = np.linspace(pose['yaw'], pose['yaw']+delta_yaw, count)
    path = QuinticPath(np.tile([pose['x'], pose['y']], (count, 1)), angles,
                       np.zeros(count), np.zeros(count), np.zeros(6), dict(pose))
    return swept_collision(path, points, **limits)
