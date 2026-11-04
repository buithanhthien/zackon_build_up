"""Perception only: fit a 250 mm front and 100 mm / 50 mm V recess."""

from dataclasses import dataclass
from math import atan2, cos, sin

import numpy as np
from scipy.optimize import least_squares
from scipy.signal import find_peaks
from scipy.spatial import cKDTree


FACE_HALF_WIDTH = 0.125
MAX_FIT_RMS = 0.020


@dataclass(frozen=True)
class ProfilePose:
    x: float
    y: float
    yaw: float
    rms: float
    full_shape: bool
    points: tuple


def wrap(angle):
    return atan2(sin(angle), cos(angle))


def scan_points(scan, transform, rear=True):
    """Convert a raw scan to a virtual rear-facing frame at base_link's origin."""
    q = transform.rotation
    yaw = atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    ranges = np.asarray(scan.ranges)
    valid = np.isfinite(ranges) & (ranges >= scan.range_min) & (ranges <= 3.0)
    angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment + yaw
    points = np.column_stack((transform.translation.x + ranges[valid] * np.cos(angles[valid]),
                              transform.translation.y + ranges[valid] * np.sin(angles[valid])))
    if rear:
        points = -points
    return points


def _downsample(points, maximum=320):
    """Keep angular coverage without rejecting a high-resolution hardware scan."""
    if len(points) <= maximum:
        return points
    return points[np.linspace(0, len(points)-1, maximum, dtype=int)]


def _recess_windows(group):
    """Find local V recesses even when their shoulders join a long facade.

    Candidate generation only: each window must still pass the same fitted
    geometry, RMS, two-shoulder and two-branch tests in detect_profile.
    """
    if np.linalg.norm(group[-1]-group[0]) <= .43:
        return
    _values, vectors = np.linalg.eigh(np.cov(group.T))
    tangent = vectors[:, -1]
    normal = np.array((tangent[1], -tangent[0]))
    if normal[0] < 0:
        normal = -normal
    along = group @ tangent
    order = np.argsort(along)
    grid = np.arange(along.min(), along.max(), .005)
    if len(grid) < 5:
        return
    depth = np.interp(grid, along[order], (group @ normal)[order])
    smooth = np.convolve(depth, np.ones(3)/3, mode='valid')
    peaks, properties = find_peaks(smooth, prominence=.018, distance=8)
    for index in np.argsort(properties['prominences'])[-8:]:
        center = grid[peaks[index]+1]
        for offset in (-.01, 0., .01):
            window = group[abs(along-center-offset) <= FACE_HALF_WIDTH]
            if len(window) >= 10:
                yield _downsample(window)


def _candidates(points):
    points = points[np.argsort(np.arctan2(points[:, 1], points[:, 0]))]
    gaps = np.linalg.norm(np.diff(points, axis=0), axis=1)
    groups = np.split(points, np.where(gaps > 0.055)[0] + 1)
    for i, group in enumerate(groups):
        if len(group) >= 10:
            yield _downsample(group)
            yield from _recess_windows(group)
        # Near the sensor's minimum range the recess can disappear between two
        # visible shoulders; only a previously acquired target may use this fit.
        if len(group) >= 4:
            joined = [group]
            # At ~0.37 m the two shoulders AND the tip of the recess are
            # visible as three separate islands, with blind gaps in between.
            for following in groups[i + 1:i + 3]:
                if len(following) < 4 or np.linalg.norm(joined[-1][-1] - following[0]) >= 0.28:
                    break
                joined.append(following)
                yield _downsample(np.concatenate(joined))


def detect_profile(points, *, allow_partial=False, prior=None, diagnostics=None):
    points = np.asarray(points, dtype=float)
    if diagnostics is not None:
        diagnostics.update(input_points=int(len(points)), roi_points=0, candidates=0,
                           rejections={}, detected=False)
    def reject(reason):
        if diagnostics is not None:
            counts = diagnostics['rejections']
            counts[reason] = counts.get(reason, 0)+1
    if points.size == 0:
        return None
    points = points[np.isfinite(points).all(axis=1)]
    minimum = 0.28 if allow_partial else 0.34
    points = points[(points[:, 0] > minimum) & (points[:, 0] < 2.8) &
                    (np.abs(points[:, 1]) < 0.9)]
    if diagnostics is not None:
        diagnostics['roi_points'] = int(len(points))
    if len(points) < 10:
        return None
    best = None
    for group in _candidates(points):
        if diagnostics is not None:
            diagnostics['candidates'] += 1
        if len(group) > 400 or not 0.20 < np.linalg.norm(group[-1] - group[0]) < 0.43:
            reject('candidate_size_or_width')
            continue
        _values, vectors = np.linalg.eigh(np.cov(group.T))
        tangent = vectors[:, -1]
        normal = np.array((tangent[1], -tangent[0]))
        if normal[0] < 0:
            normal = -normal
        angle = float(np.clip(atan2(normal[1], normal[0]), -0.8, 0.8))
        n = np.array((cos(angle), sin(angle)))
        t = np.array((-sin(angle), cos(angle)))
        along = group @ t
        center = n * np.percentile(group @ n, 25) + t * ((along.min() + along.max()) / 2)

        def residual(pose):
            c, s = cos(pose[2]), sin(pose[2])
            relative = group - pose[:2]
            x = relative @ np.array((c, s))
            y = relative @ np.array((-s, c))
            profile = np.maximum(0, 0.05 - np.abs(y))
            return np.concatenate((x - profile,
                                   np.maximum(0, np.abs(y) - (FACE_HALF_WIDTH + .005)),
                                   [(y.min() + FACE_HALF_WIDTH) * 0.4,
                                    (y.max() - FACE_HALF_WIDTH) * 0.4]))

        initial = [np.clip(center[0], minimum + .005, 2.795), np.clip(center[1], -0.895, 0.895), angle]
        fit = least_squares(residual, initial, bounds=([minimum, -0.9, -0.85], [2.8, 0.9, 0.85]),
                            loss='soft_l1', f_scale=0.004, max_nfev=35)
        x0, y0, yaw = fit.x
        relative = group - fit.x[:2]
        local_x = relative @ np.array((cos(yaw), sin(yaw)))
        local_y = relative @ np.array((-sin(yaw), cos(yaw)))
        error = local_x - np.maximum(0, 0.05 - np.abs(local_y))
        rms = float(np.sqrt(np.mean(np.minimum(error * error, 0.03 ** 2))))
        left = local_x[local_y < -0.075]
        right = local_x[local_y > 0.075]
        middle = local_x[np.abs(local_y) < 0.027]
        full = (len(left) >= 3 and len(right) >= 3 and len(middle) >= 2 and
                np.median(middle) - (np.median(left) + np.median(right)) / 2 > 0.018 and
                np.count_nonzero((local_y > 0.008) & (local_y < 0.045)) >= 2 and
                np.count_nonzero((local_y < -0.008) & (local_y > -0.045)) >= 2)
        if diagnostics is not None and rms < diagnostics.get('best_fit_rms', float('inf')):
            diagnostics.update(best_fit_rms=rms, best_fit_full_shape=bool(full),
                               best_fit_pose=[float(x0), float(y0), float(yaw)])
        if rms > MAX_FIT_RMS or len(left) < 3 or len(right) < 3:
            reject('fit_error_or_missing_shoulders')
            continue
        if not full and not allow_partial:
            reject('missing_v_branches_or_tip')
            continue
        if prior is not None:
            if np.hypot(x0 - prior.x, y0 - prior.y) > 0.12 or abs(wrap(yaw - prior.yaw)) > 0.25:
                reject('outside_previous_target')
                continue
        result = ProfilePose(float(x0), float(y0), float(yaw), rms, bool(full),
                             tuple(map(tuple, group.tolist())))
        if best is None or (full, -rms) > (best.full_shape, -best.rms):
            best = result
    if diagnostics is not None:
        diagnostics['detected'] = best is not None
    return best


def terminal_shoulder_reference(points, predicted):
    """Refine a blind terminal's translation from one visible outer shoulder.

    This is teaching only, never target acquisition. A full V in the SAME
    odometry session must already anchor the station. Its yaw is retained;
    the short visible line does not robustly determine orientation by itself.
    """
    points = np.asarray(points, dtype=float)
    if len(points) < 6:
        return None
    yaw = predicted['yaw']
    normal = np.array((cos(yaw), sin(yaw)))
    tangent = np.array((-sin(yaw), cos(yaw)))
    local = (points - [predicted['x'], predicted['y']]) @ np.column_stack((normal, tangent))
    candidates = []
    for side in (-1, 1):
        mask = (abs(local[:, 0]) < .045) & (side * local[:, 1] > .075) & (side * local[:, 1] < .21)
        group = points[mask]
        if len(group) < 6:
            continue
        nx, ty = group @ normal, group @ tangent
        width = np.ptp(ty)
        if not .012 <= width <= .10 or np.std(nx) > .004:
            continue
        edge = ty.min() if side < 0 else ty.max()
        center_t = edge - side * FACE_HALF_WIDTH
        center = normal * np.median(nx) + tangent * center_t
        if np.linalg.norm(center - [predicted['x'], predicted['y']]) < .045:
            candidates.append(center)
    if not candidates:
        return None
    center = np.mean(candidates, axis=0)
    return dict(x=float(center[0]), y=float(center[1]), yaw=yaw)


def docking_errors(current, reference):
    """Robot error in the desired rear-facing frame: x<0 means still short of goal."""
    delta = wrap(current.yaw - reference['yaw'])
    c, s = cos(delta), sin(delta)
    return (reference['x'] - c * current.x - s * current.y,
            reference['y'] + s * current.x - c * current.y, -delta)


def signature_error(points, reference_points):
    """Raw LiDAR agreement at the taught final pose; no odometry-only success."""
    points = np.asarray(points, dtype=float)
    reference_points = np.asarray(reference_points, dtype=float)
    if len(points) < 3 or len(reference_points) < 3:
        return float('inf')
    a = cKDTree(reference_points).query(points)[0]
    b = cKDTree(points).query(reference_points)[0]
    return float(max(np.percentile(a, 75), np.percentile(b, 75)))
