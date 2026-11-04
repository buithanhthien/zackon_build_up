"""Per-map teaching data; map coordinates must never be confused with odometry."""

import hashlib
from math import atan2, cos, sin, hypot
import os
from pathlib import Path
import tempfile

import yaml

from ..perception.v_shape import terminal_shoulder_reference


def compose(a, b):
    c, s = cos(a['yaw']), sin(a['yaw'])
    return dict(x=a['x'] + c*b['x'] - s*b['y'],
                y=a['y'] + s*b['x'] + c*b['y'],
                yaw=atan2(sin(a['yaw'] + b['yaw']), cos(a['yaw'] + b['yaw'])))


def inverse(pose):
    c, s = cos(pose['yaw']), sin(pose['yaw'])
    return dict(x=-c*pose['x'] - s*pose['y'], y=s*pose['x'] - c*pose['y'], yaw=-pose['yaw'])


def rear_pose(pose):
    return compose(pose, dict(x=0.0, y=0.0, yaw=3.141592653589793))


def update_reference(data):
    """Tie the taught terminal pose to a V observed at the pre-docking pose."""
    terminal = data.get('terminal_sample')
    pre = data.get('pre_docking')
    if not terminal or not pre or 'marker_in_odom' not in pre:
        return data
    same_session = pre.get('session_id') and pre.get('session_id') == terminal.get('session_id')
    if ((pre.get('session_id') and terminal.get('session_id') and not same_session) or
            (not same_session and pre['stamp'] < terminal['stamp'])):
        raise ValueError('Mô phỏng đã reset odometry: lấy lại mẫu sát dock trước khi lưu pre-docking.')
    ref = compose(inverse(rear_pose(terminal['odom_pose'])), pre['marker_in_odom'])
    if not (0.27 < ref['x'] < 0.8 and abs(ref['y']) < 0.10 and abs(ref['yaw']) < 0.30):
        raise ValueError('Mẫu đích và pre-docking không cùng trạm/hướng lùi; lấy mẫu sát dock rồi lùi ra điểm chờ.')
    if len(terminal.get('near_points', [])) < 3:
        raise ValueError('Mẫu sát dock không đủ điểm LiDAR gần để xác nhận dừng; lấy lại mẫu ở vị trí còn thấy mép trạm.')
    if terminal.get('observed_profile'):
        ref = dict(terminal['observed_profile'])
    elif terminal.get('candidate_profile'):
        candidate = terminal['candidate_profile']
        yaw_error = candidate['yaw'] - ref['yaw']
        if (hypot(candidate['x'] - ref['x'], candidate['y'] - ref['y']) < 0.12 and
                abs(atan2(sin(yaw_error), cos(yaw_error))) < 0.25):
            ref = dict(candidate)
    else:
        shoulder = terminal_shoulder_reference(terminal['near_points'], ref)
        if shoulder is not None:
            ref = shoulder
    ref.update(points=terminal.get('near_points', []), use_near_signature=True,
               blind_finish=not terminal.get('full_shape_visible', False),
               max_blind_travel=0.18)
    data['docked_reference'] = ref
    return data


def profile_path(map_path):
    return Path(map_path).with_suffix('.docking.yaml')


def map_identity(map_path):
    path = Path(map_path).resolve()
    metadata = yaml.safe_load(path.read_text())
    image = path.parent / metadata['image']
    return hashlib.sha256(path.read_bytes() + image.read_bytes()).hexdigest()


def load_profile(map_path):
    path = profile_path(map_path)
    if not path.exists():
        return {'version': 1, 'map': str(Path(map_path).resolve()), 'frame': 'map'}
    result = yaml.safe_load(path.read_text())
    if result.get('map_sha256') and result['map_sha256'] != map_identity(map_path):
        raise ValueError('Map đã thay đổi; cần lưu lại pre-docking và mẫu LiDAR cho map này.')
    return result


def save_profile(map_path, data):
    data = dict(data, version=1, map=str(Path(map_path).resolve()), frame='map',
                map_sha256=map_identity(map_path))
    has_pre = 'marker_in_odom' in data.get('pre_docking', {})
    data['calibration_status'] = (
        'ready' if has_pre and 'docked_reference' in data else
        'terminal_saved_pre_docking_required' if 'terminal_sample' in data else
        'pre_docking_saved_terminal_required' if has_pre else 'teaching_required')
    destination = profile_path(map_path)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode='w', dir=destination.parent,
                                         prefix='.dock_profile_', delete=False) as stream:
            temporary = stream.name
            yaml.safe_dump(data, stream, sort_keys=False, allow_unicode=True)
        os.replace(temporary, destination)
    finally:
        if temporary is not None and os.path.exists(temporary):
            os.unlink(temporary)
    return data
