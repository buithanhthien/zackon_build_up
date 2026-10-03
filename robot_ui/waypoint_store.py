"""Additive waypoint schema; dictionary keys remain stable legacy route references."""
import copy
import json
import math
import os
from pathlib import Path
import re
import tempfile
import uuid


class WaypointError(ValueError):
    pass


def legacy_id(key):
    return str(uuid.uuid5(uuid.NAMESPACE_URL, 'zackon:waypoint:' + key.casefold()))


# Protect original room identities even if their display name or key is edited.
_ROOM_KEYS = ([f'x5.{i}' for i in range(1, 15)] + ['x5.17']
              + [f'x5.2.{i}' for i in range(1, 6)]
              + [f'x5.3.{i}' for i in range(1, 6)])
_ROOM_IDS = {legacy_id(key) for key in _ROOM_KEYS}


def is_room(key, waypoint):
    return (re.fullmatch(r'x\d+(?:\.\d+)*', key.strip(), re.IGNORECASE) is not None
            or waypoint.get('id') in _ROOM_IDS)


def normalize_waypoints(data):
    if not isinstance(data, dict):
        raise WaypointError('Waypoint phải là một JSON object chứa các điểm đến.')
    result, ids, keys = {}, set(), set()
    for key, original in data.items():
        def fail(reason):
            raise WaypointError(f'Waypoint {key!r}: {reason}')
        if not isinstance(key, str) or not key.strip() or not isinstance(original, dict):
            fail('khóa phải là chuỗi không rỗng, dữ liệu phải là object.')
        if key.casefold() in keys:
            fail('khóa bị trùng (không phân biệt hoa/thường).')
        keys.add(key.casefold())
        wp = copy.deepcopy(original)
        for field in ('x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'):
            value = wp.get(field)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                fail(f'{field} phải là số hữu hạn.')
        if sum(wp[f] ** 2 for f in ('qx', 'qy', 'qz', 'qw')) < 1e-12:
            fail('quaternion không được bằng 0.')
        if not isinstance(wp.get('map_name'), str) or not wp['map_name'].strip():
            fail('map_name phải là chuỗi không rỗng.')
        for field, default in (('id', legacy_id(key)), ('display_name', key)):
            wp.setdefault(field, default)
            if not isinstance(wp[field], str) or not wp[field].strip():
                fail(f'{field} phải là chuỗi không rỗng.')
        if wp['id'] in ids:
            fail('id bị trùng.')
        ids.add(wp['id'])
        wp.setdefault('aliases', [])
        if not isinstance(wp['aliases'], list) or any(
                not isinstance(a, str) or not a.strip() for a in wp['aliases']):
            fail('aliases phải là danh sách chuỗi không rỗng.')
        if 'yaw_tolerance' in wp and (type(wp['yaw_tolerance']) not in (int, float)
                or not math.isfinite(wp['yaw_tolerance']) or wp['yaw_tolerance'] <= 0):
            fail('yaw_tolerance phải là số dương hữu hạn.')
        for field in ('deletable', 'deletion_permission_pending'):
            if field in wp and type(wp[field]) is not bool:
                fail(f'{field} phải là boolean true/false.')
        if 'deletable' not in wp:
            wp['deletable'] = False
            wp['deletion_permission_pending'] = not is_room(key, wp)
        if is_room(key, wp):
            wp['deletable'] = False
            wp.pop('deletion_permission_pending', None)
        if wp['deletable']:
            wp.pop('deletion_permission_pending', None)
        result[key] = wp
    return result


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise WaypointError(f'JSON chứa khóa trùng: {key!r}.')
        result[key] = value
    return result


def read_bytes(path):
    try:
        return Path(path).read_bytes()
    except FileNotFoundError:
        return None


def load_waypoint_file(path):
    raw = read_bytes(path)
    if raw is None:
        return {}, None
    return normalize_waypoints(json.loads(raw, object_pairs_hook=_unique_object)), raw


def save_waypoint_file(path, data, expected):
    """Commit to disk before the caller replaces memory. Reject stale snapshots."""
    clean = normalize_waypoints(data)
    payload = (json.dumps(clean, ensure_ascii=False, indent=2, allow_nan=False)+'\n').encode()
    if read_bytes(path) != expected:
        raise WaypointError('File waypoint đã thay đổi bên ngoài. Hãy mở lại màn hình trước khi lưu.')
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(dir=Path(path).parent, prefix='.waypoints-', delete=False) as file:
            temporary = file.name
            file.write(payload)
            file.flush()
            os.fsync(file.fileno())
        if read_bytes(path) != expected:
            raise WaypointError('File waypoint vừa thay đổi; chưa ghi đè dữ liệu.')
        os.replace(temporary, path)
    finally:
        if temporary and os.path.exists(temporary):
            os.unlink(temporary)
    return clean, payload


def deletion_reason(key, waypoint):
    try:
        wp = normalize_waypoints({key: waypoint})[key]
    except (ValueError, TypeError) as exc:
        return str(exc)
    if is_room(key, wp):
        return 'Phòng X được bảo vệ, không cho phép xóa.'
    if not wp['deletable'] and not wp.get('deletion_permission_pending', False):
        return 'Điểm đến này đã tắt quyền xóa.'
    return ''


def route_references(path, key, waypoint):
    raw = read_bytes(path)
    if raw is None:
        return []
    routes = json.loads(raw, object_pairs_hook=_unique_object)
    if not isinstance(routes, dict):
        raise WaypointError('File lộ trình phải là một JSON object.')
    identities = {s.casefold() for s in [key, waypoint['id'], waypoint['display_name'],
                                       *waypoint.get('aliases', [])]}
    references = []
    for name, route in routes.items():
        if (not isinstance(route, dict) or not isinstance(route.get('map_name'), str)
                or not isinstance(route.get('sequence'), list)
                or any(not isinstance(s, str) or not s.strip() for s in route['sequence'])):
            raise WaypointError(f'Lộ trình {name!r} sai định dạng; không thể kiểm tra tham chiếu.')
        if any(
                s.casefold() in identities for s in route['sequence']):
            references.append(name)
    return references


def resolve_waypoint(data, name, map_name):
    """Resolve live data only; ambiguous aliases must not choose a random room."""
    if not isinstance(name, str):
        return None
    name = name.casefold()
    available = {k: v for k, v in data.items() if v.get('map_name') == map_name}
    exact = [k for k, v in available.items() if name in (k.casefold(), v.get('id', '').casefold())]
    matches = exact or [k for k, v in available.items() if name in
                       [v.get('display_name', k).casefold(), *[a.casefold() for a in v.get('aliases', [])]]]
    return matches[0] if len(matches) == 1 else None
