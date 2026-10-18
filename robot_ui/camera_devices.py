"""Enumerate camera metadata without opening a capture stream."""
from pathlib import Path


def discover_cameras():
    devices, warnings = [], []
    realsense_available = False
    try:
        import pyrealsense2 as rs
        for device in rs.context().query_devices():
            serial = device.get_info(rs.camera_info.serial_number)
            model = device.get_info(rs.camera_info.name)
            devices.append((f'{model} — {serial}', {'source': 'realsense', 'serial': serial}))
            realsense_available = True
    except Exception as exc:
        warnings.append(f'Không thể liệt kê RealSense: {exc}')
    stable = {}
    for directory in ('/dev/v4l/by-id', '/dev/v4l/by-path'):
        for path in sorted(Path(directory).glob('*')):
            if path.exists():
                stable.setdefault(str(path.resolve()), str(path))
    for entry in sorted(Path('/sys/class/video4linux').glob('video*')):
        try:
            name = (entry / 'name').read_text().strip()
            # Secondary interfaces commonly expose metadata/depth, not colour.
            if (entry / 'index').read_text().strip() != '0':
                continue
            if realsense_available and 'realsense' in name.lower():
                continue
            source = stable.get('/dev/' + entry.name, '/dev/' + entry.name)
            devices.append((f'{name} — {source}', {'source': source, 'serial': ''}))
        except OSError:
            continue
    return devices, warnings
