#!/usr/bin/env python3
"""Map loading and configuration utilities"""
import os
import re
import subprocess
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from config import SOURCE_PATH


def get_current_map_path():
    """Get the currently configured map path from nav2_params.yaml"""
    nav2_params = f'{SOURCE_PATH}/src/view_robot/config/nav2_params.yaml'
    try:
        with open(nav2_params, 'r') as f:
            for line in f:
                if 'yaml_filename:' in line and '#' not in line:
                    path = line.split(':', 1)[1].strip().strip('"')
                    maps_dir = f'{SOURCE_PATH}/src/view_robot/maps/'
                    map_file = os.path.basename(path)
                    return maps_dir + map_file
    except Exception:
        pass
    return f'{SOURCE_PATH}/src/view_robot/maps/X5_08042026.yaml'


def get_current_map_name():
    """Return the stem of the current map file (e.g. 'X5_130426')"""
    path = get_current_map_path()
    return os.path.splitext(os.path.basename(path))[0]


def load_map_yaml(yaml_path):
    """Parse map YAML file and return dict with resolution, origin, image"""
    data = {}
    with open(yaml_path, 'r') as f:
        for line in f:
            if ':' in line:
                key, value = line.split(':', 1)
                key = key.strip()
                value = value.strip()
                if key == 'origin':
                    data[key] = eval(value)
                elif key == 'resolution':
                    data[key] = float(value)
                else:
                    data[key] = value
    return data


def update_map_files(map_name, log_callback=None):
    """Update all launch files and config to use the specified map
    
    Args:
        map_name: Name of map without .yaml extension
        log_callback: Optional function(message) to log progress
    
    Returns:
        bool: True if successful, False otherwise
    """
    def log(msg):
        if log_callback:
            log_callback(msg)
        else:
            print(msg)
    
    map_path = f'{SOURCE_PATH}/src/view_robot/maps/{map_name}.yaml'
    
    # Update nav2_params.yaml
    nav2_params = f'{SOURCE_PATH}/src/view_robot/config/nav2_params.yaml'
    try:
        with open(nav2_params, 'r') as f:
            content = f.read()
        updated = re.sub(
            r'(yaml_filename:\s*")[^"]*(")', rf'\1{map_path}\2', content
        )
        with open(nav2_params, 'w') as f:
            f.write(updated)
        log("✓ Đã cập nhật nav2_params.yaml")
    except Exception as e:
        log(f"✗ Lỗi cập nhật nav2_params.yaml: {e}")
        return False
    
    # Update NAV2_BRINGUP.launch.py
    synthesis_launch = f'{SOURCE_PATH}/src/view_robot/launch/NAV2_BRINGUP.launch.py'
    try:
        with open(synthesis_launch, 'r') as f:
            content = f.read()
        updated = re.sub(
            r"(map_file_path\s*=\s*PathJoinSubstitution\(\[pkg_dir,\s*'maps',\s*')[^']*('\]\))",
            rf"\1{map_name}.yaml\2", content
        )
        with open(synthesis_launch, 'w') as f:
            f.write(updated)
        log(f"✓ Đã cập nhật NAV2_BRINGUP.launch.py")
    except Exception as e:
        log(f"✗ Lỗi cập nhật NAV2_BRINGUP.launch.py: {e}")
        return False
    
    # Update zackon_localization.launch.py
    localization_launch = f'{SOURCE_PATH}/src/view_robot/launch/zackon_localization.launch.py'
    try:
        with open(localization_launch, 'r') as f:
            content = f.read()
        updated = re.sub(
            r"(default_value=os\.path\.join\(bringup_dir,\s*'maps',\s*')[^']*('\))",
            rf"\1{map_name}.yaml\2", content
        )
        with open(localization_launch, 'w') as f:
            f.write(updated)
        log("✓ Đã cập nhật zackon_localization.launch.py")
    except Exception as e:
        log(f"✗ Lỗi cập nhật zackon_localization.launch.py: {e}")
        return False
    
    # Build workspace
    log("Đang build workspace...")
    try:
        subprocess.Popen([
            'gnome-terminal', '--', 'bash', '-c',
            'cd ~/zackon_build_up && colcon build --packages-select view_robot_pkg '
            '&& source install/setup.bash '
            '&& echo "Build complete. Closing in 2 seconds..." && sleep 2'
        ])
        log(f"✓ Bản đồ '{map_name}' đã tải và đang build workspace")
        return True
    except Exception as e:
        log(f"✗ Lỗi build workspace: {e}")
        return False


def list_available_maps():
    """Return list of available map names (without .yaml extension)"""
    maps_dir = f'{SOURCE_PATH}/src/view_robot/maps'
    if not os.path.exists(maps_dir):
        return []
    return sorted(f[:-5] for f in os.listdir(maps_dir) if f.endswith('.yaml'))
