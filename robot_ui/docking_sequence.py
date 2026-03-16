#!/usr/bin/env python3
import subprocess
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from config import SOURCE_PATH

subprocess.Popen([
    'gnome-terminal', '--', 'bash', '-c',
    f'source {SOURCE_PATH}/install/setup.bash && ros2 launch auto_docking auto_docking.launch.py; exec bash'
])
