#!/usr/bin/env bash

PROJECT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/${ROS_DISTRO}/setup.bash}"
LOG_DIR="${XDG_STATE_HOME:-${HOME}/.local/state}/zackon"
mkdir -p "$LOG_DIR"

# ROS 2 (override ROS_DISTRO or ROS_SETUP for a different installation)
if [ -f "$ROS_SETUP" ]; then
    source "$ROS_SETUP"
else
    echo "ROS setup file not found: $ROS_SETUP" >&2
    exit 1
fi

# Workspace Bé Son
source "$PROJECT/install/setup.bash"

# Virtual environment
if [ -f "$PROJECT/venv/bin/activate" ]; then

    source "$PROJECT/venv/bin/activate"

fi

cd "$PROJECT/robot_ui"

exec python3 startup_layout.py \
    >> "$LOG_DIR/beson_startup.log" \
    2>&1
