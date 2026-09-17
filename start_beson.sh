#!/usr/bin/env bash

PROJECT="/home/khoaiuh/zackon_build_up"

# ROS 2 Jazzy
source /opt/ros/jazzy/setup.bash

# Workspace Bé Son
source "$PROJECT/install/setup.bash"

# Virtual environment
if [ -f "$PROJECT/venv/bin/activate" ]; then

    source "$PROJECT/venv/bin/activate"

fi

cd "$PROJECT/robot_ui"

exec python3 startup_layout.py \
    >> /home/khoaiuh/beson_startup.log \
    2>&1