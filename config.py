"""Shared paths and shell helpers for the robot workspace."""

import os
from pathlib import Path
import shlex


# Resolve the repository from this file so clones work from any directory/user.
PROJECT_ROOT = Path(__file__).resolve().parent
SOURCE_PATH = str(PROJECT_ROOT)  # Backwards-compatible name used by the UI.
ROS_DISTRO = os.environ.get("ROS_DISTRO", "jazzy")
ROS_SETUP = Path(os.environ.get("ROS_SETUP", f"/opt/ros/{ROS_DISTRO}/setup.bash"))
WORKSPACE_SETUP = PROJECT_ROOT / "install" / "setup.bash"

# Let child processes and external tools discover the same checkout.
os.environ["SOURCE_PATH"] = SOURCE_PATH


def shell_source_workspace(command: str) -> str:
    """Return a bash command that sources ROS and this workspace, then runs command."""
    parts = []
    if ROS_SETUP.is_file():
        parts.append(f"source {shlex.quote(str(ROS_SETUP))}")
    if WORKSPACE_SETUP.is_file():
        parts.append(f"source {shlex.quote(str(WORKSPACE_SETUP))}")
    parts.append(command)
    return " && ".join(parts)
