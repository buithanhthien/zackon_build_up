#!/usr/bin/env python3
"""Interactive Startup preview with inert chat/ROS, or capture with --output."""
import argparse
import os
from pathlib import Path

from preview_startup_ui import ROOT, QtWidgets, make_window


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, help="Save a PNG offscreen instead of opening a window")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--language", choices=("vi", "en"), default="vi")
    parser.add_argument("--keyboard", action="store_true")
    args = parser.parse_args()
    if args.output:
        os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    app = QtWidgets.QApplication([])
    window = make_window((ROOT / "robot_ui/startup_layout.py").read_text(), args.language)
    window.resize(args.width, args.height)
    window.keyboard_toggle.setChecked(args.keyboard)
    window.show()
    if args.output:
        app.processEvents()
        args.output.parent.mkdir(parents=True, exist_ok=True)
        if not window.grab().save(str(args.output)):
            raise RuntimeError(f"Cannot save {args.output}")
        window.close()
    else:
        app.exec()


if __name__ == "__main__":
    main()
