#!/usr/bin/env python3
"""Render only Startup presentation, without importing ROS or starting voice/processes.

Run: QT_QPA_PLATFORM=offscreen venv/bin/python tool/preview_startup_ui.py
The inert ChatPanel is intentionally a visual fixture, not a hardware simulator.
"""
import ast
import os
from pathlib import Path
import subprocess
import sys

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "robot_ui"))
from PyQt6 import QtWidgets, QtCore, QtGui
from startup_icons import startup_icon
from startup_style import STARTUP_STYLESHEET, startup_text, repolish, refresh_voice_button
from styles import MAIN_STYLESHEET
from virtual_keyboard import VirtualKeyboard
from ui_utils import setup_clock_timer
from language_config import get_ui_text


class PreviewChatPanel(QtWidgets.QWidget):
    log_signal = QtCore.pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.voice_btn = QtWidgets.QPushButton("CLICK\n TO SPEAK", self)
        self.voice_btn.setObjectName("voice-btn")
        self.voice_btn.setCheckable(True)
        self.voice_status_label = QtWidgets.QLabel("", self)
        self.interrupt_btn = QtWidgets.QPushButton("■ Dừng", self)
        self.interrupt_btn.setObjectName("interrupt-btn")
        self._recording_active = False


def make_window(source, language="vi"):
    """Extract the real UI methods; substitute only external services."""
    tree = ast.parse(source)
    robot = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "RobotUI")
    names = {"init_ui", "_make_status_card", "_set_card_status", "update_language_ui",
             "_refresh_control_presentation", "_append_chat_message"}
    selected = [n for n in robot.body if isinstance(n, ast.FunctionDef) and n.name in names]
    view = ast.ClassDef(name="PreviewWindow", bases=[ast.Name(id="QMainWindow", ctx=ast.Load())],
                        keywords=[], body=selected, decorator_list=[])
    module = ast.fix_missing_locations(ast.Module(body=[view], type_ignores=[]))
    namespace = {}
    for provider in (QtWidgets, QtCore, QtGui):
        namespace.update({k: getattr(provider, k) for k in dir(provider) if k.startswith("Q") or k == "Qt"})
    namespace.update(ChatPanel=PreviewChatPanel, VirtualKeyboard=VirtualKeyboard,
                     STARTUP_STYLESHEET=STARTUP_STYLESHEET, MAIN_STYLESHEET=MAIN_STYLESHEET, DIALOG_STYLE=MAIN_STYLESHEET,
                     startup_icon=startup_icon, startup_text=startup_text, refresh_voice_button=refresh_voice_button, repolish=repolish, get_language=lambda: language,
                     get_ui_text=get_ui_text, setup_clock_timer=setup_clock_timer)
    exec(compile(module, "startup-ui-preview", "exec"), namespace)
    cls = namespace["PreviewWindow"]
    for name in ("open_destination_dialog", "start_docking", "load_map", "start_new_map",
                 "start_reestimate", "open_language_dialog", "open_developer_mode", "mode_changed",
                 "toggle_map_window", "_send_chat_message", "show_stm32_diagnostics",
                 "update_status", "_pulse_reestimate"):
        setattr(cls, name, lambda self, *args: None)
    window = cls()
    window.init_ui()
    return window


def main():
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    output = ROOT / "docs" / "startup-ui"
    output.mkdir(parents=True, exist_ok=True)
    current = (ROOT / "robot_ui" / "startup_layout.py").read_text()
    before = subprocess.check_output(["git", "show", "HEAD:robot_ui/startup_layout.py"], cwd=ROOT, text=True)
    for revision, source in (("before", before), ("after", current)):
        for width, height in ((1280, 720), (1920, 1080)):
            window = make_window(source)
            window.resize(width, height)
            window.show()
            app.processEvents()
            window.grab().save(str(output / f"{revision}-{width}x{height}.png"))
            print(f"{revision}: requested {width}x{height}, actual {window.width()}x{window.height()}, minimum {window.minimumSizeHint().width()}x{window.minimumSizeHint().height()}")
            window.hide()
            window.deleteLater()
            app.processEvents()

    for language in ("vi", "en"):
        window = make_window(current, language)
        window.resize(1280, 720)
        window._set_card_status(window.stm32_card, True)
        window._set_card_status(window.front_lidar_card, False)
        window._append_chat_message("[Bạn] Xin chào Bé Son, cho tôi biết trạng thái robot.")
        window._append_chat_message("[Bé Son] Bạn có thể kiểm tra kết nối ở ba thẻ thiết bị phía trên. Nút Dừng luôn nằm trong khu điều khiển giọng nói.")
        panel = window.chat_panel
        panel._recording_active = True
        panel.voice_btn.setChecked(True)
        panel.voice_status_label.setText("[>>] LISTENING")
        panel.voice_status_label.show()
        window._refresh_control_presentation()
        if language == "en":
            window.keyboard_toggle.setChecked(True)
        window.show()
        app.processEvents()
        window.grab().save(str(output / f"after-states-{language}-1280x720.png"))
        window.hide()
        window.deleteLater()
        app.processEvents()


if __name__ == "__main__":
    main()
