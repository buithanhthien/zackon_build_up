#!/usr/bin/env python3
"""Preview Startup destinations without ROS, audio or subprocess startup.

QT_QPA_PLATFORM=offscreen venv/bin/python tool/preview_startup_screens.py
"""
import os
import sys
import tempfile
from pathlib import Path
from unittest.mock import patch

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'robot_ui'))
from PyQt6.QtWidgets import QApplication, QMainWindow, QPushButton
from waypoint_dialogs import (DestinationDialog, NewWaypointDialog,
                              WaypointPickerDialog, NewPathDialog, PathManagerDialog)
from language_dialog import LanguageDialog
from load_map_dialog import LoadMapDialog
from new_map_layout import NewMapUI
from docking_layout import DockingUI
from tracking_mode_layout import TrackingModeUI


class Parent(QMainWindow):
    def load_map(self): pass
    def open_waypoint_picker(self): pass
    def open_path_manager(self): pass
    def open_new_waypoint_dialog(self): pass


def main():
    app = QApplication.instance() or QApplication([])
    output = ROOT / 'docs' / 'startup-ui' / 'screens'
    output.mkdir(parents=True, exist_ok=True)
    parent = Parent()
    points = {name: dict(map_name='preview', display_name=name, x=i, y=0.,
                        z=0., qx=0., qy=0., qz=0., qw=1.)
              for i, name in enumerate(('Sảnh chính', 'Phòng thực hành', 'Trạm sạc'))}
    with tempfile.TemporaryDirectory() as directory:
        paths = Path(directory) / 'paths.json'
        paths.write_text('{}')
        dialogs = [
            ('destinations', DestinationDialog(parent)),
            ('new-waypoint', NewWaypointDialog(parent)),
            ('waypoints', WaypointPickerDialog(points, 'preview', parent)),
            ('new-path', NewPathDialog(points, 'preview', str(paths), parent)),
            ('paths', PathManagerDialog(str(paths), 'preview', parent)),
            ('language', LanguageDialog(parent)),
            ('load-map', LoadMapDialog(parent)),
            ('new-map', NewMapUI(parent)),
        ]
        # Construct only presentation; __init__ would start robot services.
        dock = DockingUI.__new__(DockingUI)
        QMainWindow.__init__(dock)
        dock.init_ui()
        dock.resize(1024, 720)
        track = TrackingModeUI.__new__(TrackingModeUI)
        QMainWindow.__init__(track)
        with patch('chat_panel_widget.VoiceEngine'), patch('chat_panel_widget.CorrectionMemory'):
            track.init_ui()
        track.showNormal()
        track.resize(1024, 720)
        dialogs += [('docking', dock), ('tracking', track)]
        for name, widget in dialogs:
            widget.show()
            app.processEvents()
            assert widget.width() <= 1024 and widget.height() <= 720, (name, widget.size())
            widget.grab().save(str(output / f'{name}.png'))
            for button in widget.findChildren(QPushButton):
                if button.isVisible():
                    assert button.height() >= 44, (name, button.text(), button.height())
            print(f'{name}: {widget.width()}x{widget.height()}')
            if hasattr(widget, 'keyboard_toggle'):
                widget.keyboard_toggle.setChecked(True)
                app.processEvents()
                assert widget.width() <= 1024 and widget.height() <= 720, (name, widget.size())
                widget.grab().save(str(output / f'{name}-keyboard.png'))
            if name == 'load-map' and widget.map_list.count():
                widget.map_list.setCurrentRow(0)
                assert widget.btn_ok.isEnabled()
                assert widget.get_selected_map() == widget.map_list.currentItem().text()
                widget.map_list.setCurrentRow(-1)
                assert not widget.btn_ok.isEnabled()
            if name == 'tracking':
                next(b for b in widget.findChildren(QPushButton) if b.text() == 'Hội thoại').click()
                app.processEvents()
                assert not widget.chat_widget.isHidden()
            widget.hide()
            widget.deleteLater()
        app.processEvents()


if __name__ == '__main__':
    main()
