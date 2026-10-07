#!/usr/bin/env python3
import os
import re
import shlex
import sys
from pathlib import Path

from PyQt6.QtWidgets import (QDialog, QWidget, QVBoxLayout, QHBoxLayout,
                             QPushButton, QLabel, QLineEdit)
from PyQt6.QtCore import QTimer, Qt
from PyQt6.QtGui import QFont
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from config import SOURCE_PATH, shell_source_workspace
from styles import MAIN_STYLESHEET
from ui_utils import setup_clock_timer
from mapping_process import MappingProcess
from virtual_keyboard import VirtualKeyboard


class NewMapUI(QDialog):
    _active_dialog = None

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setModal(False)
        self.resize(820, 480)
        self.save_process = None
        self._save_path = None
        self.mapping_process = None
        self.init_ui()
        self.process_timer = QTimer(self)
        self.process_timer.timeout.connect(self.check_processes)
        self.process_timer.start(250)

    def init_ui(self):
        self.setWindowTitle("New Map - SLAM Mapping")
        self.setStyleSheet(MAIN_STYLESHEET)

        main_layout = QHBoxLayout(self)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)

        # ── Left panel ────────────────────────────────────────────────────────
        left_panel = QWidget()
        left_panel.setObjectName("left-panel")
        left_layout = QVBoxLayout(left_panel)
        left_layout.setContentsMargins(0, 0, 0, 0)
        left_layout.setSpacing(0)

        wordmark = QLabel("BẢN ĐỒ MỚI")
        wordmark.setFont(QFont("JetBrains Mono", 14, QFont.Weight.Bold))
        wordmark.setStyleSheet("color: #fcb525; padding: 24px 24px 16px 24px;")
        left_layout.addWidget(wordmark)

        mono = QFont("JetBrains Mono", 18)
        self.btn_back   = QPushButton("Quay lại")

        self.btn_back.setObjectName("action-btn")
        self.btn_back.setFont(mono)
        self.btn_back.setMinimumHeight(72)
        left_layout.addWidget(self.btn_back)

        left_layout.addStretch()

        self.btn_back.clicked.connect(self.go_back)

        # ── Right area ────────────────────────────────────────────────────────
        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.setSpacing(0)

        # Header bar
        header = QWidget()
        header.setObjectName("header-bar")
        header.setFixedHeight(48)
        header_layout = QHBoxLayout(header)
        header_layout.setContentsMargins(20, 0, 20, 0)

        header_title = QLabel("SLAM MAPPING")
        header_title.setFont(QFont("JetBrains Mono", 15, QFont.Weight.Bold))
        header_title.setStyleSheet("color: #1a2a5e;")

        self.clock_label = QLabel()
        self.clock_label.setObjectName("clock")
        self.clock_label.setFont(QFont("JetBrains Mono", 15))

        header_layout.addWidget(header_title)
        header_layout.addStretch()
        header_layout.addWidget(self.clock_label)
        right_layout.addWidget(header)

        # Content panel
        content = QWidget()
        content.setObjectName("content-panel")
        content_layout = QVBoxLayout(content)
        content_layout.setContentsMargins(32, 24, 32, 24)
        content_layout.setSpacing(16)

        info = QLabel("Khởi động SLAM Toolbox để tạo bản đồ mới, sau đó lưu lại. Điều khiển robot để khám phá môi trường")
        info.setObjectName("info-text")
        info.setFont(QFont("DM Sans", 14))
        info.setWordWrap(True)
        content_layout.addWidget(info)

        self.btn_start = QPushButton("Bắt đầu lập bản đồ")
        self.btn_start.setObjectName("primary-btn")
        self.btn_start.setFont(QFont("JetBrains Mono", 18))
        self.btn_start.clicked.connect(self.start_mapping)
        self.btn_cancel = QPushButton("Hủy")
        self.btn_cancel.setObjectName("cancel-mapping-btn")
        self.btn_cancel.setFont(mono)
        self.btn_cancel.setMinimumHeight(56)
        self.btn_cancel.setMinimumWidth(120)
        self.btn_cancel.setStyleSheet("""
            QPushButton#cancel-mapping-btn {
                background-color: #dc2626;
                color: #ffffff;
                border: none;
                border-radius: 8px;
                font-size: 18px;
                padding: 0px 24px;
            }
            QPushButton#cancel-mapping-btn:hover { background-color: #b91c1c; }
            QPushButton#cancel-mapping-btn:pressed { background-color: #991b1b; }
        """)
        self.btn_cancel.clicked.connect(self.cancel_mapping)
        mapping_row = QHBoxLayout()
        mapping_row.setSpacing(12)
        mapping_row.addWidget(self.btn_start, 1)
        mapping_row.addWidget(self.btn_cancel)
        content_layout.addLayout(mapping_row)

        # Save map row
        save_label = QLabel("LƯU BẢN ĐỒ")
        save_label.setObjectName("section-label")
        save_label.setFont(QFont("DM Sans", 11))
        content_layout.addWidget(save_label)

        save_row = QHBoxLayout()
        self.map_name_input = QLineEdit()
        self.map_name_input.setObjectName("map-input")
        self.map_name_input.setPlaceholderText("Tên bản đồ...")
        self.map_name_input.setFont(QFont("JetBrains Mono", 16))
        save_row.addWidget(self.map_name_input)

        self.keyboard_toggle = QPushButton("⌨")
        self.keyboard_toggle.setCheckable(True)
        self.keyboard_toggle.setAutoDefault(False)
        self.keyboard_toggle.setFocusPolicy(Qt.FocusPolicy.NoFocus)
        self.keyboard_toggle.setFixedSize(50, 50)
        self.keyboard_toggle.setToolTip("Hiện/ẩn bàn phím ảo (chữ không dấu)")
        self.keyboard_toggle.setAccessibleName("Hiện hoặc ẩn bàn phím ảo cho tên bản đồ")
        self.keyboard_toggle.setStyleSheet("""
            QPushButton {
                background-color: #e0e8f8;
                color: #1a2a5e;
                border-radius: 8px;
                font-size: 24px;
                padding: 0;
            }
            QPushButton:checked { background-color: #c8d4f0; }
        """)
        save_row.addWidget(self.keyboard_toggle)

        self.btn_apply = QPushButton("Áp dụng")
        self.btn_apply.setObjectName("apply-btn")
        self.btn_apply.setFont(QFont("JetBrains Mono", 16))
        self.btn_apply.setFixedWidth(120)
        self.btn_apply.clicked.connect(self.save_map)
        save_row.addWidget(self.btn_apply)
        content_layout.addLayout(save_row)

        self.virtual_keyboard = VirtualKeyboard(self.map_name_input, enter_label="Lưu")
        self.virtual_keyboard.submitted.connect(self.save_map)
        self.virtual_keyboard.hide()
        content_layout.addWidget(self.virtual_keyboard)
        self.keyboard_toggle.toggled.connect(self._toggle_keyboard)

        content_layout.addStretch()
        right_layout.addWidget(content, 1)

        self.status_label = QLabel("Sẵn sàng lập bản đồ.")
        self.status_label.setWordWrap(True)
        self.status_label.setTextFormat(Qt.TextFormat.PlainText)
        self.status_label.setContentsMargins(24, 12, 24, 12)
        right_layout.addWidget(self.status_label)

        main_layout.addWidget(left_panel, 22)
        main_layout.addWidget(right_widget, 78)

        for button in (self.btn_cancel, self.btn_back, self.btn_start, self.btn_apply):
            button.setAutoDefault(False)
        self.map_name_input.returnPressed.connect(self.save_map)
        self.clock_timer = setup_clock_timer(self.clock_label)

    def _toggle_keyboard(self, visible):
        self.virtual_keyboard.setVisible(visible)
        if visible:
            self.map_name_input.setFocus()

    def log(self, message):
        self.status_label.setText(message)

    def start_mapping(self):
        if self.mapping_process is not None:
            return
        owner = NewMapUI._active_dialog
        if owner is not None and owner is not self:
            self.log("Một phiên lập bản đồ khác đang chạy.")
            return
        try:
            self.mapping_process = MappingProcess(shell_source_workspace(
                'exec ros2 launch view_robot_pkg MAP_GENERATING.launch.py'
            ))
        except Exception as exc:
            self.log(f"Không thể bắt đầu lập bản đồ: {exc}")
            return
        NewMapUI._active_dialog = self
        self.btn_start.setEnabled(False)
        self.btn_start.setText("Đã gửi lệnh khởi động")
        self.log("Đã gửi lệnh khởi động SLAM. Kiểm tra bản đồ trong RViz trước khi lưu.")

    def cancel_mapping(self):
        try:
            self._stop_processes()
        except Exception as exc:
            self.log(f"Không thể dừng lập bản đồ: {exc}")
            return
        self.log("Đã dừng các tiến trình của phiên lập bản đồ.")

    def _stop_processes(self):
        errors = []
        for attr in ('save_process', 'mapping_process'):
            proc = getattr(self, attr)
            if proc is not None:
                try:
                    proc.stop()
                    setattr(self, attr, None)
                except Exception as exc:
                    errors.append(str(exc))
        if errors:
            raise RuntimeError('; '.join(errors))
        if NewMapUI._active_dialog is self:
            NewMapUI._active_dialog = None
        self.btn_start.setEnabled(True)
        self.btn_start.setText("Bắt đầu lập bản đồ")
        self.btn_apply.setEnabled(True)
        self.map_name_input.setEnabled(True)
        self.virtual_keyboard.setEnabled(True)

    def save_map(self):
        if self.save_process is not None:
            return
        map_name = self.map_name_input.text().strip()
        if not map_name:
            self.log("Vui lòng nhập tên bản đồ.")
            return
        if map_name in ('.', '..') or not re.fullmatch(r'[\w .-]+', map_name):
            self.log("Tên bản đồ chỉ được chứa chữ, số, khoảng trắng, dấu gạch và dấu chấm.")
            return
        if self.mapping_process is None or self.mapping_process.poll() is not None:
            self.log("Hãy bắt đầu lập bản đồ trước khi lưu.")
            return
        self._save_path = Path(SOURCE_PATH) / 'src/view_robot/maps' / map_name
        try:
            self._save_path.parent.mkdir(parents=True, exist_ok=True)
            self.save_process = MappingProcess(shell_source_workspace(
                f'exec ros2 run nav2_map_server map_saver_cli -f {shlex.quote(str(self._save_path))}'
            ))
        except Exception as exc:
            self.log(f"Không thể lưu bản đồ: {exc}")
            return
        self.btn_apply.setEnabled(False)
        self.map_name_input.setEnabled(False)
        self.virtual_keyboard.setEnabled(False)
        self.log(f"Đang lưu bản đồ '{map_name}'…")

    def check_processes(self):
        if self.mapping_process is not None and self.mapping_process.poll() is not None:
            code = self.mapping_process.poll()
            detail = self.mapping_process.error_detail()
            try:
                self._stop_processes()
            except Exception as exc:
                self.log(f"Không thể dọn tiến trình mapping: {exc}")
                return
            self.log(f"Tiến trình mapping đã kết thúc (mã {code}). {detail}")
            return
        if self.save_process is not None and self.save_process.poll() is not None:
            code = self.save_process.poll()
            detail = self.save_process.error_detail()
            try:
                self.save_process.stop()
            except Exception as exc:
                self.log(f"Không thể dọn tiến trình lưu: {exc}")
                return
            self.save_process = None
            self.btn_apply.setEnabled(True)
            self.map_name_input.setEnabled(True)
            self.virtual_keyboard.setEnabled(True)
            yaml_file = Path(str(self._save_path) + '.yaml')
            if code == 0 and yaml_file.is_file():
                self.log(f"Đã lưu bản đồ tại: {yaml_file}")
            else:
                self.log(f"Không thể lưu bản đồ (mã {code}). {detail}")

    def go_back(self):
        self.close()

    def done(self, result):
        # Escape/reject/accept do not necessarily dispatch closeEvent.
        try:
            self._stop_processes()
        except Exception as exc:
            self.log(f"Không thể đóng: {exc}")
            return
        super().done(result)

    def closeEvent(self, event):
        try:
            self._stop_processes()
        except Exception as exc:
            self.log(f"Không thể đóng: {exc}")
            event.ignore()
            return
        event.accept()
