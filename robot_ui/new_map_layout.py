#!/usr/bin/env python3
import sys
import subprocess
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
                             QPushButton, QTextEdit, QLabel, QLineEdit)
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QFont
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from config import SOURCE_PATH
from chat_panel_widget import ChatPanel
from styles import MAIN_STYLESHEET
from ui_utils import append_log, setup_clock_timer
from process_manager import ProcessManager


class NewMapUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.process_mgr = ProcessManager()
        self.mapping_process = None
        self.init_ui()

    def init_ui(self):
        self.setWindowTitle("New Map - SLAM Mapping")
        self.setStyleSheet(MAIN_STYLESHEET)

        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QHBoxLayout(central)
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
        self.btn_cancel = QPushButton("Hủy")
        self.btn_back   = QPushButton("Quay lại")

        for btn in [self.btn_cancel, self.btn_back]:
            btn.setObjectName("action-btn")
            btn.setFont(mono)
            btn.setMinimumHeight(72)
            left_layout.addWidget(btn)

        left_layout.addStretch()

        self.btn_cancel.clicked.connect(self.cancel_mapping)
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
        content_layout.addWidget(self.btn_start)

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

        self.btn_apply = QPushButton("Áp dụng")
        self.btn_apply.setObjectName("apply-btn")
        self.btn_apply.setFont(QFont("JetBrains Mono", 16))
        self.btn_apply.setFixedWidth(120)
        self.btn_apply.clicked.connect(self.save_map)
        save_row.addWidget(self.btn_apply)
        content_layout.addLayout(save_row)

        content_layout.addStretch()
        right_layout.addWidget(content, 1)

        # Log panel
        log_panel = QWidget()
        log_panel.setObjectName("log-panel")
        log_layout = QVBoxLayout(log_panel)
        log_layout.setContentsMargins(16, 12, 16, 12)
        log_layout.setSpacing(6)

        log_header = QHBoxLayout()
        log_title = QLabel("SYSTEM LOG")
        log_title.setObjectName("log-title")
        log_title.setFont(QFont("DM Sans", 11))
        live_badge = QLabel("● LIVE")
        live_badge.setStyleSheet("color: #22c55e; font-size: 11px;")
        log_header.addWidget(log_title)
        log_header.addStretch()
        log_header.addWidget(live_badge)
        log_layout.addLayout(log_header)

        self.log_text = QTextEdit()
        self.log_text.setObjectName("log-text")
        self.log_text.setReadOnly(True)
        self.log_text.setFont(QFont("Fira Code", 13))
        log_layout.addWidget(self.log_text)

        self.chat_widget = ChatPanel()

        panels_splitter = QWidget()
        panels_layout = QHBoxLayout(panels_splitter)
        panels_layout.setContentsMargins(0, 0, 0, 0)
        panels_layout.setSpacing(8)
        panels_layout.addWidget(log_panel, 1)
        panels_layout.addWidget(self.chat_widget, 1)

        right_layout.addWidget(panels_splitter, 1)

        main_layout.addWidget(left_panel, 22)
        main_layout.addWidget(right_widget, 78)

        self.clock_timer = setup_clock_timer(self.clock_label)

    def log(self, message):
        append_log(self.log_text, message)
        
    def start_mapping(self):
        try:
            # Kill any leftover nav2/AMCL/map_server processes
            for proc in ['nav2', 'amcl', 'map_server', 'lifecycle_manager', 'MAP_NAVIGATION']:
                self.process_mgr.kill_by_pattern(proc)
            self.log("Đã dừng các tiến trình điều hướng cũ")

            self.mapping_process = self.process_mgr.launch_terminal(
                'source ~/zackon_build_up/install/setup.bash && '
                'ros2 launch view_robot_pkg MAP_GENERATING.launch.py; exec bash',
                'MAP_GENERATING'
            )
            self.log("✓ Đã khởi động MAP_GENERATING.launch.py")
            self.log("SLAM đang hoạt động")
            self.log("Điều khiển robot để khám phá môi trường")
            self.log("Nhập tên bản đồ và nhấn Áp dụng để lưu")
            self.btn_start.setEnabled(False)
            self.btn_start.setText("Mapping Active...")
        except Exception as e:
            self.log(f"[LỖI] Không thể bắt đầu lập bản đồ: {e}")
    
    def cancel_mapping(self):
        self.log("Đang hủy quá trình lập bản đồ SLAM...")
        self.process_mgr.kill_by_pattern('MAP_GENERATING.launch.py')
        self.process_mgr.kill_by_pattern('rviz2')
        self.log("Đã dừng MAP_GENERATING và RViz2")
        self.mapping_process = None
        self.btn_start.setEnabled(True)
        self.btn_start.setText("Bắt đầu lập bản đồ")
        self.log("Nút Bắt đầu đã được khôi phục")
    
    def save_map(self):
        map_name = self.map_name_input.text().strip()
        if not map_name:
            self.log("Lỗi: Vui lòng nhập tên bản đồ")
            return
        
        map_path = f"{SOURCE_PATH}/src/view_robot/maps/{map_name}"
        self.log(f"Đang lưu bản đồ '{map_name}' vào thư mục maps...")
        try:
            self.process_mgr.launch_terminal(
                f'source ~/zackon_build_up/install/setup.bash && '
                f'cd {SOURCE_PATH}/src/view_robot/maps && '
                f'ros2 run nav2_map_server map_saver_cli -f {map_name}; exec bash',
                'Map Saver'
            )
            self.log(f"Đã lưu bản đồ tại: {map_path}")
        except Exception as e:
            self.log(f"Không thể lưu bản đồ: {e}")
    
    def go_back(self):
        self.log("Quay về giao diện chính")
        self.process_mgr.cleanup_all()
        subprocess.Popen([sys.executable, f'{SOURCE_PATH}/robot_ui/startup_layout.py', '--skip-micro-ros'])
        self.close()
    
    def closeEvent(self, event):
        self.process_mgr.cleanup_all()
        self.chat_widget.cleanup()
        event.accept()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    app.setFont(QFont("Fira Sans", 12))
    window = NewMapUI()
    window.showMaximized()
    sys.exit(app.exec())
