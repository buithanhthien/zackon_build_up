#!/usr/bin/env python3
import sys
import subprocess
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
                             QPushButton, QTextEdit, QLabel, QLineEdit)
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QFont
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from config import SOURCE_PATH

STYLESHEET = """
    QMainWindow, QWidget {
        background-color: #0d0f12;
        color: #e8ecf0;
        border: none;
    }
    QWidget#left-panel {
        background-color: #141720;
        border-right: 2px solid #2a3040;
    }
    QWidget#header-bar {
        background-color: #141720;
        border-bottom: 1px solid #2a3040;
    }
    QWidget#log-panel {
        background-color: #080a0d;
        border-top: 1px solid #2a3040;
    }
    QWidget#content-panel {
        background-color: #0d0f12;
    }
    QPushButton#action-btn {
        background-color: transparent;
        color: #6b7a99;
        border: none;
        border-left: 4px solid transparent;
        border-radius: 0px;
        padding: 16px 20px 16px 24px;
        text-align: left;
        font-size: 18px;
    }
    QPushButton#action-btn:hover {
        background-color: #1a1f2e;
        color: #e8ecf0;
        border-left: 4px solid #3a4460;
    }
    QPushButton#action-btn:disabled {
        color: #3a4460;
        border-left: 4px solid transparent;
    }
    QPushButton#primary-btn {
        background-color: #1c2030;
        color: #00e5ff;
        border: 1px solid #00e5ff;
        border-radius: 4px;
        font-size: 18px;
        min-height: 56px;
        padding: 0px 24px;
    }
    QPushButton#primary-btn:hover {
        background-color: #1a2a3a;
    }
    QPushButton#primary-btn:disabled {
        color: #3a4460;
        border: 1px solid #3a4460;
    }
    QPushButton#apply-btn {
        background-color: #1c2030;
        color: #00c853;
        border: 1px solid #00c853;
        border-radius: 4px;
        font-size: 16px;
        min-height: 48px;
        padding: 0px 20px;
    }
    QPushButton#apply-btn:hover {
        background-color: #0d1f14;
    }
    QLineEdit#map-input {
        background-color: #1c2030;
        color: #e8ecf0;
        border: 1px solid #2a3040;
        border-radius: 4px;
        font-size: 16px;
        min-height: 48px;
        padding: 0px 12px;
    }
    QLineEdit#map-input:focus {
        border: 1px solid #3a4460;
    }
    QTextEdit#log-text {
        background-color: #080a0d;
        color: #e8ecf0;
        border: none;
        font-size: 13px;
    }
    QLabel#log-title {
        color: #6b7a99;
        font-size: 11px;
        letter-spacing: 2px;
    }
    QLabel#clock {
        color: #6b7a99;
        font-size: 15px;
    }
    QLabel#section-label {
        color: #6b7a99;
        font-size: 11px;
        letter-spacing: 2px;
    }
    QLabel#info-text {
        color: #6b7a99;
        font-size: 14px;
    }
"""


class NewMapUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.mapping_process = None
        self.init_ui()

    def init_ui(self):
        self.setWindowTitle("New Map - SLAM Mapping")
        self.setStyleSheet(STYLESHEET)

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

        wordmark = QLabel("NEW MAP")
        wordmark.setFont(QFont("JetBrains Mono", 14, QFont.Weight.Bold))
        wordmark.setStyleSheet("color: #00e5ff; padding: 24px 24px 16px 24px;")
        left_layout.addWidget(wordmark)

        mono = QFont("JetBrains Mono", 18)
        self.btn_cancel = QPushButton("Cancel")
        self.btn_back   = QPushButton("Back")

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
        header_title.setStyleSheet("color: #e8ecf0;")

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

        info = QLabel("Start SLAM Toolbox to generate a new map. Drive the robot to explore the environment, then save.")
        info.setObjectName("info-text")
        info.setFont(QFont("DM Sans", 14))
        info.setWordWrap(True)
        content_layout.addWidget(info)

        self.btn_start = QPushButton("Start Mapping")
        self.btn_start.setObjectName("primary-btn")
        self.btn_start.setFont(QFont("JetBrains Mono", 18))
        self.btn_start.clicked.connect(self.start_mapping)
        content_layout.addWidget(self.btn_start)

        # Save map row
        save_label = QLabel("SAVE MAP")
        save_label.setObjectName("section-label")
        save_label.setFont(QFont("DM Sans", 11))
        content_layout.addWidget(save_label)

        save_row = QHBoxLayout()
        self.map_name_input = QLineEdit()
        self.map_name_input.setObjectName("map-input")
        self.map_name_input.setPlaceholderText("Enter map name...")
        self.map_name_input.setFont(QFont("JetBrains Mono", 16))
        save_row.addWidget(self.map_name_input)

        self.btn_apply = QPushButton("Apply")
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
        live_badge.setStyleSheet("color: #00c853; font-size: 11px;")
        log_header.addWidget(log_title)
        log_header.addStretch()
        log_header.addWidget(live_badge)
        log_layout.addLayout(log_header)

        self.log_text = QTextEdit()
        self.log_text.setObjectName("log-text")
        self.log_text.setReadOnly(True)
        self.log_text.setFont(QFont("Fira Code", 13))
        log_layout.addWidget(self.log_text)

        right_layout.addWidget(log_panel, 1)

        main_layout.addWidget(left_panel, 22)
        main_layout.addWidget(right_widget, 78)

        self.clock_timer = QTimer()
        self.clock_timer.timeout.connect(self._update_clock)
        self.clock_timer.start(1000)
        self._update_clock()

    def _update_clock(self):
        from datetime import datetime
        self.clock_label.setText(datetime.now().strftime("%H:%M:%S"))
        
    def start_mapping(self):
        try:
            self.mapping_process = subprocess.Popen([
                'gnome-terminal', '--', 'bash', '-c',
                'source ~/zackon_build_up/install/setup.bash && ros2 launch view_robot_pkg MAP_GENERATING.launch.py; exec bash'
            ])
            self.log("✓ Started MAP_GENERATING.launch.py")
            self.log("SLAM mapping is now active")
            self.log("Drive the robot to explore the environment")
            self.log("Enter a map name and click Apply to save")
            self.btn_start.setEnabled(False)
            self.btn_start.setText("Mapping Active...")
        except Exception as e:
            self.log(f"[ERROR] Failed to start mapping: {e}")
    
    def cancel_mapping(self):
        self.log("Cancelling SLAM mapping process...")
        try:
            subprocess.run(['pkill', '-f', 'MAP_GENERATING.launch.py'], check=False)
            subprocess.run(['pkill', '-f', 'rviz2'], check=False)
            self.log("Killed MAP_GENERATING and RViz2 processes")
        except Exception as e:
            self.log(f"Error killing processes: {e}")
        finally:
            self.mapping_process = None
            self.btn_start.setEnabled(True)
            self.btn_start.setText("Start")
            self.log("Start button restored to normal state")
    
    def save_map(self):
        map_name = self.map_name_input.text().strip()
        if not map_name:
            self.log("Error: Please enter a map name")
            return
        
        map_path = f"{SOURCE_PATH}/src/view_robot/maps/{map_name}"
        self.log(f"Saving map as '{map_name}' to maps folder...")
        try:
            subprocess.Popen([
                'gnome-terminal', '--', 'bash', '-c',
                f'source ~/zackon_build_up/install/setup.bash && cd {SOURCE_PATH}/src/view_robot/maps && ros2 run nav2_map_server map_saver_cli -f {map_name}; exec bash'
            ])
            self.log(f"Map saved to: {map_path}")
        except Exception as e:
            self.log(f"Failed to save map: {e}")
    
    def go_back(self):
        self.log("Returning to main UI")
        if self.mapping_process:
            self.log("Stopping SLAM mapping process...")
            try:
                self.mapping_process.terminate()
                self.mapping_process.wait(timeout=3)
                self.log("SLAM process terminated")
            except:
                self.mapping_process.kill()
                self.log("SLAM process killed")
        subprocess.Popen(['python3', f'{SOURCE_PATH}/robot_ui/startup_layout.py', '--skip-micro-ros'])
        self.close()
    
    def log(self, message):
        from datetime import datetime
        ts = datetime.now().strftime("%H:%M:%S")
        if "[ERROR]" in message:
            color = "#ff3b3b"
        elif "[WARN]" in message:
            color = "#ffb300"
        elif "✓" in message:
            color = "#00c853"
        else:
            color = "#e8ecf0"
        self.log_text.append(
            f'<span style="color:#6b7a99">[{ts}]</span> <span style="color:{color}">{message}</span>'
        )
    
    def closeEvent(self, event):
        if self.mapping_process:
            try:
                self.mapping_process.terminate()
            except:
                pass
        event.accept()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    app.setFont(QFont("Fira Sans", 12))
    window = NewMapUI()
    window.showMaximized()
    sys.exit(app.exec())
