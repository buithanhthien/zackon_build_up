#!/usr/bin/env python3
import sys
import subprocess
import os
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QPushButton, QTextEdit, QLabel)
from PyQt6.QtCore import QTimer, Qt
from PyQt6.QtGui import QFont, QPixmap, QPainter, QPen, QColor
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
    QLabel#section-title {
        color: #6b7a99;
        font-size: 11px;
        letter-spacing: 2px;
        padding: 12px 24px 4px 24px;
    }
    QLabel#pos-value {
        color: #e8ecf0;
        font-size: 13px;
        padding: 0px 24px;
    }
"""


class MapWidget(QWidget):
    def __init__(self, map_path, yaml_data):
        super().__init__()
        self.map_image = QPixmap(map_path)
        self.resolution = yaml_data['resolution']
        self.origin = yaml_data['origin']
        self.robot_pose = None
        self.setMinimumSize(400, 400)
        
    def set_robot_pose(self, pose):
        self.robot_pose = pose
        self.update()
        
    def world_to_pixel(self, x, y):
        px = int((x - self.origin[0]) / self.resolution)
        py = int((self.origin[1] - y) / self.resolution + self.map_image.height())
        return px, py
        
    def paintEvent(self, event):
        painter = QPainter(self)
        
        scaled_map = self.map_image.scaled(self.size(), Qt.AspectRatioMode.KeepAspectRatio, Qt.TransformationMode.SmoothTransformation)
        x_offset = (self.width() - scaled_map.width()) // 2
        y_offset = (self.height() - scaled_map.height()) // 2
        painter.drawPixmap(x_offset, y_offset, scaled_map)
        
        if self.robot_pose:
            scale_x = scaled_map.width() / self.map_image.width()
            scale_y = scaled_map.height() / self.map_image.height()
            
            px, py = self.world_to_pixel(
                self.robot_pose.pose.pose.position.x,
                self.robot_pose.pose.pose.position.y
            )
            
            px = int(px * scale_x + x_offset)
            py = int(py * scale_y + y_offset)
            
            painter.setPen(QPen(QColor(255, 0, 0), 3))
            painter.setBrush(QColor(255, 0, 0))
            painter.drawEllipse(px - 5, py - 5, 10, 10)


class TrackingModeUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.launch_process = None
        self.ros_node = None
        self.init_ui()
        self.init_ros()
        self.launch_tracking()
        
    def init_ui(self):
        self.setWindowTitle("Tracking Mode")
        self.showMaximized()
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

        wordmark = QLabel("TRACKING")
        wordmark.setFont(QFont("JetBrains Mono", 14, QFont.Weight.Bold))
        wordmark.setStyleSheet("color: #00e5ff; padding: 24px 24px 16px 24px;")
        left_layout.addWidget(wordmark)

        self.btn_back = QPushButton("Back")
        self.btn_back.setObjectName("action-btn")
        self.btn_back.setFont(QFont("JetBrains Mono", 18))
        self.btn_back.setMinimumHeight(72)
        self.btn_back.clicked.connect(self.go_back)
        left_layout.addWidget(self.btn_back)

        pos_title = QLabel("POSITION")
        pos_title.setObjectName("section-title")
        pos_title.setFont(QFont("DM Sans", 11))
        left_layout.addWidget(pos_title)

        self.pos_label = QLabel("x: 0.00   y: 0.00   z: 0.00")
        self.pos_label.setObjectName("pos-value")
        self.pos_label.setFont(QFont("JetBrains Mono", 13))
        left_layout.addWidget(self.pos_label)

        orient_title = QLabel("ORIENTATION")
        orient_title.setObjectName("section-title")
        orient_title.setFont(QFont("DM Sans", 11))
        left_layout.addWidget(orient_title)

        self.orient_label = QLabel("qx: 0.00   qy: 0.00\nqz: 0.00   qw: 1.00")
        self.orient_label.setObjectName("pos-value")
        self.orient_label.setFont(QFont("JetBrains Mono", 13))
        left_layout.addWidget(self.orient_label)

        left_layout.addStretch()

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

        header_title = QLabel("TRACKING MODE")
        header_title.setFont(QFont("JetBrains Mono", 15, QFont.Weight.Bold))
        header_title.setStyleSheet("color: #e8ecf0;")

        self.clock_label = QLabel()
        self.clock_label.setObjectName("clock")
        self.clock_label.setFont(QFont("JetBrains Mono", 15))

        header_layout.addWidget(header_title)
        header_layout.addStretch()
        header_layout.addWidget(self.clock_label)
        right_layout.addWidget(header)

        # Map
        map_container = QWidget()
        map_container.setStyleSheet("background-color: #0d0f12; padding: 12px;")
        map_layout = QVBoxLayout(map_container)
        map_layout.setContentsMargins(12, 12, 12, 12)

        map_yaml_path = self.get_current_map_path()
        map_dir = os.path.dirname(map_yaml_path)
        yaml_data = self.load_map_yaml(map_yaml_path)
        map_image_path = os.path.join(map_dir, yaml_data['image'])

        self.map_widget = MapWidget(map_image_path, yaml_data)
        map_layout.addWidget(self.map_widget)
        right_layout.addWidget(map_container, 2)

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
        
    def init_ros(self):
        rclpy.init()
        self.ros_node = Node('tracking_ui_node')
        self.odom_sub = self.ros_node.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)
        self.lock_sub = self.ros_node.create_subscription(
            String, '/human_lock_status', self.lock_callback, 10)
        
        self.ros_timer = QTimer()
        self.ros_timer.timeout.connect(lambda: rclpy.spin_once(self.ros_node, timeout_sec=0))
        self.ros_timer.start(50)
        
        self.log("Mode changed to tracking")
    
    def get_current_map_path(self):
        nav2_params = f'{SOURCE_PATH}/src/view_robot/config/nav2_params.yaml'
        try:
            with open(nav2_params, 'r') as f:
                for line in f:
                    if 'yaml_filename:' in line and '#' not in line:
                        path = line.split(':', 1)[1].strip().strip('"')
                        return f'{SOURCE_PATH}/src/view_robot/maps/{os.path.basename(path)}'
        except Exception:
            pass
        return f'{SOURCE_PATH}/src/view_robot/maps/X5_19032026.yaml'

    def load_map_yaml(self, yaml_path):
        data = {}
        with open(yaml_path, 'r') as f:
            for line in f:
                if ':' in line:
                    key, value = line.split(':', 1)
                    key = key.strip()
                    value = value.strip()
                    if key == 'origin':
                        data[key] = eval(value)
                    elif key == 'resolution':
                        data[key] = float(value)
                    else:
                        data[key] = value
        return data
        
    def pose_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        self.pos_label.setText(f"x: {pos.x:.2f}   y: {pos.y:.2f}   z: {pos.z:.2f}")
        self.orient_label.setText(f"qx: {ori.x:.2f}   qy: {ori.y:.2f}\nqz: {ori.z:.2f}   qw: {ori.w:.2f}")
        self.map_widget.set_robot_pose(msg)
        
    def lock_callback(self, msg):
        self.log(msg.data)
        
    def launch_tracking(self):
        try:
            self.launch_process = subprocess.Popen([
                'gnome-terminal', '--', 'bash', '-c',
                f'source {SOURCE_PATH}/install/setup.bash && ros2 launch {SOURCE_PATH}/src/human_following/launch/system.launch.py; exec bash'
            ])
            self.log("Launched tracking system")
        except Exception as e:
            self.log(f"Failed to launch tracking: {e}")
    
    def go_back(self):
        if self.launch_process:
            subprocess.run(['pkill', '-f', 'system.launch.py'])
            self.launch_process.terminate()
            self.log("Stopped tracking system")
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
        if self.launch_process:
            subprocess.run(['pkill', '-f', 'system.launch.py'])
            self.launch_process.terminate()
        if self.ros_node:
            self.ros_node.destroy_node()
            rclpy.shutdown()
        event.accept()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    app.setFont(QFont("Fira Sans", 12))
    window = TrackingModeUI()
    sys.exit(app.exec())
