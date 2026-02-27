#!/usr/bin/env python3
import sys
import subprocess
import os
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QPushButton, QTextEdit, QLabel)
from PyQt6.QtCore import QTimer, Qt
from PyQt6.QtGui import QFont

class RobotUI(QMainWindow):
    def __init__(self, skip_micro_ros=False):
        super().__init__()
        self.prev_stm32_status = None
        self.prev_lidar_status = None
        self.init_ui()
        if not skip_micro_ros:
            self.start_micro_ros()
        
    def init_ui(self):
        self.setWindowTitle("Robot Control Interface")
        self.showMaximized()
        
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QHBoxLayout(central)
        main_layout.setContentsMargins(0, 0, 0, 0)
        
        # Left area - Mode selector (1/4)
        mode_widget = QWidget()
        mode_widget.setStyleSheet("background-color: #f0f0f0;")
        mode_layout = QVBoxLayout(mode_widget)
        
        self.btn_manual = QPushButton("Manual")
        self.btn_tracking = QPushButton("Tracking")
        self.btn_waypoints = QPushButton("Waypoints")
        
        for btn in [self.btn_manual, self.btn_tracking, self.btn_waypoints]:
            btn.setFont(QFont("Fira Sans", 24))
            btn.setMinimumHeight(100)
            btn.clicked.connect(lambda checked, b=btn: self.mode_changed(b.text()))
            mode_layout.addWidget(btn)
        
        mode_layout.addStretch()
        
        # Right area (3/4) - split into status and logging
        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)
        right_layout.setContentsMargins(10, 10, 10, 10)
        
        # Status area (top half)
        status_widget = QWidget()
        status_layout = QVBoxLayout(status_widget)
        
        title = QLabel("Robot Status")
        title.setFont(QFont("Fira Sans", 28, QFont.Weight.Bold))
        status_layout.addWidget(title)
        
        self.stm32_label = QLabel("STM32: Checking...")
        self.stm32_label.setFont(QFont("Fira Sans", 20))
        status_layout.addWidget(self.stm32_label)
        
        self.lidar_label = QLabel("LiDAR: Checking...")
        self.lidar_label.setFont(QFont("Fira Sans", 20))
        status_layout.addWidget(self.lidar_label)
        
        status_layout.addStretch()
        
        # Logging area (bottom half)
        log_widget = QWidget()
        log_layout = QVBoxLayout(log_widget)
        
        log_title = QLabel("System Log")
        log_title.setFont(QFont("Fira Sans", 20, QFont.Weight.Bold))
        log_layout.addWidget(log_title)
        
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setFont(QFont("Fira Sans", 14))
        log_layout.addWidget(self.log_text)
        
        right_layout.addWidget(status_widget, 1)
        right_layout.addWidget(log_widget, 1)
        
        main_layout.addWidget(mode_widget, 1)
        main_layout.addWidget(right_widget, 3)
         
        # Timer for periodic updates (5 seconds)
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_status)
        self.timer.start(5000)
        
        self.update_status()
        
    def start_micro_ros(self):
        try:
            subprocess.Popen([
                'gnome-terminal', '--', 'bash', '-c',
                'source ~/thien_ws/install/setup.bash && ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888; exec bash'
            ])
            self.log("Started micro-ROS agent in new terminal")
        except Exception as e:
            self.log(f"Failed to start micro-ROS agent: {e}")
    
    def update_status(self):
        # Check STM32 (micro-ROS agent) via topic list
        try:
            result = subprocess.run(['ros2', 'topic', 'list'], 
                                  capture_output=True, text=True, timeout=2)
            stm32_ok = '/cmd_vel' in result.stdout or result.returncode == 0
        except:
            stm32_ok = False
            
        if stm32_ok:
            self.stm32_label.setText("STM32: OK")
            self.stm32_label.setStyleSheet("color: green;")
        else:
            self.stm32_label.setText("STM32: ERROR")
            self.stm32_label.setStyleSheet("color: red;")
        
        if self.prev_stm32_status is not None and self.prev_stm32_status != stm32_ok:
            if not stm32_ok:
                self.log("STM32 connection lost")
            else:
                self.log("STM32 connection restored")
        self.prev_stm32_status = stm32_ok
        
        # Check LiDAR
        lidar_ok = os.path.exists('/dev/lidar')
        if lidar_ok:
            self.lidar_label.setText("LiDAR: OK")
            self.lidar_label.setStyleSheet("color: green;")
        else:
            self.lidar_label.setText("LiDAR: ERROR")
            self.lidar_label.setStyleSheet("color: red;")
        
        if self.prev_lidar_status is not None and self.prev_lidar_status != lidar_ok:
            if not lidar_ok:
                self.log("LiDAR connection lost")
            else:
                self.log("LiDAR connection restored")
        self.prev_lidar_status = lidar_ok
    
    def mode_changed(self, mode):
        self.log(f"Mode changed to {mode}")
        if mode == "Manual":
            subprocess.Popen(['python3', '/home/khoaiuh/thien_ws/robot_ui/manual_mode_layout.py'])
            self.close()
        elif mode == "Tracking":
            subprocess.Popen(['python3', '/home/khoaiuh/thien_ws/robot_ui/tracking_mode_layout.py'])
            self.close()
        elif mode == "Waypoints":
            subprocess.Popen(['python3', '/home/khoaiuh/thien_ws/robot_ui/waypoints_mode_layout.py'])
            self.close()
    
    def log(self, message):
        self.log_text.append(message)
    
    def closeEvent(self, event):
        event.accept()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    app.setFont(QFont("Fira Sans", 12))
    skip_micro_ros = '--skip-micro-ros' in sys.argv
    window = RobotUI(skip_micro_ros=skip_micro_ros)
    sys.exit(app.exec())
