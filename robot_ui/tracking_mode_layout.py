#!/usr/bin/env python3
import sys
import subprocess
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QPushButton, QTextEdit, QLabel)
from PyQt6.QtCore import QTimer, Qt
from PyQt6.QtGui import QFont

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
        
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QHBoxLayout(central)
        main_layout.setContentsMargins(0, 0, 0, 0)
        
        # Left area - Mode selector (1/4)
        mode_widget = QWidget()
        mode_widget.setStyleSheet("background-color: #f0f0f0;")
        mode_layout = QVBoxLayout(mode_widget)
        
        self.btn_back = QPushButton("Back")
        self.btn_back.setFont(QFont("Fira Sans", 24))
        self.btn_back.setMinimumHeight(100)
        self.btn_back.clicked.connect(self.go_back)
        mode_layout.addWidget(self.btn_back)
        mode_layout.addStretch()
        
        # Right area (3/4) - split into position and logging
        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)
        right_layout.setContentsMargins(10, 10, 10, 10)
        
        # Position area (top half)
        pos_widget = QWidget()
        pos_layout = QVBoxLayout(pos_widget)
        
        title = QLabel("Current Position")
        title.setFont(QFont("Fira Sans", 28, QFont.Weight.Bold))
        pos_layout.addWidget(title)
        
        self.pos_label = QLabel("Position:\n  x: 0.0\n  y: 0.0\n  z: 0.0\n\nOrientation:\n  x: 0.0\n  y: 0.0\n  z: 0.0\n  w: 1.0")
        self.pos_label.setFont(QFont("Fira Sans", 18))
        pos_layout.addWidget(self.pos_label)
        pos_layout.addStretch()
        
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
        
        right_layout.addWidget(pos_widget, 1)
        right_layout.addWidget(log_widget, 1)
        
        main_layout.addWidget(mode_widget, 1)
        main_layout.addWidget(right_widget, 3)
        
    def init_ros(self):
        rclpy.init()
        self.ros_node = Node('tracking_ui_node')
        self.odom_sub = self.ros_node.create_subscription(
            Odometry, '/odomfromSTM32', self.odom_callback, 10)
        
        self.ros_timer = QTimer()
        self.ros_timer.timeout.connect(lambda: rclpy.spin_once(self.ros_node, timeout_sec=0))
        self.ros_timer.start(50)
        
        self.log("Mode changed to tracking")
        
    def odom_callback(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        text = f"Position:\n  x: {pos.x:.3f}\n  y: {pos.y:.3f}\n  z: {pos.z:.3f}\n\n"
        text += f"Orientation:\n  x: {ori.x:.3f}\n  y: {ori.y:.3f}\n  z: {ori.z:.3f}\n  w: {ori.w:.3f}"
        self.pos_label.setText(text)
        
    def launch_tracking(self):
        try:
            self.launch_process = subprocess.Popen([
                'gnome-terminal', '--', 'bash', '-c',
                'source /home/khoaiuh/thien_ws/install/setup.bash && ros2 launch /home/khoaiuh/thien_ws/src/human_following/launch/system.launch.py; exec bash'
            ])
            self.log("Launched tracking system")
        except Exception as e:
            self.log(f"Failed to launch tracking: {e}")
    
    def go_back(self):
        if self.launch_process:
            subprocess.run(['pkill', '-f', 'system.launch.py'])
            self.launch_process.terminate()
            self.log("Stopped tracking system")
        subprocess.Popen(['python3', '/home/khoaiuh/thien_ws/robot_ui/startup_layout.py', '--skip-micro-ros'])
        self.close()
        
    def log(self, message):
        self.log_text.append(message)
    
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
