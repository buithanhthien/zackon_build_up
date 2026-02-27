#!/usr/bin/env python3
import sys
import subprocess
import os
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QPushButton, QTextEdit, QLabel)
from PyQt6.QtCore import QTimer
from PyQt6.QtGui import QFont

os.environ['ROS_DOMAIN_ID'] = '0'

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

class ManualModeUI(QMainWindow):
    def __init__(self):
        super().__init__()
        try:
            rclpy.init()
        except:
            pass
        self.node = Node('manual_mode_ui')
        self.odom_sub = self.node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)
        self.init_ui()
        
    def init_ui(self):
        self.setWindowTitle("Manual Mode")
        self.setStyleSheet("background-color: white; color: black;")
        
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QHBoxLayout(central)
        
        # Left area (1/4)
        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)
        
        btn_back = QPushButton("Back")
        btn_back.setFont(QFont("Fira Sans", 24))
        btn_back.clicked.connect(self.go_back)
        left_layout.addWidget(btn_back)
        left_layout.addStretch()
        
        # Right area (3/4)
        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)
        
        # Current position (top half)
        pos_widget = QWidget()
        pos_layout = QVBoxLayout(pos_widget)
        
        title = QLabel("Current Position")
        title.setFont(QFont("Fira Sans", 28))
        pos_layout.addWidget(title)
        
        self.pos_label = QLabel("position:\n    x: 0.00\n    y: 0.00\n    z: 0.00")
        self.pos_label.setFont(QFont("Fira Sans", 20))
        pos_layout.addWidget(self.pos_label)
        
        self.orient_label = QLabel("orientation:\n    x: 0.00\n    y: 0.00\n    z: 0.00\n    w: 1.00")
        self.orient_label.setFont(QFont("Fira Sans", 20))
        pos_layout.addWidget(self.orient_label)
        
        pos_layout.addStretch()
        
        # Logging area (bottom half)
        log_widget = QWidget()
        log_layout = QVBoxLayout(log_widget)
        
        log_title = QLabel("System Log")
        log_title.setFont(QFont("Fira Sans", 20))
        log_layout.addWidget(log_title)
        
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setFont(QFont("Fira Sans", 14))
        log_layout.addWidget(self.log_text)
        
        right_layout.addWidget(pos_widget, 1)
        right_layout.addWidget(log_widget, 1)
        
        main_layout.addWidget(left_widget, 1)
        main_layout.addWidget(right_widget, 3)
        
        self.timer = QTimer()
        self.timer.timeout.connect(self.spin_ros)
        self.timer.start(50)
        
        self.log("Mode changed to manual")
        
    def pose_callback(self, msg):
        self.pos_label.setText(f"position:\n    x: {msg.pose.pose.position.x:.2f}\n    y: {msg.pose.pose.position.y:.2f}\n    z: {msg.pose.pose.position.z:.2f}")
        self.orient_label.setText(f"orientation:\n    x: {msg.pose.pose.orientation.x:.2f}\n    y: {msg.pose.pose.orientation.y:.2f}\n    z: {msg.pose.pose.orientation.z:.2f}\n    w: {msg.pose.pose.orientation.w:.2f}")
    
    def spin_ros(self):
        rclpy.spin_once(self.node, timeout_sec=0)
    
    def go_back(self):
        self.log("Returning to startup layout")
        subprocess.Popen(['python3', '/home/khoaiuh/thien_ws/robot_ui/startup_layout.py', '--skip-micro-ros'])
        self.close()
    
    def log(self, message):
        self.log_text.append(message)
    
    def closeEvent(self, event):
        self.node.destroy_node()
        event.accept()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = ManualModeUI()
    window.showMaximized()
    sys.exit(app.exec())
