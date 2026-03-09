#!/usr/bin/env python3
import sys
import subprocess
import os
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from std_msgs.msg import String
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QPushButton, QTextEdit, QLabel)
from PyQt6.QtCore import QTimer, Qt
from PyQt6.QtGui import QFont, QPixmap, QPainter, QPen, QColor


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
        
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QHBoxLayout(central)
        main_layout.setContentsMargins(0, 0, 0, 0)
        
        mode_widget = QWidget()
        mode_widget.setStyleSheet("background-color: #f0f0f0;")
        mode_layout = QVBoxLayout(mode_widget)
        
        self.btn_back = QPushButton("Back")
        self.btn_back.setFont(QFont("Fira Sans", 24))
        self.btn_back.setMinimumHeight(100)
        self.btn_back.clicked.connect(self.go_back)
        mode_layout.addWidget(self.btn_back)
        
        pos_title = QLabel("Current Position")
        pos_title.setFont(QFont("Fira Sans", 18))
        mode_layout.addWidget(pos_title)
        
        self.pos_label = QLabel("x: 0.0\ny: 0.0\nz: 0.0\n\nqx: 0.0\nqy: 0.0\nqz: 0.0\nqw: 1.0")
        self.pos_label.setFont(QFont("Fira Sans", 14))
        mode_layout.addWidget(self.pos_label)
        
        mode_layout.addStretch()
        
        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)
        right_layout.setContentsMargins(10, 10, 10, 10)
        
        map_yaml_path = '/home/khoaiuh/thien_ws/src/view_robot/maps/F5.yaml'
        map_dir = os.path.dirname(map_yaml_path)
        yaml_data = self.load_map_yaml(map_yaml_path)
        map_image_path = os.path.join(map_dir, yaml_data['image'])
        
        self.map_widget = MapWidget(map_image_path, yaml_data)
        right_layout.addWidget(self.map_widget, 2)
        
        log_widget = QWidget()
        log_layout = QVBoxLayout(log_widget)
        
        log_title = QLabel("System Log")
        log_title.setFont(QFont("Fira Sans", 20, QFont.Weight.Bold))
        log_layout.addWidget(log_title)
        
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setFont(QFont("Fira Sans", 14))
        log_layout.addWidget(self.log_text)
        
        right_layout.addWidget(log_widget, 1)
        
        main_layout.addWidget(mode_widget, 1)
        main_layout.addWidget(right_widget, 3)
        
    def init_ros(self):
        rclpy.init()
        self.ros_node = Node('tracking_ui_node')
        self.odom_sub = self.ros_node.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)
        self.lock_sub = self.ros_node.create_subscription(
            String, '/human_lock_status', self.lock_callback, 10)
        self.cmd_vel_sub = self.ros_node.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.fps_sub = self.ros_node.create_subscription(
            String, '/tracking_fps', self.fps_callback, 10)
        
        self.ros_timer = QTimer()
        self.ros_timer.timeout.connect(lambda: rclpy.spin_once(self.ros_node, timeout_sec=0))
        self.ros_timer.start(50)
        
        self.log("Mode changed to tracking")
    
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
        text = f"x: {pos.x:.2f}\ny: {pos.y:.2f}\nz: {pos.z:.2f}\n\n"
        text += f"qx: {ori.x:.2f}\nqy: {ori.y:.2f}\nqz: {ori.z:.2f}\nqw: {ori.w:.2f}"
        self.pos_label.setText(text)
        self.map_widget.set_robot_pose(msg)
        
    def lock_callback(self, msg):
        self.log(msg.data)
    
    def cmd_vel_callback(self, msg):
        if msg.linear.x != 0.0 or msg.angular.z != 0.0:
            self.log(f"CMD_VEL → STM32: linear={msg.linear.x:.2f} m/s, angular={msg.angular.z:.2f} rad/s")
    
    def fps_callback(self, msg):
        self.log(msg.data)
        
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
