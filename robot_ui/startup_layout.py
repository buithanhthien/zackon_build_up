#!/usr/bin/env python3
import sys
import subprocess
import os
import threading
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QPushButton, QTextEdit, QLabel)
from PyQt6.QtCore import QTimer, Qt, pyqtSignal, QObject
from PyQt6.QtGui import QFont
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from std_srvs.srv import Empty


class LocalizationWorker(QObject):
    log_signal = pyqtSignal(str)
    finished_signal = pyqtSignal()
    
    def __init__(self):
        super().__init__()
        self.running = False
        self.stable_count = 0
        self.current_pose = None
        
    def run(self):
        try:
            rclpy.init()
        except:
            pass
            
        node = Node('localization_worker')
        
        cmd_vel_pub = node.create_publisher(Twist, '/cmd_vel', 10)
        pose_sub = node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)
        
        global_loc_client = node.create_client(Empty, '/global_localization')
        
        self.log_signal.emit("Calling /global_localization")
        if not global_loc_client.wait_for_service(timeout_sec=5.0):
            self.log_signal.emit("Global localization service not available")
            node.destroy_node()
            self.finished_signal.emit()
            return
            
        future = global_loc_client.call_async(Empty.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        
        self.log_signal.emit("Rotation started")
        self.log_signal.emit("Monitoring AMCL covariance")
        
        twist = Twist()
        twist.angular.z = 0.3
        
        rate = node.create_rate(10)
        timeout = 60
        elapsed = 0
        
        self.running = True
        while self.running and elapsed < timeout:
            cmd_vel_pub.publish(twist)
            rclpy.spin_once(node, timeout_sec=0.01)
            
            if self.is_stable():
                self.log_signal.emit("Localization stable")
                break
                
            elapsed += 0.1
            
        twist.angular.z = 0.0
        cmd_vel_pub.publish(twist)
        
        if elapsed >= timeout:
            self.log_signal.emit("Localization timeout")
        else:
            self.log_signal.emit("Stopping rotation")
            self.log_signal.emit("Localization completed")
            
        node.destroy_node()
        self.finished_signal.emit()
        
    def pose_callback(self, msg):
        self.current_pose = msg
        cov = msg.pose.covariance
        
        if cov[0] < 0.05 and cov[7] < 0.05 and cov[35] < 0.1:
            self.stable_count += 1
        else:
            self.stable_count = 0
            
    def is_stable(self):
        return self.stable_count >= 3
        
    def stop(self):
        self.running = False


class RobotUI(QMainWindow):
    def __init__(self, skip_micro_ros=False):
        super().__init__()
        self.prev_stm32_status = None
        self.prev_lidar_status = None
        self.localization_worker = None
        self.localization_thread = None
        self.init_ui()
        if not skip_micro_ros:
            self.start_micro_ros()
        
    def init_ui(self):
        self.setWindowTitle("Robot Control Interface")
        self.setStyleSheet("background-color: white; color: black;")
        
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
        self.btn_nav2 = QPushButton("Nav2")
        self.btn_reestimate = QPushButton("Re-estimate")
        
        for btn in [self.btn_manual, self.btn_tracking, self.btn_waypoints, self.btn_nav2, self.btn_reestimate]:
            btn.setFont(QFont("Fira Sans", 24))
            btn.setMinimumHeight(100)
            if btn != self.btn_reestimate:
                btn.clicked.connect(lambda checked, b=btn: self.mode_changed(b.text()))
            mode_layout.addWidget(btn)
        
        self.btn_reestimate.clicked.connect(self.start_reestimate)
        
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
        elif mode == "Nav2":
            try:
                subprocess.Popen([
                    'gnome-terminal', '--', 'bash', '-c',
                    'source ~/thien_ws/install/setup.bash && ros2 launch view_robot_pkg zackon_synthesis.launch.py; exec bash'
                ])
                self.log("Launched Nav2 navigation system")
            except Exception as e:
                self.log(f"Failed to launch Nav2: {e}")
    
    def start_reestimate(self):
        if self.prev_stm32_status == False:
            self.log("Cannot start localization: STM32 not connected")
            return
            
        if self.localization_thread and self.localization_thread.is_alive():
            self.log("Localization already running")
            return
            
        self.log("Starting adaptive localization")
        
        self.localization_worker = LocalizationWorker()
        self.localization_worker.log_signal.connect(self.log)
        self.localization_worker.finished_signal.connect(self.localization_finished)
        
        self.localization_thread = threading.Thread(target=self.localization_worker.run)
        self.localization_thread.start()
        
    def localization_finished(self):
        self.localization_thread = None
        self.localization_worker = None
    
    def log(self, message):
        self.log_text.append(message)
    
    def closeEvent(self, event):
        event.accept()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    app.setFont(QFont("Fira Sans", 12))
    skip_micro_ros = '--skip-micro-ros' in sys.argv
    window = RobotUI(skip_micro_ros=skip_micro_ros)
    window.show()
    window.setWindowState(Qt.WindowState.WindowMaximized)
    sys.exit(app.exec())
