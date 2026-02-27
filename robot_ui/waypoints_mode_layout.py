#!/usr/bin/env python3
import sys
import json
import subprocess
from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QPushButton, 
                              QLabel, QGridLayout, QTextEdit, QApplication, QMainWindow)
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QFont
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Twist


class WaypointsNode(Node):
    def __init__(self):
        super().__init__('waypoints_node')
        self.subscription = self.create_subscription(Odometry, '/odomfromSTM32', self.odom_callback, 10)
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.current_odom = None
        
    def odom_callback(self, msg):
        self.current_odom = msg
        
    def stop_robot(self):
        stop_msg = Twist()
        self.cmd_vel_pub.publish(stop_msg)


class WaypointsModeLayout(QMainWindow):
    def __init__(self):
        super().__init__()
        try:
            rclpy.init()
        except:
            pass
        self.ros_node = WaypointsNode()
        self.waypoints_file = '/home/khoaiuh/thien_ws/robot_ui/waypoints.json'
        self.waypoints = self.load_waypoints()
        self.current_mode = None
        self.zackon_process = None
        self.init_ui()
        
        self.timer = QTimer()
        self.timer.timeout.connect(self.spin_and_update)
        self.timer.start(50)
        
    def init_ui(self):
        self.setWindowTitle('Waypoints Mode')
        self.showMaximized()
        self.setStyleSheet("background-color: white; color: black;")
        
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QHBoxLayout(central)
        
        # Left area (1/4)
        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)
        
        self.save_btn = QPushButton('Save')
        self.clear_btn = QPushButton('Clear')
        self.stop_btn = QPushButton('Stop')
        self.nav2_btn = QPushButton('Nav2')
        self.back_btn = QPushButton('Back')
        
        for btn in [self.save_btn, self.clear_btn, self.stop_btn, self.nav2_btn, self.back_btn]:
            btn.setFont(QFont('Fira Sans', 24))
            left_layout.addWidget(btn)
        
        left_layout.addStretch()
        
        self.save_btn.clicked.connect(lambda: self.set_mode('save'))
        self.clear_btn.clicked.connect(lambda: self.set_mode('clear'))
        self.stop_btn.clicked.connect(self.stop_navigation)
        self.nav2_btn.clicked.connect(self.launch_zackon)
        self.back_btn.clicked.connect(self.go_back)
        
        # Right area (3/4)
        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)
        
        # Top half - Current position
        position_widget = QWidget()
        position_layout = QVBoxLayout(position_widget)
        
        pos_title = QLabel('Current Position')
        pos_title.setFont(QFont('Fira Sans', 28))
        position_layout.addWidget(pos_title)
        
        self.pos_label = QLabel('position:\n    x: 0.00\n    y: 0.00\n    z: 0.00')
        self.pos_label.setFont(QFont('Fira Sans', 20))
        position_layout.addWidget(self.pos_label)
        
        self.orient_label = QLabel('orientation:\n    x: 0.00\n    y: 0.00\n    z: 0.00\n    w: 1.00')
        self.orient_label.setFont(QFont('Fira Sans', 20))
        position_layout.addWidget(self.orient_label)
        
        # Waypoint buttons grid
        grid_layout = QGridLayout()
        self.waypoint_btns = []
        
        for i in range(10):
            btn = QPushButton(str(i + 1))
            btn.setFont(QFont('Fira Sans', 18))
            btn.clicked.connect(lambda checked, idx=i: self.waypoint_clicked(idx))
            grid_layout.addWidget(btn, i // 5, i % 5)
            self.waypoint_btns.append(btn)
        
        position_layout.addLayout(grid_layout)
        position_layout.addStretch()
        
        # Bottom half - Logging
        log_widget = QWidget()
        log_layout = QVBoxLayout(log_widget)
        
        log_title = QLabel('Logging')
        log_title.setFont(QFont('Fira Sans', 20))
        log_layout.addWidget(log_title)
        
        self.log_text = QTextEdit()
        self.log_text.setFont(QFont('Fira Sans', 14))
        self.log_text.setReadOnly(True)
        log_layout.addWidget(self.log_text)
        
        right_layout.addWidget(position_widget, 1)
        right_layout.addWidget(log_widget, 1)
        
        main_layout.addWidget(left_widget, 1)
        main_layout.addWidget(right_widget, 3)
        
        self.update_button_colors()
        self.log("Mode changed to waypoints")
        
    def spin_and_update(self):
        rclpy.spin_once(self.ros_node, timeout_sec=0)
        if self.ros_node.current_odom:
            msg = self.ros_node.current_odom
            self.pos_label.setText(f"position:\n    x: {msg.pose.pose.position.x:.2f}\n    y: {msg.pose.pose.position.y:.2f}\n    z: {msg.pose.pose.position.z:.2f}")
            self.orient_label.setText(f"orientation:\n    x: {msg.pose.pose.orientation.x:.2f}\n    y: {msg.pose.pose.orientation.y:.2f}\n    z: {msg.pose.pose.orientation.z:.2f}\n    w: {msg.pose.pose.orientation.w:.2f}")
    
    def set_mode(self, mode):
        self.current_mode = mode
        self.log(f'Mode: {mode.capitalize()}')
        
    def waypoint_clicked(self, idx):
        if self.current_mode == 'save':
            self.save_waypoint(idx)
        elif self.current_mode == 'clear':
            self.clear_waypoint(idx)
        else:
            self.navigate_to_waypoint(idx)
        self.current_mode = None
        
    def save_waypoint(self, idx):
        if str(idx) in self.waypoints:
            self.log(f'Slot {idx + 1} already has data. Clear it first.')
            return
            
        if self.ros_node.current_odom:
            odom = self.ros_node.current_odom
            self.waypoints[str(idx)] = {
                'x': odom.pose.pose.position.x,
                'y': odom.pose.pose.position.y,
                'z': odom.pose.pose.position.z,
                'qx': odom.pose.pose.orientation.x,
                'qy': odom.pose.pose.orientation.y,
                'qz': odom.pose.pose.orientation.z,
                'qw': odom.pose.pose.orientation.w
            }
            self.save_waypoints()
            self.log(f'Saved to slot {idx + 1}')
            self.update_button_colors()
        else:
            self.log('No odometry data available')
            
    def clear_waypoint(self, idx):
        if str(idx) in self.waypoints:
            del self.waypoints[str(idx)]
            self.save_waypoints()
            self.log(f'Cleared slot {idx + 1}')
            self.update_button_colors()
        else:
            self.log(f'Slot {idx + 1} is already empty')
            
    def navigate_to_waypoint(self, idx):
        if str(idx) not in self.waypoints:
            self.log(f'Slot {idx + 1} is empty')
            return
            
        wp = self.waypoints[str(idx)]
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.ros_node.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = wp['x']
        goal_msg.pose.pose.position.y = wp['y']
        goal_msg.pose.pose.position.z = wp['z']
        goal_msg.pose.pose.orientation.x = wp['qx']
        goal_msg.pose.pose.orientation.y = wp['qy']
        goal_msg.pose.pose.orientation.z = wp['qz']
        goal_msg.pose.pose.orientation.w = wp['qw']
        
        self.ros_node.nav_client.wait_for_server()
        self.ros_node.nav_client.send_goal_async(goal_msg)
        self.log(f'Go to position saved in slot {idx + 1}')
        
    def stop_navigation(self):
        self.ros_node.nav_client.cancel_goal_async()
        self.ros_node.stop_robot()
        self.log('Navigation stopped')
        
    def launch_zackon(self):
        try:
            self.zackon_process = subprocess.Popen(
                ['gnome-terminal', '--', 'bash', '-c',
                 'source /home/khoaiuh/thien_ws/install/setup.bash && ros2 launch view_robot_pkg zackon_synthesis.launch.py; exec bash']
            )
            self.log('Zackon synthesis launched (includes Nav2)')
        except Exception as e:
            self.log(f'Failed to launch zackon: {e}')
            
    def kill_processes(self):
        if self.zackon_process:
            subprocess.run(['pkill', '-f', 'zackon_synthesis.launch.py'])
            self.zackon_process.terminate()
            self.log('Zackon synthesis stopped')
        
    def update_button_colors(self):
        for i, btn in enumerate(self.waypoint_btns):
            if str(i) in self.waypoints:
                btn.setStyleSheet('background-color: lightgreen;')
            else:
                btn.setStyleSheet('')
                
    def load_waypoints(self):
        try:
            with open(self.waypoints_file, 'r') as f:
                return json.load(f)
        except FileNotFoundError:
            return {}
            
    def save_waypoints(self):
        with open(self.waypoints_file, 'w') as f:
            json.dump(self.waypoints, f, indent=2)
            
    def log(self, message):
        self.log_text.append(message)
        
    def go_back(self):
        self.ros_node.stop_robot()
        self.kill_processes()
        subprocess.Popen(['python3', '/home/khoaiuh/thien_ws/robot_ui/startup_layout.py', '--skip-micro-ros'])
        self.close()
    
    def closeEvent(self, event):
        self.ros_node.stop_robot()
        self.kill_processes()
        if self.ros_node:
            self.ros_node.destroy_node()
        event.accept()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = WaypointsModeLayout()
    sys.exit(app.exec())
