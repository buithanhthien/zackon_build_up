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
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Twist
from nav2_msgs.action import NavigateToPose


class WaypointsNode(Node):
    def __init__(self):
        super().__init__('waypoints_node')
        self.subscription = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)
        # FIXED: Proper action client with goal handle management
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.current_pose = None
        self.current_goal_handle = None  # ADDED: Track active goal
        self.nav_server_available = False
        
        # FIXED: Check server availability once at init
        self.nav_server_available = self.nav_client.wait_for_server(timeout_sec=2.0)
        
    def pose_callback(self, msg):
        self.current_pose = msg


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
        self.selected_sequence = []  # Track clicked waypoints in order
        self.running_sequence = False
        self.current_sequence_index = 0
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
        self.run_btn = QPushButton('Run')
        self.reset_btn = QPushButton('Reset')
        self.stop_btn = QPushButton('Stop')
        self.back_btn = QPushButton('Back')
        
        for btn in [self.save_btn, self.clear_btn, self.run_btn, self.reset_btn, self.stop_btn, self.back_btn]:
            btn.setFont(QFont('Fira Sans', 24))
            left_layout.addWidget(btn)
        
        left_layout.addStretch()
        
        self.save_btn.clicked.connect(lambda: self.set_mode('save'))
        self.clear_btn.clicked.connect(lambda: self.set_mode('clear'))
        self.run_btn.clicked.connect(self.run_sequence)
        self.reset_btn.clicked.connect(self.reset_sequence)
        self.stop_btn.clicked.connect(self.stop_navigation)
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
        
        # FIXED: Disable navigation if server unavailable
        if not self.ros_node.nav_server_available:
            self.log("WARNING: Nav2 action server not available")
            for btn in self.waypoint_btns:
                btn.setEnabled(False)
        
    def spin_and_update(self):
        rclpy.spin_once(self.ros_node, timeout_sec=0)
        if self.ros_node.current_pose:
            msg = self.ros_node.current_pose
            self.pos_label.setText(f"position:\n    x: {msg.pose.pose.position.x:.2f}\n    y: {msg.pose.pose.position.y:.2f}\n    z: {msg.pose.pose.position.z:.2f}")
            self.orient_label.setText(f"orientation:\n    x: {msg.pose.pose.orientation.x:.2f}\n    y: {msg.pose.pose.orientation.y:.2f}\n    z: {msg.pose.pose.orientation.z:.2f}\n    w: {msg.pose.pose.orientation.w:.2f}")
    
    def set_mode(self, mode):
        # FIXED: Mode persists until action completes
        if self.current_mode == mode:
            self.current_mode = None  # Toggle off
            self.log(f'Mode: {mode.capitalize()} cancelled')
        else:
            self.current_mode = mode
            self.log(f'Mode: {mode.capitalize()} - select a slot')
        
    def waypoint_clicked(self, idx):
        # FIXED: Use 1-based indexing consistently
        slot_num = idx + 1
        
        if self.current_mode == 'save':
            self.save_waypoint(slot_num)
            self.current_mode = None  # Reset after action
        elif self.current_mode == 'clear':
            self.clear_waypoint(slot_num)
            self.current_mode = None  # Reset after action
        else:
            # Add to sequence if not in save/clear mode
            if str(slot_num) not in self.waypoints:
                self.log(f'Slot {slot_num} is empty')
                return
            
            # Prevent consecutive duplicates
            if self.selected_sequence and self.selected_sequence[-1] == slot_num:
                self.log(f'Cannot select slot {slot_num} consecutively')
                return
            
            self.selected_sequence.append(slot_num)
            self.log(f'Added slot {slot_num} to sequence: {self.selected_sequence}')
        
    def save_waypoint(self, slot_num):
        # FIXED: Use 1-based slot numbering
        if str(slot_num) in self.waypoints:
            self.log(f'Slot {slot_num} already has data. Clear it first.')
            return
            
        if self.ros_node.current_pose:
            pose = self.ros_node.current_pose
            self.waypoints[str(slot_num)] = {
                'x': pose.pose.pose.position.x,
                'y': pose.pose.pose.position.y,
                'z': pose.pose.pose.position.z,
                'qx': pose.pose.pose.orientation.x,
                'qy': pose.pose.pose.orientation.y,
                'qz': pose.pose.pose.orientation.z,
                'qw': pose.pose.pose.orientation.w
            }
            self.save_waypoints()
            self.log(f'Saved to slot {slot_num}')
            self.update_button_colors()
        else:
            self.log('No pose data available')
            
    def clear_waypoint(self, slot_num):
        # FIXED: Use 1-based slot numbering
        if str(slot_num) in self.waypoints:
            del self.waypoints[str(slot_num)]
            self.save_waypoints()
            self.log(f'Cleared slot {slot_num}')
            self.update_button_colors()
        else:
            self.log(f'Slot {slot_num} is already empty')
            
    def navigate_to_waypoint(self, slot_num):
        # FIXED: Proper goal handle management and cancellation
        if str(slot_num) not in self.waypoints:
            self.log(f'Slot {slot_num} is empty')
            return
        
        if not self.ros_node.nav_server_available:
            self.log('Nav2 server not available')
            return
        
        # FIXED: Cancel previous goal if active
        if self.ros_node.current_goal_handle is not None:
            self.log('Cancelling previous navigation...')
            self.ros_node.current_goal_handle.cancel_goal_async()
            self.ros_node.current_goal_handle = None
            
        wp = self.waypoints[str(slot_num)]
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
        
        # FIXED: Store goal handle for proper cancellation
        send_goal_future = self.ros_node.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(lambda future: self._goal_response_callback(future, slot_num))
        
    def _goal_response_callback(self, future, slot_num):
        # FIXED: Callback to store goal handle
        goal_handle = future.result()
        if goal_handle.accepted:
            self.ros_node.current_goal_handle = goal_handle
            self.log(f'Navigating to slot {slot_num}')
            
            # If running sequence, set up result callback
            if self.running_sequence:
                result_future = goal_handle.get_result_async()
                result_future.add_done_callback(self._goal_result_callback)
        else:
            self.log(f'Goal to slot {slot_num} rejected')
            self.ros_node.current_goal_handle = None
            if self.running_sequence:
                self.running_sequence = False
    
    def _goal_result_callback(self, future):
        # When goal completes, navigate to next in sequence
        if self.running_sequence and self.current_sequence_index < len(self.selected_sequence) - 1:
            self.current_sequence_index += 1
            next_slot = self.selected_sequence[self.current_sequence_index]
            
            # Skip if same as current position (already there)
            current_slot = self.selected_sequence[self.current_sequence_index - 1]
            if next_slot == current_slot:
                self.log(f'Already at slot {next_slot}, skipping...')
                self._goal_result_callback(future)  # Recursively check next
            else:
                self.navigate_to_waypoint(next_slot)
        else:
            self.running_sequence = False
            self.log('Sequence completed')
    
    def run_sequence(self):
        if not self.selected_sequence:
            self.log('No waypoints selected. Click slots to build sequence.')
            return
        
        self.log(f'Running sequence: {self.selected_sequence}')
        self.running_sequence = True
        self.current_sequence_index = 0
        self.navigate_to_waypoint(self.selected_sequence[0])
    
    def reset_sequence(self):
        self.selected_sequence = []
        self.running_sequence = False
        self.current_sequence_index = 0
        self.log('Sequence reset')
        
    def stop_navigation(self):
        # FIXED: Proper goal cancellation without cmd_vel
        if self.ros_node.current_goal_handle is not None:
            self.ros_node.current_goal_handle.cancel_goal_async()
            self.ros_node.current_goal_handle = None
            self.log('Navigation cancelled')
        else:
            self.log('No active navigation to stop')
        
        # Stop sequence and clear selection
        self.running_sequence = False
        self.selected_sequence = []
        self.log('Sequence cleared')
        
    def update_button_colors(self):
        # FIXED: Use 1-based slot numbering
        for i, btn in enumerate(self.waypoint_btns):
            slot_num = i + 1
            if str(slot_num) in self.waypoints:
                btn.setStyleSheet('background-color: lightgreen;')
            else:
                btn.setStyleSheet('')
                
    def load_waypoints(self):
        try:
            with open(self.waypoints_file, 'r') as f:
                content = f.read().strip()
                if not content:
                    return {}
                return json.loads(content)
        except (FileNotFoundError, json.JSONDecodeError):
            return {}
            
    def save_waypoints(self):
        with open(self.waypoints_file, 'w') as f:
            json.dump(self.waypoints, f, indent=2)
            
    def log(self, message):
        self.log_text.append(message)
        
    def go_back(self):
        # FIXED: Cancel active navigation before going back
        if self.ros_node.current_goal_handle is not None:
            self.ros_node.current_goal_handle.cancel_goal_async()
        subprocess.Popen(['python3', '/home/khoaiuh/thien_ws/robot_ui/startup_layout.py', '--skip-micro-ros'])
        self.close()
    
    def closeEvent(self, event):
        # FIXED: Proper ROS shutdown
        if self.ros_node.current_goal_handle is not None:
            self.ros_node.current_goal_handle.cancel_goal_async()
        if self.ros_node:
            self.ros_node.destroy_node()
        try:
            rclpy.shutdown()
        except:
            pass
        event.accept()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = WaypointsModeLayout()
    sys.exit(app.exec())
