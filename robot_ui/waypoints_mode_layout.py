#!/usr/bin/env python3
import sys
import json
import subprocess
import os
from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QPushButton, 
                              QLabel, QGridLayout, QTextEdit, QApplication, QMainWindow)
from PyQt6.QtCore import Qt, QTimer, QPointF
from PyQt6.QtGui import QFont, QPixmap, QPainter, QPen, QColor, QTransform
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
# from nav2_msgs.action import NavigateToPose
from nav2_msgs.action import NavigateToPose, DockRobot
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from config import SOURCE_PATH


class WaypointsNode(Node):
    def __init__(self):
        super().__init__('waypoints_node')
        self.subscription = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.dock_client = ActionClient(self, DockRobot, '/dock_robot')
        self.current_pose = None
        self.current_goal_handle = None
        self.nav_server_available = False
        self.nav_server_available = self.nav_client.wait_for_server(timeout_sec=2.0)
        self.dock_after_nav = False
        
    def pose_callback(self, msg):
        self.current_pose = msg


class MapWidget(QWidget):
    def __init__(self, map_path, yaml_data):
        super().__init__()
        self.map_image = QPixmap(map_path)
        self.resolution = yaml_data['resolution']
        self.origin = yaml_data['origin']
        self.robot_pose = None
        self.waypoints = {}
        self.setMinimumSize(400, 400)
        
    def set_robot_pose(self, pose):
        self.robot_pose = pose
        self.update()

    def set_waypoints(self, waypoints):
        self.waypoints = waypoints
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

        scale_x = scaled_map.width() / self.map_image.width()
        scale_y = scaled_map.height() / self.map_image.height()

        # Draw waypoint arrows
        for slot, wp in self.waypoints.items():
            px, py = self.world_to_pixel(wp['x'], wp['y'])
            px = int(px * scale_x + x_offset)
            py = int(py * scale_y + y_offset)
            self._draw_arrow(painter, px, py, slot)

        if self.robot_pose:
            px, py = self.world_to_pixel(
                self.robot_pose.pose.pose.position.x,
                self.robot_pose.pose.pose.position.y
            )
            px = int(px * scale_x + x_offset)
            py = int(py * scale_y + y_offset)
            painter.setPen(QPen(QColor(255, 0, 0), 3))
            painter.setBrush(QColor(255, 0, 0))
            painter.drawEllipse(px - 5, py - 5, 10, 10)

    def _draw_arrow(self, painter, px, py, label):
        painter.setPen(QPen(QColor(0, 120, 255), 2))
        painter.setBrush(QColor(0, 120, 255))
        # Arrow shaft
        painter.drawLine(px, py - 20, px, py)
        # Arrowhead (downward triangle)
        from PyQt6.QtGui import QPolygon
        from PyQt6.QtCore import QPoint
        tip = QPoint(px, py)
        left = QPoint(px - 6, py - 12)
        right = QPoint(px + 6, py - 12)
        painter.drawPolygon(QPolygon([tip, left, right]))
        # Label
        painter.setPen(QPen(QColor(0, 0, 0), 1))
        painter.drawText(px + 8, py - 10, label)


class WaypointsModeLayout(QMainWindow):
    def __init__(self):
        super().__init__()
        try:
            rclpy.init()
        except:
            pass
        self.ros_node = WaypointsNode()
        self.waypoints_file = f'{SOURCE_PATH}/robot_ui/waypoints.json'
        self.waypoints = self.load_waypoints()
        self.current_mode = None
        self.selected_sequence = []
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
        main_layout.setContentsMargins(0, 0, 0, 0)
        
        left_widget = QWidget()
        left_widget.setStyleSheet("background-color: #f0f0f0;")
        left_layout = QVBoxLayout(left_widget)
        
        self.save_btn = QPushButton('Save')
        self.clear_btn = QPushButton('Clear')
        self.run_btn = QPushButton('Run')
        self.reset_btn = QPushButton('Reset')
        self.stop_btn = QPushButton('Stop')
        self.back_btn = QPushButton('Back')
        self.dock_btn = QPushButton('Dock After Nav')
        self.dock_btn.setCheckable(True)
        
        # for btn in [self.save_btn, self.clear_btn, self.run_btn, self.reset_btn, self.stop_btn, self.back_btn]:
        for btn in [self.save_btn, self.clear_btn, self.run_btn, self.reset_btn, self.stop_btn, self.dock_btn, self.back_btn]:
            btn.setFont(QFont('Fira Sans', 24))
            btn.setMinimumHeight(80)
            left_layout.addWidget(btn)
        
        pos_title = QLabel('Current Position')
        pos_title.setFont(QFont('Fira Sans', 18))
        left_layout.addWidget(pos_title)
        
        self.pos_label = QLabel('x: 0.00\ny: 0.00\nz: 0.00')
        self.pos_label.setFont(QFont('Fira Sans', 14))
        left_layout.addWidget(self.pos_label)
        
        self.orient_label = QLabel('qx: 0.00\nqy: 0.00\nqz: 0.00\nqw: 1.00')
        self.orient_label.setFont(QFont('Fira Sans', 14))
        left_layout.addWidget(self.orient_label)
        
        left_layout.addStretch()
        
        self.save_btn.clicked.connect(lambda: self.set_mode('save'))
        self.clear_btn.clicked.connect(lambda: self.set_mode('clear'))
        self.run_btn.clicked.connect(self.run_sequence)
        self.reset_btn.clicked.connect(self.reset_sequence)
        self.stop_btn.clicked.connect(self.stop_navigation)
        self.back_btn.clicked.connect(self.go_back)
        self.dock_btn.toggled.connect(self.toggle_dock_after_nav)
        
        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)
        right_layout.setContentsMargins(10, 10, 10, 10)
        
        map_and_buttons_widget = QWidget()
        map_and_buttons_layout = QVBoxLayout(map_and_buttons_widget)
        
        map_yaml_path = self.get_current_map_path()
        map_dir = os.path.dirname(map_yaml_path)
        yaml_data = self.load_map_yaml(map_yaml_path)
        map_image_path = os.path.join(map_dir, yaml_data['image'])
        
        self.map_widget = MapWidget(map_image_path, yaml_data)
        self.map_widget.set_waypoints(self.waypoints)
        map_and_buttons_layout.addWidget(self.map_widget, 2)
        
        grid_layout = QGridLayout()
        self.waypoint_btns = []
        
        for i in range(10):
            btn = QPushButton(str(i + 1))
            btn.setFont(QFont('Fira Sans', 18))
            btn.setMinimumHeight(50)
            btn.clicked.connect(lambda checked, idx=i: self.waypoint_clicked(idx))
            grid_layout.addWidget(btn, i // 5, i % 5)
            self.waypoint_btns.append(btn)
        
        map_and_buttons_layout.addLayout(grid_layout)
        
        log_widget = QWidget()
        log_layout = QVBoxLayout(log_widget)
        
        log_title = QLabel('Logging')
        log_title.setFont(QFont('Fira Sans', 18))
        log_layout.addWidget(log_title)
        
        self.log_text = QTextEdit()
        self.log_text.setFont(QFont('Fira Sans', 12))
        self.log_text.setReadOnly(True)
        log_layout.addWidget(self.log_text)
        
        right_layout.addWidget(map_and_buttons_widget, 2)
        right_layout.addWidget(log_widget, 1)
        
        main_layout.addWidget(left_widget, 1)
        main_layout.addWidget(right_widget, 3)
        
        self.update_button_colors()
        self.log("Mode changed to waypoints")
        
        if not self.ros_node.nav_server_available:
            self.log("WARNING: Nav2 action server not available")
            for btn in self.waypoint_btns:
                btn.setEnabled(False)
    
    def get_current_map_path(self):
        nav2_params = f'{SOURCE_PATH}/src/view_robot/config/nav2_params.yaml'
        try:
            with open(nav2_params, 'r') as f:
                for line in f:
                    if 'yaml_filename:' in line and '#' not in line:
                        return line.split(':', 1)[1].strip().strip('"')
        except Exception:
            pass
        return f'{SOURCE_PATH}/src/view_robot/maps/F5.yaml'

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
        
    def spin_and_update(self):
        rclpy.spin_once(self.ros_node, timeout_sec=0)
        if self.ros_node.current_pose:
            msg = self.ros_node.current_pose
            self.pos_label.setText(f"x: {msg.pose.pose.position.x:.2f}\ny: {msg.pose.pose.position.y:.2f}\nz: {msg.pose.pose.position.z:.2f}")
            self.orient_label.setText(f"qx: {msg.pose.pose.orientation.x:.2f}\nqy: {msg.pose.pose.orientation.y:.2f}\nqz: {msg.pose.pose.orientation.z:.2f}\nqw: {msg.pose.pose.orientation.w:.2f}")
            self.map_widget.set_robot_pose(msg)
    
    def set_mode(self, mode):
        if self.current_mode == mode:
            self.current_mode = None
            self.log(f'Mode: {mode.capitalize()} cancelled')
        else:
            self.current_mode = mode
            self.log(f'Mode: {mode.capitalize()} - select a slot')
        
    def waypoint_clicked(self, idx):
        slot_num = idx + 1
        
        if self.current_mode == 'save':
            self.save_waypoint(slot_num)
            self.current_mode = None
        elif self.current_mode == 'clear':
            self.clear_waypoint(slot_num)
            self.current_mode = None
        else:
            if str(slot_num) not in self.waypoints:
                self.log(f'Slot {slot_num} is empty')
                return
            
            if self.selected_sequence and self.selected_sequence[-1] == slot_num:
                self.log(f'Cannot select slot {slot_num} consecutively')
                return
            
            self.selected_sequence.append(slot_num)
            self.log(f'Added slot {slot_num} to sequence: {self.selected_sequence}')
        
    def save_waypoint(self, slot_num):
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
            self.map_widget.set_waypoints(self.waypoints)
        else:
            self.log('No pose data available')
            
    def clear_waypoint(self, slot_num):
        if str(slot_num) in self.waypoints:
            del self.waypoints[str(slot_num)]
            self.save_waypoints()
            self.log(f'Cleared slot {slot_num}')
            self.update_button_colors()
            self.map_widget.set_waypoints(self.waypoints)
        else:
            self.log(f'Slot {slot_num} is already empty')
            
    def navigate_to_waypoint(self, slot_num):
        if str(slot_num) not in self.waypoints:
            self.log(f'Slot {slot_num} is empty')
            return
        
        if not self.ros_node.nav_server_available:
            self.log('Nav2 server not available')
            return
        
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
        
        send_goal_future = self.ros_node.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(lambda future: self._goal_response_callback(future, slot_num))
        
    def _goal_response_callback(self, future, slot_num):
        goal_handle = future.result()
        if goal_handle.accepted:
            self.ros_node.current_goal_handle = goal_handle
            self.log(f'Navigating to slot {slot_num}')
            
            if self.running_sequence:
                result_future = goal_handle.get_result_async()
                result_future.add_done_callback(self._goal_result_callback)
        else:
            self.log(f'Goal to slot {slot_num} rejected')
            self.ros_node.current_goal_handle = None
            if self.running_sequence:
                self.running_sequence = False
    
    def _goal_result_callback(self, future):
        if self.running_sequence and self.current_sequence_index < len(self.selected_sequence) - 1:
            self.current_sequence_index += 1
            next_slot = self.selected_sequence[self.current_sequence_index]
            
            current_slot = self.selected_sequence[self.current_sequence_index - 1]
            if next_slot == current_slot:
                self.log(f'Already at slot {next_slot}, skipping...')
                self._goal_result_callback(future)
            else:
                self.navigate_to_waypoint(next_slot)
        else:
            self.running_sequence = False
            self.log('Sequence completed')
            # Auto-dock after last waypoint if enabled
            if self.ros_node.dock_after_nav:
                self._send_dock_goal('home_dock')

    def toggle_dock_after_nav(self, checked):
        self.ros_node.dock_after_nav = checked
        self.log(f'Dock after nav: {"ON" if checked else "OFF"}')
        self.dock_btn.setStyleSheet('background-color: lightblue;' if checked else '')

    def _send_dock_goal(self, dock_id):
        self.log(f'Sending dock goal: {dock_id}')
        goal = DockRobot.Goal()
        goal.dock_id = dock_id
        self.ros_node.dock_client.send_goal_async(goal).add_done_callback(
            lambda f: self.log('Docking started' if f.result().accepted else 'Dock goal rejected')
        )
    
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
        if self.ros_node.current_goal_handle is not None:
            self.ros_node.current_goal_handle.cancel_goal_async()
            self.ros_node.current_goal_handle = None
            self.log('Navigation cancelled')
        else:
            self.log('No active navigation to stop')
        
        self.running_sequence = False
        self.selected_sequence = []
        self.log('Sequence cleared')
        
    def update_button_colors(self):
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
        if self.ros_node.current_goal_handle is not None:
            self.ros_node.current_goal_handle.cancel_goal_async()
        subprocess.Popen(['python3', f'{SOURCE_PATH}/robot_ui/startup_layout.py', '--skip-micro-ros'])
        self.close()
    
    def closeEvent(self, event):
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
