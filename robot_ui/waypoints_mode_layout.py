#!/usr/bin/env python3
import sys
import json
import subprocess
import os
from PyQt6.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QPushButton, 
                              QLabel, QGridLayout, QTextEdit, QApplication, QMainWindow, QSizePolicy)
from PyQt6.QtCore import Qt, QTimer, QPointF
from PyQt6.QtGui import QFont, QPixmap, QPainter, QPen, QColor, QTransform, QFontDatabase
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
# from nav2_msgs.action import NavigateToPose
from nav2_msgs.action import NavigateToPose
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from config import SOURCE_PATH


class WaypointsNode(Node):
    def __init__(self):
        super().__init__('waypoints_node')
        self.subscription = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.current_pose = None
        self.current_goal_handle = None
        self.nav_server_available = False
        self.nav_server_available = self.nav_client.wait_for_server(timeout_sec=2.0)
        
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
        painter.setPen(QPen(QColor(0, 229, 255), 2))
        painter.setBrush(QColor(0, 229, 255))
        painter.drawLine(px, py - 20, px, py)
        from PyQt6.QtGui import QPolygon
        from PyQt6.QtCore import QPoint
        tip = QPoint(px, py)
        left = QPoint(px - 6, py - 12)
        right = QPoint(px + 6, py - 12)
        painter.drawPolygon(QPolygon([tip, left, right]))
        painter.setPen(QPen(QColor(232, 236, 240), 1))
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
            QPushButton#action-btn:checked {
                background-color: #1c2030;
                color: #00e5ff;
                border-left: 4px solid #00e5ff;
            }
            QPushButton#wp-btn {
                background-color: #1c2030;
                color: #6b7a99;
                border: 1px solid #2a3040;
                border-radius: 4px;
                font-size: 16px;
                min-height: 48px;
            }
            QPushButton#wp-btn:hover {
                background-color: #1a1f2e;
                color: #e8ecf0;
                border: 1px solid #3a4460;
            }
            QPushButton#wp-btn[filled="true"] {
                color: #00c853;
                border: 1px solid #00c853;
            }
            QPushButton#wp-btn[selected="true"] {
                color: #00e5ff;
                border: 1px solid #00e5ff;
                background-color: #1c2030;
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
            QLabel#clock {
                color: #6b7a99;
                font-size: 15px;
            }
            QLabel#header-title {
                color: #e8ecf0;
                font-size: 15px;
            }
        """
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

        wordmark = QLabel("WAYPOINTS")
        wordmark.setFont(QFont("JetBrains Mono", 14, QFont.Weight.Bold))
        wordmark.setStyleSheet("color: #00e5ff; padding: 24px 24px 16px 24px;")
        left_layout.addWidget(wordmark)

        mono = QFont("JetBrains Mono", 18)
        self.save_btn  = QPushButton("Save")
        self.clear_btn = QPushButton("Clear")
        self.run_btn   = QPushButton("Run")
        self.reset_btn = QPushButton("Reset")
        self.stop_btn  = QPushButton("Stop")
        self.back_btn  = QPushButton("Back")

        for btn in [self.save_btn, self.clear_btn, self.run_btn,
                    self.reset_btn, self.stop_btn, self.back_btn]:
            btn.setObjectName("action-btn")
            btn.setFont(mono)
            btn.setMinimumHeight(64)
            btn.setCheckable(True)
            btn.setAutoExclusive(False)
            left_layout.addWidget(btn)

        # Position section
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

        self.save_btn.clicked.connect(lambda: self.set_mode('save'))
        self.clear_btn.clicked.connect(lambda: self.set_mode('clear'))
        self.run_btn.clicked.connect(self.run_sequence)
        self.reset_btn.clicked.connect(self.reset_sequence)
        self.stop_btn.clicked.connect(self.stop_navigation)
        self.back_btn.clicked.connect(self.go_back)

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

        header_title = QLabel("WAYPOINTS MODE")
        header_title.setObjectName("header-title")
        header_title.setFont(QFont("JetBrains Mono", 15, QFont.Weight.Bold))

        self.clock_label = QLabel()
        self.clock_label.setObjectName("clock")
        self.clock_label.setFont(QFont("JetBrains Mono", 15))

        header_layout.addWidget(header_title)
        header_layout.addStretch()
        header_layout.addWidget(self.clock_label)
        right_layout.addWidget(header)

        # Map + waypoint grid
        map_container = QWidget()
        map_container.setStyleSheet("background-color: #0d0f12; padding: 12px;")
        map_layout = QVBoxLayout(map_container)
        map_layout.setContentsMargins(12, 12, 12, 12)
        map_layout.setSpacing(8)

        map_yaml_path = self.get_current_map_path()
        map_dir = os.path.dirname(map_yaml_path)
        yaml_data = self.load_map_yaml(map_yaml_path)
        map_image_path = os.path.join(map_dir, yaml_data['image'])

        self.map_widget = MapWidget(map_image_path, yaml_data)
        self.map_widget.set_waypoints(self.waypoints)
        map_layout.addWidget(self.map_widget, 2)

        grid_layout = QGridLayout()
        grid_layout.setSpacing(6)
        self.waypoint_btns = []
        for i in range(10):
            btn = QPushButton(str(i + 1))
            btn.setObjectName("wp-btn")
            btn.setFont(QFont("JetBrains Mono", 16))
            btn.setMinimumHeight(48)
            btn.clicked.connect(lambda checked, idx=i: self.waypoint_clicked(idx))
            grid_layout.addWidget(btn, i // 5, i % 5)
            self.waypoint_btns.append(btn)
        map_layout.addLayout(grid_layout)

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

        self.update_button_colors()
        self.log("Mode changed to waypoints")

        if not self.ros_node.nav_server_available:
            self.log("[WARN] Nav2 action server not available")
            for btn in self.waypoint_btns:
                btn.setEnabled(False)

    def _update_clock(self):
        from datetime import datetime
        self.clock_label.setText(datetime.now().strftime("%H:%M:%S"))
    
    def get_current_map_path(self):
        nav2_params = f'{SOURCE_PATH}/src/view_robot/config/nav2_params.yaml'
        try:
            with open(nav2_params, 'r') as f:
                for line in f:
                    if 'yaml_filename:' in line and '#' not in line:
                        path = line.split(':', 1)[1].strip().strip('"')
                        maps_dir = f'{SOURCE_PATH}/src/view_robot/maps/'
                        map_file = os.path.basename(path)
                        return maps_dir + map_file
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
            p = msg.pose.pose.position
            o = msg.pose.pose.orientation
            self.pos_label.setText(f"x: {p.x:.2f}   y: {p.y:.2f}   z: {p.z:.2f}")
            self.orient_label.setText(f"qx: {o.x:.2f}   qy: {o.y:.2f}\nqz: {o.z:.2f}   qw: {o.w:.2f}")
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
            self.update_button_colors()
        
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
            filled = str(i + 1) in self.waypoints
            selected = (i + 1) in self.selected_sequence
            btn.setProperty("filled", "true" if filled else "false")
            btn.setProperty("selected", "true" if selected else "false")
            btn.style().unpolish(btn)
            btn.style().polish(btn)
                
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
        from datetime import datetime
        ts = datetime.now().strftime("%H:%M:%S")
        if "[ERROR]" in message:
            color = "#ff3b3b"
        elif "[WARN]" in message or "WARNING" in message:
            color = "#ffb300"
        elif "✓" in message or "completed" in message.lower():
            color = "#00c853"
        else:
            color = "#e8ecf0"
        self.log_text.append(
            f'<span style="color:#6b7a99">[{ts}]</span> <span style="color:{color}">{message}</span>'
        )
        
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
