#!/usr/bin/env python3
import sys
import subprocess
import os
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                             QHBoxLayout, QPushButton, QLabel, QProgressBar)
from PyQt6.QtCore import QTimer, Qt
from PyQt6.QtGui import QFont
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Bool, Float32

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from config import SOURCE_PATH


class DockingUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.docking_process = None
        self.battery_level = 0
        
        try:
            rclpy.init()
        except Exception:
            pass
        
        self._ros_node = Node('docking_ui_node')
        self._ros_node.create_subscription(
            Int32, '/rear_docking_status', self._docking_status_callback, 10
        )
        self._ros_node.create_subscription(
            Bool, '/charging_status', self._charging_status_callback, 10
        )
        self._ros_node.create_subscription(
            Float32, '/battery_level', self._battery_callback, 10
        )
        
        self._charge_pub = self._ros_node.create_publisher(Bool, '/charge_command', 10)
        
        self._ros_spin_timer = QTimer()
        self._ros_spin_timer.timeout.connect(self._ros_spin_once)
        self._ros_spin_timer.start(100)
        
        self.init_ui()
    
    def _ros_spin_once(self):
        try:
            if rclpy.ok():
                rclpy.spin_once(self._ros_node, timeout_sec=0)
        except Exception:
            self._ros_spin_timer.stop()
    
    def _docking_status_callback(self, msg):
        """Monitor docking status from /rear_docking_status topic"""
        print(f"[DEBUG] Received /rear_docking_status: data={msg.data}")
        
        if msg.data == 1:  # Docked
            # Turn on docked light, turn off docking light
            self.docked_light.setStyleSheet("color: #22c55e;")
            self.docking_light.setStyleSheet("color: #c8d4f0;")
            
            print("✓ Robot đã docked thành công! Đang tắt terminal...")
            
            # Kill the docking terminal using pkill
            try:
                subprocess.run(['pkill', '-f', 'zackon_docking.launch.py'])
                self.docking_process = None
                print("✓ Đã dừng tiến trình docking")
            except Exception as e:
                print(f"✗ Lỗi khi dừng tiến trình: {e}")
        else:  # Not docked
            self.docked_light.setStyleSheet("color: #c8d4f0;")
            self.docking_light.setStyleSheet("color: #fcb525;")
    
    def _charging_status_callback(self, msg):
        """Monitor charging status"""
        print(f"[DEBUG] Received /charging_status: data={msg.data}")
        
        if msg.data:  # Charging
            self.charging_light.setStyleSheet("color: #fcb525;")
        else:  # Not charging
            self.charging_light.setStyleSheet("color: #c8d4f0;")
    
    def _battery_callback(self, msg):
        """Update battery level"""
        self.battery_level = int(msg.data)
        self.battery_bar.setValue(self.battery_level)
        self.battery_label.setText(f"Battery: {self.battery_level}%")
    
    def init_ui(self):
        self.setWindowTitle("Docking Control")
        self.setStyleSheet("""
            QMainWindow, QWidget {
                background-color: #f0f4ff;
                color: #1a2a5e;
            }
            QPushButton#dock-btn {
                background-color: #214196;
                color: #ffffff;
                border: none;
                border-radius: 8px;
                padding: 20px;
                font-size: 18px;
                min-height: 60px;
            }
            QPushButton#dock-btn:hover {
                background-color: #1a3278;
            }
            QPushButton#dock-btn:disabled {
                background-color: #c8d4f0;
                color: #8fa3cc;
            }
            QPushButton#back-btn {
                background-color: #ef4444;
                color: #ffffff;
                border: none;
                border-radius: 8px;
                padding: 15px 30px;
                font-size: 16px;
            }
            QPushButton#back-btn:hover {
                background-color: #dc2626;
            }
            QLabel#status-light {
                font-size: 48px;
                padding: 10px;
            }
            QLabel#status-text {
                font-size: 16px;
                color: #5a7abf;
            }
            QProgressBar {
                border: 2px solid #c8d4f0;
                border-radius: 8px;
                text-align: center;
                height: 30px;
                background-color: #ffffff;
            }
            QProgressBar::chunk {
                background-color: #22c55e;
                border-radius: 6px;
            }
        """)
        
        central = QWidget()
        self.setCentralWidget(central)
        layout = QHBoxLayout(central)
        layout.setContentsMargins(20, 20, 20, 20)
        layout.setSpacing(20)
        
        # Left panel - Back button and battery
        left_panel = QWidget()
        left_layout = QVBoxLayout(left_panel)
        left_layout.setAlignment(Qt.AlignmentFlag.AlignTop)
        
        back_btn = QPushButton("← Back")
        back_btn.setObjectName("back-btn")
        back_btn.clicked.connect(self.go_back)
        left_layout.addWidget(back_btn)
        
        # Battery section
        battery_section = QWidget()
        battery_layout = QVBoxLayout(battery_section)
        battery_layout.setSpacing(10)
        
        self.battery_label = QLabel("Battery: 0%")
        self.battery_label.setStyleSheet("color: #214196; font-size: 16px; font-weight: bold;")
        battery_layout.addWidget(self.battery_label)
        
        self.battery_bar = QProgressBar()
        self.battery_bar.setRange(0, 100)
        self.battery_bar.setValue(0)
        battery_layout.addWidget(self.battery_bar)
        
        left_layout.addWidget(battery_section)
        
        # Instructions - moved to left panel
        instructions = QLabel(
            "📋 <b>Instructions:</b><br><br>"
            "<b>Buttons:</b><br>"
            "• <b>Start Dock:</b> Begin docking<br>"
            "• <b>Undock:</b> Prepare to leave<br>"
            "• <b>Charge:</b> Start charging<br><br>"
            "<b>Status Lights:</b><br>"
            "• <b>Docking:</b> Moving to dock<br>"
            "• <b>Docked:</b> Successfully docked<br>"
            "• <b>Charging:</b> Receiving power<br>"
            "• <b>Ready:</b> Ready to operate"
        )
        instructions.setStyleSheet("color: #5a7abf; font-size: 14px; padding: 15px; "
                                   "background-color: #ffffff; border: 1px solid #c8d4f0; "
                                   "border-radius: 8px; margin-top: 20px;")
        instructions.setWordWrap(True)
        left_layout.addWidget(instructions)
        
        left_layout.addStretch()
        
        # Right panel - Status and controls
        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)
        right_layout.setSpacing(30)
        
        # Title
        title = QLabel("Docking Control")
        title.setFont(QFont("JetBrains Mono", 24, QFont.Weight.Bold))
        title.setStyleSheet("color: #214196;")
        right_layout.addWidget(title)
        
        # Status indicators (4 lights in 2 rows)
        status_container = QWidget()
        status_layout = QVBoxLayout(status_container)
        status_layout.setSpacing(20)
        
        # First row: Docking and Docked
        row1 = QWidget()
        row1_layout = QHBoxLayout(row1)
        row1_layout.setSpacing(40)
        
        # Docking status
        docking_widget = QWidget()
        docking_layout = QVBoxLayout(docking_widget)
        docking_layout.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.docking_light = QLabel("●")
        self.docking_light.setObjectName("status-light")
        self.docking_light.setStyleSheet("color: #c8d4f0;")
        self.docking_light.setAlignment(Qt.AlignmentFlag.AlignCenter)
        docking_text = QLabel("Docking")
        docking_text.setObjectName("status-text")
        docking_text.setAlignment(Qt.AlignmentFlag.AlignCenter)
        docking_layout.addWidget(self.docking_light)
        docking_layout.addWidget(docking_text)
        
        # Docked status
        docked_widget = QWidget()
        docked_layout = QVBoxLayout(docked_widget)
        docked_layout.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.docked_light = QLabel("●")
        self.docked_light.setObjectName("status-light")
        self.docked_light.setStyleSheet("color: #c8d4f0;")
        self.docked_light.setAlignment(Qt.AlignmentFlag.AlignCenter)
        docked_text = QLabel("Docked")
        docked_text.setObjectName("status-text")
        docked_text.setAlignment(Qt.AlignmentFlag.AlignCenter)
        docked_layout.addWidget(self.docked_light)
        docked_layout.addWidget(docked_text)
        
        row1_layout.addWidget(docking_widget)
        row1_layout.addWidget(docked_widget)
        
        # Second row: Charging and Undock
        row2 = QWidget()
        row2_layout = QHBoxLayout(row2)
        row2_layout.setSpacing(40)
        
        # Charging status
        charging_widget = QWidget()
        charging_layout = QVBoxLayout(charging_widget)
        charging_layout.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.charging_light = QLabel("●")
        self.charging_light.setObjectName("status-light")
        self.charging_light.setStyleSheet("color: #c8d4f0;")
        self.charging_light.setAlignment(Qt.AlignmentFlag.AlignCenter)
        charging_text = QLabel("Charging")
        charging_text.setObjectName("status-text")
        charging_text.setAlignment(Qt.AlignmentFlag.AlignCenter)
        charging_layout.addWidget(self.charging_light)
        charging_layout.addWidget(charging_text)
        
        # Undock status
        undock_widget = QWidget()
        undock_layout = QVBoxLayout(undock_widget)
        undock_layout.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.undock_light = QLabel("●")
        self.undock_light.setObjectName("status-light")
        self.undock_light.setStyleSheet("color: #c8d4f0;")  # Default: off
        self.undock_light.setAlignment(Qt.AlignmentFlag.AlignCenter)
        undock_text = QLabel("Ready")
        undock_text.setObjectName("status-text")
        undock_text.setAlignment(Qt.AlignmentFlag.AlignCenter)
        undock_layout.addWidget(self.undock_light)
        undock_layout.addWidget(undock_text)
        
        row2_layout.addWidget(charging_widget)
        row2_layout.addWidget(undock_widget)
        
        status_layout.addWidget(row1)
        status_layout.addWidget(row2)
        right_layout.addWidget(status_container)
        
        # Buttons
        dock_btn = QPushButton("Start Dock")
        dock_btn.setObjectName("dock-btn")
        dock_btn.clicked.connect(self.execute_docking)
        right_layout.addWidget(dock_btn)
        
        undock_btn = QPushButton("Undock")
        undock_btn.setObjectName("dock-btn")
        undock_btn.clicked.connect(self.execute_undock)
        right_layout.addWidget(undock_btn)
        
        charge_btn = QPushButton("Charge")
        charge_btn.setObjectName("dock-btn")
        charge_btn.clicked.connect(self.send_charge_command)
        right_layout.addWidget(charge_btn)
        
        right_layout.addStretch()
        
        layout.addWidget(left_panel, 2)
        layout.addWidget(right_panel, 5)
    
    def execute_docking(self):
        """Launch docking process and monitor via ROS topic"""
        print("Bắt đầu trình tự về trạm sạc")
        
        # Turn on docking light, turn off ready light
        self.docking_light.setStyleSheet("color: #fcb525;")
        self.undock_light.setStyleSheet("color: #c8d4f0;")
        
        # Launch docking process
        try:
            self.docking_process = subprocess.Popen([
                'gnome-terminal', '--', 'bash', '-c',
                f'source {SOURCE_PATH}/install/setup.bash && '
                f'ros2 launch {SOURCE_PATH}/src/lidar_dock_detector/launch/zackon_docking.launch.py; exec bash'
            ])
            
            print("Đang theo dõi trạng thái docking qua /rear_docking_status...")
            
        except Exception as e:
            print(f"Lỗi khởi động docking: {e}")
            self.docking_light.setStyleSheet("color: #ef4444;")
    
    def execute_undock(self):
        """Prepare robot to undock"""
        print("Chuẩn bị undock...")
        
        # Turn on ready light
        self.undock_light.setStyleSheet("color: #22c55e;")
        
        # Turn off other lights
        self.docking_light.setStyleSheet("color: #c8d4f0;")
        self.docked_light.setStyleSheet("color: #c8d4f0;")
        self.charging_light.setStyleSheet("color: #c8d4f0;")
        
        print("✓ Robot sẵn sàng để sử dụng")
    
    def send_charge_command(self):
        """Send charge command to robot"""
        print("Gửi lệnh sạc đến robot...")
        msg = Bool()
        msg.data = True
        self._charge_pub.publish(msg)
        print("✓ Đã gửi lệnh sạc")
    
    def go_back(self):
        """Return to startup layout"""
        subprocess.Popen([sys.executable, f'{SOURCE_PATH}/robot_ui/startup_layout.py', '--skip-micro-ros'])
        self.close()
    
    def closeEvent(self, event):
        self._ros_spin_timer.stop()
        self._ros_node.destroy_node()
        event.accept()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = DockingUI()
    window.showMaximized()
    sys.exit(app.exec())
