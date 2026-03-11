#!/usr/bin/env python3
import sys
import subprocess
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
                             QPushButton, QTextEdit, QLabel, QLineEdit)
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QFont


class NewMapUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.mapping_process = None
        self.init_ui()
        
    def init_ui(self):
        self.setWindowTitle("New Map - SLAM Mapping")
        self.setStyleSheet("background-color: white; color: black;")
        
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QHBoxLayout(central)
        main_layout.setContentsMargins(0, 0, 0, 0)
        
        # Left panel - Back button
        left_panel = QWidget()
        left_panel.setStyleSheet("background-color: #f0f0f0;")
        left_layout = QVBoxLayout(left_panel)
        
        self.btn_cancel = QPushButton("Cancel")
        self.btn_cancel.setFont(QFont("Fira Sans", 24))
        self.btn_cancel.setMinimumHeight(100)
        self.btn_cancel.clicked.connect(self.cancel_mapping)
        left_layout.addWidget(self.btn_cancel)
        
        self.btn_back = QPushButton("Back")
        self.btn_back.setFont(QFont("Fira Sans", 24))
        self.btn_back.setMinimumHeight(100)
        self.btn_back.clicked.connect(self.go_back)
        left_layout.addWidget(self.btn_back)
        
        left_layout.addStretch()
        
        # Right panel - Main content
        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)
        right_layout.setContentsMargins(10, 10, 10, 10)
        
        title = QLabel("SLAM Mapping")
        title.setFont(QFont("Fira Sans", 28, QFont.Weight.Bold))
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        right_layout.addWidget(title)
        
        info = QLabel("Click Start to begin generating a new map using SLAM Toolbox")
        info.setFont(QFont("Fira Sans", 16))
        info.setAlignment(Qt.AlignmentFlag.AlignCenter)
        right_layout.addWidget(info)
        
        # Start button
        self.btn_start = QPushButton("Start")
        self.btn_start.setFont(QFont("Fira Sans", 24))
        self.btn_start.setMinimumHeight(100)
        self.btn_start.clicked.connect(self.start_mapping)
        right_layout.addWidget(self.btn_start)
        
        # Save map section
        save_widget = QWidget()
        save_layout = QVBoxLayout(save_widget)
        
        save_label = QLabel("Save Map")
        save_label.setFont(QFont("Fira Sans", 20, QFont.Weight.Bold))
        save_layout.addWidget(save_label)
        
        input_layout = QHBoxLayout()
        self.map_name_input = QLineEdit()
        self.map_name_input.setPlaceholderText("Enter map name you want...")
        self.map_name_input.setFont(QFont("Fira Sans", 16))
        self.map_name_input.setMinimumHeight(50)
        input_layout.addWidget(self.map_name_input)
        
        self.btn_apply = QPushButton("Apply")
        self.btn_apply.setFont(QFont("Fira Sans", 16))
        self.btn_apply.setMinimumHeight(50)
        self.btn_apply.setMaximumWidth(120)
        self.btn_apply.clicked.connect(self.save_map)
        input_layout.addWidget(self.btn_apply)
        
        save_layout.addLayout(input_layout)
        right_layout.addWidget(save_widget)
        
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setFont(QFont("Fira Sans", 14))
        right_layout.addWidget(self.log_text)
        
        main_layout.addWidget(left_panel, 1)
        main_layout.addWidget(right_panel, 4)
        
    def start_mapping(self):
        try:
            self.mapping_process = subprocess.Popen([
                'gnome-terminal', '--', 'bash', '-c',
                'source ~/zackon_build_up/install/setup.bash && ros2 launch view_robot_pkg MAP_GENERATING.launch.py; exec bash'
            ])
            self.log("Started MAP_GENERATING.launch.py")
            self.log("SLAM mapping is now active")
            self.log("Drive the robot to explore the environment")
            self.log("Enter a map name and click Apply to save the map")
            self.btn_start.setEnabled(False)
            self.btn_start.setText("Mapping Active")
        except Exception as e:
            self.log(f"Failed to start mapping: {e}")
    
    def cancel_mapping(self):
        self.log("Cancelling SLAM mapping process...")
        try:
            subprocess.run(['pkill', '-f', 'MAP_GENERATING.launch.py'], check=False)
            subprocess.run(['pkill', '-f', 'rviz2'], check=False)
            self.log("Killed MAP_GENERATING and RViz2 processes")
        except Exception as e:
            self.log(f"Error killing processes: {e}")
        finally:
            self.mapping_process = None
            self.btn_start.setEnabled(True)
            self.btn_start.setText("Start")
            self.log("Start button restored to normal state")
    
    def save_map(self):
        map_name = self.map_name_input.text().strip()
        if not map_name:
            self.log("Error: Please enter a map name")
            return
        
        map_path = f"/home/khoaiuh/zackon_build_up/src/view_robot/maps/{map_name}"
        self.log(f"Saving map as '{map_name}' to maps folder...")
        try:
            subprocess.Popen([
                'gnome-terminal', '--', 'bash', '-c',
                f'source ~/zackon_build_up/install/setup.bash && cd /home/khoaiuh/zackon_build_up/src/view_robot/maps && ros2 run nav2_map_server map_saver_cli -f {map_name}; exec bash'
            ])
            self.log(f"Map saved to: {map_path}")
        except Exception as e:
            self.log(f"Failed to save map: {e}")
    
    def go_back(self):
        self.log("Returning to main UI")
        if self.mapping_process:
            self.log("Stopping SLAM mapping process...")
            try:
                self.mapping_process.terminate()
                self.mapping_process.wait(timeout=3)
                self.log("SLAM process terminated")
            except:
                self.mapping_process.kill()
                self.log("SLAM process killed")
        subprocess.Popen(['python3', '/home/khoaiuh/zackon_build_up/robot_ui/startup_layout.py', '--skip-micro-ros'])
        self.close()
    
    def log(self, message):
        self.log_text.append(message)
    
    def closeEvent(self, event):
        if self.mapping_process:
            try:
                self.mapping_process.terminate()
            except:
                pass
        event.accept()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    app.setFont(QFont("Fira Sans", 12))
    window = NewMapUI()
    window.showMaximized()
    sys.exit(app.exec())
