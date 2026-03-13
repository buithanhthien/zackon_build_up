#!/usr/bin/env python3
import os
from PyQt6.QtWidgets import (QDialog, QVBoxLayout, QHBoxLayout, QPushButton, 
                             QListWidget, QLabel, QListWidgetItem)
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QFont
import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from config import SOURCE_PATH


class LoadMapDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.selected_map = None
        self.maps_dir = f'{SOURCE_PATH}/src/view_robot/maps'
        self.init_ui()
        
    def init_ui(self):
        self.setWindowTitle("Load Map")
        self.setModal(True)
        self.resize(500, 600)
        
        layout = QVBoxLayout(self)
        
        title = QLabel("Select a map to load:")
        title.setFont(QFont("Fira Sans", 16, QFont.Weight.Bold))
        layout.addWidget(title)
        
        self.map_list = QListWidget()
        self.map_list.setFont(QFont("Fira Sans", 14))
        self.map_list.itemClicked.connect(self.on_item_clicked)
        layout.addWidget(self.map_list)
        
        self.load_maps()
        
        btn_layout = QHBoxLayout()
        
        self.btn_ok = QPushButton("OK")
        self.btn_ok.setFont(QFont("Fira Sans", 14))
        self.btn_ok.setMinimumHeight(50)
        self.btn_ok.clicked.connect(self.accept)
        self.btn_ok.setEnabled(False)
        
        self.btn_cancel = QPushButton("Cancel")
        self.btn_cancel.setFont(QFont("Fira Sans", 14))
        self.btn_cancel.setMinimumHeight(50)
        self.btn_cancel.clicked.connect(self.reject)
        
        btn_layout.addWidget(self.btn_ok)
        btn_layout.addWidget(self.btn_cancel)
        
        layout.addLayout(btn_layout)
        
    def load_maps(self):
        if not os.path.exists(self.maps_dir):
            return
            
        yaml_files = [f[:-5] for f in os.listdir(self.maps_dir) 
                      if f.endswith('.yaml')]
        yaml_files.sort()
        
        for map_name in yaml_files:
            item = QListWidgetItem(map_name)
            self.map_list.addItem(item)
    
    def on_item_clicked(self, item):
        self.selected_map = item.text()
        self.btn_ok.setEnabled(True)
        
        for i in range(self.map_list.count()):
            list_item = self.map_list.item(i)
            if list_item.text() == self.selected_map:
                list_item.setBackground(Qt.GlobalColor.lightGray)
            else:
                list_item.setBackground(Qt.GlobalColor.white)
    
    def get_selected_map(self):
        return self.selected_map
