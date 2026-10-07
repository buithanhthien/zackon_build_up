#!/usr/bin/env python3
import os
from PyQt6.QtWidgets import (QDialog, QVBoxLayout, QHBoxLayout, QPushButton,
                             QListWidget, QLabel, QListWidgetItem)
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QFont
from styles import DIALOG_STYLESHEET
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from config import SOURCE_PATH



class LoadMapDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.selected_map = None
        self.maps_dir = f'{SOURCE_PATH}/src/view_robot/maps'
        self.init_ui()

    def init_ui(self):
        self.setWindowTitle("Tải bản đồ")
        self.setModal(True)
        self.resize(480, 560)
        self.setStyleSheet(DIALOG_STYLESHEET)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(20, 20, 20, 20)
        layout.setSpacing(12)

        title = QLabel("Chọn bản đồ")
        title.setObjectName("title")
        title.setFont(QFont("DM Sans", 11))
        layout.addWidget(title)

        self.map_list = QListWidget()
        self.map_list.setFont(QFont("JetBrains Mono", 15))
        self.map_list.currentItemChanged.connect(self.on_item_clicked)
        layout.addWidget(self.map_list)

        self.load_maps()

        btn_layout = QHBoxLayout()
        btn_layout.setSpacing(8)

        self.btn_ok = QPushButton("Tải bản đồ")
        self.btn_ok.setObjectName("ok-btn")
        self.btn_ok.setFont(QFont("JetBrains Mono", 15))
        self.btn_ok.clicked.connect(self.accept)
        self.btn_ok.setEnabled(False)

        self.btn_cancel = QPushButton("Hủy")
        self.btn_cancel.setObjectName("cancel-btn")
        self.btn_cancel.setFont(QFont("JetBrains Mono", 15))
        self.btn_cancel.clicked.connect(self.reject)

        btn_layout.addWidget(self.btn_ok)
        btn_layout.addWidget(self.btn_cancel)
        layout.addLayout(btn_layout)

    def load_maps(self):
        if not os.path.exists(self.maps_dir):
            return
        for map_name in sorted(f[:-5] for f in os.listdir(self.maps_dir) if f.endswith('.yaml')):
            self.map_list.addItem(QListWidgetItem(map_name))

    def on_item_clicked(self, item, previous=None):
        self.selected_map = item.text() if item else None
        self.btn_ok.setEnabled(item is not None)

    def get_selected_map(self):
        return self.selected_map
