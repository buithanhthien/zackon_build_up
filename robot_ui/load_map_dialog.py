#!/usr/bin/env python3
import os
from PyQt6.QtWidgets import (QDialog, QVBoxLayout, QHBoxLayout, QPushButton,
                             QListWidget, QLabel, QListWidgetItem)
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QFont
import sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from config import SOURCE_PATH

STYLESHEET = """
    QDialog {
        background-color: #0d0f12;
        color: #e8ecf0;
    }
    QLabel#title {
        color: #6b7a99;
        font-size: 11px;
        letter-spacing: 2px;
        padding-bottom: 8px;
    }
    QListWidget {
        background-color: #1c2030;
        color: #e8ecf0;
        border: 1px solid #2a3040;
        border-radius: 4px;
        font-size: 15px;
        outline: none;
    }
    QListWidget::item {
        padding: 12px 16px;
        border-bottom: 1px solid #2a3040;
    }
    QListWidget::item:hover {
        background-color: #1a1f2e;
        color: #e8ecf0;
    }
    QListWidget::item:selected {
        background-color: #1c2030;
        color: #00e5ff;
        border-left: 3px solid #00e5ff;
    }
    QPushButton#ok-btn {
        background-color: #1c2030;
        color: #00e5ff;
        border: 1px solid #00e5ff;
        border-radius: 4px;
        font-size: 15px;
        min-height: 44px;
    }
    QPushButton#ok-btn:hover {
        background-color: #1a2a3a;
    }
    QPushButton#ok-btn:disabled {
        color: #3a4460;
        border: 1px solid #3a4460;
    }
    QPushButton#cancel-btn {
        background-color: transparent;
        color: #6b7a99;
        border: 1px solid #2a3040;
        border-radius: 4px;
        font-size: 15px;
        min-height: 44px;
    }
    QPushButton#cancel-btn:hover {
        background-color: #1a1f2e;
        color: #e8ecf0;
        border: 1px solid #3a4460;
    }
"""


class LoadMapDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.selected_map = None
        self.maps_dir = f'{SOURCE_PATH}/src/view_robot/maps'
        self.init_ui()

    def init_ui(self):
        self.setWindowTitle("Load Map")
        self.setModal(True)
        self.resize(480, 560)
        self.setStyleSheet(STYLESHEET)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(20, 20, 20, 20)
        layout.setSpacing(12)

        title = QLabel("SELECT MAP")
        title.setObjectName("title")
        title.setFont(QFont("DM Sans", 11))
        layout.addWidget(title)

        self.map_list = QListWidget()
        self.map_list.setFont(QFont("JetBrains Mono", 15))
        self.map_list.itemClicked.connect(self.on_item_clicked)
        layout.addWidget(self.map_list)

        self.load_maps()

        btn_layout = QHBoxLayout()
        btn_layout.setSpacing(8)

        self.btn_ok = QPushButton("Load")
        self.btn_ok.setObjectName("ok-btn")
        self.btn_ok.setFont(QFont("JetBrains Mono", 15))
        self.btn_ok.clicked.connect(self.accept)
        self.btn_ok.setEnabled(False)

        self.btn_cancel = QPushButton("Cancel")
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

    def on_item_clicked(self, item):
        self.selected_map = item.text()
        self.btn_ok.setEnabled(True)

    def get_selected_map(self):
        return self.selected_map
