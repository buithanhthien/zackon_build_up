#!/usr/bin/env python3

from PyQt6.QtWidgets import (
    QDialog,
    QVBoxLayout,
    QHBoxLayout,
    QLabel,
    QListWidget,
    QListWidgetItem,
    QPushButton,
)

from PyQt6.QtCore import Qt
from PyQt6.QtGui import QFont

from language_config import (
    get_language,
    set_language,
    get_ui_text,
)


class LanguageDialog(QDialog):

    def __init__(self, parent=None):
        super().__init__(parent)

        self.selected_language = None

        self.ui_text = get_ui_text()

        self.setWindowTitle(
            self.ui_text["language_window"]
        )

        self.setFixedSize(
            480,
            500
        )

        self._build_ui()

    def _build_ui(self):

        layout = QVBoxLayout(self)

        layout.setContentsMargins(
            20,
            20,
            20,
            20
        )

        layout.setSpacing(12)

        # ============================================================
        # Tiêu đề
        # ============================================================

        title = QLabel(
            self.ui_text[
                "select_language"
            ]
        )

        title.setFont(
            QFont(
                "JetBrains Mono",
                10,
                QFont.Weight.Bold
            )
        )

        title.setStyleSheet(
            """
            color: #5674c8;
            letter-spacing: 2px;
            padding-bottom: 8px;
            """
        )

        layout.addWidget(title)

        # ============================================================
        # Danh sách ngôn ngữ
        # ============================================================

        self.language_list = QListWidget()

        self.language_list.setFont(
            QFont(
                "Fira Sans",
                12
            )
        )

        self.language_list.setStyleSheet(
            """
            QListWidget {
                background: white;
                border: 1px solid #c9d4ee;
                border-radius: 5px;
                outline: none;
            }

            QListWidget::item {
                height: 50px;
                padding-left: 14px;
                border-bottom: 1px solid #e1e6f2;
                color: #17306d;
            }

            QListWidget::item:selected {
                background: #e7edff;
                color: #17306d;
            }

            QListWidget::item:hover {
                background: #f1f4fc;
            }
            """
        )

        languages = [
            ("Tiếng Việt", "vi"),
            ("English", "en"),
        ]

        current_language = (
            get_language()
        )

        for name, code in languages:

            item = QListWidgetItem(
                name
            )

            item.setData(
                Qt.ItemDataRole.UserRole,
                code
            )

            self.language_list.addItem(
                item
            )

            if code == current_language:

                self.language_list.setCurrentItem(
                    item
                )

        self.language_list.itemDoubleClicked.connect(
            self._select_language
        )

        layout.addWidget(
            self.language_list
        )

        # ============================================================
        # Buttons
        # ============================================================

        button_layout = QHBoxLayout()

        button_layout.setSpacing(10)

        self.btn_select = QPushButton(
            self.ui_text["select"]
        )

        self.btn_cancel = QPushButton(
            self.ui_text["cancel"]
        )

        self.btn_select.setFixedHeight(
            46
        )

        self.btn_cancel.setFixedHeight(
            46
        )

        self.btn_select.setStyleSheet(
            """
            QPushButton {
                background: #dce5fb;
                color: #5674c8;
                border: none;
                border-radius: 6px;
            }

            QPushButton:hover {
                background: #cddaf7;
            }

            QPushButton:pressed {
                background: #bdccf2;
            }
            """
        )

        self.btn_cancel.setStyleSheet(
            """
            QPushButton {
                background: white;
                color: #5674c8;
                border: 1px solid #aebee7;
                border-radius: 6px;
            }

            QPushButton:hover {
                background: #f1f4fc;
            }
            """
        )

        self.btn_select.clicked.connect(
            self._select_language
        )

        self.btn_cancel.clicked.connect(
            self.reject
        )

        button_layout.addWidget(
            self.btn_select
        )

        button_layout.addWidget(
            self.btn_cancel
        )

        layout.addLayout(
            button_layout
        )

    def _select_language(self):

        item = (
            self.language_list.currentItem()
        )

        if item is None:
            return

        language = item.data(
            Qt.ItemDataRole.UserRole
        )

        if set_language(language):

            self.selected_language = language

            self.accept()

    def get_selected_language(self):

        return self.selected_language