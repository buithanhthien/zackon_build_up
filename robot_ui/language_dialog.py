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
from styles import DIALOG_STYLESHEET

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

        self.resize(
            480,
            500
        )

        self.setStyleSheet(DIALOG_STYLESHEET)
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

        title.setObjectName("title")

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

        self.btn_select.setObjectName("primary-btn")

        self.btn_cancel.setObjectName("secondary-btn")

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