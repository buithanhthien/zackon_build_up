"""Reusable touch keyboard for a QLineEdit; the owner handles submission.

Supports basic Latin, digits and punctuation (no Vietnamese composition).
Each instance owns its Shift/symbol state and never redirects to another field.
"""
from PyQt6.QtCore import Qt, pyqtSignal
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QSizePolicy


class VirtualKeyboard(QWidget):
    submitted = pyqtSignal()

    def __init__(self, target, enter_label="Gửi", parent=None):
        super().__init__(parent)
        self.target = target
        self.enter_label = enter_label
        self.setObjectName("virtual-keyboard")
        self.setStyleSheet("""
            QWidget#virtual-keyboard {
                background-color: #e8edf7;
                border: 1px solid #c7d5f3;
                border-radius: 10px;
            }
            QWidget#virtual-keyboard QPushButton {
                min-height: 42px;
                background-color: #ffffff;
                color: #172554;
                border: 1px solid #c7d5f3;
                border-radius: 6px;
                font-size: 16px;
                font-weight: 600;
            }
            QWidget#virtual-keyboard QPushButton:hover {
                background-color: #dbeafe;
            }
        """)
        self._virtual_keyboard_layout = QVBoxLayout(self)
        self._virtual_keyboard_layout.setContentsMargins(8, 8, 8, 8)
        self._virtual_keyboard_layout.setSpacing(4)
        self._keyboard_symbols = False
        self._keyboard_shift = False
        self._render_virtual_keyboard()

    def _render_virtual_keyboard(self):
        while self._virtual_keyboard_layout.count():
            row = self._virtual_keyboard_layout.takeAt(0).layout()
            while row.count():
                button = row.takeAt(0).widget()
                button.hide()
                button.deleteLater()
            row.deleteLater()

        if self._keyboard_symbols:
            rows = [
                [(digit, digit, 1) for digit in "1234567890"],
                [(symbol, symbol, 1) for symbol in "@-'?!, ." if symbol != " "],
                [("letters", "ABC", 1), ("space", "space", 4),
                 ("backspace", "⌫", 1), ("enter", self.enter_label, 1)],
            ]
        else:
            rows = [
                [(letter, letter.upper() if self._keyboard_shift else letter, 1)
                 for letter in "qwertyuiop"],
                [(letter, letter.upper() if self._keyboard_shift else letter, 1)
                 for letter in "asdfghjkl"],
                [("shift", "Shift", 2)] +
                [(letter, letter.upper() if self._keyboard_shift else letter, 1)
                 for letter in "zxcvbnm"] +
                [("backspace", "⌫", 2)],
                [("symbols", "?123", 2), (",", ",", 1),
                 ("space", "space", 5), (".", ".", 1),
                 ("enter", self.enter_label, 2)],
            ]

        for row in rows:
            row_layout = QHBoxLayout()
            row_layout.setSpacing(4)
            for key, label, stretch in row:
                button = QPushButton(label)
                button.setObjectName(f"key-{key}")
                button.setFocusPolicy(Qt.FocusPolicy.NoFocus)
                button.setAutoDefault(False)
                button.setMinimumWidth(32)
                button.setSizePolicy(
                    QSizePolicy.Policy.Expanding,
                    QSizePolicy.Policy.Fixed,
                )
                button.clicked.connect(
                    lambda checked=False, key=key: self._handle_virtual_key(key)
                )
                row_layout.addWidget(button, stretch)
            self._virtual_keyboard_layout.addLayout(row_layout)

    def _handle_virtual_key(self, key):
        if key == "symbols":
            self._keyboard_symbols = True
            self._render_virtual_keyboard()
        elif key == "letters":
            self._keyboard_symbols = False
            self._render_virtual_keyboard()
        elif key == "shift":
            self._keyboard_shift = not self._keyboard_shift
            self._render_virtual_keyboard()
        elif key == "backspace":
            self.target.backspace()
        elif key == "space":
            self.target.insert(" ")
        elif key == "enter":
            self.submitted.emit()
        else:
            value = key.upper() if self._keyboard_shift else key
            self.target.insert(value)
            was_shifted = self._keyboard_shift
            self._keyboard_shift = False
            if was_shifted and not self._keyboard_symbols:
                self._render_virtual_keyboard()
