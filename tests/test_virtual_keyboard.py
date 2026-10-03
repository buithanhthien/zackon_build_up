"""Touch input, modal isolation and route persistence without ROS processes."""
import json
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import Mock

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'robot_ui'))
from PyQt6.QtCore import Qt, QCoreApplication, QEvent
from PyQt6.QtTest import QTest
from PyQt6.QtWidgets import QApplication, QDialog, QLineEdit, QPushButton, QVBoxLayout, QWidget
from virtual_keyboard import VirtualKeyboard
from waypoint_dialogs import NewPathDialog


class VirtualKeyboardTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.route_file = Path(self.tmp.name) / 'routes.json'
        self.parent = QWidget()
        layout = QVBoxLayout(self.parent)
        self.chat_input = QLineEdit('Tin nhan dang soan')
        self.chat_keyboard = VirtualKeyboard(self.chat_input)
        self.send = Mock()
        self.chat_keyboard.submitted.connect(self.send)
        layout.addWidget(self.chat_input)
        layout.addWidget(self.chat_keyboard)
        self.parent.show()
        self.dialog = NewPathDialog({'home': {'map_name': 'test'}}, 'test',
                                    str(self.route_file), self.parent)
        self.dialog.show()
        self.app.processEvents()
        self.addCleanup(self.parent.deleteLater)
        self.addCleanup(self.parent.hide)

    def press(self, keyboard, *keys):
        for key in keys:
            # Process deleted buttons after each rerender so lookup tests the live UI.
            QCoreApplication.sendPostedEvents(None, QEvent.Type.DeferredDelete)
            button = keyboard.findChild(QPushButton, f'key-{key}')
            self.assertIsNotNone(button, key)
            QTest.mouseClick(button, Qt.MouseButton.LeftButton)
            self.app.processEvents()

    def enable_keyboard(self):
        QTest.mouseClick(self.dialog.keyboard_toggle, Qt.MouseButton.LeftButton)
        self.app.processEvents()
        self.assertTrue(self.dialog.virtual_keyboard.isVisible())
        return self.dialog.virtual_keyboard

    def test_touch_only_name_and_save(self):
        keyboard = self.enable_keyboard()
        item_rect = self.dialog.goal_list.visualItemRect(self.dialog.goal_list.item(0))
        QTest.mouseClick(self.dialog.goal_list.viewport(), Qt.MouseButton.LeftButton,
                         pos=item_rect.center())
        self.press(keyboard, 'shift', 't', 'o', 'u', 'r', 'space',
                   'symbols', '1', '-', '2', 'letters', 'enter')
        self.assertEqual(json.loads(self.route_file.read_text()),
                         {'Tour 1-2': {'map_name': 'test', 'sequence': ['home']}})
        self.assertEqual(self.dialog.result(), QDialog.DialogCode.Accepted)
        self.send.assert_not_called()
        self.assertEqual(self.chat_input.text(), 'Tin nhan dang soan')

    def test_cursor_selection_shift_space_digits_punctuation_backspace(self):
        keyboard = self.enable_keyboard()
        field = self.dialog.name_input
        self.press(keyboard, 'a', 'c')
        field.setCursorPosition(1)
        self.press(keyboard, 'shift', 'b')
        self.assertEqual(field.text(), 'aBc')
        self.press(keyboard, 'backspace')
        self.assertEqual(field.text(), 'ac')
        field.setSelection(0, 2)
        self.press(keyboard, 'x', 'y')
        self.assertEqual(field.text(), 'xy')
        field.setSelection(0, 1)
        self.press(keyboard, 'backspace')
        self.assertEqual(field.text(), 'y')
        self.press(keyboard, 'backspace')  # cursor at zero
        self.assertEqual(field.text(), 'y')
        field.setCursorPosition(1)
        self.press(keyboard, 'space', 'symbols', '0', '@', '-', "'", '?', '!', ',', '.', 'letters', 'z')
        self.assertEqual(field.text(), "y 0@-'?!,.z")
        self.assertTrue(field.hasFocus())

    def test_enter_requires_name_and_sequence_and_physical_enter_saves(self):
        keyboard = self.enable_keyboard()
        self.press(keyboard, 'enter')
        self.assertFalse(self.route_file.exists())
        self.press(keyboard, 'a', 'enter')
        self.assertFalse(self.route_file.exists())
        self.dialog._add_goal(self.dialog.goal_list.item(0))
        QTest.keyClick(self.dialog.name_input, Qt.Key.Key_Return)
        self.assertEqual(json.loads(self.route_file.read_text())['a']['sequence'], ['home'])
        self.send.assert_not_called()

    def test_toggle_cancel_preserves_file_and_chat_state(self):
        self.route_file.write_text('{"existing": {}}')
        # Keep the chat keyboard in symbols with Shift enabled while modal is open.
        self.chat_keyboard._handle_virtual_key('shift')
        self.chat_keyboard._handle_virtual_key('symbols')
        before = (self.chat_input.text(), self.chat_input.cursorPosition(),
                  self.chat_keyboard.isHidden(),
                  self.chat_keyboard._keyboard_shift, self.chat_keyboard._keyboard_symbols)
        keyboard = self.enable_keyboard()
        self.press(keyboard, 'a')
        QTest.mouseClick(self.dialog.keyboard_toggle, Qt.MouseButton.LeftButton)
        self.assertTrue(keyboard.isHidden())
        self.assertEqual(self.dialog.name_input.text(), 'a')
        cancel = next(b for b in self.dialog.findChildren(QPushButton) if b.text() == 'Hủy')
        QTest.mouseClick(cancel, Qt.MouseButton.LeftButton)
        self.assertEqual(self.dialog.result(), QDialog.DialogCode.Rejected)
        self.assertEqual(self.route_file.read_text(), '{"existing": {}}')
        self.assertEqual(before, (self.chat_input.text(), self.chat_input.cursorPosition(),
                         self.chat_keyboard.isHidden(),
                         self.chat_keyboard._keyboard_shift, self.chat_keyboard._keyboard_symbols))
        self.send.assert_not_called()

    def test_chat_keyboard_still_edits_and_submits(self):
        self.dialog.reject()
        self.chat_input.clear()
        self.press(self.chat_keyboard, 'shift', 'h', 'i', 'enter')
        self.assertEqual(self.chat_input.text(), 'Hi')
        self.send.assert_called_once_with()
        self.assertEqual(self.dialog.name_input.text(), '')

    def test_switching_modes_does_not_accumulate_buttons(self):
        keyboard = self.enable_keyboard()
        for _ in range(5):
            self.press(keyboard, 'symbols', 'letters', 'shift', 'shift')
        QCoreApplication.sendPostedEvents(None, QEvent.Type.DeferredDelete)
        self.assertEqual(len(keyboard.findChildren(QPushButton)), 33)
        self.assertEqual(keyboard._virtual_keyboard_layout.count(), 4)


if __name__ == '__main__':
    unittest.main()
