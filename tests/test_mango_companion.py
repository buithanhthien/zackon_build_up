"""Real Qt input, containment and lifecycle without ROS or network calls."""
import sys
import time
from pathlib import Path
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "robot_ui"))
from PyQt6.QtCore import QPoint, Qt
from PyQt6.QtTest import QTest
from PyQt6.QtWidgets import QApplication
from mango_companion import MangoCompanion


class MangoCompanionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        self.widget = MangoCompanion()
        self.widget.resize(420, 320)
        self.widget.show()
        self.app.processEvents()
        self.addCleanup(self.widget.deleteLater)
        self.addCleanup(self.widget.close)

    def test_click_and_keyboard_react_and_return_to_idle(self):
        w = self.widget
        point = w.character_rect().center().toPoint()
        QTest.mousePress(w, Qt.MouseButton.LeftButton, pos=point)
        self.assertEqual(w.state, "touched")
        QTest.mouseRelease(w, Qt.MouseButton.LeftButton, pos=point)
        self.assertEqual(w.state, "happy")
        w._state_started = time.monotonic() - 2
        w._tick()
        self.assertEqual(w.state, "idle")

        QTest.keyClick(w, Qt.Key.Key_Space)
        self.assertEqual(w.state, "happy")
        w._set_state("idle")
        QTest.mouseClick(w, Qt.MouseButton.LeftButton, pos=QPoint(2, 2))
        self.assertEqual(w.state, "idle")
        QTest.mouseClick(w, Qt.MouseButton.RightButton, pos=point)
        self.assertEqual(w.state, "idle")

    def test_speaking_animates_mouth_and_restores_expression(self):
        w = self.widget
        with patch("mango_companion.time.monotonic", return_value=100.):
            normal = w.grab().toImage()
            w.set_speaking(True)
            first = w.grab().toImage()
            w._speech_started -= .06
            second = w.grab().toImage()
            self.assertNotEqual(first, second)
            self.assertNotEqual(normal, first)
            QTest.keyClick(w, Qt.Key.Key_Space)
            self.assertTrue(w.speaking)
            self.assertEqual(w.state, "happy")
            w.set_speaking(False)
            w._set_state("idle")
            self.assertEqual(normal, w.grab().toImage())

    def test_drag_outside_is_bounded_and_survives_resize(self):
        w = self.widget
        for target in (QPoint(-500, -500), QPoint(2000, 2000)):
            QTest.mousePress(w, Qt.MouseButton.LeftButton, pos=w.character_rect().center().toPoint())
            QTest.mouseMove(w, target)
            self.assertEqual(w.state, "dragging")
            self.assertTrue(w.rect().contains(w.character_rect().toAlignedRect()))
            QTest.mouseRelease(w, Qt.MouseButton.LeftButton, pos=target)
            self.assertEqual(w.state, "released")
        for width, height in ((200, 110), (900, 600), (140, 260)):
            w.resize(width, height)
            self.app.processEvents()
            rect = w.character_rect()
            self.assertTrue(w.rect().contains(rect.toAlignedRect()))
            self.assertAlmostEqual(rect.width() / rect.height(), 240 / 260)
            self.assertFalse(w.grab().isNull())

    def test_animation_runs_only_while_visible_and_hide_cancels_drag(self):
        w = self.widget
        self.assertTrue(w._timer.isActive())
        idle = w.grab().toImage()
        w._started -= .8
        self.assertNotEqual(idle, w.grab().toImage())
        QTest.mousePress(w, Qt.MouseButton.LeftButton, pos=w.character_rect().center().toPoint())
        w.hide()
        self.assertFalse(w._timer.isActive())
        self.assertIsNone(w._press)
        self.assertEqual(w.state, "idle")
        w.show()
        self.assertTrue(w._timer.isActive())
        w.close()
        self.assertFalse(w._timer.isActive())
