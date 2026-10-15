"""Deterministic roaming and native Qt masked-overlay input routing."""
import random
import sys
from pathlib import Path
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'robot_ui'))
from PyQt6.QtCore import QPointF, Qt
from PyQt6.QtTest import QTest, QSignalSpy
from PyQt6.QtWidgets import QApplication, QWidget, QPushButton
from mango_companion import MangoCompanion


class RoamingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        self.now = 100.
        self.m = MangoCompanion(clock=lambda: self.now, rng=random.Random(4))
        self.m.resize(1024, 720)
        self.m.show()
        self.m._timer.stop()
        self.addCleanup(self.m.close)
        self.addCleanup(self.m.deleteLater)

    def step(self, seconds):
        for _ in range(round(seconds / .025)):
            self.now += .025
            self.m._tick()

    def test_seeded_run_turn_jump_landing_and_rest(self):
        states, directions, heights = set(), set(), []
        for _ in range(7200):  # Three simulated minutes, no wall-clock waiting.
            self.step(.025)
            states.add(self.m.motion)
            directions.add(self.m.facing)
            heights.append(self.m.jump_height)
            self.assertTrue(self.m.rect().contains(self.m.character_rect().toAlignedRect()))
        self.assertEqual(states, {'rest', 'turn', 'run', 'jump', 'landing'})
        self.assertEqual(directions, {-1, 1})
        self.assertGreater(max(heights), 15)
        self.assertLessEqual(max(heights), 16)

    def test_arc_and_activity_playback_drag_preempt_motion(self):
        m = self.m
        m.motion, m._duration = 'jump', 2.
        for t, expected in ((0, 0), (.5, 12), (1, 16), (1.5, 12), (2, 0)):
            m._motion_age = t
            self.assertAlmostEqual(m.jump_height, expected)
        for activity in ('listening', 'thinking', 'success', 'error'):
            m.set_activity(activity)
            pos = QPointF(m._position)
            self.step(.5)
            self.assertEqual(m._position, pos)
            self.assertEqual(m.motion, 'rest')
        m.set_activity('idle')
        m.set_speaking(True)
        pos = QPointF(m._position)
        self.step(5)
        self.assertEqual(m._position, pos)
        m.set_speaking(False)
        point = m.character_rect().center().toPoint()
        QTest.mousePress(m, Qt.MouseButton.LeftButton, pos=point)
        QTest.mouseMove(m, point + m.rect().bottomRight())
        pos = QPointF(m._position)
        self.step(5)
        self.assertEqual(m._position, pos)
        QTest.mouseRelease(m, Qt.MouseButton.LeftButton, pos=point)
        self.step(1)
        self.assertEqual(m._position, pos)
        self.step(8)
        self.assertNotEqual(m._position, pos)

    def test_resize_scale_four_edges_and_hide_resume(self):
        m = self.m
        for size in ((1024, 720), (1280, 720), (120, 110)):
            m.resize(*size)
            for x, y in ((0, 0), (1, 0), (0, 1), (1, 1)):
                m._position = QPointF(x, y)
                rect = m.character_rect()
                self.assertTrue(m.rect().contains(rect.toAlignedRect()))
                self.assertAlmostEqual(rect.width()/rect.height(), 240/260)
            old_scale = min((m.width()-16)/280, (m.height()-16)/300, 1.25)
            self.assertAlmostEqual(m.character_rect().width()/240, old_scale*.65)
        pos = QPointF(m._position)
        m.hide()
        self.now += 1000
        m.show()
        self.assertEqual(m._position, pos)
        self.assertTrue(m._timer.isActive())

    def test_mask_routes_click_to_button_even_under_character(self):
        host = QWidget()
        host.resize(1024, 720)
        home = QWidget(host)
        home.setGeometry(100, 100, 400, 300)
        button = QPushButton('Stop', host)
        button.setGeometry(400, 300, 180, 60)
        m = self.m
        m.attach_to(host, [button], home)
        host.show()
        self.app.processEvents()
        m._place(QPointF(button.geometry().center()))
        m._sync_mask()
        self.assertIs(host.childAt(button.geometry().center()), m)
        spy = QSignalSpy(button.clicked)
        QTest.mouseClick(m, Qt.MouseButton.LeftButton,
                         pos=button.geometry().center())
        self.assertEqual(len(spy), 1)
        # Empty mask must never accidentally turn into a full-window input shield.
        button.setGeometry(host.rect())
        m._sync_mask()
        self.assertIs(host.childAt(QPointF(0, 0).toPoint()), button)
        m.setParent(None)
        host.close()
        host.deleteLater()
