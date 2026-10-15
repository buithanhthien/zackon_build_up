"""Exercise real GUI boundaries; AI, speech and robot services remain inert."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "robot_ui"))

import threading
import time
import unittest
from unittest.mock import Mock, patch

from PyQt6.QtCore import QTimer, Qt, QThread
from PyQt6.QtTest import QTest
from PyQt6.QtWidgets import QApplication, QWidget, QPushButton, QLabel
from chat_panel_widget import ChatPanel
from mango_companion import MangoCompanion, Activity


class ActivityTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        self.mango = MangoCompanion()
        self.mango.resize(420, 320)
        self.mango.show()
        # Real QObject/signals/slots without initializing hardware or models.
        self.panel = ChatPanel.__new__(ChatPanel)
        QWidget.__init__(self.panel)
        p = self.panel
        p.activity_changed.connect(self.mango.set_activity)
        p._voice_engine = Mock()
        p._typing_timer = QTimer(p)
        p._ai_thread = p._intent_thread = None
        p._pending_reply = None
        p._recording_active = p._voice_enabled = p._did_speak = False
        p._companion_activity = 'idle'
        p._companion_waiting = False
        p._language = p._conversation_language = 'vi'
        p._chat_history = []
        p._correction_memory = p._local_search = None
        p._waypoints_provider = lambda: []
        p.voice_btn = QPushButton(p)
        p.voice_status_label = QLabel(p)
        self.addCleanup(p.deleteLater)
        self.addCleanup(self.mango.deleteLater)
        self.addCleanup(self.mango.close)

    def test_voice_capture_and_end_without_transcript(self):
        p, m = self.panel, self.mango
        for state, expected in [('[>>] LISTENING', 'listening'),
                                ('[..] THINKING', 'thinking'), ('', 'idle')]:
            p._on_voice_state_changed(state)
            self.assertEqual(m.display_state, expected)
        p._on_voice_state_changed('[<<] SPEAKING')
        self.assertFalse(m.speaking)
        m.set_speaking(True)
        self.assertEqual(m.display_state, 'speaking')
        m.set_speaking(False)
        self.assertEqual(m.display_state, 'idle')

    def test_text_and_intent_wait_survive_capture_end(self):
        p, m = self.panel, self.mango
        for method, worker in [('_ask_ai', '_AIChatWorker'), ('_classify_intent', '_IntentWorker')]:
            with patch('chat_panel_widget.QThread') as thread, patch('chat_panel_widget.' + worker):
                getattr(p, method)('xin chào')
                self.assertEqual(m.display_state, 'thinking')
                p._on_voice_state_changed('')
                self.assertEqual(m.display_state, 'thinking')
                p._on_error('offline')
                self.assertEqual(m.display_state, 'error')
                p._ai_thread = p._intent_thread = None

    def test_empty_reply_error_cancel_and_success(self):
        p, m = self.panel, self.mango
        p._set_companion_activity('thinking')
        p._on_response('😊')
        self.assertEqual(m.display_state, 'idle')
        p._voice_engine.speak_in_language.assert_not_called()
        p._set_companion_activity('thinking')
        p._on_response('Xin chào')
        self.assertEqual(m.display_state, 'thinking')
        self.assertFalse(m.speaking)
        p._on_utterance_finished('error')
        p._on_voice_state_changed('')
        self.assertEqual(m.display_state, 'error')
        p._cancel_companion()
        self.assertEqual(m.display_state, 'idle')
        p._on_utterance_finished('success')
        p._on_voice_state_changed('')
        self.assertEqual(m.display_state, 'success')
        m._activity_started = time.monotonic() - 2
        m._tick()
        self.assertEqual(m.display_state, 'idle')
        p._companion_waiting = True
        p._set_companion_activity('thinking')
        p._on_utterance_finished('success')
        self.assertEqual(m.display_state, 'thinking')

    def test_interaction_priority_and_new_activity(self):
        m = self.mango
        point = m.character_rect().center().toPoint()
        QTest.mousePress(m, Qt.MouseButton.LeftButton, pos=point)
        QTest.mouseMove(m, point + m.rect().bottomRight())
        position = m._position
        m.set_activity('error')
        m.set_speaking(True)
        self.assertEqual(m.display_state, 'dragging')
        self.assertEqual(m._position, position)
        self.assertTrue(m.speaking)
        QTest.mouseRelease(m, Qt.MouseButton.LeftButton, pos=point)
        m.set_activity('listening')
        m._state_started = time.monotonic() - 2
        m._tick()
        self.assertEqual(m.display_state, 'speaking')
        m.set_speaking(False)
        self.assertEqual(m.display_state, 'listening')
        with self.assertRaises(ValueError):
            m.set_activity('unknown')
        with self.assertRaises(ValueError):
            m.set_activity('speaking')

    def test_hidden_transients_expire_and_timer_stops(self):
        m = self.mango
        m.set_activity('thinking')
        m.hide()
        self.assertEqual(m.activity, Activity.IDLE)
        self.assertFalse(m._timer.isActive())
        m.set_activity('error')
        m._activity_started = time.monotonic() - 2
        m.show()
        self.assertEqual(m.activity, Activity.IDLE)
        self.assertTrue(m._timer.isActive())

    def test_distinct_poses_and_queued_delivery(self):
        m = self.mango
        with patch('mango_companion.time.monotonic', return_value=m._started + 1):
            images = []
            for state in ('idle', 'listening', 'thinking', 'success', 'error'):
                m.set_activity(state)
                images.append(m.grab().toImage())
            for i, image in enumerate(images):
                for other in images[i + 1:]:
                    self.assertNotEqual(image, other)
        m.set_activity('idle')
        thread = threading.Thread(target=lambda: self.panel.activity_changed.emit('thinking'))
        thread.start()
        thread.join()
        self.assertEqual(m.activity, Activity.IDLE)
        self.app.processEvents()
        self.assertEqual(m.activity, Activity.THINKING)
        self.assertEqual(m.thread(), QThread.currentThread())
