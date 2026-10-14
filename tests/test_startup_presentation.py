"""Exercise real Startup layout methods with inert ROS/voice dependencies."""
import ast
import importlib.util
from pathlib import Path
import unittest

from PyQt6.QtWidgets import QApplication, QPushButton
from PyQt6.QtTest import QTest, QSignalSpy

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location("startup_preview", ROOT / "tool" / "preview_startup_ui.py")
preview = importlib.util.module_from_spec(spec)
spec.loader.exec_module(preview)


class StartupPresentationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])
        cls.source = (ROOT / "robot_ui" / "startup_layout.py").read_text()

    def make_window(self, language="vi"):
        window = preview.make_window(self.source, language)
        self.addCleanup(window.deleteLater)
        self.addCleanup(window.hide)
        return window

    def test_learning_card_states_and_accessibility(self):
        from unittest.mock import Mock
        for language, title in (("vi", "Chế độ học"), ("en", "Learning mode")):
            w = self.make_window(language)
            w.hindsight = Mock()
            for state, enabled in (("checking", False), ("starting", False),
                                   ("stopping", False), ("on", True),
                                   ("off", True), ("error", True)):
                w._update_learning_status(state, '')
                self.assertEqual(w.learning_toggle.isEnabled(), enabled)
                self.assertIn(title, w.learning_card['widget'].accessibleName())
            w._update_learning_status('on', '')
            w.learning_toggle.click()
            w.hindsight.set_enabled.assert_called_once_with(False)
            w._update_learning_status('off', 'command-timeout')
            self.assertIn('OFF', w.learning_card['state'].text())
            self.assertIn('command-timeout', w.learning_card['state'].toolTip())

    def test_resolutions_languages_and_touch_keyboard_fit(self):
        for language in ("vi", "en"):
            for width, height in ((1024, 720), (1280, 720), (1920, 1080)):
                with self.subTest(language=language, size=(width, height)):
                    w = self.make_window(language)
                    w.resize(width, height)
                    w.show()
                    self.app.processEvents()
                    for keyboard in (False, True):
                        w.keyboard_toggle.setChecked(keyboard)
                        w._append_chat_message("[Bé Son] " + "Nội dung dài có dấu. " * 160)
                        self.app.processEvents()
                        self.assertEqual((w.width(), w.height()), (width, height))
                        self.assertGreaterEqual(w.chat_history_box.height(), 80)
                        mango = w.mango_companion
                        self.assertGreaterEqual(mango.height(), 110)
                        self.assertTrue(mango.rect().contains(mango.character_rect().toAlignedRect()))
                        mango_right = mango.mapTo(w, mango.rect().topRight()).x()
                        chat_left = w.chat_history_box.mapTo(w, w.chat_history_box.rect().topLeft()).x()
                        self.assertLess(mango_right, chat_left)
                        for button in w.findChildren(QPushButton):
                            if button.isVisible():
                                point = button.mapTo(w, button.rect().topLeft())
                                self.assertGreaterEqual(point.x(), 0)
                                self.assertGreaterEqual(point.y(), 0)
                                self.assertLessEqual(point.x() + button.width(), width)
                                self.assertLessEqual(point.y() + button.height(), height)
                                self.assertGreaterEqual(button.height(), 44)
                    w.hide()

    def test_mango_interaction_does_not_trigger_chat_or_voice(self):
        w = self.make_window()
        w.show()
        self.app.processEvents()
        spies = [QSignalSpy(button.clicked) for button in
                 (w.chat_panel.voice_btn, w.chat_panel.interrupt_btn, w.send_chat_button)]
        w.chat_input.setText("Giữ nội dung đang nhập")
        history = w.chat_history_box.toPlainText()
        mango = w.mango_companion
        QTest.mouseClick(mango, preview.QtCore.Qt.MouseButton.LeftButton,
                         pos=mango.character_rect().center().toPoint())
        self.assertEqual(mango.state, "happy")
        self.assertTrue(all(len(spy) == 0 for spy in spies))
        self.assertEqual(w.chat_input.text(), "Giữ nội dung đang nhập")
        self.assertEqual(w.chat_history_box.toPlainText(), history)

    def test_playback_signal_controls_mango_even_while_hidden(self):
        w = self.make_window()
        w.show()
        w.chat_panel._voice_engine.playback_changed.emit(True)
        self.assertTrue(w.mango_companion.speaking)
        w.hide()
        w.chat_panel._voice_engine.playback_changed.emit(False)
        w.show()
        self.assertFalse(w.mango_companion.speaking)

    def test_speaking_locks_mic_across_playback_gaps_and_keeps_stop_available(self):
        from test_voice_recording_flow import _extract_method, CHAT_PANEL
        for language in ("vi", "en"):
            w = self.make_window(language)
            p = w.chat_panel
            p._pending_reply = None
            p._typing_timer = preview.QtCore.QTimer(p)
            p._voice_enabled = False
            p._reset_recording_controls = lambda: _extract_method(
                CHAT_PANEL, "ChatPanel", "_reset_recording_controls")(p)
            state_changed = _extract_method(CHAT_PANEL, "ChatPanel", "_on_voice_state_changed")
            stop_calls = []
            p.interrupt_btn.clicked.connect(lambda: stop_calls.append(True))
            state_changed(p, "[<<] SPEAKING")
            self.assertFalse(p.voice_btn.isEnabled())
            for playing in (True, False, True, False):
                p._voice_engine.playback_changed.emit(playing)
                w._refresh_control_presentation()
                self.assertFalse(p.voice_btn.isEnabled())
                self.assertEqual(p.voice_btn.text(), preview.startup_text(language)["speaking_button"])
                self.assertEqual(p.voice_btn.accessibleName(), p.voice_btn.text())
                self.assertIn(preview.startup_text(language)["mic_locked"], p.voice_btn.toolTip())
                self.assertEqual(p.voice_btn.accessibleDescription(), p.voice_btn.toolTip())
            mic_clicks = QSignalSpy(p.voice_btn.clicked)
            p.voice_btn.click()
            self.assertEqual(len(mic_clicks), 0)
            self.assertTrue(p.interrupt_btn.isEnabled())
            p.interrupt_btn.click()
            self.assertEqual(stop_calls, [True])
            state_changed(p, "")
            w._refresh_control_presentation()
            self.assertTrue(p.voice_btn.isEnabled())
            self.assertEqual(p.voice_btn.text(), preview.startup_text(language)["listen_button"])
            self.assertEqual(p.voice_btn.toolTip(), preview.startup_text(language)["voice_hint"])

    def test_device_states_have_labels_and_diagnostics_remains_available(self):
        w = self.make_window()
        card = w.stm32_card
        for value, state, text in ((None, "checking", "Đang kiểm tra"),
                                   (True, "online", "Hoạt động"),
                                   (False, "offline", "Mất kết nối")):
            w._set_card_status(card, value)
            self.assertEqual(card["widget"].property("state"), state)
            self.assertEqual(card["state"].text(), text)
            self.assertIn(text, card["widget"].accessibleName())
            self.assertTrue(card["detail_btn"].isEnabled())
            self.assertEqual(card["detail_btn"].accessibleName(), "Chẩn đoán STM32")

    def test_presentation_observes_voice_and_cancel_without_changing_controls(self):
        w = self.make_window("en")
        p = w.chat_panel
        p._recording_active = True
        p.voice_btn.setChecked(True)
        p.voice_status_label.setText("[>>] LISTENING")
        w._refresh_control_presentation()
        self.assertIn("Finish recording", p.voice_btn.text())
        self.assertTrue(p.voice_btn.isChecked())
        self.assertTrue(p._recording_active)
        self.assertEqual(p.voice_status_label.text(), "[>>] LISTENING")
        self.assertFalse(hasattr(w, "voice_status_display"))
        p._recording_active = False
        p.voice_btn.setChecked(False)
        p._voice_state = "[..] THINKING"
        w._refresh_control_presentation()
        self.assertEqual(p.voice_btn.text(), "Processing…")
        self.assertFalse(p.voice_btn.isEnabled())
        w._nav_goal_handle = object()
        w._nav_cancel_requested = True
        w._refresh_control_presentation()
        self.assertIn("Waiting", w.stop_hint.text())
        self.assertTrue(p.interrupt_btn.isEnabled())
        w._nav_goal_handle = None
        w._refresh_control_presentation()
        self.assertNotIn("Waiting", w.stop_hint.text())

    def test_thinking_animation_and_startup_refresh_do_not_overwrite_each_other(self):
        tree = ast.parse((ROOT / "robot_ui" / "chat_panel_widget.py").read_text())
        panel_class = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "ChatPanel")
        animate_method = next(n for n in panel_class.body if isinstance(n, ast.FunctionDef) and n.name == "_animate_status")
        namespace = {}
        exec(compile(ast.Module(body=[animate_method], type_ignores=[]), "chat-animation", "exec"), namespace)
        for language, thinking, speech in (("vi", "Đang suy nghĩ", "Đang xử lý…"),
                                          ("en", "Thinking", "Processing…")):
            w = self.make_window(language)
            p = w.chat_panel
            p._typing_dots = 0
            p._typing_timer = preview.QtCore.QTimer(p)
            p.voice_status_label.show()
            p._voice_state = "[..] THINKING"
            p.voice_status_label.setText("[..] THINKING")
            w._refresh_control_presentation()
            self.assertEqual(p.voice_btn.text(), speech)
            self.assertEqual(p.voice_status_label.text(), "[..] THINKING")
            p._typing_timer.start(400)
            # Even before the first animation tick, identify AI processing.
            w._refresh_control_presentation()
            self.assertEqual(p.voice_btn.text(), thinking)
            for tick in range(5):
                namespace["_animate_status"](p)
                source_text = p.voice_status_label.text()
                for _ in range(3):
                    w._refresh_control_presentation()
                    self.assertEqual(p.voice_status_label.text(), source_text)
                    self.assertEqual(p.voice_btn.text(), thinking + "." * (tick % 4))
            p._typing_timer.stop()
            p._voice_state = ""
            p.voice_status_label.hide()
            w._refresh_control_presentation()
            self.assertEqual(p.voice_btn.text(), preview.startup_text(language)["listen_button"])
            p._voice_state = "[<<] SPEAKING"
            p.voice_status_label.setText("[<<] SPEAKING")
            p.voice_status_label.show()
            w._refresh_control_presentation()
            self.assertEqual(p.voice_btn.text(), preview.startup_text(language)["speaking_button"])



if __name__ == "__main__":
    unittest.main()
