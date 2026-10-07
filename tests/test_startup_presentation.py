"""Exercise real Startup layout methods with inert ROS/voice dependencies."""
import ast
import importlib.util
from pathlib import Path
import unittest

from PyQt6.QtWidgets import QApplication, QPushButton

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
                        for button in w.findChildren(QPushButton):
                            if button.isVisible():
                                point = button.mapTo(w, button.rect().topLeft())
                                self.assertGreaterEqual(point.x(), 0)
                                self.assertGreaterEqual(point.y(), 0)
                                self.assertLessEqual(point.x() + button.width(), width)
                                self.assertLessEqual(point.y() + button.height(), height)
                                self.assertGreaterEqual(button.height(), 44)
                    w.hide()

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
        p.voice_btn.setEnabled(False)
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
            p.voice_status_label.hide()
            w._refresh_control_presentation()
            self.assertEqual(p.voice_btn.text(), preview.startup_text(language)["listen_button"])
            p.voice_status_label.setText("[>>] SPEAKING")
            p.voice_status_label.show()
            w._refresh_control_presentation()
            self.assertEqual(p.voice_btn.text(), preview.startup_text(language)["speaking_button"])



if __name__ == "__main__":
    unittest.main()
