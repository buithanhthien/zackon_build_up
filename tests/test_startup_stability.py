"""Regression checks for startup ownership, close vetoes and plain chat output."""
import os
import sys
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'robot_ui'))
from PyQt6.QtWidgets import QApplication
from chat_text import plain_chat_text
from chat_panel_widget import ChatPanel
from startup_layout import RobotUI


class PlainChatTests(unittest.TestCase):
    def test_emoji_sequences_removed_without_damaging_text(self):
        for emoji in ('😊', '👩🏽\u200d💻', '🇻🇳', '1️⃣', '❤️', '✅'):
            self.assertEqual(plain_chat_text('Xin chào! ' + emoji), 'Xin chào!')
        text = 'Điện áp 24 V; 2 × 3 = 6; x ≤ 5 → đúng.\n#1: 50% ©'
        self.assertEqual(plain_chat_text(text), text)

    def test_display_history_and_tts_receive_same_clean_reply(self):
        panel = SimpleNamespace(_typing_timer=Mock(), _chat_history=[],
                                log_signal=Mock(), _voice_engine=Mock(), _language='vi')
        ChatPanel._on_response(panel, 'Chào bạn! 😊')
        self.assertEqual(panel._chat_history[-1]['content'], 'Chào bạn!')
        panel.log_signal.emit.assert_called_once_with('[Bé Son] Chào bạn!')
        panel._voice_engine.speak_in_language.assert_called_once_with('Chào bạn!', 'vi')

    def test_emoji_only_reply_does_not_start_empty_speech(self):
        panel = SimpleNamespace(_typing_timer=Mock(), voice_status_label=Mock(),
                                _voice_engine=Mock(), _chat_history=[])
        ChatPanel._on_response(panel, '😊')
        self.assertEqual(panel._chat_history, [])
        panel._voice_engine.speak_in_language.assert_not_called()
        panel.voice_status_label.hide.assert_called_once()


class StartupLifecycleTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def test_constructor_owns_one_motion_controller_and_one_signal_connection(self):
        def init_ui(window):
            window.chat_panel = Mock()
        with patch('startup_layout.rclpy.init'), patch('startup_layout.Node'), \
             patch('startup_layout.ActionClient'), patch('startup_layout.QTimer'), \
             patch.object(RobotUI, '_load_waypoints', return_value={}), \
             patch.object(RobotUI, 'init_ui', init_ui), \
             patch.object(RobotUI, '_report_motion_status') as report, \
             patch('startup_layout.RosMotionController') as controller:
            window = RobotUI()
            controller.assert_called_once()
            window.chat_panel.motion_command.connect.assert_called_once()
            window.motion_status.emit('test status')
            report.assert_called_once_with('test status')
            window.deleteLater()

    def test_mapping_close_veto_keeps_motion_and_ros_alive(self):
        window = SimpleNamespace(cancel_voice_navigation=Mock(), _nav_goal_handle=None,
                                 _nav_pending=False, _motion=Mock(), _ros_node=Mock(),
                                 _new_map_dialog=Mock())
        window._new_map_dialog.close.return_value = False
        event = Mock()
        RobotUI.closeEvent(window, event)
        event.ignore.assert_called_once()
        window._motion.close.assert_not_called()
        window._ros_node.destroy_node.assert_not_called()

    def test_mode_switch_does_not_launch_when_close_is_rejected(self):
        window = SimpleNamespace(log=Mock(), close=Mock(return_value=False))
        with patch('startup_layout.subprocess.Popen') as launch:
            RobotUI.start_docking(window)
            RobotUI.mode_changed(window, 'Tracking')
            launch.assert_not_called()

    def test_stop_also_cancels_localization_rotation(self):
        window = SimpleNamespace(localization_worker=Mock(), _motion=Mock(),
                                 _navigation_generation=0, _voice_nav_queue=[],
                                 _nav_goal_handle=None, log=Mock())
        RobotUI.cancel_voice_navigation(window)
        window.localization_worker.stop.assert_called_once()
        window._motion.stop.assert_called_once()

    def test_close_waits_for_localization_before_destroying_resources(self):
        window = SimpleNamespace(cancel_voice_navigation=Mock(), _nav_goal_handle=None,
                                 _nav_pending=False, localization_thread=Mock(),
                                 _motion=Mock(), _ros_node=Mock(), log=Mock())
        window.localization_thread.is_alive.return_value = True
        event = Mock()
        RobotUI.closeEvent(window, event)
        event.ignore.assert_called_once()
        window._motion.close.assert_not_called()
        window._ros_node.destroy_node.assert_not_called()


if __name__ == '__main__':
    unittest.main()
