"""UI dispatch and ROS adapter guards, with no robot or network requests."""
import os
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'robot_ui'))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/view_robot'))
from chat_panel_widget import ChatPanel
from startup_layout import RobotUI
from motion_commands import parse_motion
from motion_ros import RosMotionController


class DispatchTests(unittest.TestCase):
    def panel(self):
        return SimpleNamespace(
            _language='vi', _voice_engine=Mock(), navigation_stop=Mock(),
            motion_command=Mock(), waypoint_command=Mock(), log_signal=Mock(),
            _waypoints_provider=lambda: [], _classify_intent=Mock(), _ask_ai=Mock(),
            _answer_current_location=Mock(), _typing_timer=Mock(), voice_status_label=Mock())

    def test_motion_bypasses_waypoint_classifier(self):
        panel = self.panel()
        text = 'Bé Son xoay trái 3 vòng rồi đi thẳng 5 giây sau đó đi lùi 3 mét'
        ChatPanel._on_voice_transcript(panel, text)
        panel.motion_command.emit.assert_called_once_with(parse_motion(text))
        panel._classify_intent.assert_not_called()
        panel.navigation_stop.emit.assert_not_called()

    def test_continuous_is_not_a_stop_and_stop_cancels_generation(self):
        panel = self.panel()
        ChatPanel._on_voice_transcript(panel, 'xoay trái đến khi tôi bảo dừng')
        panel.navigation_stop.emit.assert_not_called()
        generation = panel._intent_generation
        ChatPanel._on_voice_transcript(panel, 'dừng')
        panel.navigation_stop.emit.assert_called_once()
        panel._voice_engine.stop_speaking.assert_called_once()
        self.assertGreater(panel._intent_generation, generation)

    def test_ambiguous_and_quoted_stop_do_not_actuate(self):
        panel = self.panel()
        ChatPanel._on_voice_transcript(panel, 'xoay trái 180')
        ChatPanel._on_voice_transcript(panel, 'giải thích câu dừng lại')
        panel.motion_command.emit.assert_not_called()
        panel.navigation_stop.emit.assert_not_called()
        panel._ask_ai.assert_called_once_with('giải thích câu dừng lại')

    def test_waypoint_wake_word_path_preserved(self):
        panel = self.panel()
        ChatPanel._on_voice_transcript(panel, 'Bé Son đi đến phòng A1')
        panel._classify_intent.assert_called_once()
        panel.motion_command.emit.assert_not_called()

    def test_classifier_cannot_invent_motion(self):
        panel = self.panel()
        ChatPanel._on_intent_ready(panel, parse_motion('đi thẳng 3 giây'), 'xin chào')
        panel.motion_command.emit.assert_not_called()
        panel.waypoint_command.emit.assert_not_called()

    def test_late_classifier_result_after_stop_is_ignored(self):
        panel = self.panel()
        ChatPanel._on_voice_transcript(panel, 'dừng lại')
        ChatPanel._on_intent_ready(panel, {'intent': 'navigate', 'waypoints': ['A1']}, 'Bé Son đi đến A1', 0)
        ChatPanel._on_intent_error(panel, 'network', 'Bé Son đi đến A1', 0)
        panel.waypoint_command.emit.assert_not_called()
        panel.motion_command.emit.assert_not_called()
        panel._ask_ai.assert_not_called()

    def test_shared_stop_cancels_both_controllers(self):
        goal = Mock()
        window = SimpleNamespace(_motion=Mock(), _navigation_generation=0,
                                 _voice_nav_queue=['A1', 'A2'], _nav_goal_handle=goal,
                                 chat_panel=self.panel(), log=Mock())
        RobotUI.cancel_voice_navigation(window)
        window._motion.stop.assert_called_once()
        goal.cancel_goal_async.assert_called_once()
        self.assertEqual(window._voice_nav_queue, [])
        self.assertIsNone(window._nav_goal_handle)

    def test_typed_stop_bypasses_busy_chat_worker(self):
        panel = self.panel()
        panel._ai_thread = Mock()
        panel._ai_thread.isRunning.return_value = True
        panel._on_voice_transcript = lambda text: ChatPanel._on_voice_transcript(panel, text)
        window = SimpleNamespace(chat_panel=panel, chat_input=Mock())
        window.chat_input.text.return_value = 'dừng lại'
        RobotUI._send_chat_message(window)
        panel.navigation_stop.emit.assert_called_once()
        panel._ai_thread.isRunning.assert_not_called()
        window.chat_input.clear.assert_called_once()

    def test_active_waypoint_rejects_direct_motion(self):
        window = SimpleNamespace(_motion=Mock(), _nav_goal_handle=Mock(),
                                 _voice_nav_queue=[], _report_motion_status=Mock())
        RobotUI.execute_motion(window, parse_motion('đi thẳng 3 giây'))
        window._motion.start.assert_not_called()
        window._report_motion_status.assert_called_once()


class AdapterGuardTests(unittest.TestCase):
    def test_arbiter_output_state_and_heartbeat_required(self):
        import time
        adapter = SimpleNamespace(node=Mock(), publisher=Mock(), ui_heartbeat=time.monotonic(),
                                  arbiter_stamp=time.monotonic(),
                                  arbiter_state=dict(healthy=True, emergency_stop=False, owner='', ui_granted=False))
        adapter.node.get_publishers_info_by_topic.return_value = [SimpleNamespace(node_name='velocity_arbiter')]
        adapter.publisher.get_subscription_count.return_value = 1
        self.assertIsNone(RosMotionController._ownership_error(adapter))
        adapter.node.get_publishers_info_by_topic.return_value.append(SimpleNamespace(node_name='controller_server'))
        self.assertIsNotNone(RosMotionController._ownership_error(adapter))
        adapter.node.get_publishers_info_by_topic.return_value.pop()
        adapter.arbiter_stamp -= 1
        self.assertIsNotNone(RosMotionController._ownership_error(adapter))
        adapter.arbiter_stamp = time.monotonic()
        adapter.ui_heartbeat -= 1
        self.assertIsNotNone(RosMotionController._ownership_error(adapter))

    def test_idle_stop_does_not_override_nav2(self):
        import threading
        adapter = SimpleNamespace(lock=threading.RLock(), controller=Mock(active=False),
                                  _pending=None, _release=Mock())
        RosMotionController.stop(adapter)
        adapter.controller.stop.assert_not_called()


class TeleopDeadmanTests(unittest.TestCase):
    def setUp(self):
        from view_robot_pkg.teleop_node import JoyTeleopNode
        self.callback = JoyTeleopNode.joy_callback
        self.node = SimpleNamespace(_held=False, publisher=Mock(), release_pub=Mock(),
                       get_parameter=lambda name: SimpleNamespace(value=0.3))

    def test_idle_does_not_publish_and_release_is_sent_once(self):
        idle = SimpleNamespace(buttons=[0] * 5, axes=[0., 0.])
        held = SimpleNamespace(buttons=[0, 0, 0, 0, 1], axes=[0., 1.])
        self.callback(self.node, idle)
        self.node.publisher.publish.assert_not_called()
        self.callback(self.node, held)
        self.assertEqual(self.node.publisher.publish.call_args.args[0].linear.x, 0.3)
        self.callback(self.node, idle)
        self.callback(self.node, idle)
        self.node.release_pub.publish.assert_called_once()
        self.node.publisher.publish.assert_called_once()

    def test_short_joystick_message_releases_instead_of_crashing(self):
        self.node._held = True
        self.callback(self.node, SimpleNamespace(buttons=[], axes=[]))
        self.node.release_pub.publish.assert_called_once()
        self.node.publisher.publish.assert_not_called()


if __name__ == '__main__':
    unittest.main()
