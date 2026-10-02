import os
import sys
import unittest
from types import SimpleNamespace
from unittest.mock import Mock

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'robot_ui'))

from PyQt6.QtWidgets import QApplication, QLabel, QMainWindow, QTextEdit
from action_msgs.msg import GoalStatus

from waypoints_mode_layout import WaypointsModeLayout


class ResultFuture:
    def __init__(self, value):
        self.value = value

    def result(self):
        return self.value


class GoalHandle:
    accepted = True

    def __init__(self):
        self.cancel_goal_async = Mock()


class WaypointNavigationCallbackTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        self.window = WaypointsModeLayout.__new__(WaypointsModeLayout)
        QMainWindow.__init__(self.window)
        self.window._navigation_generation = 4
        self.window.ros_node = SimpleNamespace(current_goal_handle=None)
        self.window.running_sequence = True
        self.window.selected_sequence = []
        self.window.current_sequence_index = 0
        self.window.log_text = QTextEdit()
        self.window.navigation_status = QLabel()

    def test_late_accepted_goal_is_cancelled(self):
        goal_handle = GoalHandle()

        self.window._goal_response_callback(
            ResultFuture(goal_handle),
            'old-target',
            3,
        )

        goal_handle.cancel_goal_async.assert_called_once_with()
        self.assertIsNone(self.window.ros_node.current_goal_handle)

    def test_successful_goal_advances_captured_route(self):
        self.window.selected_sequence = ['X5.1', 'X5.2']
        self.window.waypoints = {}
        self.window.ros_node.current_goal_handle = object()
        self.window._announce_arrival = Mock()
        self.window.navigate_to_waypoint = Mock()
        result = ResultFuture(SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED))

        self.window._goal_result_callback(result, 'X5.1', 4)

        self.window._announce_arrival.assert_called_once_with('X5.1')
        self.window.navigate_to_waypoint.assert_called_once_with('X5.2', 4)
        self.assertEqual(self.window.current_sequence_index, 1)
        self.assertIsNone(self.window.ros_node.current_goal_handle)


if __name__ == '__main__':
    unittest.main()