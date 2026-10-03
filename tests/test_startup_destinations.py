"""Offscreen UI/navigation integration without starting ROS or robot processes."""
import json
import os
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'robot_ui'))
from PyQt6.QtWidgets import QApplication, QDialog, QMainWindow, QPushButton
from PyQt6.QtGui import QPixmap
from action_msgs.msg import GoalStatus
from startup_layout import RobotUI
from waypoint_dialogs import DestinationDialog, NewPathDialog, WaypointPickerDialog
from waypoint_store import load_waypoint_file


def waypoint(map_name='test'):
    return dict(x=1., y=2., z=0., qx=0., qy=0., qz=0., qw=1., map_name=map_name)


class StartupDestinationsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.window = RobotUI.__new__(RobotUI)
        QMainWindow.__init__(self.window)
        w = self.window
        w.log = Mock()
        w._waypoints_file = str(Path(self.tmp.name) / 'waypoints.json')
        Path(w._waypoints_file).write_text(json.dumps({'home': waypoint(), 'other': waypoint('other')}))
        w._waypoints = w._load_waypoints()
        w._latest_pose = SimpleNamespace(position=SimpleNamespace(x=3., y=4., z=0.),
                                        orientation=SimpleNamespace(x=0., y=0., z=0., w=1.))
        w._latest_amcl_msg = None
        w._navigation_generation = 0
        w._nav_goal_handle = None
        w._voice_nav_queue = []
        self.map_patch = patch('startup_layout.get_current_map_name', return_value='test')
        self.map_patch.start()
        self.addCleanup(self.map_patch.stop)
        self.addCleanup(w.deleteLater)

    def test_destination_window_only_four_actions_and_close_without_process_changes(self):
        w = self.window
        for name in ('load_map', 'open_waypoint_picker', 'open_path_manager', 'open_new_waypoint_dialog'):
            setattr(w, name, Mock())
        with patch('startup_layout.subprocess.Popen') as popen, patch.object(w, 'close') as close:
            w.mode_changed('Waypoints')
            dialog = w._destination_dialog
            self.assertIsInstance(dialog, DestinationDialog)
            buttons = dialog.findChildren(QPushButton)
            self.assertEqual([b.text() for b in buttons],
                             ['Tải bản đồ', 'Địa điểm', 'Tạo lộ trình', 'Tạo địa điểm mới', 'Đóng'])
            self.assertEqual(len(dialog.findChildren(QPushButton)), 5)
            for button in buttons:
                button.click()
            for name in ('load_map', 'open_waypoint_picker', 'open_path_manager', 'open_new_waypoint_dialog'):
                getattr(w, name).assert_called_once()
            self.assertFalse(dialog.isVisible())
            w.open_destination_dialog()
            self.assertIs(w._destination_dialog, dialog)
            self.assertTrue(dialog.isVisible())
            close.assert_not_called()
            popen.assert_not_called()
            dialog.close()

    def test_picker_uses_latest_file_current_map_and_shared_navigation(self):
        w = self.window
        Path(w._waypoints_file).write_text(json.dumps({'fresh': waypoint(), 'other': waypoint('other')}))
        def choose(dialog):
            self.assertEqual(dialog.list_widget.count(), 1)
            dialog.list_widget.setCurrentRow(0)
            self.assertEqual(dialog.get_selected_key(), 'fresh')
            self.assertTrue(dialog.btn_delete.isHidden())
            return QDialog.DialogCode.Accepted
        with patch.object(WaypointPickerDialog, 'exec', choose), patch.object(w, '_send_next_voice_goal') as send:
            w.open_waypoint_picker()
            self.assertEqual(w._voice_nav_queue, ['fresh'])
            send.assert_called_once()

    def test_save_amcl_pose_and_refresh_source(self):
        w = self.window
        with patch('startup_layout.NewWaypointDialog') as dialog:
            dialog.return_value.exec.return_value = 1
            dialog.return_value.get_name.return_value = 'Cafe, tầng 1'
            dialog.return_value.get_deletable.return_value = True
            w.open_new_waypoint_dialog()
        data, _ = load_waypoint_file(w._waypoints_file)
        self.assertEqual(data['Cafe, tầng 1']['x'], 3.)
        self.assertEqual(data['Cafe, tầng 1']['map_name'], 'test')
        self.assertEqual(w._waypoints, data)
        with patch.object(w, '_send_next_voice_goal'):
            w._run_waypoint_sequence(['Cafe, tầng 1'])
            self.assertEqual(w._voice_nav_queue, ['Cafe, tầng 1'])

    def test_missing_pose_cannot_save(self):
        w = self.window
        w._latest_pose = None
        with patch('startup_layout.QMessageBox.warning') as warning, patch('startup_layout.NewWaypointDialog') as dialog:
            w.open_new_waypoint_dialog()
            warning.assert_called_once()
            dialog.assert_not_called()

    def test_corrupt_file_cannot_be_overwritten(self):
        w = self.window
        Path(w._waypoints_file).write_text('{broken')
        with patch('startup_layout.NewWaypointDialog') as dialog, patch('startup_layout.QMessageBox.warning') as warning:
            dialog.return_value.exec.return_value = 1
            dialog.return_value.get_name.return_value = 'New'
            w.open_new_waypoint_dialog()
            warning.assert_called_once()
        self.assertEqual(Path(w._waypoints_file).read_text(), '{broken')

    def test_sequence_sends_nav2_goal_using_startup_client(self):
        from builtin_interfaces.msg import Time
        w = self.window
        w._ros_node = Mock()
        w._ros_node.get_clock.return_value.now.return_value.to_msg.return_value = Time()
        w._nav_client = Mock()
        w._nav_client.wait_for_server.return_value = True
        w._run_waypoint_sequence(['home'])
        goal = w._nav_client.send_goal_async.call_args.args[0]
        self.assertEqual(goal.pose.header.frame_id, 'map')
        self.assertEqual(goal.pose.pose.position.x, 1.)
        self.assertEqual(goal.pose.pose.position.y, 2.)
        handle = Mock(accepted=True)
        callback = w._nav_client.send_goal_async.return_value.add_done_callback.call_args.args[0]
        callback(Mock(result=Mock(return_value=handle)))
        self.assertIs(w._nav_goal_handle, handle)
        handle.get_result_async.assert_called_once()

    def test_route_creation_and_run_use_same_store_and_queue(self):
        w = self.window
        route_file = Path(self.tmp.name) / 'multi_waypoints.json'
        dialog = NewPathDialog(w._waypoints, 'test', str(route_file), w)
        dialog.sequence = ['home', 'home']
        dialog.name_input.setText('Tour')
        dialog._confirm()
        self.assertEqual(json.loads(route_file.read_text())['Tour']['sequence'], ['home', 'home'])
        with patch.object(w, '_send_next_voice_goal') as send:
            w._run_waypoint_sequence(json.loads(route_file.read_text())['Tour']['sequence'])
            self.assertEqual(w._voice_nav_queue, ['home', 'home'])
            send.assert_called_once()
        with patch('startup_layout.QMessageBox.warning') as warning, patch.object(w, '_send_next_voice_goal') as send:
            w._run_waypoint_sequence(['other'])
            warning.assert_called_once()
            send.assert_not_called()

    def test_map_toggle_and_window_close_do_not_cancel_navigation(self):
        w = self.window
        path = Path(self.tmp.name) / 'map.yaml'
        pixmap = QPixmap(10, 10)
        pixmap.fill()
        pixmap.save(str(path.with_suffix('.png')))
        path.write_text('image: map.png\nresolution: 0.05\norigin: [0, 0, 0]\n')
        with patch('startup_layout.get_current_map_path', return_value=str(path)), patch.object(w, 'cancel_voice_navigation') as cancel:
            w.toggle_map_window()
            self.assertTrue(w._map_window.isVisible())
            self.assertEqual(set(w._map_view.waypoints), {'home'})
            w.toggle_map_window()
            self.assertFalse(w._map_window.isVisible())
            w.toggle_map_window()
            w._map_window.close()
            cancel.assert_not_called()

    def test_load_map_refreshes_data_and_invalidates_old_pose(self):
        w = self.window
        with patch('startup_layout.LoadMapDialog') as dialog, patch('startup_layout.update_map_files', return_value=True), patch.object(w, '_refresh_map_window') as refresh:
            dialog.return_value.exec.return_value = 1
            dialog.return_value.get_selected_map.return_value = 'new'
            w.load_map()
            self.assertIsNone(w._latest_pose)
            refresh.assert_called_once()

    def test_late_goal_and_result_cannot_modify_new_route(self):
        w = self.window
        w._navigation_generation = 2
        w._voice_nav_queue = ['home']
        handle = Mock(accepted=True)
        w._nav_goal_response_callback(Mock(result=Mock(return_value=handle)), 1)
        handle.cancel_goal_async.assert_called_once()
        self.assertIsNone(w._nav_goal_handle)
        w._nav_result_callback(Mock(result=Mock(return_value=SimpleNamespace(status=GoalStatus.STATUS_CANCELED))), 1)
        self.assertEqual(w._voice_nav_queue, ['home'])

    def test_success_advances_route_and_cancel_invalidates_delayed_advance(self):
        w = self.window
        w._voice_nav_queue = ['home']
        with patch('startup_layout.QTimer.singleShot') as timer:
            w._nav_result_callback(Mock(result=Mock(return_value=SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED))), 0)
            callback = timer.call_args.args[1]
            w.cancel_voice_navigation()
            w._voice_nav_queue = ['new-command']
            callback()
            self.assertEqual(w._voice_nav_queue, ['new-command'])


if __name__ == '__main__':
    unittest.main()
