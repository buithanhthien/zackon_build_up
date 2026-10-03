import copy
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
from PyQt6.QtWidgets import QApplication, QMainWindow, QMessageBox
from waypoint_store import (WaypointError, normalize_waypoints, load_waypoint_file,
                            save_waypoint_file, deletion_reason, route_references,
                            resolve_waypoint)
from waypoints_mode_layout import WaypointsModeLayout, WaypointPickerDialog, NewWaypointDialog


def waypoint(**fields):
    return dict(x=1., y=2., z=0., qx=0., qy=0., qz=0., qw=1.,
                map_name='test', **fields)


class SchemaTests(unittest.TestCase):
    def test_legacy_migration_preserves_all_data_and_stable_identity(self):
        original = {'X5.1': waypoint(aliases=['room']), 'home': waypoint(aliases=['nhà'])}
        clean = normalize_waypoints(original)
        self.assertEqual(set(clean), set(original))
        self.assertEqual(clean['home']['x'], original['home']['x'])
        self.assertFalse(clean['X5.1']['deletable'])
        self.assertFalse(clean['home']['deletable'])
        self.assertTrue(clean['home']['deletion_permission_pending'])
        identifier = clean['home']['id']
        clean['home'].update(display_name='Tên mới', aliases=['alias mới'])
        self.assertEqual(normalize_waypoints(clean)['home']['id'], identifier)
        self.assertNotIn('id', original['home'])

    def test_x_rooms_cannot_be_unlocked_or_renamed_to_bypass(self):
        for permission in (None, True, False):
            wp = waypoint()
            if permission is not None:
                wp['deletable'] = permission
            self.assertTrue(deletion_reason('x5.2.4', wp))
        clean = normalize_waypoints({'X5.1': waypoint()})['X5.1']
        clean.update(display_name='Cafe', aliases=['coffee'], deletable=True)
        self.assertTrue(deletion_reason('renamed-key', clean))

    def test_invalid_data_is_rejected_not_silently_discarded(self):
        for fields in ({'deletable': 'false'}, {'deletable': 1}, {'x': float('nan')},
                       {'x': True}, {'aliases': 'home'}, {'aliases': [1]},
                       {'map_name': ''}, {'id': ''}, {'qw': 0.},
                       {'deletion_permission_pending': 'true'}):
            wp = waypoint()
            wp.update(fields)
            with self.subTest(fields=fields), self.assertRaises(WaypointError):
                normalize_waypoints({'bad': wp})
        with self.assertRaises(WaypointError):
            normalize_waypoints({'a': waypoint(id='same'), 'b': waypoint(id='same')})

    def test_atomic_roundtrip_and_failed_replace_preserves_file(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'waypoints.json'
            clean, raw = save_waypoint_file(path, {'a': waypoint(deletable=True),
                                                 'b': waypoint(deletable=False)}, None)
            self.assertEqual(load_waypoint_file(path)[0], clean)
            with patch('waypoint_store.os.replace', side_effect=OSError('disk full')):
                with self.assertRaises(OSError):
                    save_waypoint_file(path, {}, raw)
            self.assertEqual(path.read_bytes(), raw)
            self.assertEqual(list(Path(directory).iterdir()), [path])
            path.write_text('{}')
            with self.assertRaises(WaypointError):
                save_waypoint_file(path, clean, raw)
            path.write_text('{"a": {}, "a": {}}')
            with self.assertRaises(WaypointError):
                load_waypoint_file(path)


class DeletionUITests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.path = Path(self.directory.name)/'waypoints.json'
        self.path.write_text(json.dumps({'X5.1': waypoint(deletable=True),
                                        'home': waypoint(deletable=True, aliases=['nhà']),
                                        'locked': waypoint(deletable=False),
                                        'legacy': waypoint()}))
        self.window = WaypointsModeLayout.__new__(WaypointsModeLayout)
        QMainWindow.__init__(self.window)
        w = self.window
        w.waypoints_file = str(self.path)
        w.waypoints = w.load_waypoints()
        w._current_nav_target = None
        w.running_sequence = False
        w.selected_sequence = []
        w.ros_node = SimpleNamespace(current_goal_handle=None)
        w.map_widget = Mock()
        w.log = Mock()
        self.question = patch('waypoints_mode_layout.QMessageBox.question', return_value=QMessageBox.StandardButton.Yes).start()
        self.warning = patch('waypoints_mode_layout.QMessageBox.warning').start()
        patch('waypoints_mode_layout.get_current_map_name', return_value='test').start()
        self.addCleanup(patch.stopall)

    def picker(self):
        return WaypointPickerDialog(self.window.waypoints, 'test', delete_callback=self.window.delete_waypoint)

    def test_delete_updates_list_map_alias_provider_and_disk(self):
        w = self.window
        self.assertEqual(resolve_waypoint(w.waypoints, 'nhà', 'test'), 'home')
        picker = self.picker()
        picker.list_widget.setCurrentRow(1)
        picker.btn_delete.click()
        self.assertNotIn('home', w.waypoints)
        self.assertNotIn('home', json.loads(self.path.read_text()))
        self.assertEqual(picker.list_widget.count(), 3)
        self.assertNotIn('home', w.map_widget.set_waypoints.call_args.args[0])
        self.assertNotIn('home', [d['key'] for d in w._get_current_map_waypoint_descriptors()])
        self.assertIsNone(resolve_waypoint(w.waypoints, 'nhà', 'test'))
        w.log.assert_called_once()

    def test_startup_voice_provider_and_goal_recheck_use_current_file(self):
        from startup_layout import RobotUI
        startup = SimpleNamespace(_waypoints_file=str(self.path), log=Mock(),
                                  _voice_nav_queue=['home'], _nav_client=Mock())
        startup._load_waypoints = lambda: RobotUI._load_waypoints(startup)
        with patch('startup_layout.get_current_map_name', return_value='test'):
            self.assertIn('home', [d['key'] for d in RobotUI._get_voice_waypoints(startup)])
            self.assertTrue(self.window.delete_waypoint('home'))
            self.assertNotIn('home', [d['key'] for d in RobotUI._get_voice_waypoints(startup)])
            RobotUI._send_next_voice_goal(startup)
        startup._nav_client.wait_for_server.assert_not_called()

    def test_display_rename_keeps_picker_and_route_key(self):
        from waypoints_mode_layout import NewPathDialog
        original_id = self.window.waypoints['home']['id']
        self.window.waypoints['home']['display_name'] = 'Tên mới'
        picker = self.picker()
        picker.list_widget.setCurrentRow(1)
        self.assertEqual(picker.list_widget.currentItem().text(), 'Tên mới')
        self.assertEqual(picker.get_selected_key(), 'home')
        route = NewPathDialog(self.window.waypoints, 'test', str(self.path.with_name('routes.json')))
        route._add_goal(route.goal_list.item(1))
        self.assertEqual(route.sequence, ['home'])
        self.assertEqual(self.window.waypoints['home']['id'], original_id)

    def test_cancel_leaves_memory_file_and_list_unchanged(self):
        self.question.return_value = QMessageBox.StandardButton.No
        before, raw = copy.deepcopy(self.window.waypoints), self.path.read_bytes()
        picker = self.picker()
        picker.list_widget.setCurrentRow(1)
        picker.btn_delete.click()
        self.assertEqual(self.window.waypoints, before)
        self.assertEqual(self.path.read_bytes(), raw)
        self.assertEqual(picker.list_widget.count(), 4)
        self.window.log.assert_not_called()

    def test_protection_enforced_by_button_and_handler(self):
        picker = self.picker()
        for row, key in ((0, 'X5.1'), (2, 'locked')):
            picker.list_widget.setCurrentRow(row)
            self.assertFalse(picker.btn_delete.isEnabled())
            self.assertFalse(self.window.delete_waypoint(key))
        for invalid in (True, 'true', 1, None):
            self.window.waypoints['X5.1']['deletable'] = invalid
            self.assertFalse(self.window.delete_waypoint('X5.1'))
        self.question.assert_not_called()

    def test_legacy_requires_explicit_permission_confirmation(self):
        self.question.return_value = QMessageBox.StandardButton.No
        self.assertFalse(self.window.delete_waypoint('legacy'))
        self.assertIn('chưa có quyền xóa', self.question.call_args.args[2])
        self.question.return_value = QMessageBox.StandardButton.Yes
        self.assertTrue(self.window.delete_waypoint('legacy'))

    def test_saved_route_and_bad_route_file_block_deletion(self):
        routes = self.path.with_name('multi_waypoints.json')
        for reference in ('home', 'nhà', self.window.waypoints['home']['id']):
            routes.write_text(json.dumps({'tour': {'map_name': 'test', 'sequence': [reference]}}))
            self.assertFalse(self.window.delete_waypoint('home'))
        routes.write_text('{bad json')
        self.assertFalse(self.window.delete_waypoint('home'))
        self.question.assert_not_called()
        self.assertIn('home', self.window.waypoints)

    def test_active_route_and_reference_added_during_confirmation_block(self):
        self.window.running_sequence = True
        self.window.selected_sequence = ['home']
        self.assertFalse(self.window.delete_waypoint('home'))
        self.window.running_sequence = False
        def add_route(*args):
            self.path.with_name('multi_waypoints.json').write_text(json.dumps(
                {'new': {'map_name': 'test', 'sequence': ['home']}}))
            return QMessageBox.StandardButton.Yes
        self.question.side_effect = add_route
        self.assertFalse(self.window.delete_waypoint('home'))
        self.assertIn('home', self.window.waypoints)

    def test_save_failure_keeps_memory_disk_picker_and_success_log_unchanged(self):
        before, raw = copy.deepcopy(self.window.waypoints), self.path.read_bytes()
        picker = self.picker()
        picker.list_widget.setCurrentRow(1)
        with patch('waypoint_store.os.replace', side_effect=OSError('permission denied')):
            picker.btn_delete.click()
        self.assertEqual(self.window.waypoints, before)
        self.assertEqual(self.path.read_bytes(), raw)
        self.assertEqual(picker.list_widget.count(), 4)
        self.window.log.assert_not_called()
        self.window.map_widget.set_waypoints.assert_not_called()
        self.warning.assert_called_once()

    def test_invalid_load_cannot_overwrite_original(self):
        raw = '{"broken": {"x": 1}}'
        self.path.write_text(raw)
        self.window.waypoints = self.window.load_waypoints()
        self.assertFalse(self.window.save_waypoints({'new': waypoint()}))
        self.assertEqual(self.path.read_text(), raw)

    def test_new_form_default_and_both_permissions_roundtrip(self):
        form = NewWaypointDialog()
        self.assertTrue(form.get_deletable())
        vec = SimpleNamespace(x=1., y=2., z=0.)
        orientation = SimpleNamespace(x=0., y=0., z=0., w=1.)
        self.window.ros_node.current_pose = SimpleNamespace(pose=SimpleNamespace(
            pose=SimpleNamespace(position=vec, orientation=orientation)))
        for permission in (False, True):
            form.name_input.setText('new'+str(permission))
            form.deletable_checkbox.setChecked(permission)
            with patch('waypoints_mode_layout.NewWaypointDialog', return_value=form), patch.object(form, 'exec', return_value=1):
                self.window.open_new_waypoint_dialog()
            saved, _ = load_waypoint_file(self.path)
            self.assertEqual(saved['new'+str(permission)]['deletable'], permission)
            self.assertTrue(saved['new'+str(permission)]['id'])

    def test_new_save_failure_does_not_add_to_memory(self):
        vec = SimpleNamespace(x=1., y=2., z=0.)
        quat = SimpleNamespace(x=0., y=0., z=0., w=1.)
        self.window.ros_node.current_pose = SimpleNamespace(pose=SimpleNamespace(
            pose=SimpleNamespace(position=vec, orientation=quat)))
        form = NewWaypointDialog()
        form.name_input.setText('new')
        with patch('waypoints_mode_layout.NewWaypointDialog', return_value=form), patch.object(form, 'exec', return_value=1), patch('waypoint_store.os.replace', side_effect=OSError('disk full')):
            self.window.open_new_waypoint_dialog()
        self.assertNotIn('new', self.window.waypoints)
        self.window.log.assert_not_called()


if __name__ == '__main__':
    unittest.main()
