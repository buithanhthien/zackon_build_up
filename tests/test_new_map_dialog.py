"""Mapping popup behavior without ROS launches or robot hardware."""
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import Mock, patch

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'robot_ui'))
from PyQt6.QtWidgets import QApplication, QDialog, QMainWindow, QLabel, QTextEdit
from new_map_layout import NewMapUI
from mapping_process import MappingProcess


class NewMapDialogTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        self.parent = QMainWindow()
        self.dialog = NewMapUI(self.parent)
        self.parent.show()
        self.dialog.show()
        self.addCleanup(self.parent.close)
        self.addCleanup(self.dialog.close)

    def process(self, code=None):
        process = Mock()
        process.poll.return_value = code
        process.error_detail.return_value = 'diagnostic'
        return process

    def test_popup_has_no_chat_or_log_and_back_keeps_parent_open(self):
        self.assertIsInstance(self.dialog, QDialog)
        self.assertIs(self.dialog.parent(), self.parent)
        self.assertFalse(self.dialog.isModal())
        self.assertFalse(self.dialog.findChildren(QTextEdit))
        labels = [label.text() for label in self.dialog.findChildren(QLabel)]
        self.assertNotIn('SYSTEM LOG', labels)
        self.assertFalse(hasattr(self.dialog, 'chat_widget'))
        self.dialog.btn_back.click()
        self.assertFalse(self.dialog.isVisible())
        self.assertTrue(self.parent.isVisible())

    def test_single_session_cancel_and_restart(self):
        process = self.process()
        other = NewMapUI(self.parent)
        self.addCleanup(other.close)
        with patch('new_map_layout.MappingProcess', return_value=process) as launch:
            self.dialog.start_mapping()
            self.dialog.start_mapping()
            other.start_mapping()
            launch.assert_called_once()
            self.assertIn('khác đang chạy', other.status_label.text())
            self.dialog.btn_cancel.click()
            process.stop.assert_called_once()
            self.assertIsNone(self.dialog.mapping_process)
            self.assertTrue(self.dialog.btn_start.isEnabled())
            other.start_mapping()
            self.assertEqual(launch.call_count, 2)

    def test_close_and_escape_stop_only_owned_mapping_and_saver(self):
        for close in (self.dialog.close, self.dialog.reject):
            mapping, saver = self.process(), self.process()
            self.dialog.mapping_process = mapping
            self.dialog.save_process = saver
            self.dialog.show()
            close()
            mapping.stop.assert_called_once()
            saver.stop.assert_called_once()
            self.assertTrue(self.parent.isVisible())

    def test_cleanup_attempts_mapping_even_if_saver_stop_fails(self):
        mapping, saver = self.process(), self.process()
        saver.stop.side_effect = OSError('stop failed')
        self.dialog.mapping_process = mapping
        self.dialog.save_process = saver
        self.dialog.cancel_mapping()
        mapping.stop.assert_called_once()
        self.assertIsNone(self.dialog.mapping_process)
        self.assertIs(self.dialog.save_process, saver)
        self.assertIn('stop failed', self.dialog.status_label.text())
        saver.stop.side_effect = None
        self.dialog.cancel_mapping()
        self.assertIsNone(self.dialog.save_process)

    def test_mapping_launch_failure_and_early_exit_show_error(self):
        with patch('new_map_layout.MappingProcess', side_effect=OSError('missing executable')):
            self.dialog.start_mapping()
        self.assertIn('missing executable', self.dialog.status_label.text())
        self.assertTrue(self.dialog.btn_start.isEnabled())
        self.dialog.mapping_process = self.process(1)
        self.dialog.check_processes()
        self.assertIn('mã 1', self.dialog.status_label.text())
        self.assertIn('diagnostic', self.dialog.status_label.text())
        self.assertTrue(self.dialog.btn_start.isEnabled())

    def test_save_waits_for_exit_and_checks_result(self):
        self.dialog.mapping_process = self.process()
        saver = self.process()
        with tempfile.TemporaryDirectory() as tmp, patch('new_map_layout.SOURCE_PATH', tmp), \
                patch('new_map_layout.MappingProcess', return_value=saver) as launch:
            self.dialog.map_name_input.setText('Tầng 1')
            self.dialog.save_map()
            self.dialog.save_map()
            launch.assert_called_once()
            self.assertIn('Đang lưu', self.dialog.status_label.text())
            self.assertFalse(self.dialog.btn_apply.isEnabled())
            saver.poll.return_value = 1
            self.dialog.check_processes()
            self.assertIn('Không thể lưu', self.dialog.status_label.text())
            self.assertTrue(self.dialog.btn_apply.isEnabled())
            saver.poll.return_value = None
            self.dialog.save_map()
            saver.poll.return_value = 0
            self.dialog.check_processes()
            self.assertIn('Không thể lưu', self.dialog.status_label.text())
            saver.poll.return_value = None
            self.dialog.save_map()
            Path(str(self.dialog._save_path) + '.yaml').write_text('image: map.pgm')
            saver.poll.return_value = 0
            self.dialog.check_processes()
            self.assertIn('Đã lưu bản đồ tại', self.dialog.status_label.text())

    def test_save_rejects_path_escape_and_requires_mapping(self):
        with patch('new_map_layout.MappingProcess') as launch:
            for name in ('', '../outside', 'a/b', '..', 'a;command'):
                self.dialog.map_name_input.setText(name)
                self.dialog.save_map()
            self.dialog.map_name_input.setText('valid')
            self.dialog.save_map()
            self.assertIn('bắt đầu lập bản đồ', self.dialog.status_label.text())
            launch.assert_not_called()


class MappingProcessTests(unittest.TestCase):
    def test_stop_kills_owned_descendants_and_keeps_unrelated_process(self):
        unrelated = subprocess.Popen(['sleep', '30'], start_new_session=True)
        self.addCleanup(unrelated.wait)
        self.addCleanup(unrelated.terminate)
        proc = MappingProcess('sleep 30 & wait')
        self.addCleanup(lambda: proc.stop() if not proc.output.closed else None)
        proc.stop()
        self.assertIsNotNone(proc.poll())
        self.assertIsNone(unrelated.poll())
        self.assertTrue(proc.output.closed)

    def test_group_is_stopped_even_after_launch_parent_exits(self):
        # Verify the child itself stops, even when the launch parent has exited.
        with tempfile.TemporaryDirectory() as tmp:
            pidfile = Path(tmp) / 'child.pid'
            proc = MappingProcess(f'sleep 30 & echo $! > {pidfile}; exit 7')
            self.addCleanup(lambda: proc.stop() if not proc.output.closed else None)
            self.assertEqual(proc.process.wait(timeout=2), 7)
            child_pid = int(pidfile.read_text())
            with patch('mapping_process.os.killpg', wraps=os.killpg) as kill:
                proc.stop()
            self.assertIn(signal.SIGINT, [call.args[1] for call in kill.call_args_list])
            deadline = time.monotonic() + 2
            while time.monotonic() < deadline:
                stat = Path(f'/proc/{child_pid}/stat')
                if not stat.exists() or stat.read_text().split()[2] == 'Z':
                    break
                time.sleep(0.02)
            else:
                self.fail('Mapping child is still running after cleanup')


if __name__ == '__main__':
    unittest.main()
