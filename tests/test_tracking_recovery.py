"""Tracking source, failures, motion gate and owned process lifecycle regression."""
import os
import sys
import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'robot_ui'), str(ROOT / 'src/human_following')]
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
from human_following.camera import Camera
from tracking_process import TrackingProcess


class CameraTests(unittest.TestCase):
    def test_realsense_requests_colour_bgr_and_serial(self):
        rs = Mock()
        colour = np.zeros((480, 640, 3), dtype=np.uint8)
        rs.pipeline.return_value.wait_for_frames.return_value.get_color_frame.return_value.get_data.return_value = colour
        with patch.dict(sys.modules, pyrealsense2=rs):
            camera = Camera(serial='123')
            rs.config.return_value.enable_device.assert_called_once_with('123')
            rs.config.return_value.enable_stream.assert_called_once_with(
                rs.stream.color, 640, 480, rs.format.bgr8, 30)
            self.assertTrue(camera.read()[0])
            camera.release()
            camera.release()
            rs.pipeline.return_value.stop.assert_called_once()

    def test_stable_colour_device_and_depth_rejection(self):
        with patch('human_following.camera.cv2.VideoCapture') as capture:
            camera = Camera('/dev/v4l/by-id/colour')
            capture.assert_called_once_with('/dev/v4l/by-id/colour')
            capture.return_value.read.return_value = (True, np.zeros((480, 640), dtype=np.uint16))
            self.assertEqual(camera.read(), (False, None))
            capture.return_value.read.return_value = (False, None)
            self.assertEqual(camera.read(), (False, None))
            camera.release()

    def test_open_failure_releases_handle(self):
        with patch('human_following.camera.cv2.VideoCapture') as capture:
            capture.return_value.isOpened.return_value = False
            with self.assertRaisesRegex(RuntimeError, 'Cannot open colour camera'):
                Camera('/missing')
            capture.return_value.release.assert_called_once()

    def test_missing_sdk_is_actionable(self):
        with patch.dict(sys.modules, pyrealsense2=None):
            with self.assertRaisesRegex(RuntimeError, 'pyrealsense2'):
                Camera()


class ProcessTests(unittest.TestCase):
    def test_launch_error_is_not_success(self):
        process = TrackingProcess()
        with patch('tracking_process.subprocess.Popen', side_effect=OSError('launch failed')):
            with self.assertRaisesRegex(OSError, 'launch failed'):
                process.start('irrelevant')
        self.assertIsNone(process.process)
        self.assertTrue(process.log.closed)

    def test_exit_code_and_log(self):
        process = TrackingProcess()
        try:
            process.start('echo detector_failed; exit 7')
            self.assertEqual(process.process.wait(timeout=3), 7)
            self.assertIn('detector_failed', process.read_log())
        finally:
            process.stop()

    def test_stop_releases_child_resource(self):
        # A real descendant holds a file lock until the process group is stopped.
        import fcntl
        import shlex
        with tempfile.TemporaryDirectory() as directory:
            marker = Path(directory) / 'ready'
            resource = Path(directory) / 'camera-lock'
            code = ('import fcntl,time,pathlib; '
                    f's=open({str(resource)!r}, "w"); '
                    'fcntl.flock(s,fcntl.LOCK_EX); '
                    f'pathlib.Path({str(marker)!r}).touch(); '
                    'time.sleep(30)')
            process = TrackingProcess()
            try:
                process.start(f'{shlex.quote(sys.executable)} -c {shlex.quote(code)} & wait')
                deadline = time.monotonic() + 3
                while not marker.exists() and time.monotonic() < deadline:
                    time.sleep(.02)
                self.assertTrue(marker.exists(), process.read_log())
                process.stop()
                with resource.open('w') as stream:
                    fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
            finally:
                process.stop()


class NodeTests(unittest.TestCase):
    def test_no_motion_without_explicit_enable_and_status_for_no_detection(self):
        from human_following.human_following_node import HumanFollowingNode
        from geometry_msgs.msg import Twist
        node = SimpleNamespace(camera=Mock(), detector=Mock(), tracker=Mock(),
                               enable_motion=False, use_nav2=False, status={},
                               publisher=Mock(), publish_frame=Mock(), last_locked_id=None, get_logger=Mock())
        node.camera.read.return_value = (True, np.zeros((480, 640, 3)))
        node.tracker.get_human_position.return_value = (0.4, 0.2, [], None)
        node.tracker.locked_id = None
        HumanFollowingNode.process_frame(node)
        msg = node.publisher.publish.call_args.args[0]
        self.assertIsInstance(msg, Twist)
        self.assertEqual((msg.linear.x, msg.angular.z), (0, 0))
        self.assertEqual(node.status['tracking'], 'searching')

    def test_enabled_motion_publishes_standalone_following_command(self):
        from human_following.human_following_node import HumanFollowingNode
        from geometry_msgs.msg import PoseStamped, Twist
        node = SimpleNamespace(
            camera=Mock(), detector=Mock(), tracker=Mock(),
            enable_motion=True, use_nav2=False, status={},
            publisher=Mock(), goal_publisher=Mock(), publish_frame=Mock(),
            last_locked_id=None, get_logger=Mock(),
            get_clock=Mock(return_value=SimpleNamespace(
                now=Mock(return_value=SimpleNamespace(to_msg=Mock())))))
        node.camera.read.return_value = (True, np.zeros((480, 640, 3)))
        node.tracker.get_human_position.return_value = (
            0.4, 0.2, [], {'x': 1.0, 'y': 0.25})
        node.tracker.locked_id = None

        HumanFollowingNode.process_frame(node)

        velocity = node.publisher.publish.call_args.args[0]
        goal = node.goal_publisher.publish.call_args.args[0]
        self.assertIsInstance(velocity, Twist)
        self.assertEqual((velocity.linear.x, velocity.angular.z), (0.4, 0.2))
        self.assertIsInstance(goal, PoseStamped)
        self.assertEqual((goal.pose.position.x, goal.pose.position.y), (1.0, 0.25))

    def test_no_frame_stops_before_inference(self):
        from human_following.human_following_node import HumanFollowingNode
        node = SimpleNamespace(camera=Mock(), detector=Mock(), fail=Mock())
        node.camera.read.return_value = (False, None)
        HumanFollowingNode.process_frame(node)
        node.fail.assert_called_once()
        node.detector.detect.assert_not_called()

    def test_failure_cancels_timer_and_releases_camera(self):
        from human_following.human_following_node import HumanFollowingNode
        node = SimpleNamespace(status={'camera': 'connected', 'detector': 'ready'},
                               get_logger=Mock(), publish_status=Mock(), timer=Mock(),
                               stop=Mock(), create_timer=Mock(), exit_failed=Mock())
        HumanFollowingNode.fail(node, RuntimeError('unplugged'))
        node.timer.cancel.assert_called_once()
        node.stop.assert_called_once()
        self.assertEqual(node.status['error'], 'unplugged')

    def test_camera_preview_node_publishes_frames_without_tracking_or_motion(self):
        import rclpy
        from rclpy.context import Context
        from human_following.camera_preview_node import CameraPreviewNode
        camera = Mock()
        camera.read.return_value = (True, np.zeros((2, 3, 3), dtype=np.uint8))
        context = Context()
        rclpy.init(context=context)
        try:
            with patch('human_following.camera_preview_node.Camera', return_value=camera):
                node = CameraPreviewNode(context=context)
            node.publisher = Mock()
            node.publish_frame()
            image = node.publisher.publish.call_args.args[0]
            self.assertEqual((image.width, image.height), (3, 2))
            self.assertEqual(image.encoding, 'bgr8')
            self.assertFalse(hasattr(node, 'detector'))
            self.assertFalse(hasattr(node, 'tracker'))
            self.assertEqual(node.count_publishers('/cmd_vel_sources/following'), 0)
        finally:
            if 'node' in locals():
                node.destroy_node()
            if context.ok():
                context.shutdown()


class UiTests(unittest.TestCase):
    def test_close_and_escape_stop_tracking_without_shutting_down_startup(self):
        from PyQt6.QtWidgets import QApplication, QDialog
        from tracking_mode_layout import TrackingModeUI
        app = QApplication.instance() or QApplication([])
        for action in ('close', 'reject'):
            with patch.object(TrackingModeUI, 'init_ros'), patch.object(TrackingModeUI, 'launch_tracking'):
                window = TrackingModeUI()
            window.tracking_process = Mock()
            window.show()
            self.assertIsInstance(window, QDialog)
            self.assertFalse(hasattr(window, 'chat_widget'))
            self.assertFalse(hasattr(window, 'log_text'))
            self.assertFalse(hasattr(window, 'pos_label'))
            with patch('tracking_mode_layout.rclpy.shutdown') as shutdown:
                getattr(window, action)()
                shutdown.assert_not_called()
            self.assertTrue(window._cleaned)
            window.tracking_process.stop.assert_called_once()
            self.assertFalse(window.isVisible())
            window.deleteLater()

    def test_failed_process_is_reported_and_cleaned(self):
        from tracking_mode_layout import TrackingModeUI
        window = SimpleNamespace(tracking_process=Mock(), status_label=Mock(), log=Mock(), last_error='', _launch_output='', _stopping=None, last_frame_time=time.monotonic(), stop_tracking=Mock())
        window.tracking_process.read_log.return_value = "malformed launch argument 'camera_serial:='"
        window.tracking_process.process.poll.return_value = 9
        TrackingModeUI.check_tracking(window)
        self.assertIn('9', window.stop_tracking.call_args.kwargs['message'])
        self.assertIn('malformed launch argument', window.status_label.setText.call_args.args[0])
        window.stop_tracking.assert_called_once()

    def test_node_error_is_shown_without_full_launch_command(self):
        from tracking_mode_layout import TrackingModeUI
        window = SimpleNamespace(tracking_process=Mock(), status_label=Mock(), log=Mock(),
                                 last_error='', _launch_output='', _stopping=None, last_frame_time=time.monotonic(), stop_tracking=Mock())
        window.tracking_process.read_log.return_value = (
            '[human_following_node-1] RuntimeError: No device connected\n'
            '[ERROR] [human_following_node-1]: process has died [exit code 1, cmd very_long_command]\n')
        window.tracking_process.process.poll.return_value = 0
        TrackingModeUI.check_tracking(window)
        shown = window.status_label.setText.call_args.args[0]
        self.assertIn('No device connected', shown)
        self.assertNotIn('very_long_command', shown)
        self.assertNotIn('mã 0', shown)

    def test_camera_serial_is_only_passed_when_nonempty(self):
        import shlex
        from tracking_mode_layout import TrackingModeUI
        for serial in ('', '123456'):
            window = SimpleNamespace(ros_node=Mock(), tracking_process=Mock(),
                motion_enabled=Mock(), camera_source=Mock(), status_label=Mock(), log=Mock(),
                _stopping=None, video=Mock(), update_controls=Mock())
            window.tracking_process.process = None
            window.motion_enabled.isChecked.return_value = False
            window.camera_source.currentData.return_value = {'source': 'realsense', 'serial': serial}
            with patch.dict(os.environ, TRACKING_CAMERA_SERIAL=serial), \
                 patch('tracking_mode_layout.TrackingProcess') as process:
                TrackingModeUI.launch_tracking(window)
            command = process.return_value.start.call_args.args[0]
            args = shlex.split(command)
            self.assertIn('camera_url:=realsense', args)
            self.assertIn('enable_motion:=false', args)
            if serial.strip():
                self.assertIn('camera_serial:=' + serial.strip(), args)
            else:
                self.assertFalse(any(arg.startswith('camera_serial:=') for arg in args))

    def test_tracking_opens_popup_without_closing_startup(self):
        from startup_layout import RobotUI
        window = SimpleNamespace(log=Mock(), close=Mock(), open_tracking_dialog=Mock())
        with patch('startup_layout.subprocess.Popen') as launch:
            RobotUI.mode_changed(window, 'Tracking')
        window.open_tracking_dialog.assert_called_once()
        window.close.assert_not_called()
        launch.assert_not_called()

    def test_reopening_reuses_open_popup_and_recreates_closed_popup(self):
        from startup_layout import RobotUI
        old = Mock(_cleaned=False)
        window = SimpleNamespace(_tracking_dialog=old)
        with patch('tracking_mode_layout.TrackingModeUI') as create:
            RobotUI.open_tracking_dialog(window)
            create.assert_not_called()
            old.show.assert_called_once()
            old._cleaned = True
            RobotUI.open_tracking_dialog(window)
            create.assert_called_once_with(window)
            old.deleteLater.assert_called_once()

    def test_camera_test_opens_its_own_preview_window(self):
        from camera_test_layout import CameraTestUI
        devices = [('D435i', {'source': 'realsense', 'serial': '123'})]
        with patch('camera_test_layout.discover_cameras', return_value=(devices, [])), \
                patch.object(CameraTestUI, 'init_ros'), \
                patch('camera_test_layout.TrackingProcess') as process:
            process.return_value.process = None
            window = CameraTestUI()
            command = process.return_value.start.call_args.args[0]
            self.assertIn('camera_preview.launch.py', command)
            self.assertIn('camera_url:=realsense', command)
            self.assertIn('camera_serial:=123', command)
            self.assertNotIn('enable_motion', command)
            self.assertNotIn('system.launch.py', command)
            window.cleanup()
            process.return_value.stop.assert_called_once()
            window.deleteLater()


class CameraDiscoveryTests(unittest.TestCase):
    def test_models_serials_and_no_capture_opened(self):
        from camera_devices import discover_cameras
        rs = Mock()
        devices = [Mock(), Mock()]
        devices[0].get_info.side_effect = ['123', 'Intel RealSense D435i']
        devices[1].get_info.side_effect = ['456', 'Intel RealSense D435i']
        rs.context.return_value.query_devices.return_value = devices
        with patch.dict(sys.modules, pyrealsense2=rs), patch('camera_devices.Path') as paths, \
                patch('human_following.camera.cv2.VideoCapture') as capture:
            paths.return_value.glob.return_value = []
            found, warnings = discover_cameras()
        self.assertEqual([item[1]['serial'] for item in found], ['123', '456'])
        self.assertTrue(all(item[1]['source'] == 'realsense' for item in found))
        self.assertIn('D435i', found[0][0])
        self.assertEqual(warnings, [])
        capture.assert_not_called()

    def test_v4l_stable_path_and_empty_refresh(self):
        from camera_devices import discover_cameras
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'sys/class/video4linux/video7').mkdir(parents=True)
            entry = root / 'sys/class/video4linux/video7'
            (entry / 'name').write_text('USB Colour')
            (entry / 'index').write_text('0')
            (root / 'dev/v4l/by-id').mkdir(parents=True)
            (root / 'dev/video7').touch()
            link = root / 'dev/v4l/by-id/usb-colour'
            link.symlink_to(root / 'dev/video7')
            # Device paths are rooted only for metadata reads; resolved links mimic /dev.
            original_resolve = Path.resolve
            def resolved(path):
                return Path(str(original_resolve(path)).removeprefix(directory))
            with patch.dict(sys.modules, pyrealsense2=None), \
                    patch('camera_devices.Path', side_effect=lambda p: root / p.lstrip('/')), \
                    patch.object(Path, 'resolve', resolved):
                found, _ = discover_cameras()
                self.assertEqual(found[0][1]['source'], str(link))
                (entry / 'index').write_text('1')
                self.assertEqual(discover_cameras()[0], [])
                (entry / 'index').write_text('0')
                self.assertEqual(len(discover_cameras()[0]), 1)


class TrackingVideoTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from PyQt6.QtWidgets import QApplication
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        from tracking_mode_layout import TrackingModeUI
        self.devices = [('D435i', {'source': 'realsense', 'serial': '123'})]
        with patch.object(TrackingModeUI, 'init_ros'), \
                patch.object(TrackingModeUI, 'launch_tracking'), \
                patch('tracking_mode_layout.discover_cameras', return_value=(self.devices, [])):
            self.window = TrackingModeUI()
        self.window.ros_node = Mock()

    def tearDown(self):
        self.window.cleanup()
        self.window.deleteLater()

    def test_no_camera_blocks_launch_and_refresh_restores_selection(self):
        window = self.window
        with patch('tracking_mode_layout.discover_cameras', return_value=([], [])):
            window.refresh_cameras()
        self.assertFalse(window.restart_button.isEnabled())
        with patch('tracking_mode_layout.TrackingProcess') as create:
            window.launch_tracking()
            create.assert_not_called()
        with patch('tracking_mode_layout.discover_cameras', return_value=(self.devices, [])):
            window.refresh_cameras()
        self.assertTrue(window.restart_button.isEnabled())
        self.assertFalse(window.motion_enabled.isChecked())

    def test_selected_v4l_source_and_stop_restart(self):
        window = self.window
        window.camera_source.addItem('USB', {'source': '/dev/v4l/by-id/usb camera', 'serial': ''})
        window.camera_source.setCurrentIndex(1)
        with patch('tracking_mode_layout.TrackingProcess') as create:
            owner = create.return_value
            owner.read_log.return_value = ''
            owner.process.poll.return_value = None
            owner.stop.side_effect = lambda: setattr(owner, 'process', None)
            window.launch_tracking()
            import shlex
            args = shlex.split(owner.start.call_args.args[0])
            self.assertIn('camera_url:=/dev/v4l/by-id/usb camera', args)
            self.assertIn('enable_motion:=false', args)
            window.stop_tracking()
            self.assertFalse(window.restart_button.isEnabled())
            window._stopping.result(timeout=2)
            window.check_tracking()
            self.assertIn('Đã dừng', window.status_label.text())
            self.assertTrue(window.restart_button.isEnabled())
            self.assertFalse(window.stop_button.isEnabled())
            window.launch_tracking()
            self.assertEqual(owner.start.call_count, 2)

    def test_preview_copies_new_frames_and_reports_timeout(self):
        from sensor_msgs.msg import Image
        window = self.window
        window.tracking_process = Mock()
        window.tracking_process.read_log.return_value = ''
        window.tracking_process.process.poll.return_value = None
        window.last_status_time = time.monotonic()
        window.launch_started = time.monotonic()
        msg = Image(width=2, height=1, encoding='bgr8', step=6, data=bytes([0, 0, 255] * 2))
        window.image_callback(msg)
        self.assertEqual(window.video.frame.pixelColor(0, 0).red(), 255)
        msg.data = bytes([255, 0, 0] * 2)
        self.assertEqual(window.video.frame.pixelColor(0, 0).red(), 255)
        window.image_callback(msg)
        self.assertEqual(window.video.frame.pixelColor(0, 0).blue(), 255)
        window.last_frame_time -= 4
        window.check_tracking()
        self.assertIsNone(window.video.frame)
        self.assertIn('Không nhận được frame', window.video.text())
        msg.step = 1
        window.image_callback(msg)
        self.assertIn('không hợp lệ', window.video.text())

    def test_node_image_matches_capture_and_stop_publishes_zero(self):
        from human_following.human_following_node import HumanFollowingNode
        node = SimpleNamespace(image_publisher=Mock(), get_clock=Mock(),
                               publisher=Mock(), camera=Mock())
        from builtin_interfaces.msg import Time
        node.get_clock.return_value.now.return_value.to_msg.return_value = Time()
        frame = np.arange(18, dtype=np.uint8).reshape(2, 3, 3)
        HumanFollowingNode.publish_frame(node, frame)
        msg = node.image_publisher.publish.call_args.args[0]
        self.assertEqual(bytes(msg.data), frame.tobytes())
        self.assertEqual((msg.width, msg.height, msg.step, msg.encoding), (3, 2, 9, 'bgr8'))
        with patch('human_following.human_following_node.rclpy.ok', return_value=True):
            HumanFollowingNode.stop(node)
        velocity = node.publisher.publish.call_args.args[0]
        self.assertEqual((velocity.linear.x, velocity.angular.z), (0, 0))
        node.camera.release.assert_called_once()


if __name__ == '__main__':
    unittest.main()
