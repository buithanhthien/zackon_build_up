"""Offline sequence and failure-path regression tests; no actuator access."""
import hashlib
import json
from pathlib import Path
import tempfile
import threading
import unittest
from types import SimpleNamespace

from elevator_detector import DEFAULT_CONFIDENCE_THRESHOLD, ElevatorDetector, classify, freshness_error
from elevator_sequence import ElevatorSequence
from motion_controller import MotionController


class Port:
    def __init__(self):
        self.now = 10.0
        self.events = []
        self.navigation = SimpleNamespace(active=False, outcome=None)
        self.controller = SimpleNamespace(active=False, outcome=None)
        self.ready = True
        self.confidence = .95
        self.label = 'unavailable'
        self.error = None
        self.released = False
        self.detector = SimpleNamespace(enable=lambda: self.events.append('detect'), snapshot=self.snapshot,
                                        threshold=DEFAULT_CONFIDENCE_THRESHOLD)

    def snapshot(self):
        return self.ready, '', (self.label, self.confidence, self.now), self.now

    def status(self, message):
        pass

    def elevator_preflight(self):
        if not self.ready:
            raise ValueError('not ready')

    def elevator_map_ready(self):
        return True

    def elevator_navigate(self):
        self.events.append('navigate')
        self.navigation.active = True

    def elevator_hold(self):
        self.events.append('hold')
        return True

    def elevator_move(self):
        self.events.append('move')
        self.controller.active = True

    def elevator_halt(self):
        self.events.append('halt')

    def elevator_released(self):
        return self.released

    def elevator_safety(self):
        return self.error


class SequenceTests(unittest.TestCase):
    def setUp(self):
        self.port = Port()
        self.seq = ElevatorSequence(self.port, lambda: self.port.now)

    def waiting(self):
        self.seq.start()
        self.seq.tick()
        self.assertEqual(self.port.events, ['navigate'])
        self.port.navigation.active = False
        self.port.navigation.outcome = 'succeeded'
        self.seq.tick()
        self.seq.tick()
        self.assertEqual(self.seq.state, 'waiting')

    def test_unavailable_waits_days_and_stop_waits_for_release(self):
        self.waiting()
        for day in range(1, 31):
            self.port.now += 86400
            self.seq.tick()
            self.assertEqual(self.seq.state, 'waiting')
        self.assertNotIn('move', self.port.events)
        self.seq.stop()
        self.seq.tick()
        self.assertEqual(self.seq.state, 'stopping')
        self.port.released = True
        self.seq.tick()
        self.assertEqual(self.seq.state, 'done')

    def test_available_requires_three_distinct_frames_after_arrival(self):
        self.waiting()
        self.port.label = 'available'
        for i in range(3):
            self.port.now += .1
            self.seq.tick()
            if i < 2:
                self.assertNotIn('move', self.port.events)
                for _ in range(10):
                    self.seq.tick()
                self.assertNotIn('move', self.port.events)
        self.assertEqual(self.port.events, ['navigate', 'hold', 'detect', 'move'])
        self.port.controller.active = False
        self.port.controller.outcome = 'succeeded'
        self.seq.tick()
        self.assertEqual(self.seq.state, 'stopping')

    def test_unknown_error_and_camera_loss_hold(self):
        self.waiting()
        for label in ('unknown', 'error'):
            self.port.label = label
            self.port.now += 100000
            self.seq.tick()
            self.assertEqual(self.seq.state, 'waiting')
        self.port.ready = False
        self.seq.tick()
        self.assertEqual(self.seq.state, 'waiting')
        self.assertNotIn('move', self.port.events)

    def test_motion_uses_detector_threshold(self):
        self.waiting()
        self.port.detector.threshold = .9
        self.port.label, self.port.confidence = 'available', .87
        for _ in range(3):
            self.port.now += .1
            self.seq.tick()
        self.assertEqual(self.seq.state, 'waiting')
        self.assertNotIn('move', self.port.events)

    def test_uncertain_available_is_distinct_from_camera_timeout(self):
        self.waiting()
        self.port.label, self.port.confidence = 'unknown', .8
        self.seq.tick()
        self.assertIn('unknown', self.seq.message)
        self.assertIn('85%', self.seq.message)
        self.assertNotIn('camera', self.seq.message)
        self.port.detector.snapshot = lambda: (True, '', ('available', .95, 1.0), self.port.now)
        self.seq.tick()
        self.assertIn('Kết quả nhận diện đã cũ', self.seq.message)
        self.assertEqual(self.seq.state, 'waiting')

    def test_nav_failure_never_detects(self):
        self.seq.start()
        self.seq.tick()
        self.port.navigation.active = False
        self.port.navigation.outcome = 'failed'
        self.seq.tick()
        self.assertEqual(self.port.events, ['navigate', 'halt'])

    def test_stop_in_every_state_prevents_late_motion(self):
        for state in ('preflight', 'navigating', 'acquiring', 'waiting', 'moving'):
            self.seq.state = state
            self.seq.stop()
            self.port.label = 'available'
            self.seq.tick()
            self.assertEqual(self.seq.state, 'stopping')
            self.assertNotIn('move', self.port.events)

    def test_safety_independent_of_wait_timeout(self):
        self.waiting()
        self.port.error = 'emergency / lidar / odom / heartbeat'
        self.seq.tick()
        self.assertEqual(self.seq.state, 'stopping')

    def test_lost_available_during_motion_stops(self):
        self.waiting()
        self.port.label = 'available'
        for _ in range(3):
            self.port.now += .1
            self.seq.tick()
        self.port.ready = False
        self.seq.tick()
        self.assertEqual(self.seq.state, 'stopping')

    def test_preflight_failure_no_motion(self):
        self.port.ready = False
        with self.assertRaises(ValueError):
            self.seq.start()
        self.assertFalse(self.seq.active)
        self.assertEqual(self.port.events, [])


class DetectorTests(unittest.TestCase):
    def test_labels_and_unknown(self):
        self.assertEqual(classify([.9, .1])[0], 'available')
        self.assertEqual(classify([.1, .9])[0], 'unavailable')
        for scores in ([.6, .4], [float('nan'), .9], [.9, .9]):
            self.assertEqual(classify(scores)[0], 'unknown')

    def test_threshold_explains_difference_from_original_predictor(self):
        self.assertEqual(classify([.8, .2], .7)[0], 'available')
        self.assertEqual(classify([.8, .2], .85)[0], 'unknown')
        self.assertEqual(classify([.15, .85], .85)[0], 'unavailable')
        self.assertEqual(classify([.85, .15], .85)[0], 'available')
        for threshold in (0, .5, 1.1, float('nan'), '0.7', True):
            with self.assertRaises(ValueError):
                classify([.8, .2], threshold)

    def test_freshness_reasons(self):
        self.assertIsNone(freshness_error(10, 10, 10.5))
        self.assertIn('Chưa nhận được frame', freshness_error(0, 0, 10))
        self.assertIn('đầu tiên', freshness_error(0, 10, 10.5))
        self.assertIn('Frame camera đã cũ', freshness_error(10, 8, 10.5))
        self.assertIn('Kết quả nhận diện đã cũ', freshness_error(8, 10, 10.5))
        self.assertIn('sai thời gian', freshness_error(11, 10, 10.5))

    def test_diagnostics_show_probabilities_and_specific_reason(self):
        detector = ElevatorDetector.__new__(ElevatorDetector)
        detector.lock = threading.Lock()
        detector.ready, detector.error, detector.input_error = True, '', ''
        detector.enabled = True
        detector.threshold = .85
        detector.frame = (None, 10.0)
        detector.result = ('unknown', .8, 10.0)
        detector.probabilities = (.8, .2)
        detector.inference_seconds = .03
        message = detector.describe(now=10.5)
        self.assertIn('available: 80.0%', message)
        self.assertIn('unavailable: 20.0%', message)
        self.assertIn('chưa đạt ngưỡng', message)
        self.assertNotIn('đã cũ', message)
        self.assertIn('Frame camera đã cũ', detector.describe(now=12))

    def test_missing_corrupt_and_unlicensed_model(self):
        for kind in ('missing', 'corrupt', 'unlicensed'):
            with tempfile.TemporaryDirectory() as directory:
                root = Path(directory)
                manifest = {'license_status': 'pending' if kind == 'unlicensed' else 'authorized',
                            'sha256': hashlib.sha256(b'bad checkpoint').hexdigest()}
                (root / 'manifest.json').write_text(json.dumps(manifest))
                if kind == 'corrupt':
                    (root / 'best.pt').write_bytes(b'bad checkpoint')
                detector = ElevatorDetector(root)
                detector.thread.join(30)
                detector.close()
                ready, error, result, _ = detector.snapshot()
                self.assertFalse(ready)
                self.assertTrue(error)
                self.assertEqual(result[0], 'error')

    def test_distance_then_rotation_uses_odometry(self):
        import math
        now = [10.0]
        commands = []
        controller = MotionController(lambda v, w: commands.append((v, w)), lambda _: None,
                                      lambda: now[0])
        def sensors(x, yaw):
            now[0] += .1
            controller.update_odom(x, 0, yaw)
            for side in ('front', 'rear'):
                controller.update_scan(side, [10.0], .1, 20.0)
        sensors(0, 0)
        controller.start({'intent': 'motion', 'actions': [
            {'type': 'move', 'direction': 'forward', 'distance_meters': 2},
            {'type': 'rotate', 'direction': 'left', 'degrees': 180}]})
        controller.tick()
        for i in range(1, 11):
            sensors(i * .2, 0)
            controller.tick()
        self.assertFalse(any(w for v, w in commands))
        sensors(2, 0)
        controller.tick()
        for i in range(1, 12):
            sensors(2, min(i * .3, math.pi))
            controller.tick()
        self.assertFalse(controller.active)
        self.assertEqual(controller.outcome, 'succeeded')
        self.assertEqual(commands[-1], (0.0, 0.0))

# These tests use ROS message classes only; no node, executor or robot connection.
try:
    from elevator_ros import ElevatorMixin
    from nav_msgs.msg import OccupancyGrid
except ImportError:
    ElevatorMixin = None


@unittest.skipIf(ElevatorMixin is None, 'Source ROS setup to test message adapter')
class AdapterTests(unittest.TestCase):
    def test_camera_converts_bgr_with_padding_and_reports_bad_encoding(self):
        from rclpy.clock import Clock
        from sensor_msgs.msg import Image
        adapter = ElevatorMixin()
        clock = Clock()
        adapter.node = SimpleNamespace(get_clock=lambda: clock)
        frames, errors = [], []
        adapter.detector = SimpleNamespace(submit=lambda rgb, stamp: frames.append(rgb),
                                           reject_frame=errors.append)
        msg = Image()
        msg.header.stamp = clock.now().to_msg()
        msg.height = msg.width = 1
        msg.step = 4
        msg.encoding = 'bgr8'
        msg.data = [10, 20, 30, 0]
        adapter._elevator_frame(msg)
        self.assertEqual(frames[0].tolist(), [[[30, 20, 10]]])
        msg.encoding = 'mono8'
        adapter._elevator_frame(msg)
        self.assertEqual(len(frames), 1)
        self.assertIn('rgb8 hoặc bgr8', errors[-1])

    def test_map_requires_cells_and_pose_not_only_filename(self):
        import numpy as np
        adapter = ElevatorMixin()
        adapter._expected_map = (np.array([[0, 100], [-1, 0]], dtype=np.int8), .05, [1, 2, 0])
        msg = OccupancyGrid()
        msg.header.frame_id = 'map'
        msg.info.width = msg.info.height = 2
        msg.info.resolution = .05
        msg.info.origin.position.x = 1.0
        msg.info.origin.position.y = 2.0
        msg.info.origin.orientation.w = 1.0
        msg.data = [0, 100, -1, 0]
        self.assertTrue(adapter._matches_elevator_map(msg))
        msg.data = [0, 0, -1, 0]
        self.assertFalse(adapter._matches_elevator_map(msg))
        msg.data = [0, 100, -1, 0]
        msg.info.origin.position.x = 1.1
        self.assertFalse(adapter._matches_elevator_map(msg))

    def test_manual_confirmation_needs_terminal_goal_and_fresh_arbiter(self):
        import time
        adapter = ElevatorMixin()
        adapter.navigation = SimpleNamespace(active=True)
        adapter.controller = SimpleNamespace(active=False)
        adapter._pending = adapter._claim_future = adapter._release_future = None
        adapter._leased = False
        adapter.arbiter_stamp = time.monotonic()
        adapter.arbiter_state = dict(healthy=True, emergency_stop=False, ui_granted=False,
                                    navigation_busy=False, owner='')
        self.assertFalse(adapter.elevator_released())
        adapter.navigation.active = False
        self.assertTrue(adapter.elevator_released())
        adapter.arbiter_state['ui_granted'] = True
        self.assertFalse(adapter.elevator_released())
        adapter.arbiter_state['ui_granted'] = False
        adapter.arbiter_stamp -= 1
        self.assertFalse(adapter.elevator_released())


if __name__ == '__main__':
    unittest.main()
