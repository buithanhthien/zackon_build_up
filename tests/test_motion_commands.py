import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'robot_ui'))
from motion_commands import parse_motion, validate_motion
from motion_controller import MotionController, SENSOR_TIMEOUT, OBSTACLE_CLEARANCE


class GrammarTests(unittest.TestCase):
    def test_requested_commands(self):
        examples = [
            ('xoay trái 1 vòng', {'type': 'rotate', 'direction': 'left', 'revolutions': 1}),
            ('xoay phải 5 vòng', {'type': 'rotate', 'direction': 'right', 'revolutions': 5}),
            ('đi thẳng 3 giây', {'type': 'move', 'direction': 'forward', 'duration_seconds': 3}),
            ('đi lùi 4 giây', {'type': 'move', 'direction': 'backward', 'duration_seconds': 4}),
            ('xoay trái đến khi tôi bảo dừng', {'type': 'rotate_continuous', 'direction': 'left'}),
            ('xoay trái 180 độ', {'type': 'rotate', 'direction': 'left', 'degrees': 180}),
            ('Bé Son, quay sang trái một trăm tám mươi độ', {'type': 'rotate', 'direction': 'left', 'degrees': 180}),
            ('đi lùi một phẩy năm mét', {'type': 'navigate_move', 'direction': 'backward', 'distance_meters': 1.5}),
            ('ha ha tiến không chấm năm mét', {'type': 'navigate_move', 'direction': 'forward', 'distance_meters': 0.5}),
            ('ha ha tiến không phẩy năm mét', {'type': 'navigate_move', 'direction': 'forward', 'distance_meters': 0.5}),
            ('tiến 0.5 mét', {'type': 'navigate_move', 'direction': 'forward', 'distance_meters': 0.5}),
        ]
        for text, action in examples:
            with self.subTest(text=text):
                self.assertEqual(parse_motion(text), {'intent': 'motion', 'actions': [action]})

    def test_sequence(self):
        data = parse_motion('xoay trái 3 vòng rồi đi thẳng 5 giây sau đó đi lùi 3 giây')
        self.assertEqual(data['actions'], [
            {'type': 'rotate', 'direction': 'left', 'revolutions': 3},
            {'type': 'move', 'direction': 'forward', 'duration_seconds': 5},
            {'type': 'move', 'direction': 'backward', 'duration_seconds': 3}])

    def test_stop(self):
        for text in ('dừng', 'dừng lại', 'Bé Son dừng lại', 'stop', 'hủy hành trình'):
            self.assertEqual(parse_motion(text)['actions'], [{'type': 'stop'}])

    def test_conversation_and_waypoints(self):
        for text in ('xin chào', 'Bé Son đi đến phòng A1', 'tôi muốn biết xoay trái nghĩa là gì',
                     'đừng xoay trái 1 vòng', 'xoay trái 180 độ được không?',
                     'Bé Son hãy giải thích lệnh dừng lại', 'Bé Son quay về chỗ cũ', 'nếu xoay trái 1 vòng thì sao'):
            with self.subTest(text=text):
                self.assertIsNone(parse_motion(text))

    def test_reject_ambiguous_out_of_bounds_and_partial(self):
        for text in ('xoay trái', 'đi thẳng 3', 'xoay trái 6 vòng', 'đi lùi 0 giây',
                     'đi thẳng -3 mét', 'xoay trái 1 vòng rồi đi thẳng', 'xoay phải 2 giây',
                     'xoay trái liên tục rồi đi thẳng 3 giây', 'đi thẳng 31 giây', 'đi lùi 6 mét'):
            with self.subTest(text=text), self.assertRaises(ValueError):
                parse_motion(text)

    def test_untrusted_payload(self):
        for value in (True, '3', float('nan'), float('inf'), -1, 0, 31):
            with self.subTest(value=value), self.assertRaises(ValueError):
                validate_motion({'intent': 'motion', 'actions': [dict(type='move', direction='forward', duration_seconds=value)]})
        for actions in ([], [dict(type='fly')], [dict(type='stop'), dict(type='stop')],
                        [dict(type='rotate', direction='left', degrees=180, revolutions=1)],
                        [dict(type='rotate_continuous', direction='left', speed=3)]):
            with self.assertRaises(ValueError):
                validate_motion({'intent': 'motion', 'actions': actions})


class ExecutionTests(unittest.TestCase):
    def setUp(self):
        self.now = 0.0
        self.commands = []
        self.status = []
        self.controller = MotionController(lambda x, z: self.commands.append((x, z)), self.status.append, lambda: self.now)
        self.x = self.yaw = 0.0
        self.refresh()

    def refresh(self):
        self.controller.update_odom(self.x, 0., self.yaw)
        for side in ('front', 'rear'):
            self.controller.update_scan(side, [2.0], 0.1, 10.)

    def advance(self, dt=0.05):
        self.now += dt
        self.refresh()
        self.controller.tick()

    def start(self, text):
        self.controller.start(parse_motion(text))
        self.controller.tick()

    def test_order_and_measured_distance(self):
        # The direct controller still supports legacy programmatic distance payloads.
        self.controller.start({'intent': 'motion', 'actions': [
            dict(type='rotate', direction='left', degrees=180),
            dict(type='move', direction='forward', duration_seconds=1),
            dict(type='move', direction='backward', distance_meters=1)]})
        self.controller.tick()
        self.assertGreater(self.commands[-1][1], 0)
        for i in range(32):
            self.yaw += 0.1
            self.advance()
        self.assertEqual(self.commands[-1], (0., 0.))
        self.advance()
        self.assertGreater(self.commands[-1][0], 0)
        for i in range(21):
            self.advance()
        self.assertEqual(self.commands[-1], (0., 0.))
        self.advance()
        self.assertLess(self.commands[-1][0], 0)
        for i in range(11):
            self.x += 0.1
            self.advance()
        self.assertFalse(self.controller.active)
        self.assertEqual(self.commands[-1], (0., 0.))

    def test_stop_clears_sequence_and_continuous(self):
        for text in ('xoay trái đến khi tôi bảo dừng', 'xoay trái 3 vòng rồi đi thẳng 5 giây'):
            self.start(text)
            self.controller.stop()
            count = len(self.commands)
            self.advance()
            self.assertEqual(len(self.commands), count)
            self.assertFalse(self.controller.active)
            self.assertEqual(self.commands[-1], (0., 0.))

    def test_continuous_has_no_default_timeout(self):
        self.start('xoay trái đến khi tôi bảo dừng')
        for i in range(2400):
            self.advance()
        self.assertTrue(self.controller.active)

    def test_obstacle_sensor_loss_and_loop_stall(self):
        for fault in ('obstacle', 'sensor', 'stall'):
            self.setUp()
            self.start('đi thẳng 3 giây rồi đi lùi 1 giây')
            if fault == 'obstacle':
                self.controller.update_scan('rear', [OBSTACLE_CLEARANCE / 2], 0.01, 10.)
            elif fault == 'sensor':
                self.now += SENSOR_TIMEOUT + 0.1
                self.controller.last_tick = self.now
            else:
                self.now += SENSOR_TIMEOUT + 0.1
                self.refresh()
            self.controller.tick()
            self.assertFalse(self.controller.active)
            self.assertEqual(self.commands[-1], (0., 0.))

    def test_stalled_odometry_times_out(self):
        self.controller.start({'intent': 'motion', 'actions': [
            dict(type='move', direction='forward', distance_meters=1)]})
        self.controller.tick()
        for i in range(520):
            self.advance()
        self.assertFalse(self.controller.active)
        self.assertIn('thời gian', self.status[-1])

    def test_invalid_sequence_never_publishes(self):
        with self.assertRaises(ValueError):
            self.controller.start({'intent': 'motion', 'actions': [
                dict(type='move', direction='forward', duration_seconds=1), dict(type='fly')]})
        self.assertEqual(self.commands, [])

    def test_publish_error_clears_queue(self):
        self.start('xoay phải 5 vòng rồi đi thẳng 3 giây')
        def fail_nonzero(x, z):
            if x or z:
                raise RuntimeError('disconnect')
            self.commands.append((x, z))
        self.controller.publish = fail_nonzero
        self.advance()
        self.assertFalse(self.controller.active)
        self.assertEqual(self.commands[-1], (0., 0.))


if __name__ == '__main__':
    unittest.main()
