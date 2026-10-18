"""Status boundary regression tests using real ROS messages, without a camera."""
import json
import unittest
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np

from human_following.human_following_node import HumanFollowingNode


class TrackingStatusTests(unittest.TestCase):
    def setUp(self):
        self.node = SimpleNamespace(
            camera=Mock(requires_depth=False), detector=Mock(), tracker=Mock(),
            enable_motion=False, use_nav2=False,
            status=dict(camera='connected', detector='ready'),
            publisher=Mock(), status_publisher=Mock(), publish_frame=Mock(),
            get_clock=Mock(), get_logger=Mock(), last_locked_id=None,
            fail=Mock(), max_frame_age=10.)
        self.node.camera.read.return_value = (
            True, np.zeros((4, 4, 3), dtype=np.uint8))
        self.node.tracker.locked_id = None

    def frame(self, distance):
        self.node.tracker.measurement_status = 'estimated' if distance is not None else 'searching'
        pose = dict(x=1., y=0., distance=distance) if distance is not None else None
        self.node.tracker.get_human_position.return_value = (0., 0., [], pose)
        HumanFollowingNode.process_frame(self.node)
        self.node.fail.assert_not_called()

    def publish(self):
        HumanFollowingNode.publish_status(self.node)
        return json.loads(self.node.status_publisher.publish.call_args.args[0].data)

    def test_numpy_float32_remains_json_number(self):
        self.frame(np.float32(1.25))
        self.assertIs(type(self.node.status['distance']), float)
        status = self.publish()
        self.assertIs(type(status['distance']), float)
        self.assertEqual(status['distance'], 1.25)
        self.assertEqual(self.publish()['distance'], 1.25)

    def test_no_person_publishes_null_and_next_publish_survives(self):
        self.frame(np.float32(1.25))
        self.publish()
        self.frame(None)
        for _ in range(2):
            status = self.publish()
            self.assertIsNone(status['distance'])
            self.assertEqual(status['tracking'], 'searching')
        self.frame(np.float32(2.5))
        self.assertEqual(self.publish()['distance'], 2.5)

    def test_no_person_on_startup(self):
        self.frame(None)
        self.assertIsNone(self.publish()['distance'])
        self.assertIsNone(self.publish()['distance'])


if __name__ == '__main__':
    unittest.main()
