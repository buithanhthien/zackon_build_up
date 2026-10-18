"""Depth measurements and fail-closed following; no hardware required."""
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/human_following'))
from human_following.tracker import HumanTracker
from human_following.camera import Camera

OPTICAL_TO_BASE = np.array([[0, 0, 1, .1], [-1, 0, 0, 0],
                            [0, -1, 0, .5], [0, 0, 0, 1]], dtype=float)


def results(*boxes):
    detections = []
    for bbox in boxes:
        tensor = Mock()
        tensor.cpu.return_value.numpy.return_value = np.array(bbox)
        detections.append(NS(xyxy=[tensor]))
    return [NS(boxes=detections)]


def deproject(pixel, depth):
    return [(pixel[0]-320)*depth/600, (pixel[1]-240)*depth/600, depth]


class DepthTrackerTests(unittest.TestCase):
    def setUp(self):
        self.tracker = HumanTracker()
        self.depth = np.full((480, 640), 2., dtype=np.float32)
        self.boxes = results([220, 30, 420, 450])
        self.now = 0.

    def measure(self, depth=None, boxes=None, matrix=OPTICAL_TO_BASE):
        self.now += .1
        return self.tracker.get_human_position(
            self.boxes if boxes is None else boxes, self.now,
            self.depth if depth is None else depth, deproject, matrix, True)

    def ready(self):
        for _ in range(4):
            output = self.measure()
        return output

    def test_metric_position_transform_and_speed_limit(self):
        linear, angular, tracked, pose = self.ready()
        self.assertAlmostEqual(pose['x'], 2.1, places=5)
        self.assertAlmostEqual(pose['y'], 1/600, places=5)
        self.assertEqual(pose['frame_id'], 'base_link')
        self.assertGreater(linear, 0)
        self.assertLessEqual(linear, .2)
        self.assertLessEqual(abs(angular), .5)

    def test_robust_median_rejects_isolated_noise(self):
        self.depth[::4, ::4] = 0
        self.depth[1::4, 1::4] = 2.9
        self.depth[2::8, 2::8] = np.nan
        self.assertAlmostEqual(self.ready()[3]['x'], 2.1, places=5)

    def test_invalid_depth_clears_history_and_never_estimates(self):
        self.ready()
        for value in (0, np.nan, np.inf, .1, 10):
            output = self.measure(np.full_like(self.depth, value))
            self.assertEqual(output[:2], (0., 0.))
            self.assertIsNone(output[3])
            self.assertEqual(self.tracker.measurement_status, 'depth_invalid')
        self.assertIsNone(self.measure()[3])  # reacquisition needs fresh samples

    def test_missing_depth_intrinsics_tf_and_mismatched_shape(self):
        for depth, project, matrix in [(None, deproject, OPTICAL_TO_BASE),
                                      (self.depth, None, OPTICAL_TO_BASE),
                                      (self.depth, deproject, None),
                                      (self.depth[:10], deproject, OPTICAL_TO_BASE)]:
            output = self.tracker.get_human_position(
                self.boxes, .1, depth, project, matrix, True)
            self.assertEqual(output[:2], (0., 0.))
            self.assertIsNone(output[3])

    def test_mixed_surface_and_abrupt_jump_stop(self):
        self.ready()
        mixed = self.depth.copy()
        mixed[:, 320:] = 2.8
        self.assertIsNone(self.measure(mixed)[3])
        self.ready()
        self.assertIsNone(self.measure(np.full_like(self.depth, 2.8))[3])
        self.assertEqual(self.tracker.measurement_status, 'depth_warming')

    def test_locked_person_missing_does_not_follow_other_or_stale_box(self):
        self.ready()
        self.tracker.locked_id = 1
        other = results([10, 40, 100, 430])
        self.assertIsNone(self.measure(boxes=other)[3])
        self.assertEqual(self.tracker.measurement_status, 'target_lost')
        self.assertIsNone(self.measure(boxes=results())[3])

    def test_selected_person_depth_not_largest_background_area(self):
        self.boxes = results([220, 30, 420, 450], [20, 60, 120, 420])
        self.depth[:, :150] = 1.
        self.assertAlmostEqual(self.ready()[3]['x'], 2.1, places=5)

    def test_camera_right_is_robot_right_and_no_reverse_when_close(self):
        self.boxes = results([340, 30, 540, 450])
        _, angular, _, pose = self.ready()
        self.assertLess(pose['y'], 0)
        self.assertLess(angular, 0)
        self.depth.fill(.5)
        self.assertEqual(self.ready()[0], 0)

    def test_invalid_configuration_is_rejected(self):
        for options in (dict(depth_min=4), dict(max_linear=float('inf')),
                        dict(desired_distance=float('nan')), dict(max_angular=-1)):
            with self.assertRaises(ValueError):
                HumanTracker(**options)

    def test_colour_only_compatibility(self):
        output = self.tracker.get_human_position(self.boxes, .1)
        self.assertIsNotNone(output[3])
        self.assertEqual(self.tracker.measurement_status, 'estimated')


class DepthCameraTests(unittest.TestCase):
    def test_start_failure_stops_pipeline(self):
        rs = Mock()
        rs.pipeline.return_value.start.side_effect = RuntimeError('disconnected')
        with patch.dict(sys.modules, pyrealsense2=rs):
            with self.assertRaisesRegex(RuntimeError, 'disconnected'):
                Camera()
        rs.pipeline.return_value.stop.assert_called_once()

    def test_url_read_api_stays_colour_only(self):
        with patch('human_following.camera.cv2.VideoCapture') as capture:
            capture.return_value.read.return_value = (True, np.zeros((20, 30, 3)))
            camera = Camera('rtsp://example/camera')
            self.assertTrue(camera.read()[0])
            self.assertFalse(camera.requires_depth)
            self.assertIsNone(camera.depth_m)
            camera.release()


class DepthNodeTests(unittest.TestCase):
    def node(self, nav=False):
        from human_following.human_following_node import HumanFollowingNode
        from rclpy.time import Time
        node = NS(
            camera=NS(requires_depth=True, depth_m=np.zeros((480, 640)),
                      intrinsics=True, deproject=deproject,
                      read=lambda: (True, np.zeros((480, 640, 3)))),
            tracker=HumanTracker(), detector=Mock(), use_nav2=nav, enable_motion=True,
            status={}, get_logger=Mock(), get_clock=Mock(), publisher=Mock(),
            goal_publisher=Mock(), publish_frame=Mock(), last_locked_id=None,
            camera_frame='camera', tf_buffer=Mock(), fail=Mock(),
            cancel_navigation=Mock(), send_nav2_goal=Mock())
        node.get_clock.return_value.now.return_value = Time()
        node.detector.detect.return_value = results([220, 30, 420, 450])
        node.tf_buffer.lookup_transform.return_value.transform = NS(
            translation=NS(x=.1, y=0., z=.5),
            rotation=NS(x=-.5, y=.5, z=-.5, w=.5))
        return node, HumanFollowingNode

    def test_invalid_depth_stops_direct_and_nav(self):
        for nav in (False, True):
            node, cls = self.node(nav)
            cls.process_frame(node)
            node.fail.assert_not_called()
            self.assertEqual(node.status['tracking'], 'depth_invalid')
            command = node.publisher.publish.call_args.args[0]
            self.assertEqual((command.linear.x, command.angular.z), (0., 0.))
            node.goal_publisher.publish.assert_not_called()
            node.send_nav2_goal.assert_not_called()
            if nav:
                node.cancel_navigation.assert_called_once()

    def test_valid_depth_then_loss_clears_command_and_reported_distance(self):
        node, cls = self.node()
        node.camera.depth_m.fill(2.)
        for _ in range(4):
            cls.process_frame(node)
        self.assertGreater(node.status['distance'], 2.)
        self.assertGreater(node.publisher.publish.call_args.args[0].linear.x, 0.)
        node.camera.depth_m = None
        cls.process_frame(node)
        self.assertIsNone(node.status['distance'])
        self.assertEqual(node.publisher.publish.call_args.args[0].linear.x, 0.)

    def test_motion_disabled_even_with_valid_depth(self):
        node, cls = self.node()
        node.enable_motion = False
        node.camera.depth_m.fill(2.)
        for _ in range(4):
            cls.process_frame(node)
        self.assertIsNotNone(node.status['distance'])
        for call in node.publisher.publish.call_args_list:
            self.assertEqual((call.args[0].linear.x, call.args[0].angular.z), (0., 0.))
        node.goal_publisher.publish.assert_not_called()

    def test_tf_missing_and_slow_inference_stop(self):
        from tf2_ros import TransformException
        node, cls = self.node()
        node.camera.depth_m.fill(2)
        node.tf_buffer.lookup_transform.side_effect = TransformException('missing')
        cls.process_frame(node)
        self.assertEqual(node.status['tracking'], 'tf_unavailable')
        node, cls = self.node()
        with patch('human_following.human_following_node.time.monotonic',
                   side_effect=[0., 0., 1.]):
            cls.process_frame(node)
        self.assertEqual(node.status['tracking'], 'frame_stale')

    def test_late_accepted_goal_is_cancelled(self):
        from human_following.human_following_node import HumanFollowingNode
        node = NS(goal_pending=True, goal_generation=2, current_goal_handle=None,
                  get_logger=Mock(), goal_result_callback=Mock())
        handle = Mock(accepted=True)
        HumanFollowingNode.goal_response_callback(node, Mock(result=lambda: handle), 1)
        handle.cancel_goal_async.assert_called_once()
        self.assertFalse(node.goal_pending)
        self.assertIs(node.current_goal_handle, handle)

    def test_nav_path_fixed_frame_standoff_and_cancel_before_replace(self):
        from human_following.human_following_node import HumanFollowingNode
        from rclpy.time import Time
        node = NS(tracker=HumanTracker(), tf_buffer=Mock(), frame_stamp=Time(),
                  nav_frame='odom', max_frame_age=.5, goal_pending=False, last_goal_time=0,
                  goal_interval=1., current_goal_handle=None, goal_generation=0,
                  nav2_client=Mock(), goal_response_callback=Mock())
        node.tf_buffer.lookup_transform.return_value.transform = NS(
            translation=NS(x=10., y=2., z=0.), rotation=NS(x=0., y=0., z=0., w=1.))
        node.tf_buffer.lookup_transform.return_value.header.stamp = Time().to_msg()
        HumanFollowingNode.send_nav2_goal(node, dict(x=2., y=0., distance=2.))
        path = node.nav2_client.send_goal_async.call_args.args[0].path
        self.assertEqual(path.header.frame_id, 'odom')
        self.assertEqual(path.poses[0].pose.position.x, 10.)
        self.assertEqual(path.poses[-1].pose.position.x, 11.)
        self.assertEqual(path.poses[-1].pose.position.y, 2.)
        node.goal_pending = False
        node.last_goal_time = 0
        node.current_goal_handle = Mock()
        HumanFollowingNode.send_nav2_goal(node, dict(x=2., y=0., distance=2.))
        node.current_goal_handle.cancel_goal_async.assert_called_once()
        node.nav2_client.send_goal_async.assert_called_once()
        node.frame_stamp = Time(seconds=3)
        node.tf_buffer.lookup_transform.return_value.header.stamp = Time(seconds=1).to_msg()
        node.cancel_navigation, node.publisher, node.status = Mock(), Mock(), {}
        HumanFollowingNode.send_nav2_goal(node, dict(x=2., y=0., distance=2.))
        node.cancel_navigation.assert_called_once()
        self.assertEqual(node.status['tracking'], 'tf_unavailable')
        self.assertEqual(node.publisher.publish.call_args.args[0].linear.x, 0.)
        node.nav2_client.send_goal_async.assert_called_once()


if __name__ == '__main__':
    unittest.main()
