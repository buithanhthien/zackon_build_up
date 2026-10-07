"""Deterministic action races and relative map targets, without a robot."""
import math
from pathlib import Path
import sys
import threading
from types import SimpleNamespace
from concurrent.futures import Future
from unittest.mock import Mock
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'robot_ui'))
from motion_commands import parse_motion, validate_motion
from motion_navigation import NavigationTask, relative_target
from motion_controller import MotionController


@pytest.mark.parametrize('text,direction,distance', [
    ('tiến 2 mét', 'forward', 2),
    ('đi tiến 2 m', 'forward', 2),
    ('đi thẳng 2 mét', 'forward', 2),
    ('lùi 2 mét', 'backward', 2),
    ('đi lùi 1,5 mét', 'backward', 1.5),
    ('tiến 2 mét né vật cản', 'forward', 2),
    ('Bé Son lùi một phẩy năm mét né vật cản', 'backward', 1.5),
    ('đi thẳng 5 m né vật cản', 'forward', 5),
])
def test_navigation_grammar(text, direction, distance):
    assert parse_motion(text)['actions'] == [dict(type='navigate_move', direction=direction, distance_meters=distance)]


@pytest.mark.parametrize('text', [
    'tiến 2', 'lùi mét', 'tiến 6 mét', 'lùi 0 mét', 'lùi -1 mét',
    'tiến vài mét', 'xoay trái 1 vòng rồi tiến 2 mét', 'tiến 2 mét rồi lùi 1 mét',
    'tiến 2 né vật cản', 'lùi mét né vật cản', 'tiến 2 giây né vật cản',
    'tiến 6 mét né vật cản', 'lùi 0 mét né vật cản', 'lùi -1 mét né vật cản',
    'tiến 2 mét né vật cản rồi lùi 1 mét', 'tiến 2 mét rồi lùi 1 mét né vật cản',
    'tiến vài mét né vật cản', 'tiến 2 mét tránh vật cản',
])
def test_invalid_navigation_never_partially_executes(text):
    with pytest.raises(ValueError):
        parse_motion(text)


@pytest.mark.parametrize('extra', [dict(duration_seconds=2), dict(speed=1), dict(frame='odom')])
def test_navigation_payload_rejects_extra_fields(extra):
    with pytest.raises(ValueError):
        validate_motion(dict(intent='motion', actions=[dict(type='navigate_move', direction='forward', distance_meters=2, **extra)]))


def test_navigation_cannot_enter_twist_controller():
    publish = Mock()
    controller = MotionController(publish, Mock())
    with pytest.raises(ValueError, match='Nav2'):
        controller.start(parse_motion('tiến 1 mét né vật cản'))
    publish.assert_not_called()


@pytest.mark.parametrize('yaw', [0, math.pi/2, math.pi, -math.pi/2, 0.73])
@pytest.mark.parametrize('direction,sign', [('forward', 1), ('backward', -1)])
def test_relative_target(yaw, direction, sign):
    x, y, angle = relative_target(4, -3, yaw, direction, 2)
    assert x == pytest.approx(4 + sign*2*math.cos(yaw))
    assert y == pytest.approx(-3 + sign*2*math.sin(yaw))
    assert angle == yaw


class Harness:
    def __init__(self):
        self.now = 0
        self.status = []
        self.client = Mock()
        self.acceptance, self.result, self.cancel = Future(), Future(), Future()
        self.client.send_goal_async.return_value = self.acceptance
        self.handle = Mock(accepted=True)
        self.handle.get_result_async.return_value = self.result
        self.handle.cancel_goal_async.return_value = self.cancel
        self.task = NavigationTask(self.client, self.status.append, threading.RLock(), lambda: self.now)
        self.task.start(object())

    def accept(self):
        self.acceptance.set_result(self.handle)


@pytest.mark.parametrize('status,word', [(4, 'thành công'), (5, 'đã hủy'), (6, 'thất bại')])
def test_terminal_results(status, word):
    h = Harness()
    h.accept()
    assert h.task.active
    h.result.set_result(SimpleNamespace(status=status))
    assert not h.task.active
    assert word in h.status[-1]


def test_rejection_and_send_failure():
    h = Harness()
    h.acceptance.set_result(Mock(accepted=False))
    assert not h.task.active
    assert 'từ chối' in h.status[-1]
    h = Harness()
    h.acceptance.set_exception(RuntimeError('transport'))
    assert not h.task.active


def test_stop_before_acceptance_cancels_late_goal_and_blocks_new_work():
    h = Harness()
    h.task.stop()
    with pytest.raises(ValueError):
        h.task.start(object())
    h.accept()
    h.handle.cancel_goal_async.assert_called_once()
    h.cancel.set_result(SimpleNamespace(goals_canceling=[object()]))
    assert h.task.active  # Cancel service acknowledgement does not imply stopped.
    h.result.set_result(SimpleNamespace(status=5))
    assert not h.task.active


@pytest.mark.parametrize('accepted', [False, True])
def test_timeout_requests_cancellation_without_claiming_stopped(accepted):
    h = Harness()
    if accepted:
        h.accept()
    h.now = 121 if accepted else 6
    h.task.tick()
    assert h.task.cancel_requested and h.task.active
    assert 'timeout' in h.status[-1]
    if not accepted:
        h.accept()
    h.handle.cancel_goal_async.assert_called_once()


def test_failed_cancellation_keeps_guard_and_retries():
    h = Harness()
    h.accept()
    h.task.stop()
    h.cancel.set_exception(RuntimeError('cancel transport'))
    assert h.task.active
    h.handle.cancel_goal_async.return_value = Future()
    h.now = 3
    h.task.tick()
    assert h.handle.cancel_goal_async.call_count == 2
    assert h.task.active


def adapter_fixture():
    from geometry_msgs.msg import TransformStamped
    from rclpy.time import Time
    from motion_ros import RosMotionController
    adapter = RosMotionController.__new__(RosMotionController)
    adapter.node = Mock()
    adapter.node.get_clock.return_value.now.return_value = Time(seconds=10)
    adapter.node.get_publishers_info_by_topic.return_value = [SimpleNamespace(node_name='controller_server')]
    adapter.arbiter_state = dict(ui_granted=False, owner='', navigation_busy=False)
    adapter._ownership_error = Mock(return_value=None)
    adapter._tf = Mock()
    transform = TransformStamped()
    transform.header.stamp = Time(seconds=10).to_msg()
    transform.transform.translation.x = 3.
    transform.transform.translation.y = 4.
    transform.transform.rotation.z = math.sin(math.pi/4)
    transform.transform.rotation.w = math.cos(math.pi/4)
    adapter._tf.lookup_transform.return_value = transform
    adapter.navigation, adapter._control, adapter.publisher = Mock(), Mock(), Mock()
    return adapter


def test_adapter_uses_fresh_map_tf_without_ui_lease_or_twist():
    adapter = adapter_fixture()
    adapter._start_navigation(parse_motion('lùi 2 mét')['actions'][0])
    goal = adapter.navigation.start.call_args.args[0]
    assert goal.pose.header.frame_id == 'map'
    assert goal.pose.pose.position.x == pytest.approx(3)
    assert goal.pose.pose.position.y == pytest.approx(2)
    assert goal.pose.pose.orientation.z == pytest.approx(math.sin(math.pi/4))
    adapter._control.call_async.assert_not_called()
    adapter.publisher.publish.assert_not_called()
    assert adapter._tf.lookup_transform.call_args.args[:2] == ('map', 'base_link')


@pytest.mark.parametrize('fault', ['tf', 'stale', 'future', 'quaternion', 'route', 'owner', 'ui', 'navigation_busy'])
def test_adapter_refuses_invalid_pose_or_competing_source(fault):
    from rclpy.time import Time
    adapter = adapter_fixture()
    transform = adapter._tf.lookup_transform.return_value
    if fault == 'tf':
        adapter._tf.lookup_transform.side_effect = RuntimeError('no transform')
    elif fault == 'stale':
        transform.header.stamp = Time(seconds=8).to_msg()
    elif fault == 'future':
        transform.header.stamp = Time(seconds=11).to_msg()
    elif fault == 'quaternion':
        transform.transform.rotation.z = transform.transform.rotation.w = 0.
    elif fault == 'route':
        adapter.node.get_publishers_info_by_topic.return_value = []
    elif fault == 'owner':
        adapter.arbiter_state['owner'] = 'docking'
    elif fault == 'ui':
        adapter.arbiter_state['ui_granted'] = True
    else:
        adapter.arbiter_state['navigation_busy'] = True
    with pytest.raises(ValueError):
        adapter._start_navigation(parse_motion('tiến 1 mét né vật cản')['actions'][0])
    adapter.navigation.start.assert_not_called()
    adapter.publisher.publish.assert_not_called()
    adapter._control.call_async.assert_not_called()


def test_close_waits_for_confirmed_nav_cancel_before_destroying_node():
    from motion_ros import RosMotionController
    adapter = adapter_fixture()
    adapter.navigation.active = True
    adapter.stop = Mock(side_effect=lambda: setattr(adapter.navigation, 'active', False))
    adapter.executor, adapter.thread = Mock(), Mock()
    assert adapter.close() is True
    adapter.stop.assert_called_once()
    adapter.executor.shutdown.assert_called_once()
    adapter.node.destroy_node.assert_called_once()


def test_unconfirmed_close_keeps_executor_alive(monkeypatch):
    from motion_ros import RosMotionController
    adapter = adapter_fixture()
    adapter.navigation.active = True
    adapter.stop, adapter.status, adapter.executor, adapter.thread = Mock(), Mock(), Mock(), Mock()
    ticks = iter([0, 6])
    monkeypatch.setattr('motion_ros.time.monotonic', lambda: next(ticks))
    assert adapter.close() is False
    adapter.executor.shutdown.assert_not_called()
    adapter.node.destroy_node.assert_not_called()
    assert 'giữ giao diện' in adapter.status.call_args.args[0]
