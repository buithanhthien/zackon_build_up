import json
import math
import time
from types import SimpleNamespace as NS

import numpy as np
import pytest
from std_msgs.msg import Bool, String

from dock_shape_sim.safety import DockingSafety
from dock_shape_sim.mission.calibration import teach_pre, teach_terminal
from dock_shape_sim.mission.calibration import DockCalibration
from dock_shape_sim.perception.v_shape import ProfilePose
from dock_shape_sim.mission.node import DockMission


def safety():
    node = NS(declare_parameter=lambda name, default: NS(value=True if name=='require_safety_permit' else default),
              create_subscription=lambda *args: None)
    clock = [10.]
    guard = DockingSafety(node, clock=lambda: clock[0])
    return guard, clock


def ready(guard, owner=''):
    guard._heartbeat(Bool(data=True))
    guard._arbiter(String(data=json.dumps(dict(owner=owner, healthy=True, emergency_stop=False))))


def test_hardware_requires_both_ui_heartbeat_and_healthy_arbiter():
    guard, clock = safety()
    assert guard.error()
    ready(guard)
    assert guard.error() is None
    clock[0] += .51
    assert 'arbiter' in guard.error()
    ready(guard)
    guard._heartbeat(Bool(data=False))
    assert guard.error()


@pytest.mark.parametrize('owner', ['teleop', 'localization', 'following', 'ui'])
def test_displaced_controller_must_abort_instead_of_resuming_later(owner):
    guard, _ = safety()
    ready(guard, owner)
    assert 'giành quyền' in guard.error()


def test_invalid_arbiter_message_cannot_renew_permission():
    guard, clock = safety()
    ready(guard)
    clock[0] += .6
    guard._heartbeat(Bool(data=True))
    guard._arbiter(String(data='{"owner":"", "healthy":"true"}'))
    assert guard.error()


def test_map_mismatch_and_expired_map_verification_block_mission():
    guard, clock = safety()
    ready(guard)
    guard.expected_map = '/room.yaml'
    guard.map_error = 'Sai map'
    assert guard.error() == 'Sai map'
    guard.map_error, guard.map_stamp = None, clock[0]
    assert guard.error() is None
    guard.map_stamp -= 4.
    assert 'map_server' in guard.error()


def sample(session='one'):
    return dict(map_pose=dict(x=0., y=0., yaw=0.), odom_pose=dict(x=0., y=0., yaw=0.),
                stamp=1., session_id=session)


def test_teaching_new_session_discards_old_odom_calibration():
    profile = {'terminal_sample': sample('old'), 'docked_reference': {'x': .4}}
    teach_pre(profile, sample(), ProfilePose(.9, 0., 0., 0., True, ()))
    assert 'terminal_sample' not in profile and 'docked_reference' not in profile
    assert profile['pre_docking']['session_id'] == 'one'


def test_hardware_terminal_teaching_never_enables_blind_finish():
    points = np.array([[.4, -.05], [.4, 0.], [.4, .05]])
    with pytest.raises(ValueError, match='đầy đủ'):
        teach_terminal({}, sample(), points, None)
    profile = {'pre_docking': {'session_id': 'old'}}
    teach_terminal(profile, sample(), points, ProfilePose(.4, 0., 0., 0., True, ()))
    assert not profile['docked_reference']['blind_finish']
    assert 'pre_docking' not in profile


def test_cancel_is_not_confirmed_until_nav_result_is_terminal():
    callbacks = []
    handle = NS(cancel_goal_async=lambda: None,
                get_result_async=lambda: NS(add_done_callback=callbacks.append))
    node = NS(_cancelling_goals={}, _failure_detail='')
    node._cancel_result = lambda future, key: DockMission._cancel_result(node, future, key)
    DockMission._cancel_goal(node, handle)
    assert node._cancelling_goals
    callbacks[0](NS(result=lambda: NS(status=2)))
    assert node._cancelling_goals
    callbacks[0](NS(result=lambda: NS(status=5)))
    assert not node._cancelling_goals


def test_teaching_captures_current_map_tf_separately_from_filtered_odometry(monkeypatch):
    from nav_msgs.msg import Odometry
    from builtin_interfaces.msg import Time
    from dock_shape_sim.mission import calibration
    odom = Odometry()
    odom.header.stamp = Time(sec=10)
    odom.pose.pose.position.x = 1.
    odom.pose.pose.orientation.w = 1.
    scan = NS(header=NS(stamp=Time(sec=10), frame_id='rear_lidar'))
    map_tf = NS(header=NS(stamp=Time(sec=10)), transform=NS(
        translation=NS(x=5., y=6.),
        rotation=NS(x=0., y=0., z=math.sin(.2), w=math.cos(.2))))
    requested = []
    def lookup(target, source, stamp):
        requested.append((target, source))
        return map_tf if target == 'map' else NS(transform='lidar-transform')
    points = np.array([[.4, 0.]])
    monkeypatch.setattr(calibration, 'scan_points', lambda *args: points)
    node = NS(_mission_time=time.monotonic(), _mission_state='IDLE', _map_verified=True,
        _mission_cancel_pending=False, _scan_time=time.monotonic(), _odom_time=time.monotonic(),
        _scan=scan, _odom=odom, _session='current-session', _tf=NS(lookup_transform=lookup),
        get_clock=lambda: NS(now=lambda: NS(nanoseconds=10_000_000_000)))
    sample, captured = DockCalibration._sample(node)
    assert sample['map_pose'] == pytest.approx(dict(x=5., y=6., yaw=.4))
    assert sample['odom_pose'] == dict(x=1., y=0., yaw=0.)
    assert sample['session_id'] == 'current-session'
    assert captured is points
    assert requested == [('map', 'base_link'), ('base_link', 'rear_lidar')]


@pytest.mark.parametrize('terminal_first', [False, True])
def test_two_capture_services_save_both_current_map_poses(tmp_path, monkeypatch, terminal_first):
    from dock_shape_sim.mission import calibration
    from dock_shape_sim.mission.profile import load_profile
    map_file = tmp_path/'room.yaml'
    map_file.write_text('image: room.pgm\nresolution: 0.05\n')
    (tmp_path/'room.pgm').write_bytes(b'P5\n1 1\n255\n\xff')
    node = NS(map_file=str(map_file))
    node._save = lambda response, terminal: DockCalibration._save(node, response, terminal)
    monkeypatch.setattr(calibration, 'detect_profile',
        lambda points: ProfilePose(float(points[0, 0]), 0., 0., 0., True, ()))
    for terminal in ([True, False] if terminal_first else [False, True]):
        offset = .6 if terminal else 0.
        captured = dict(map_pose=dict(x=5.+offset, y=6., yaw=math.pi),
            odom_pose=dict(x=offset, y=0., yaw=math.pi), stamp=10.+offset, session_id='one')
        distance = .4 if terminal else 1.
        points = np.array([[distance, -.05], [distance, 0.], [distance, .05]])
        node._sample = lambda: (captured, points)
        service = DockCalibration._terminal if terminal else DockCalibration._pre
        response = service(node, None, NS(success=False, message=''))
        assert response.success, response.message
        assert f"x={captured['map_pose']['x']:.3f}" in response.message
    saved = load_profile(map_file)
    assert saved['pre_docking']['x'] == 5.
    assert saved['terminal_sample']['map_pose']['x'] == 5.6
    assert saved['calibration_status'] == 'ready'
    assert saved['hardware_calibration'] and not saved['docked_reference']['blind_finish']
