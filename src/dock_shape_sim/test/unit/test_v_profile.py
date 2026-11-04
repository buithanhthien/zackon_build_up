import math

import numpy as np
import pytest

from dock_shape_sim.perception.v_shape import (detect_profile, docking_errors, signature_error,
                                     terminal_shoulder_reference)
from dock_shape_sim.mission.profile import compose, inverse, update_reference


def profile(x=1.4, y=0.0, yaw=0.0, minimum_range=0.0):
    lateral = np.linspace(-0.125, 0.125, 160)
    points = np.column_stack((np.maximum(0.0, 0.05-np.abs(lateral)), lateral))
    matrix = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
    points = points @ matrix.T + [x, y]
    return points[np.linalg.norm(points-[0.26014,0], axis=1) >= minimum_range]


@pytest.mark.parametrize('pose', [(1.4,0,0), (1.2,.12,.25), (.46,-.03,-.20)])
def test_250mm_face_pose(pose):
    found = detect_profile(profile(*pose))
    assert found is not None and found.full_shape
    assert (found.x, found.y, found.yaw) == pytest.approx(pose, abs=0.002)


def test_no_acquisition_from_a_flat_wall_or_blind_shoulders():
    wall = np.column_stack((np.full(160, 1.0), np.linspace(-.125,.125,160)))
    assert detect_profile(wall) is None
    partial = profile(.36, minimum_range=.15)
    assert detect_profile(partial) is None
    tracked = detect_profile(partial, allow_partial=True)
    assert tracked is not None and not tracked.full_shape


def test_three_islands_at_sensor_minimum_range_remain_trackable():
    # Actual Gazebo returns: shoulders at |y|>.10 and a small central tip;
    # both sloping branches are inside the scanner's 0.15 m minimum range.
    points = profile(.37, minimum_range=.15)
    tracked = detect_profile(points, allow_partial=True)
    assert tracked is not None
    assert tracked.x == pytest.approx(.37, abs=.002)
    assert abs(tracked.y) < .002


def test_250mm_face_inside_sensor_minimum_range_cannot_be_tracked():
    points = profile(.32, minimum_range=.15)
    assert len(points) == 0
    assert detect_profile(points) is None
    assert detect_profile(points, allow_partial=True) is None


def test_terminal_visible_outer_edge_refines_translation_not_yaw():
    points = np.column_stack((np.full(20, .302), np.linspace(-.152, -.122, 20)))
    predicted = dict(x=.31, y=0., yaw=0.)
    measured = terminal_shoulder_reference(points, predicted)
    assert measured == pytest.approx(dict(x=.302, y=-.027, yaw=0.))
    assert detect_profile(points) is None  # Teaching evidence is NOT V acquisition.
    assert terminal_shoulder_reference(points + [.2, 0.], predicted) is None


def test_controller_errors_and_teaching_use_rigid_frames():
    data = dict(pre_docking=dict(stamp=10, session_id='same',
                                marker_in_odom=dict(x=1.4,y=0,yaw=0)),
                terminal_sample=dict(stamp=20, session_id='same',
                                     odom_pose=dict(x=1.04,y=0,yaw=math.pi),
                                     full_shape_visible=False, near_points=[[.3, -.1], [.3, 0], [.3, .1]]))
    update_reference(data)
    ref = data['docked_reference']
    assert ref['x'] == pytest.approx(.36)
    assert ref['blind_finish']
    current = detect_profile(profile(1.4))
    ex, ey, etheta = docking_errors(current, ref)
    assert ex == pytest.approx(-1.04)
    assert abs(ey) < 1e-8 and abs(etheta) < 1e-8
    data['terminal_sample']['session_id'] = 'different-reset'
    with pytest.raises(ValueError):
        update_reference(data)


def test_pose_inverse_and_signature():
    a = dict(x=1.2,y=-3.4,yaw=1.1)
    assert compose(a,inverse(a)) == pytest.approx(dict(x=0,y=0,yaw=0), abs=1e-10)
    points = profile(.4)
    assert signature_error(points, points) == 0.0
    assert signature_error(points + [0.05,0], points) > 0.02
    assert math.isinf(signature_error([], []))


@pytest.mark.parametrize('noise, accepted', [(.015, True), (.025, False)])
def test_fit_rms_limit_is_20mm(noise, accepted):
    points = profile()
    points[::2, 0] += noise
    points[1::2, 0] -= noise
    diagnostic = {}
    found = detect_profile(points, diagnostics=diagnostic)
    assert diagnostic['best_fit_full_shape']
    if accepted:
        assert found is not None
        assert .009 < found.rms <= .020
    else:
        assert found is None
        assert diagnostic['best_fit_rms'] > .020
