import math

import numpy as np
import pytest

from dock_shape_sim.planning.quintic import PlanningError, plan_quintic, swept_collision, rotation_collision
from dock_shape_sim.control.tracker import TrajectoryTracker
from dock_shape_sim.control.teleop import approach


def test_quintic_endpoints_tangents_and_curvature_in_rotated_frame():
    goal = dict(x=2., y=-1., yaw=1.2)
    c, s = math.cos(1.2), math.sin(1.2)
    start = dict(x=2.-.7*c-.26*s, y=-1.-.7*s+.26*c, yaw=1.1)
    path = plan_quintic(start, goal)
    assert path.xy[0] == pytest.approx([start['x'], start['y']])
    assert path.xy[-1] == pytest.approx([goal['x'], goal['y']])
    assert path.yaw[[0, -1]] == pytest.approx([start['yaw'], goal['yaw']])
    assert path.curvature[[0, -1]] == pytest.approx([0., 0.], abs=1e-10)
    assert np.all(np.diff(path.distance) > 0)


@pytest.mark.parametrize('start', [
    dict(x=.1, y=0., yaw=0.), dict(x=-.01, y=.2, yaw=0.),
    dict(x=-.6, y=.8, yaw=0.), dict(x=-.6, y=0., yaw=1.5),
    dict(x=-.05, y=.3, yaw=0.), dict(x=float('nan'), y=0., yaw=0.),
])
def test_infeasible_paths_are_rejected(start):
    with pytest.raises(PlanningError):
        plan_quintic(start, dict(x=0., y=0., yaw=0.))


def test_collision_checks_body_width_and_end_clearance():
    path = plan_quintic(dict(x=-.7, y=0., yaw=0.), dict(x=0., y=0., yaw=0.))
    assert swept_collision(path, [[-.3, .2]])  # not on the centreline
    assert not swept_collision(path, [[-.3, .4], [.43, 0.]])
    assert swept_collision(path, [[.20, 0.]])


@pytest.mark.parametrize('distance,lateral,yaw,slip', [
    (.609, .262, -.100, 1.),  # Actual FAILED_ALIGNMENT case; NOT rejected at 22 cm.
    (.609, .262, -.100, .8),
    (.61, -.262, .10, .8),
    (.24, .018, .020, .8),
    (1., .08, .05, .8),
])
def test_tracker_converges_with_acceleration_limits(distance, lateral, yaw, slip):
    pose = dict(x=-distance, y=lateral, yaw=yaw)
    path = plan_quintic(pose, dict(x=0., y=0., yaw=0.))
    tracker = TrajectoryTracker(path)
    v = w = 0.
    dt = .02
    for _ in range(6000):
        if abs(pose['x'])<.015 and abs(pose['y'])<.015 and abs(pose['yaw'])<.05:
            return
        assert pose['x']<.03, pose
        command = tracker.command(pose, dt)
        assert 0<=command.speed<=.08 and abs(command.omega)<=.30+1e-12
        v, w = approach(v, command.speed, .15*dt), approach(w, command.omega, .60*dt)
        pose['x'] += v*math.cos(pose['yaw'])*dt
        pose['y'] += v*math.sin(pose['yaw'])*dt
        pose['yaw'] += slip*w*dt
    pytest.fail(f'No convergence: {pose}')


def test_reverse_approach_reaches_requested_cruise_speed_then_slows():
    start = dict(x=-.28, y=0., yaw=0.)
    path = plan_quintic(start, dict(x=0., y=0., yaw=0.))
    tracker = TrajectoryTracker(path)
    cruise = tracker.command(start, .05, max_speed=.10, max_omega=.20)
    assert cruise.speed == pytest.approx(.10)
    speed = 0.
    for _ in range(7):
        speed = approach(speed, cruise.speed, .30*.05)
    assert speed == pytest.approx(.10)
    near_goal = tracker.command(dict(x=-.08, y=0., yaw=0.), .05,
                                max_speed=.10, max_omega=.20)
    assert 0. < near_goal.speed < .10


def test_tracker_rejects_departure_from_planned_path():
    path = plan_quintic(dict(x=-.6, y=0., yaw=0.), dict(x=0., y=0., yaw=0.))
    with pytest.raises(ValueError, match='Cross-track'):
        TrajectoryTracker(path).command(dict(x=-.6, y=.3, yaw=0.), .05)


def test_final_rotation_checks_intermediate_corners():
    assert rotation_collision(dict(x=0., y=0., yaw=0.), -.3, [[.32, .20]])
    assert not rotation_collision(dict(x=0., y=0., yaw=0.), -.3, [[.6, .6]])
