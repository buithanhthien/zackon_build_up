from types import SimpleNamespace as NS

from dock_shape_sim.mission.node import DockMission


def test_late_accepted_nav_goal_is_cancelled_after_mission_cancel():
    cancelled = []
    goal = NS(accepted=True, cancel_goal_async=lambda: cancelled.append(True))
    node = NS(_generation=2, _state='IDLE')
    DockMission._goal_response(node, NS(result=lambda: goal), 1)
    assert cancelled == [True]


def test_late_docking_start_response_cannot_restart_cancelled_motion():
    requests = []
    service = NS(service_is_ready=lambda: True, call_async=requests.append)
    node = NS(_generation=2, _state='IDLE', _dock_cancel=service)
    DockMission._dock_response(node, None, 1)
    assert len(requests) == 1


def test_mission_cancel_invalidates_callbacks_and_cancels_both_controllers():
    cancelled, requests = [], []
    goal = NS(cancel_goal_async=lambda: cancelled.append(True))
    service = NS(service_is_ready=lambda: True, call_async=requests.append)
    node = NS(_generation=7, _goal_handle=goal, _dock_cancel=service, _steps=['STAGING'])
    node._cancel_active = lambda: DockMission._cancel_active(node)
    node._set_state = lambda state: setattr(node, '_state', state)
    response = DockMission._cancel(node, None, NS())
    assert response.success and node._state == 'IDLE'
    assert node._generation == 8 and node._goal_handle is None and node._steps == []
    assert cancelled == [True] and len(requests) == 1
