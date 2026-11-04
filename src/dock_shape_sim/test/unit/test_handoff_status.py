import json
import time
from types import SimpleNamespace as NS

from std_msgs.msg import String

from dock_shape_sim.mission.node import DockMission


def mission(state='DOCKING', run='new-run'):
    published=[]
    cancelled=[]
    node=NS(_state=state,_state_since=time.monotonic(),_active_run_id=run,
            _dock_ack_wall=time.monotonic(),_last_controller_status=None,
            _controller_receipt=None,_dock_status=None,_failure_detail='',
            _saw_docking_active=False,_generation=1,_nav_active=True,
            _nav=NS(server_is_ready=lambda:True),_dock_start=NS(service_is_ready=lambda:False),
            _cmd=NS(publish=published.append),_status_pub=NS(publish=published.append))
    node.get_parameter=lambda key:NS(value=10. if key=='handoff_timeout' else '')
    node.resolve_topic_name=lambda name:'/odometry/filtered' if name=='/odom' else name
    node.get_logger=lambda:NS(info=lambda message:None)
    node._set_state=lambda state:DockMission._set_state(node,state)
    node._cancel_active=lambda:cancelled.append(True)
    node._status_data=lambda:DockMission._status_data(node)
    node._consume_controller_status=lambda data:DockMission._consume_controller_status(node,data)
    node._consume_dock_state=lambda state,allow_failure=False:DockMission._consume_dock_state(node,state,allow_failure)
    return node,published,cancelled


def test_stale_run_failure_and_success_are_ignored():
    node,_,_=mission()
    for state in ('FAILED_REFERENCE_MISMATCH','DOCKED_GEOMETRY'):
        DockMission._controller_status(node,String(data=json.dumps(dict(run_id='old-run',state=state))))
        assert node._state=='DOCKING'
    DockMission._controller_status(node,String(data=json.dumps(dict(run_id='new-run',state='FAILED_SCAN'))))
    assert node._state=='FAILED_SCAN'


def test_early_failure_before_start_response_is_not_lost():
    node,_,_=mission('STARTING_V_DOCK',None)
    DockMission._controller_status(node,String(data=json.dumps(dict(run_id='new-run',state='FAILED_TF'))))
    assert node._state=='STARTING_V_DOCK'
    response=NS(success=True,message=json.dumps(dict(run_id='new-run')))
    DockMission._dock_response(node,NS(result=lambda:response),1)
    assert node._state=='FAILED_TF'


def test_handoff_watchdog_times_out_missing_service_and_response():
    for state,expected in [('WAIT_V_DOCK','FAILED_DOCK_SERVICE_TIMEOUT'),
                           ('STARTING_V_DOCK','FAILED_DOCK_START_TIMEOUT')]:
        node,published,cancelled=mission(state)
        node._state_since-=11.
        DockMission._watchdog(node)
        assert node._state==expected and cancelled and published


def test_controller_silence_after_accepted_start_is_reported():
    node,_,cancelled=mission()
    node._state_since-=11.
    DockMission._watchdog(node)
    assert node._state=='FAILED_CONTROLLER_STATE_TIMEOUT' and cancelled


def test_legacy_queued_failure_waits_for_new_active_state():
    node,_,_=mission(run=None)
    DockMission._dock_state(node,String(data='FAILED_REFERENCE_MISMATCH'))
    assert node._state=='DOCKING'
    DockMission._dock_state(node,String(data='SEARCH'))
    DockMission._dock_state(node,String(data='FAILED_SCAN'))
    assert node._state=='FAILED_SCAN'


def test_late_response_from_old_run_does_not_cancel_new_run():
    node,_,_=mission()
    requests=[]
    node._dock_cancel=NS(service_is_ready=lambda:True,call_async=requests.append)
    response=NS(success=True,message=json.dumps(dict(run_id='old-run')))
    DockMission._dock_response(node,NS(result=lambda:response),0)
    assert not requests and node._state=='DOCKING'
