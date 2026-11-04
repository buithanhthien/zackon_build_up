"""Exercise the actual Qt dialog with a fake transport; no ROS graph or motion."""
import os
from pathlib import Path
import sys
import time
from types import SimpleNamespace as NS

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
from PyQt6.QtCore import QObject, pyqtSignal
from PyQt6.QtWidgets import QApplication
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[4]/'robot_ui'))
import docking_layout


class FakeBridge(QObject):
    updated = pyqtSignal(str, object)
    replied = pyqtSignal(str, bool, str)
    error = pyqtSignal(str)

    def __init__(self, parent):
        super().__init__(parent)
        self.latest, self.received, self.pending = {}, {}, {}
        self.requests, self.emergencies, self.charges = [], [], []
        self.permit = False
        self.clients = {'start': NS(service_is_ready=lambda: True)}
        self.charge = NS(publish=self.charges.append, get_subscription_count=lambda: 1)

    def fresh(self, key, timeout=1.):
        return key in self.received and time.monotonic()-self.received[key] < timeout

    def request(self, key):
        self.requests.append(key)
        return True

    def set_permit(self, value):
        self.permit = value

    def emergency_stop(self, value):
        self.emergencies.append(value)

    def close(self):
        pass


@pytest.fixture(scope='module')
def app():
    return QApplication.instance() or QApplication([])


@pytest.fixture
def dialog(app, monkeypatch, tmp_path):
    monkeypatch.setattr(docking_layout, 'DockingBridge', FakeBridge)
    window = docking_layout.DockingUI()
    window.timer.stop()
    window._map_file = str(tmp_path/'room.yaml')
    window._odom_topic = '/odometry/filtered'
    bridge = window.bridge
    bridge.latest = {
        'mission': {'state': 'IDLE', 'cancel_pending': False, 'profile_file': window._profile(),
                    'odom_topic': window._odom_topic},
        'controller': {'state': 'IDLE', 'profile_file': window._profile(), 'odom_topic': window._odom_topic},
        'nav2_map': window._map_file, 'nav2_odom': window._odom_topic,
        'arbiter': {'owner': '', 'healthy': True, 'emergency_stop': False, 'navigation_busy': False},
        'odom': {'linear': 0., 'angular': 0.}, 'charging': False,
    }
    bridge.received = {key: time.monotonic() for key in bridge.latest}
    yield window
    window._active = window._start_pending = window._stopping = False
    bridge.latest['mission']['state'] = bridge.latest['controller']['state'] = 'IDLE'
    window.close()


def test_cancel_before_delayed_start_prevents_start_request(dialog):
    dialog.execute_docking()
    assert dialog.bridge.permit
    dialog.cancel_docking()
    dialog._send_start()
    assert not dialog.bridge.permit
    assert 'start' not in dialog.bridge.requests
    assert set(dialog.bridge.requests) == {'cancel', 'controller_cancel'}


def test_late_successful_start_reply_is_cancelled(dialog):
    dialog._stopping = True
    dialog._replied('start', True, 'Accepted')
    assert dialog.bridge.requests == ['cancel', 'controller_cancel']
    assert not dialog.bridge.permit


def test_navigation_busy_or_wrong_profile_blocks_start(dialog):
    dialog.bridge.latest['arbiter']['navigation_busy'] = True
    dialog.execute_docking()
    assert not dialog.busy
    dialog.bridge.latest['arbiter']['navigation_busy'] = False
    dialog.bridge.latest['controller']['profile_file'] = '/wrong.docking.yaml'
    dialog.execute_docking()
    assert not dialog.busy


def test_close_and_escape_wait_for_confirmed_stop(dialog):
    dialog.show()
    dialog._active = True
    assert dialog.close() is False
    assert dialog._stopping and not dialog._cleaned
    dialog.reject()
    assert not dialog._cleaned


def test_stop_confirmation_requires_fresh_ack_and_no_pending_nav(dialog):
    dialog._stop_time = time.monotonic()-.01
    bridge = dialog.bridge
    bridge.received = {key: time.monotonic() for key in bridge.latest}
    assert dialog._stop_confirmed()
    bridge.latest['mission']['cancel_pending'] = True
    assert not dialog._stop_confirmed()
    bridge.latest['mission']['cancel_pending'] = False
    bridge.received['controller'] -= 2.
    assert not dialog._stop_confirmed()


def test_motion_invalidates_charge_permission_until_new_docking(dialog):
    dialog.bridge.latest['controller']['state'] = 'DOCKED_GEOMETRY'
    dialog._docked_confirmed = True
    dialog.send_charge_command()
    assert len(dialog.bridge.charges) == 1
    dialog._updated('odom', {'linear': .05, 'angular': 0.})
    dialog.send_charge_command()
    assert len(dialog.bridge.charges) == 1


def test_emergency_stop_never_restarts_mission(dialog):
    dialog._emergency_stop()
    assert dialog.bridge.emergencies == [True]
    dialog._reset_stop()
    assert dialog.bridge.emergencies == [True, False]
    assert not dialog.bridge.permit and not dialog.bridge.requests


def test_service_timeout_keeps_start_pending_until_its_late_reply(monkeypatch):
    from docking_bridge import DockingBridge
    emitted = []
    future = NS(result=lambda: NS(success=True, message='late accepted'))
    node = NS(executor=NS(spin_once=lambda **kwargs: None), permit=False,
        _check_navigation=lambda: None,
        heartbeat=NS(publish=lambda msg: None), _last_heartbeat=0.,
        pending={'start': (future, time.monotonic()-9.)},
        replied=NS(emit=lambda *args: emitted.append(args)), error=NS(emit=lambda text: None))
    DockingBridge._tick(node)
    DockingBridge._tick(node)
    assert 'start' in node.pending and len(emitted) == 1
    DockingBridge._done(node, 'start', future)
    assert 'start' not in node.pending
    assert emitted[-1] == ('start', True, 'late accepted')


def test_main_screen_stop_also_stops_docking():
    import ast
    source = Path(docking_layout.__file__).with_name('startup_layout.py').read_text()
    cls = next(item for item in ast.parse(source).body if isinstance(item, ast.ClassDef) and item.name=='RobotUI')
    names = {'_docking_busy', 'cancel_voice_navigation'}
    module = ast.Module(body=[item for item in cls.body if isinstance(item, ast.FunctionDef) and item.name in names], type_ignores=[])
    functions = {}
    exec(compile(module, '<startup docking controls>', 'exec'), functions)
    stopped = []
    node = NS(_docking_dialog=NS(_cleaned=False, busy=True, _emergency_stop=lambda: stopped.append(True)),
        _navigation_generation=0, _voice_nav_queue=[], _nav_goal_handle=None, log=lambda text: None)
    node._docking_busy = lambda: functions['_docking_busy'](node)
    functions['cancel_voice_navigation'](node)
    assert stopped == [True]


def test_opening_dialog_uses_live_nav2_map_without_manual_controls(dialog):
    from PyQt6.QtWidgets import QPushButton
    dialog._map_file = dialog._odom_topic = None
    dialog._sync_navigation()
    assert dialog._map_file == dialog.bridge.latest['nav2_map']
    assert dialog._odom_topic == '/odometry/filtered'
    assert dialog._matching_backend()
    labels = [button.text() for button in dialog.findChildren(QPushButton)]
    assert 'Chọn map' not in labels and 'Khởi động backend' not in labels


def test_waits_for_nav2_before_launching_and_launches_only_once(dialog, monkeypatch):
    started = []
    monkeypatch.setattr(docking_layout.subprocess, 'Popen',
                        lambda *args, **kwargs: started.append(args) or NS(poll=lambda: None))
    monkeypatch.setattr(docking_layout, 'SOURCE_PATH', str(Path(dialog._map_file).parent))
    Path(dialog._map_file).write_text('image: room.pgm\n')
    dialog.bridge.clients['start'].service_is_ready = lambda: False
    dialog.bridge.received['nav2_map'] -= 4.
    dialog._sync_navigation()
    assert not started
    dialog.bridge.received['nav2_map'] = time.monotonic()
    dialog._sync_navigation()
    dialog._sync_navigation()
    assert len(started) == 1
    command = started[0][0][2]
    assert 'map_file:='+dialog._map_file in command
    assert 'odom_topic:=/odometry/filtered' in command
    # Drop the fake process; fixture cleanup must never signal a real process.
    dialog._process = None


def test_stale_nav2_or_wrong_odometry_blocks_teaching_and_start(dialog):
    dialog.bridge.received['nav2_map'] -= 4.
    dialog._teach('pre')
    dialog.execute_docking()
    assert not dialog.bridge.requests and not dialog.bridge.permit
    dialog.bridge.received['nav2_map'] = time.monotonic()
    dialog.bridge.latest['controller']['odom_topic'] = '/odom'
    assert not dialog._matching_backend()
    dialog._teach('terminal')
    dialog.execute_docking()
    assert not dialog.bridge.requests and not dialog.bridge.permit


def test_two_teaching_buttons_capture_without_starting_motion(dialog):
    dialog.pre_btn.click()
    dialog.terminal_btn.click()
    assert dialog.bridge.requests == ['pre', 'terminal']
    assert not dialog.bridge.permit and not dialog.busy
    assert not dialog.bridge.charges


def test_changing_map_during_docking_cancels_before_switching(dialog):
    old_map = dialog._map_file
    dialog._active = True
    dialog.bridge.latest['nav2_map'] = '/new/room.yaml'
    dialog._sync_navigation()
    assert dialog._stopping and dialog._map_file == old_map
    assert not dialog.bridge.permit
    assert dialog.bridge.requests == ['cancel', 'controller_cancel']
    assert not dialog._matching_backend()
    dialog._active = dialog._start_pending = dialog._stopping = False
    dialog._sync_navigation()
    assert dialog._map_file == '/new/room.yaml'


def test_teaching_waits_for_pending_service_or_motion_to_stop(dialog):
    dialog.bridge.pending['pre'] = object()
    dialog._teach('terminal')
    dialog.execute_docking()
    assert not dialog.bridge.requests and not dialog.bridge.permit
    dialog.bridge.pending.clear()
    dialog.bridge.latest['odom']['linear'] = .1
    dialog._teach('pre')
    assert not dialog.bridge.requests


def bridge_for_navigation_result():
    from docking_bridge import DockingBridge
    subscriptions = []
    destroyed = []
    node = NS(_closed=False, _nav_queries={}, latest={'odom': {}}, received={'odom': time.monotonic()},
              _odom_topic='/old', _odom_subscription='old-subscription', _odom=lambda msg: None)
    node.node = NS(resolve_topic_name=lambda value: '/'+value.lstrip('/'),
        create_subscription=lambda *args: subscriptions.append(args) or 'new-subscription',
        destroy_subscription=destroyed.append)
    def update(key, value):
        node.latest[key], node.received[key] = value, time.monotonic()
    node._update = update
    return DockingBridge, node, subscriptions, destroyed


def test_bridge_uses_nav2_odom_parameter_and_discards_old_velocity():
    cls, bridge, subscriptions, destroyed = bridge_for_navigation_result()
    future = NS(result=lambda: NS(values=[NS(string_value='odometry/filtered')]))
    bridge._nav_queries['nav2_odom'] = (future, time.monotonic())
    cls._navigation_result(bridge, 'nav2_odom', future)
    assert bridge.latest['nav2_odom'] == '/odometry/filtered'
    assert subscriptions[0][1] == '/odometry/filtered' and destroyed == ['old-subscription']
    assert 'odom' not in bridge.received and 'odom' not in bridge.latest


def test_bridge_resolves_live_map_and_ignores_late_parameter_reply(tmp_path):
    cls, bridge, _, _ = bridge_for_navigation_result()
    path = tmp_path/'live.yaml'
    path.write_text('image: live.pgm\n')
    future = NS(result=lambda: NS(values=[NS(string_value=str(path))]))
    bridge._nav_queries['nav2_map'] = (future, time.monotonic())
    cls._navigation_result(bridge, 'nav2_map', future)
    assert bridge.latest['nav2_map'] == str(path.resolve())
    bridge._nav_queries['nav2_map'] = (object(), time.monotonic())
    cls._navigation_result(bridge, 'nav2_map', future)
    assert 'nav2_map' in bridge._nav_queries


def test_empty_map_reply_invalidates_previous_map_instead_of_using_config():
    cls, bridge, _, _ = bridge_for_navigation_result()
    bridge.received['nav2_map'] = time.monotonic()
    future = NS(result=lambda: NS(values=[NS(string_value='')]))
    bridge._nav_queries['nav2_map'] = (future, time.monotonic())
    cls._navigation_result(bridge, 'nav2_map', future)
    assert 'nav2_map' not in bridge.received
    assert 'navigation_error' in bridge.latest
