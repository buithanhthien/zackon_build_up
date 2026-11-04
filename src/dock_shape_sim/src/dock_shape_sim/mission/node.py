"""Return-to-dock mission: Nav2 staging first, LiDAR V control second."""

from math import sin, cos
from pathlib import Path
import json
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from lifecycle_msgs.srv import GetState
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from std_srvs.srv import Trigger
import yaml

from .profile import load_profile
from ..safety import DockingSafety


class DockMission(Node):
    def __init__(self):
        super().__init__('dock_mission')
        self.safety = DockingSafety(self)
        self._pending_nav_goals = 0
        self._cancelling_goals = {}
        self.declare_parameter('staging_x', 0.0)
        self.declare_parameter('staging_y', 0.0)
        self.declare_parameter('staging_yaw', 0.0)
        self.declare_parameter('demo_route', False)
        self.declare_parameter('demo_x', -1.0)
        self.declare_parameter('demo_y', 0.0)
        self.declare_parameter('max_mission_time', 600.0)
        self.declare_parameter('auto_start', False)
        self.declare_parameter('profile_file', '')
        self.declare_parameter('handoff_timeout', 10.0)
        self._nav_state = self.create_client(GetState, '/bt_navigator/get_state')
        self._nav_active = False
        self._nav_state_pending = False
        self._nav = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self._dock_start = self.create_client(Trigger, '/v_dock/start')
        self._dock_cancel = self.create_client(Trigger, '/v_dock/cancel')
        self._state_pub = self.create_publisher(String, '/dock_mission/state', 10)
        self._status_pub = self.create_publisher(String, '/dock_mission/status', QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(String, '/v_dock/state', self._dock_state, 10)
        self.create_subscription(String, '/v_dock/status', self._controller_status, QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_service(Trigger, '/dock_mission/start', self._start)
        self.create_service(Trigger, '/dock_mission/cancel', self._cancel)
        self.create_service(Trigger, '/dock_mission/get_status', self._get_status)
        self.create_timer(0.2, self._tick)
        self._state = 'IDLE'
        self._steps = []
        self._goal_handle = None
        self._dock_status = None
        self._start_ns = None
        self._auto_started = False
        self._generation = 0
        self._saw_docking_active = False
        self._state_since = time.monotonic()
        self._active_run_id = None
        self._last_controller_status = None
        self._controller_receipt = None
        self._dock_ack_wall = None
        self._failure_detail = ''
        self.create_timer(.2, self._watchdog, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def _status_data(self):
        reason = {
            'WAIT_NAV2':'Chờ action server và lifecycle bt_navigator active.',
            'NAVIGATING_STAGING':'Chưa có kết quả NavigateToPose SUCCEEDED.',
            'WAIT_V_DOCK':'Nav2 đã SUCCEEDED; chờ /v_dock/start sẵn sàng.',
            'STARTING_V_DOCK':'Đã gửi /v_dock/start; chờ phản hồi.',
            'DOCKING':'Controller đã nhận yêu cầu; theo dõi nhận dạng và điều khiển.',
            'IDLE':'Chưa chạy hoặc đã hủy nhiệm vụ.',
            'DOCKED_GEOMETRY':'Đã đạt hình học docking.'}.get(self._state, self._failure_detail)
        return dict(state=self._state, reason=reason, detail=self._failure_detail,
                    state_age=time.monotonic()-self._state_since,
                    nav_server_ready=self._nav.server_is_ready(), nav_active=self._nav_active,
                    dock_service_ready=self._dock_start.service_is_ready(),
                    controller_state=self._dock_status, run_id=self._active_run_id,
                    profile_file=self.get_parameter('profile_file').value,
                    odom_topic=self.resolve_topic_name('/odom'),
                    safety_error=(self.safety.error() if getattr(self, 'safety', None) else None),
                    map_verified=bool(getattr(self, 'safety', None) and self.safety.expected_map
                        and not self.safety.map_error and self.safety.map_stamp is not None
                        and time.monotonic()-self.safety.map_stamp < 3.),
                    source_file=__file__, cancel_pending=bool(
                        getattr(self, '_pending_nav_goals', 0) or getattr(self, '_cancelling_goals', {})))

    def _get_status(self, _request, response):
        response.success, response.message = True, json.dumps(self._status_data())
        return response

    def _watchdog(self):
        now = time.monotonic()
        timeout = self.get_parameter('handoff_timeout').value
        failures = {'WAIT_V_DOCK':'FAILED_DOCK_SERVICE_TIMEOUT',
                    'STARTING_V_DOCK':'FAILED_DOCK_START_TIMEOUT'}
        code = failures.get(self._state) if now-self._state_since > timeout else None
        if self._state == 'DOCKING':
            if self._active_run_id:
                fresh = (self._last_controller_status and
                         self._last_controller_status.get('run_id') == self._active_run_id and
                         self._controller_receipt is not None and now-self._controller_receipt < timeout)
                if not fresh and now-self._state_since > timeout:
                    code = 'FAILED_CONTROLLER_STATE_TIMEOUT'
            elif self._dock_ack_wall is not None and now-self._dock_ack_wall > .5:
                # Legacy String-only publishers have no request id. Wait out
                # queued failure messages before accepting a repeated failure.
                self._consume_dock_state(self._dock_status, allow_failure=True)
        if code:
            self._failure_detail = 'Handoff timeout; kiểm tra trạng thái/service controller.'
            self._cancel_active()
            self._set_state(code)
        self._status_pub.publish(String(data=json.dumps(self._status_data())))

    def _set_state(self, state):
        if self._state != state:
            self._state = state
            self._state_since = time.monotonic()
            self.get_logger().info('Dock mission: %s' % state)
            if state in ('IDLE', 'DOCKED_GEOMETRY') or state.startswith('FAILED'):
                self._cmd.publish(Twist())

    def _begin(self):
        safety_error = self.safety.error()
        if safety_error:
            raise ValueError(safety_error)
        filename = self.get_parameter('profile_file').value
        if self.safety.required and not filename:
            raise ValueError('Thiếu profile phần cứng.')
        if self._pending_nav_goals or self._cancelling_goals:
            raise ValueError('Chưa xác nhận Nav2 đã hủy; hãy chờ kết quả dừng.')
        self._staging = (
            self.get_parameter('staging_x').value,
            self.get_parameter('staging_y').value,
            self.get_parameter('staging_yaw').value)
        if filename:
            data = yaml.safe_load(Path(filename).read_text())
            if (self.safety.required and self.safety.expected_map
                    and Path(data['map']).resolve() != Path(self.safety.expected_map).resolve()):
                raise ValueError('Profile không thuộc map đang chọn.')
            data = load_profile(data['map'])
            if self.safety.required and not data.get('hardware_calibration'):
                raise ValueError('Hãy dạy hai mẫu trên robot thật cho map này.')
            staging = data['pre_docking']
            if 'marker_in_odom' not in staging:
                raise ValueError('Điểm chờ mới là gợi ý, chưa LƯU PRE-DOCKING bằng LiDAR.')
            if 'docked_reference' not in data:
                raise ValueError('Chưa lấy mẫu LiDAR tại vị trí đã sát dock.')
            self._staging = (staging['x'], staging['y'], staging['yaw'])
        self._generation += 1
        self._nav_active = False
        self._steps = []
        if self.get_parameter('demo_route').value:
            self._steps.append((
                self.get_parameter('demo_x').value,
                self.get_parameter('demo_y').value, 0.0, 'WAYPOINT'))
        self._steps.append((*self._staging, 'STAGING'))
        self._start_ns = self.get_clock().now().nanoseconds
        self._dock_status = None
        self._saw_docking_active = False
        self._active_run_id = None
        self._dock_ack_wall = None
        self._failure_detail = ''
        self._set_state('WAIT_NAV2')

    def _start(self, _request, response):
        if self._state not in ('IDLE', 'DOCKED_GEOMETRY') and not self._state.startswith('FAILED'):
            response.success = False
            response.message = 'Mission already active'
        else:
            try:
                self._begin()
                response.success = True
                response.message = 'Return-to-dock mission started'
            except (OSError, KeyError, ValueError, TypeError) as exc:
                response.success = False
                response.message = str(exc)
        return response

    def _cancel(self, _request, response):
        self._cancel_active()
        self._steps = []
        self._set_state('IDLE')
        response.success = True
        response.message = 'Mission cancelled'
        return response

    def _cancel_active(self):
        self._generation += 1
        if self._goal_handle is not None:
            getattr(self, '_cancel_goal', lambda handle: handle.cancel_goal_async())(self._goal_handle)
            self._goal_handle = None
        if self._dock_cancel.service_is_ready():
            self._dock_cancel.call_async(Trigger.Request())

    def _cancel_goal(self, handle):
        key = id(handle)
        self._cancelling_goals[key] = handle
        try:
            handle.cancel_goal_async()
            handle.get_result_async().add_done_callback(
                lambda future: self._cancel_result(future, key))
        except Exception as exc:
            self._failure_detail = 'Chưa xác nhận Nav2 dừng: '+str(exc)

    def _cancel_result(self, future, key):
        try:
            if future.result().status in (4, 5, 6):
                self._cancelling_goals.pop(key, None)
        except Exception as exc:
            self._failure_detail = 'Chưa xác nhận Nav2 dừng: '+str(exc)

    def _dock_state(self, message):
        self._dock_status = message.data
        if getattr(self, '_active_run_id', None) is None:
            self._consume_dock_state(message.data)

    def _consume_dock_state(self, state, allow_failure=False):
        if self._state != 'DOCKING' or not state:
            return
        if state in ('SEARCH', 'APPROACH', 'WAIT_DETECTION'):
            self._saw_docking_active = True
        if state == 'DOCKED_GEOMETRY' and self._saw_docking_active:
            self._set_state('DOCKED_GEOMETRY')
        elif state.startswith('FAILED') and (self._saw_docking_active or allow_failure):
            self._set_state(state)

    def _controller_status(self, message):
        try:
            data = json.loads(message.data)
        except (ValueError, TypeError):
            return
        if not isinstance(data, dict):
            return
        self._last_controller_status = data
        self._controller_receipt = time.monotonic()
        self._consume_controller_status(data)

    def _consume_controller_status(self, data):
        if (not self._active_run_id or
                data.get('run_id') != self._active_run_id):
            return
        state = data.get('state', '')
        # The legacy String state can arrive before the structured status.
        if self._state.startswith('FAILED') and state == self._state:
            self._failure_detail = data.get('detail', self._failure_detail)
            return
        if self._state != 'DOCKING':
            return
        if state.startswith('FAILED') or state == 'DOCKED_GEOMETRY':
            self._failure_detail = data.get('detail', '')
            self._set_state(state)

    def _send_next_goal(self):
        x, y, yaw, kind = self._steps[0]
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = sin(yaw / 2.0)
        goal.pose.pose.orientation.w = cos(yaw / 2.0)
        self._set_state('NAVIGATING_' + kind)
        generation = self._generation
        self._pending_nav_goals += 1
        self._nav.send_goal_async(goal).add_done_callback(
            lambda future: self._goal_response(future, generation))

    def _goal_response(self, future, generation):
        self._pending_nav_goals = max(0, getattr(self, '_pending_nav_goals', 0)-1)
        if generation != self._generation or not self._state.startswith('NAVIGATING_'):
            try:
                handle = future.result()
                if handle.accepted:
                    getattr(self, '_cancel_goal', lambda goal: goal.cancel_goal_async())(handle)
            except Exception:
                pass
            return
        try:
            self._goal_handle = future.result()
            if not self._goal_handle.accepted:
                self._failure_detail = 'NavigateToPose goal bị từ chối.'
                self._set_state('FAILED_NAV2')
                return
            self._goal_handle.get_result_async().add_done_callback(
                lambda result: self._goal_result(result, generation))
        except Exception as exc:
            self.get_logger().error('Nav2 goal error: %s' % exc)
            self._failure_detail = str(exc)
            self._set_state('FAILED_NAV2')

    def _goal_result(self, future, generation):
        if generation != self._generation or not self._state.startswith('NAVIGATING_'):
            return
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().error('Nav2 result error: %s' % exc)
            self._failure_detail = str(exc)
            self._set_state('FAILED_NAV2')
            return
        self._goal_handle = None
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error('Nav2 goal failed, status %d' % result.status)
            self._failure_detail = f'Nav2 action result status={result.status}; cần SUCCEEDED=4.'
            self._set_state('FAILED_NAV2')
            return
        self._steps.pop(0)
        self._set_state('WAIT_NAV2' if self._steps else 'WAIT_V_DOCK')

    def _dock_response(self, future, generation):
        if self._state != 'STARTING_V_DOCK' or generation != self._generation:
            try:
                old_run = json.loads(future.result().message).get('run_id')
            except (AttributeError, ValueError, TypeError):
                old_run = None
            current_run = getattr(self, '_active_run_id', None)
            if old_run and current_run and old_run != current_run:
                return  # Never cancel a newer run because an older reply arrived late.
            if self._dock_cancel.service_is_ready():
                self._dock_cancel.call_async(Trigger.Request())
            return
        try:
            response = future.result()
            if response.success:
                try:
                    self._active_run_id = json.loads(response.message).get('run_id')
                except (ValueError, TypeError, AttributeError):
                    self._active_run_id = None
                self._dock_ack_wall = time.monotonic()
                self._set_state('DOCKING')
                if self._active_run_id and self._last_controller_status:
                    self._consume_controller_status(self._last_controller_status)
                return
            self.get_logger().error(response.message)
            self._failure_detail = response.message
        except Exception as exc:
            self.get_logger().error('V dock service error: %s' % exc)
            self._failure_detail = str(exc)
        self._set_state('FAILED_DOCK_START')

    def _tick(self):
        if self.get_parameter('auto_start').value and not self._auto_started:
            self._auto_started = True
            try:
                self._begin()
            except (OSError, KeyError, ValueError, TypeError) as exc:
                self.get_logger().error(str(exc))
                self._set_state('FAILED_PROFILE')
        self._state_pub.publish(String(data=self._state))
        if self._state in ('IDLE', 'DOCKED_GEOMETRY') or self._state.startswith('FAILED'):
            return
        safety = getattr(self, 'safety', None)
        if safety is not None and safety.error():
            self._failure_detail = safety.error()
            self._cancel_active()
            self._steps = []
            self._set_state('FAILED_SAFETY_PERMIT')
            return
        if (self.get_clock().now().nanoseconds - self._start_ns) / 1e9 > \
                self.get_parameter('max_mission_time').value:
            self.get_logger().error('Mission timeout')
            self._cancel_active()
            self._set_state('FAILED_MISSION_TIMEOUT')
            return
        if self._state == 'WAIT_NAV2' and self._nav.server_is_ready():
            if self._nav_active:
                self._send_next_goal()
            elif self._nav_state.service_is_ready() and not self._nav_state_pending:
                self._nav_state_pending = True
                self._nav_state.call_async(GetState.Request()).add_done_callback(self._ready_response)
        elif self._state == 'WAIT_V_DOCK' and self._dock_start.service_is_ready():
            self._set_state('STARTING_V_DOCK')
            generation = self._generation
            self._dock_start.call_async(Trigger.Request()).add_done_callback(
                lambda future: self._dock_response(future, generation))

    def _ready_response(self, future):
        self._nav_state_pending = False
        try:
            self._nav_active = future.result().current_state.id == 3
        except Exception:
            self._nav_active = False


def main():
    rclpy.init()
    node = DockMission()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            if rclpy.ok():
                node._cmd.publish(Twist())
        except RuntimeError:
            pass  # ROS context may shut down between ok() and publish().
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
