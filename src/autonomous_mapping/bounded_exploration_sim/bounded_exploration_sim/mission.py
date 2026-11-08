"""CLI-controlled exploration mission with a frozen 10 x 10 m ROI.

The frontier package selects goals using /bounded_exploration/map. Nav2 sees
the unmodified SLAM map and a keepout mask on both costmaps. This node owns
start/stop, return-to-start and saving; no AI or robot UI is involved.
"""

import json
import math
import os
from pathlib import Path
import subprocess
import time
import uuid

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from action_msgs.msg import GoalStatus, GoalStatusArray
from frontier_exploration_ros2.srv import ControlExploration
from geometry_msgs.msg import Point, PoseStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import CostmapFilterInfo
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool, Empty, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker

from .geometry import (bounded_map_data, inside_rectangle, keepout_mask,
                       observation_grid, occupancy_at, rectangle_corners)


LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


class ExplorationMission(Node):
    def __init__(self):
        super().__init__('bounded_exploration_mission')
        self.declare_parameter('half_width_m', 5.0)
        self.declare_parameter('half_length_m', 5.0)
        # sqrt(0.30^2 + 0.25^2) = 0.391 m for the configured robot footprint.
        self.declare_parameter('footprint_margin_m', 0.50)
        self.declare_parameter('map_output_dir', '/tmp/bounded_exploration_maps')
        self.declare_parameter('map_stale_seconds', 8.0)
        self.declare_parameter('mission_timeout_seconds', 1800.0)
        self.declare_parameter('idle_frontier_timeout_seconds', 45.0)
        self.declare_parameter('return_timeout_seconds', 180.0)
        self.declare_parameter('observation_points_enabled', True)
        self.declare_parameter('observation_spacing_m', 2.0)
        self.declare_parameter('observation_inset_m', 0.60)
        self.declare_parameter('observation_visited_radius_m', 1.0)
        self.declare_parameter('observation_max_points', 6)
        self.declare_parameter('observation_goal_timeout_seconds', 60.0)
        self.half_x = float(self.get_parameter('half_length_m').value)
        self.half_y = float(self.get_parameter('half_width_m').value)
        self.margin = float(self.get_parameter('footprint_margin_m').value)
        if not all(math.isfinite(v) for v in (self.half_x, self.half_y, self.margin)) or (
                self.margin < 0.391 or min(self.half_x, self.half_y) <= self.margin):
            raise ValueError('Rectangle or footprint margin is invalid')
        observation_radius = float(self.get_parameter('observation_visited_radius_m').value)
        observation_timeout = float(self.get_parameter('observation_goal_timeout_seconds').value)
        if (not all(math.isfinite(v) and v > 0 for v in (
                observation_radius, observation_timeout)) or
                int(self.get_parameter('observation_max_points').value) < 0):
            raise ValueError('Observation mission parameters are invalid')

        self.phase = 'idle'
        self.detail = ''
        self.mission_id = None
        self.home = None
        self.center = None
        self.map = None
        self.map_at = 0.0
        self.global_costmap_at = 0.0
        self.local_costmap_at = 0.0
        self.global_costmap = None
        self.mask = None
        self.info = None
        self.start_future = None
        self.stop_future = None
        self.stop_last_at = 0.0
        self.stop_reason = ''
        self.partial_reason = ''
        self.return_goal = None
        self.return_attempts = 0
        self.return_started_at = 0.0
        self.return_retry_at = 0.0
        self.return_error = ''
        self.saver = None
        self.save_base = None
        self.save_started_at = 0.0
        self.return_succeeded = False
        self.max_local_x = 0.0
        self.max_local_y = 0.0
        self.started_at = 0.0
        self.last_goal_at = 0.0
        self.nav_goal_active = False
        self.observed_path = []
        self.observation_remaining = []
        self.observation_goal = None
        self.observation_target = None
        self.observation_started_at = 0.0
        self.observation_generation = 0
        self.observation_done = 0
        self.observation_failed = 0
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)

        self.map_pub = self.create_publisher(OccupancyGrid, '/bounded_exploration/map', LATCHED)
        self.mask_pub = self.create_publisher(OccupancyGrid, '/bounded_exploration/keepout_mask', LATCHED)
        self.info_pub = self.create_publisher(CostmapFilterInfo, '/bounded_exploration/filter_info', LATCHED)
        self.marker_pub = self.create_publisher(Marker, '/bounded_exploration/boundary', LATCHED)
        self.status_pub = self.create_publisher(String, '/bounded_exploration/status', LATCHED)
        self.estop_pub = self.create_publisher(Bool, '/velocity_arbiter/emergency_stop', 1)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, LATCHED)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap',
                                 lambda msg: self.note_costmap('global', msg), 10)
        self.create_subscription(OccupancyGrid, '/local_costmap/costmap',
                                 lambda msg: self.note_costmap('local', msg), 10)
        self.create_subscription(Empty, '/exploration_complete', self.on_exploration_complete, LATCHED)
        self.create_subscription(PoseStamped, '/explore/selected_frontier',
                                 self.on_selected_goal, 10)
        self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status',
                                 self.on_nav_status, LATCHED)
        self.create_service(Trigger, '/bounded_exploration/start', self.on_start)
        self.create_service(Trigger, '/bounded_exploration/cancel', self.on_cancel)
        self.create_service(Trigger, '/bounded_exploration/get_status', self.on_get_status)
        self.control_client = self.create_client(ControlExploration, '/control_exploration')
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.create_timer(0.25, self.tick)
        self.report('idle')

    def report(self, phase, detail=''):
        self.phase, self.detail = phase, detail
        status = String()
        status.data = json.dumps({'mission_id': self.mission_id, 'state': phase,
                                  'detail': detail, 'returned_home': self.return_succeeded,
                                  'max_local_x_m': round(self.max_local_x, 3),
                                  'max_local_y_m': round(self.max_local_y, 3),
                                  'observation_points_done': self.observation_done,
                                  'observation_points_failed': self.observation_failed,
                                  'map_path': str(self.save_base) if self.save_base else ''})
        self.status_pub.publish(status)
        self.get_logger().info(status.data)

    def on_get_status(self, _request, response):
        response.success = True
        response.message = json.dumps({'mission_id': self.mission_id, 'state': self.phase,
                                       'detail': self.detail, 'returned_home': self.return_succeeded,
                                       'max_local_x_m': round(self.max_local_x, 3),
                                       'max_local_y_m': round(self.max_local_y, 3),
                                       'observation_points_done': self.observation_done,
                                       'observation_points_failed': self.observation_failed,
                                       'map_path': str(self.save_base) if self.save_base else ''})
        return response

    def current_pose(self):
        try:
            transform = self.tf.lookup_transform('map', 'base_link', Time())
        except TransformException:
            return None
        stamp = transform.header.stamp
        age = (self.get_clock().now().nanoseconds -
               (stamp.sec * 1_000_000_000 + stamp.nanosec)) / 1e9
        if age < -0.2 or age > 1.0:
            return None
        p = PoseStamped()
        p.header = transform.header
        p.pose.position.x = transform.transform.translation.x
        p.pose.position.y = transform.transform.translation.y
        p.pose.orientation = transform.transform.rotation
        q = p.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        if not all(math.isfinite(v) for v in (p.pose.position.x, p.pose.position.y, yaw)):
            return None
        return p, yaw

    def on_start(self, _request, response):
        if self.phase not in ('idle', 'completed', 'return_failed', 'failed', 'cancelled'):
            response.message = 'A mission is already active'
            return response
        if self.map is None or time.monotonic() - self.map_at > float(
                self.get_parameter('map_stale_seconds').value):
            response.message = 'SLAM map is not fresh'
            return response
        pose = self.current_pose()
        if pose is None:
            response.message = 'map -> base_link TF is unavailable or stale'
            return response
        if not self.control_client.service_is_ready() or not self.nav_client.server_is_ready():
            response.message = 'Explorer or Nav2 is not ready'
            return response
        self.mission_id = uuid.uuid4().hex
        self.home, yaw = pose
        self.center = (self.home.pose.position.x, self.home.pose.position.y, yaw)
        self.return_succeeded = False
        self.max_local_x = 0.0
        self.max_local_y = 0.0
        self.save_base = None
        self.stop_reason = ''
        self.partial_reason = ''
        self.start_future = None
        self.stop_future = None
        self.started_at = time.monotonic()
        self.last_goal_at = self.started_at
        self.nav_goal_active = False
        self.observed_path = [(self.home.pose.position.x, self.home.pose.position.y)]
        self.observation_remaining = []
        self.observation_goal = None
        self.observation_target = None
        self.observation_generation += 1
        self.observation_done = 0
        self.observation_failed = 0
        self.return_goal = None
        self.return_attempts = 0
        self.return_started_at = 0.0
        self.return_retry_at = 0.0
        self.return_error = ''
        self.build_boundary()
        self.publish_observation_markers()
        self.publish_bounded_map()
        self.report('arming', 'Waiting for both Nav2 costmap filters')
        response.success = True
        response.message = self.mission_id
        return response

    def on_cancel(self, _request, response):
        if self.phase not in ('arming', 'exploring', 'observing', 'stopping', 'returning'):
            response.message = 'No active mission to cancel'
            return response
        if self.return_goal is not None:
            self.return_goal.cancel_goal_async()
        if self.observation_goal is not None:
            self.observation_goal.cancel_goal_async()
        self.observation_generation += 1
        self.stop_reason = 'cancelled'
        self.report('stopping', 'Operator cancelled')
        response.success = True
        response.message = self.mission_id or ''
        return response

    def note_costmap(self, name, msg):
        if name == 'global':
            self.global_costmap_at = time.monotonic()
            self.global_costmap = msg
        else:
            self.local_costmap_at = time.monotonic()

    def on_map(self, msg):
        if msg.header.frame_id != 'map' or not msg.data:
            return
        self.map, self.map_at = msg, time.monotonic()
        if self.center is not None and self.phase in (
                'arming', 'exploring', 'observing', 'stopping', 'returning'):
            self.publish_bounded_map()

    def publish_bounded_map(self):
        if self.map is None or self.center is None:
            return
        msg = OccupancyGrid()
        msg.header = self.map.header
        msg.info = self.map.info
        q = msg.info.origin.orientation
        origin_yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                                1 - 2 * (q.y * q.y + q.z * q.z))
        msg.data = bounded_map_data(
            self.map.data, msg.info.width, msg.info.height, msg.info.resolution,
            msg.info.origin.position.x, msg.info.origin.position.y, origin_yaw,
            *self.center, self.half_x - self.margin, self.half_y - self.margin)
        self.map_pub.publish(msg)

    def build_boundary(self):
        x, y, yaw = self.center
        ox, oy, width, height, data = keepout_mask(
            x, y, yaw, self.half_x - self.margin, self.half_y - self.margin)
        mask = OccupancyGrid()
        mask.header.frame_id = 'map'
        mask.info.resolution = 0.10
        mask.info.width, mask.info.height = width, height
        mask.info.origin.position.x, mask.info.origin.position.y = ox, oy
        mask.info.origin.orientation.w = 1.0
        mask.data = data
        self.mask = mask
        info = CostmapFilterInfo()
        info.header.frame_id = 'map'
        info.type = 0
        info.filter_mask_topic = '/bounded_exploration/keepout_mask'
        info.base, info.multiplier = 0.0, 1.0
        self.info = info
        self.publish_boundary()

    def publish_boundary(self):
        if self.mask is None:
            return
        stamp = self.get_clock().now().to_msg()
        self.mask.header.stamp = stamp
        self.info.header.stamp = stamp
        self.mask_pub.publish(self.mask)
        self.info_pub.publish(self.info)
        for marker_id, half_x, half_y, color in (
                (0, self.half_x, self.half_y, (0.1, 0.7, 1.0)),
                (1, self.half_x - self.margin, self.half_y - self.margin, (1.0, 0.6, 0.1))):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = stamp
            marker.ns = 'bounded_exploration'
            marker.id = marker_id
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.05
            marker.color.r, marker.color.g, marker.color.b = color
            marker.color.a = 1.0
            corners = rectangle_corners(*self.center, half_x, half_y)
            marker.points = [Point(x=x, y=y, z=0.08) for x, y in corners + corners[:1]]
            self.marker_pub.publish(marker)

    def publish_observation_markers(self):
        stamp = self.get_clock().now().to_msg()
        points = Marker()
        points.header.frame_id = 'map'
        points.header.stamp = stamp
        points.ns = 'bounded_observation'
        points.id = 0
        points.action = Marker.ADD if self.observation_remaining else Marker.DELETE
        points.type = Marker.SPHERE_LIST
        points.pose.orientation.w = 1.0
        points.scale.x = points.scale.y = points.scale.z = 0.17
        points.color.r, points.color.g, points.color.b, points.color.a = 0.8, 0.2, 0.8, 0.9
        points.points = [Point(x=x, y=y, z=0.14)
                         for x, y in self.observation_remaining]
        self.marker_pub.publish(points)

        target = Marker()
        target.header.frame_id = 'map'
        target.header.stamp = stamp
        target.ns = 'bounded_observation'
        target.id = 1
        target.action = Marker.ADD if self.observation_target else Marker.DELETE
        target.type = Marker.SPHERE
        target.pose.orientation.w = 1.0
        target.scale.x = target.scale.y = target.scale.z = 0.28
        target.color.r, target.color.g, target.color.b, target.color.a = 0.1, 1.0, 0.2, 0.9
        if self.observation_target:
            target.pose.position.x, target.pose.position.y = self.observation_target
            target.pose.position.z = 0.18
        self.marker_pub.publish(target)

    def on_selected_goal(self, msg):
        if self.phase != 'exploring' or self.center is None:
            return
        self.last_goal_at = time.monotonic()
        if not inside_rectangle(msg.pose.position.x, msg.pose.position.y, *self.center,
                                self.half_x - self.margin, self.half_y - self.margin):
            self.get_logger().error('Explorer selected a goal outside safe ROI')
            self.stop_reason = 'failed'
            self.report('stopping', 'Explorer selected a goal outside safe ROI')

    def on_nav_status(self, msg):
        self.nav_goal_active = any(item.status in (
            GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING,
            GoalStatus.STATUS_CANCELING) for item in msg.status_list)

    def on_exploration_complete(self, _msg):
        if self.phase == 'exploring':
            self.stop_reason = 'complete'
            self.report('stopping', 'No frontier found inside bounded map')

    def guard_healthy(self):
        if self.mask is None or self.info is None:
            return False, 'Missing ROI mask'
        if self.info_pub.get_subscription_count() < 2 or self.mask_pub.get_subscription_count() < 2:
            return False, 'Both Nav2 keepout filters are not subscribed'
        now = time.monotonic()
        if now - self.global_costmap_at > 5 or now - self.local_costmap_at > 5:
            return False, 'Nav2 costmap is unavailable or stale'
        if now - self.map_at > float(self.get_parameter('map_stale_seconds').value):
            return False, 'SLAM map is stale'
        pose = self.current_pose()
        if pose is None:
            return False, 'Robot pose is unavailable or stale'
        dx = pose[0].pose.position.x - self.center[0]
        dy = pose[0].pose.position.y - self.center[1]
        c, s = math.cos(self.center[2]), math.sin(self.center[2])
        self.max_local_x = max(self.max_local_x, abs(c * dx + s * dy))
        self.max_local_y = max(self.max_local_y, abs(-s * dx + c * dy))
        if not self.observed_path or math.hypot(
                pose[0].pose.position.x - self.observed_path[-1][0],
                pose[0].pose.position.y - self.observed_path[-1][1]) >= 0.25:
            self.observed_path.append((pose[0].pose.position.x,
                                       pose[0].pose.position.y))
        if not inside_rectangle(pose[0].pose.position.x, pose[0].pose.position.y,
                                *self.center, self.half_x - self.margin,
                                self.half_y - self.margin):
            return False, 'Robot center reached or crossed safe ROI boundary'
        return True, ''

    def request_explorer(self, action):
        request = ControlExploration.Request()
        request.action = action
        request.delay_seconds = 0.0
        request.quit_after_stop = False
        return self.control_client.call_async(request)

    def tick(self):
        if self.phase == 'arming':
            healthy, reason = self.guard_healthy()
            if not healthy:
                self.detail = reason
                return
            if self.start_future is None:
                self.start_future = self.request_explorer(ControlExploration.Request.ACTION_START)
            elif self.start_future.done():
                result = self.start_future.result()
                self.start_future = None
                if result and result.accepted:
                    self.last_goal_at = time.monotonic()
                    self.report('exploring')
                else:
                    self.report('failed', 'Explorer did not accept start')
        elif self.phase in ('exploring', 'observing', 'returning'):
            healthy, reason = self.guard_healthy()
            if not healthy:
                if self.phase == 'returning' and self.return_goal is not None:
                    self.return_goal.cancel_goal_async()
                if self.phase == 'observing' and self.observation_goal is not None:
                    self.observation_goal.cancel_goal_async()
                    self.observation_generation += 1
                self.estop_pub.publish(Bool(data=True))
                self.stop_reason = 'failed'
                self.report('stopping', reason)
            elif self.phase == 'exploring':
                now = time.monotonic()
                if now - self.started_at > float(self.get_parameter('mission_timeout_seconds').value):
                    self.stop_reason = 'partial'
                    self.partial_reason = 'Mission time limit reached'
                    self.report('stopping', self.partial_reason)
                elif (not self.nav_goal_active and
                      now - self.last_goal_at > float(
                          self.get_parameter('idle_frontier_timeout_seconds').value)):
                    self.stop_reason = 'partial'
                    self.partial_reason = 'No reachable frontier was selected before idle timeout'
                    self.report('stopping', self.partial_reason)
            elif self.phase == 'observing':
                if self.observation_target is not None and time.monotonic() - (
                        self.observation_started_at) > float(self.get_parameter(
                            'observation_goal_timeout_seconds').value):
                    if self.observation_goal is not None:
                        self.observation_goal.cancel_goal_async()
                    self.observation_generation += 1
                    self.observation_goal = None
                    self.observation_target = None
                    self.observation_failed += 1
                    self.send_next_observation()
            elif self.phase == 'returning':
                if time.monotonic() - self.return_started_at > float(
                        self.get_parameter('return_timeout_seconds').value):
                    if self.return_goal is not None:
                        self.return_goal.cancel_goal_async()
                    self.return_error = 'Return time limit reached'
                    self.report('return_failed', self.return_error)
                    self.begin_save()
                elif self.return_retry_at and time.monotonic() >= self.return_retry_at:
                    self.return_retry_at = 0.0
                    self.begin_return()
        elif self.phase == 'stopping':
            self.tick_stopping()
        elif self.phase == 'saving':
            self.tick_saving()

    def tick_stopping(self):
        if not self.control_client.service_is_ready():
            self.estop_pub.publish(Bool(data=True))
            self.report('failed', 'Explorer control service disappeared during stop')
            return
        now = time.monotonic()
        if self.stop_future is not None:
            if not self.stop_future.done():
                return
            result = self.stop_future.result()
            self.stop_future = None
            if result and result.state == ControlExploration.Request.STATE_IDLE:
                if self.stop_reason == 'cancelled':
                    self.report('cancelled')
                elif self.stop_reason == 'complete':
                    self.begin_observations()
                elif self.stop_reason == 'partial':
                    self.begin_return()
                else:
                    self.report('failed', self.detail)
                return
        if now - self.stop_last_at > 0.5:
            self.stop_future = self.request_explorer(ControlExploration.Request.ACTION_STOP)
            self.stop_last_at = now

    @staticmethod
    def grid_cost(grid, x, y):
        q = grid.info.origin.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        return occupancy_at(grid.data, grid.info.width, grid.info.height,
                            grid.info.resolution, grid.info.origin.position.x,
                            grid.info.origin.position.y, yaw, x, y)

    def observation_point_valid(self, x, y):
        if self.map is None or self.global_costmap is None or (
                self.map.header.frame_id != 'map' or
                self.global_costmap.header.frame_id != 'map'):
            return False
        map_value = self.grid_cost(self.map, x, y)
        cost = self.grid_cost(self.global_costmap, x, y)
        return (map_value is not None and 0 <= map_value <= 20 and
                cost is not None and 0 <= cost <= 30)

    def begin_observations(self):
        if not bool(self.get_parameter('observation_points_enabled').value):
            self.begin_return()
            return
        try:
            points = observation_grid(
                *self.center, self.half_x - self.margin, self.half_y - self.margin,
                float(self.get_parameter('observation_spacing_m').value),
                float(self.get_parameter('observation_inset_m').value))
        except ValueError as exc:
            self.stop_reason = 'partial'
            self.partial_reason = str(exc)
            self.get_logger().error(str(exc))
            self.begin_return()
            return
        self.observation_remaining = [point for point in points
                                      if self.observation_point_valid(*point)]
        self.publish_observation_markers()
        self.report('observing', 'Checking additional observation points')
        self.send_next_observation()

    def send_next_observation(self):
        if self.phase != 'observing':
            return
        if not self.nav_client.server_is_ready():
            self.observation_failed += 1
            self.finish_observations()
            return
        pose = self.current_pose()
        if pose is None:
            self.observation_failed += 1
            self.finish_observations()
            return
        x0, y0 = pose[0].pose.position.x, pose[0].pose.position.y
        radius = float(self.get_parameter('observation_visited_radius_m').value)
        self.observation_remaining.sort(key=lambda point: math.hypot(
            point[0] - x0, point[1] - y0))
        limit = max(0, int(self.get_parameter('observation_max_points').value))
        while self.observation_remaining and self.observation_done + self.observation_failed < limit:
            x, y = self.observation_remaining.pop(0)
            if not self.observation_point_valid(x, y) or any(
                    math.hypot(x - px, y - py) < radius
                    for px, py in self.observed_path):
                continue
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = 'map'
            goal.pose.header.stamp = self.get_clock().now().to_msg()
            goal.pose.pose.position.x = x
            goal.pose.pose.position.y = y
            goal.pose.pose.orientation = pose[0].pose.orientation
            self.observation_generation += 1
            generation = self.observation_generation
            self.observation_target = (x, y)
            self.observation_started_at = time.monotonic()
            self.publish_observation_markers()
            self.report('observing', f'Nav2 observation goal ({x:.2f}, {y:.2f})')
            future = self.nav_client.send_goal_async(goal)
            future.add_done_callback(
                lambda done, token=generation: self.on_observation_accepted(done, token))
            return
        self.finish_observations()

    def on_observation_accepted(self, future, generation):
        if self.phase != 'observing' or generation != self.observation_generation:
            try:
                stale_handle = future.result()
                if stale_handle.accepted:
                    stale_handle.cancel_goal_async()
            except Exception:
                pass
            return
        try:
            handle = future.result()
            if not handle.accepted:
                raise RuntimeError('Nav2 rejected observation goal')
            self.observation_goal = handle
            handle.get_result_async().add_done_callback(
                lambda done, token=generation: self.on_observation_result(done, token))
        except Exception as exc:
            self.get_logger().warn('Observation goal rejected: ' + str(exc))
            self.observation_target = None
            self.observation_failed += 1
            self.send_next_observation()

    def on_observation_result(self, future, generation):
        if self.phase != 'observing' or generation != self.observation_generation:
            return
        try:
            result = future.result()
            succeeded = (result.status == GoalStatus.STATUS_SUCCEEDED and
                         result.result.error_code == 0)
        except Exception:
            succeeded = False
        if succeeded:
            self.observation_done += 1
        else:
            self.observation_failed += 1
        self.observation_goal = None
        self.observation_target = None
        self.send_next_observation()

    def finish_observations(self):
        self.observation_remaining = []
        self.observation_target = None
        self.publish_observation_markers()
        if self.observation_failed:
            self.stop_reason = 'partial'
            self.partial_reason = f'{self.observation_failed} observation goal(s) failed'
        self.begin_return()

    def begin_return(self):
        if self.return_started_at == 0.0:
            self.return_started_at = time.monotonic()
        if self.home is None or not self.nav_client.server_is_ready():
            self.return_failure('Nav2 unavailable for return')
            return
        goal = NavigateToPose.Goal()
        goal.pose = self.home
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        self.return_attempts += 1
        self.report('returning', 'Return attempt ' + str(self.return_attempts))
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self.on_return_accepted)

    def on_return_accepted(self, future):
        if self.phase != 'returning':
            return
        try:
            handle = future.result()
            if not handle.accepted:
                raise RuntimeError('Nav2 rejected home goal')
            self.return_goal = handle
            handle.get_result_async().add_done_callback(self.on_return_result)
        except Exception as exc:
            self.return_failure(str(exc))

    def on_return_result(self, future):
        if self.phase != 'returning':
            return
        try:
            result = future.result()
            self.return_succeeded = (result.status == GoalStatus.STATUS_SUCCEEDED and
                                     result.result.error_code == 0)
        except Exception:
            self.return_succeeded = False
        if self.return_succeeded:
            pose = self.current_pose()
            if pose is None or self.home is None:
                self.return_succeeded = False
            else:
                dx = pose[0].pose.position.x - self.home.pose.position.x
                dy = pose[0].pose.position.y - self.home.pose.position.y
                home_q = self.home.pose.orientation
                home_yaw = math.atan2(2 * (home_q.w * home_q.z + home_q.x * home_q.y),
                                      1 - 2 * (home_q.y * home_q.y + home_q.z * home_q.z))
                yaw_error = math.atan2(math.sin(pose[1] - home_yaw),
                                       math.cos(pose[1] - home_yaw))
                self.return_succeeded = math.hypot(dx, dy) <= 0.20 and abs(yaw_error) <= 0.25
        self.return_goal = None
        if not self.return_succeeded:
            self.return_failure('Nav2 did not reach the start pose')
        else:
            self.begin_save()

    def return_failure(self, reason):
        self.return_error = reason
        if self.return_attempts < 3 and time.monotonic() - self.return_started_at < float(
                self.get_parameter('return_timeout_seconds').value):
            self.return_retry_at = time.monotonic() + 1.0
            self.report('returning', reason + '; retry scheduled')
        else:
            self.report('return_failed', reason)
            self.begin_save()

    def begin_save(self):
        output_dir = Path(str(self.get_parameter('map_output_dir').value)).expanduser()
        try:
            output_dir.mkdir(parents=True, exist_ok=True)
            self.save_base = output_dir / ('exploration_' + self.mission_id)
            command = ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f',
                       str(self.save_base), '--fmt', 'pgm', '--ros-args',
                       '-p', 'save_map_timeout:=10.0',
                       '-p', 'map_subscribe_transient_local:=true']
            self.saver = subprocess.Popen(command, env=os.environ.copy())
            self.save_started_at = time.monotonic()
            self.report('saving')
        except Exception as exc:
            self.report('failed', 'Cannot start map saver: ' + str(exc))

    def tick_saving(self):
        result = self.saver.poll()
        if result is None and time.monotonic() - self.save_started_at > 25:
            self.saver.terminate()
            self.report('failed', 'Map save timed out')
            return
        if result is None:
            return
        if result == 0 and self.save_base.with_suffix('.yaml').is_file() and (
                self.save_base.with_suffix('.pgm').is_file()):
            phase = ('return_failed' if not self.return_succeeded else
                     'partial' if self.stop_reason == 'partial' else 'completed')
            detail = ('Map saved; ' + self.return_error if phase == 'return_failed' else
                      'Map saved; ' + self.partial_reason if phase == 'partial' else
                      f'Map saved; frontier pass finished and {self.observation_done} '
                      'additional observation point(s) reached')
            self.report(phase, detail)
        else:
            self.report('failed', 'Map saver failed with exit code ' + str(result))


def main():
    rclpy.init()
    node = ExplorationMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
