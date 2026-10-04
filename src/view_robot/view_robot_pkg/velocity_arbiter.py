"""The sole robot velocity output. Inputs use /cmd_vel_sources/<source>."""
import json

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from action_msgs.msg import GoalStatusArray
from rclpy.clock import Clock, ClockType
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Empty, String
from std_srvs.srv import SetBool
from .velocity_arbiter_core import VelocityArbiterCore, PRIORITIES, TIMEOUTS


class VelocityArbiter(Node):
    def __init__(self):
        super().__init__('velocity_arbiter')
        priorities, timeouts = {}, {}
        for source in PRIORITIES:
            priorities[source] = self.declare_parameter(f'{source}.priority', PRIORITIES[source]).value
            timeouts[source] = self.declare_parameter(f'{source}.timeout', TIMEOUTS[source]).value
        if len(set(priorities.values())) != len(priorities) or any(not 0 < t <= 2 for t in timeouts.values()):
            raise ValueError('Arbiter priorities must be unique; timeouts must be in (0, 2].')
        self.core = VelocityArbiterCore(priorities, timeouts)
        self.output_pub = self.create_publisher(Twist, '/cmd_vel', 1)
        self.state_pub = self.create_publisher(String, '/velocity_arbiter/state', 1)
        for source in PRIORITIES:
            self.create_subscription(Twist, f'/cmd_vel_sources/{source}',
                                     lambda msg, source=source: self._input(source, msg), 1)
        self.nav_status = {}
        status_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for action in ('navigate_to_pose', 'navigate_through_poses', 'follow_path',
                       'spin', 'backup', 'drive_on_heading'):
            topic = f'/{action}/_action/status'
            self.create_subscription(GoalStatusArray, topic,
                                     lambda msg, topic=topic: self._nav_status(topic, msg), status_qos)
        self.create_service(SetBool, '/velocity_arbiter/ui_control', self._ui_control)
        self.create_subscription(Empty, '/velocity_arbiter/release/teleop',
                                 lambda _: self._release('teleop'), 1)
        self.create_subscription(Bool, '/velocity_arbiter/emergency_stop', self._emergency, 1)
        # A paused /clock must not pause a command-loss watchdog.
        self.create_timer(0.05, self._tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def _nav_status(self, topic, msg):
        self.nav_status[topic] = any(item.status in (1, 2, 3) for item in msg.status_list)

    def _health(self):
        self.core.navigation_busy = any(busy and self.count_publishers(topic) > 0
                                        for topic, busy in self.nav_status.items())
        outputs = self.get_publishers_info_by_topic('/cmd_vel')
        healthy = (len(outputs) == 1 and outputs[0].node_name == self.get_name()
                   and outputs[0].node_namespace == self.get_namespace()
                   and self.output_pub.get_subscription_count() > 0)
        self.core.set_health(healthy)

    def _input(self, source, msg):
        self._health()
        self.core.receive(source, msg.linear.x, msg.angular.z)

    def _release(self, source):
        self.core.release(source)
        self._tick()

    def _ui_control(self, request, response):
        self._health()
        if request.data:
            response.success, response.message = self.core.claim_ui()
        else:
            self.core.release('ui')
            response.success, response.message = True, 'Đã trả quyền điều khiển.'
        self._tick()
        return response

    def _emergency(self, msg):
        self.core.set_emergency_stop(msg.data)
        self._tick()

    def _tick(self):
        self._health()
        msg = Twist()
        msg.linear.x, msg.angular.z = self.core.output()
        self.output_pub.publish(msg)
        state = String()
        state.data = json.dumps({'owner': self.core.owner, 'ui_granted': self.core.ui_granted,
                                 'healthy': self.core.healthy, 'navigation_busy': self.core.navigation_busy, 'emergency_stop': self.core.emergency_stop})
        self.state_pub.publish(state)


def main(args=None):
    rclpy.init(args=args)
    node = VelocityArbiter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.output_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
