"""Real DDS test graph, synthetic sensors/receiver only; opt-in isolated domain."""
import os
from pathlib import Path
import sys
import time
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'src/view_robot'), str(ROOT / 'robot_ui')]


@unittest.skipUnless(os.environ.get('RUN_ARBITER_ROS_TESTS') == '1', 'isolated ROS test is opt-in')
class ArbiterRosTests(unittest.TestCase):
    def setUp(self):
        self.assertEqual(os.environ.get('ROS_DOMAIN_ID'), '181', 'Use only the isolated test domain 181')
        import rclpy
        from rclpy.node import Node
        from rclpy.executors import SingleThreadedExecutor
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import LaserScan
        from view_robot_pkg.velocity_arbiter import VelocityArbiter
        from motion_ros import RosMotionController
        rclpy.init()
        self.rclpy = rclpy
        self.arbiter = VelocityArbiter()
        self.probe = Node('arbiter_test_probe')
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.arbiter)
        self.executor.add_node(self.probe)
        self.velocities = []
        self.probe.create_subscription(Twist, '/cmd_vel',
                                       lambda msg: self.velocities.append((msg.linear.x, msg.angular.z)), 10)
        # Reproduce six idle Nav2 publisher endpoints; none writes final cmd_vel.
        self.nav = [self.probe.create_publisher(Twist, '/cmd_vel_sources/navigation', 1) for _ in range(6)]
        self.teleop = self.probe.create_publisher(Twist, '/cmd_vel_sources/teleop', 1)
        self.odom_pub = self.probe.create_publisher(Odometry, '/odomfromSTM32', 1)
        self.scan_pubs = [self.probe.create_publisher(LaserScan, f'/{side}_lidar/scan', 1)
                          for side in ('front', 'rear')]
        self.messages = []
        self.motion = RosMotionController(self.messages.append)
        self.pump_until(lambda: self.motion.arbiter_state is not None
                        and self.motion._control.service_is_ready()
                        and self.motion._ownership_error() is None
                        and self.motion.controller.safety_error() is None)

    def tearDown(self):
        if hasattr(self, 'motion'):
            self.motion.close()
        self.executor.shutdown()
        self.probe.destroy_node()
        self.arbiter.destroy_node()
        self.rclpy.shutdown()

    def pump(self, duration=0.1, send=None):
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import LaserScan
        end = time.monotonic() + duration
        while time.monotonic() < end:
            self.motion.heartbeat()
            odom = Odometry()
            odom.pose.pose.orientation.w = 1.
            self.odom_pub.publish(odom)
            scan = LaserScan(range_min=0.1, range_max=10., ranges=[2., 2., 2.])
            for pub in self.scan_pubs:
                pub.publish(scan)
            if send:
                send()
            self.executor.spin_once(timeout_sec=0.005)
            time.sleep(0.005)

    def pump_until(self, predicate, timeout=4., send=None):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            self.pump(0.03, send)
        self.assertTrue(predicate(), str(self.messages))

    @staticmethod
    def twist(x):
        from geometry_msgs.msg import Twist
        msg = Twist()
        msg.linear.x = x
        return msg

    def test_idle_nav_publishers_allow_khang_command_and_only_one_output(self):
        from motion_commands import parse_motion
        self.assertEqual(self.probe.count_publishers('/cmd_vel_sources/navigation'), 6)
        outputs = self.probe.get_publishers_info_by_topic('/cmd_vel')
        self.assertEqual([info.node_name for info in outputs], ['velocity_arbiter'])
        self.motion.start(parse_motion('KHANG ĐI LÙI HAI GIÂY'))
        self.pump_until(lambda: any(x < 0 for x, _ in self.velocities))
        self.pump_until(lambda: any('Hoàn thành chuỗi' in msg for msg in self.messages))
        self.pump_until(lambda: self.arbiter.core.owner == '' and self.velocities[-1] == (0., 0.))

    def test_active_navigation_denies_claim(self):
        from motion_commands import parse_motion
        send = lambda: self.nav[0].publish(self.twist(0.2))
        self.pump_until(lambda: self.arbiter.core.owner == 'navigation', send=send)
        self.motion.start(parse_motion('đi lùi 2 giây'))
        self.pump_until(lambda: any('Không thể thực hiện' in msg for msg in self.messages), send=send)
        self.assertFalse(any(x < 0 for x, _ in self.velocities))

    def test_teleop_preemption_cancels_ui_and_cannot_resume_it(self):
        from motion_commands import parse_motion
        self.motion.start(parse_motion('đi lùi 3 giây rồi xoay trái 1 vòng'))
        self.pump_until(lambda: any(x < 0 for x, _ in self.velocities))
        send = lambda: self.teleop.publish(self.twist(0.3))
        self.pump_until(lambda: any('mất quyền' in msg for msg in self.messages), send=send)
        self.pump(0.2, send)
        self.assertFalse(self.motion.controller.active)
        self.velocities.clear()
        self.pump(0.7)
        self.assertEqual(self.velocities[-1], (0., 0.))
        self.assertFalse(any(x < 0 for x, _ in self.velocities))

    def test_stop_while_claim_is_pending_never_moves(self):
        from motion_commands import parse_motion
        self.motion.start(parse_motion('đi lùi 3 giây'))
        self.motion.stop()
        self.pump(0.8)
        self.assertTrue(all(value == (0., 0.) for value in self.velocities))
        self.assertEqual(self.arbiter.core.owner, '')

    def test_source_loss_times_out_and_does_not_replay_nav(self):
        self.nav[0].publish(self.twist(0.2))
        self.pump_until(lambda: any(x > 0 for x, _ in self.velocities))
        self.pump(0.7)
        self.assertEqual(self.velocities[-1], (0., 0.))
        self.assertEqual(self.arbiter.core.owner, '')

    def test_stop_continuous_and_release_lease(self):
        from motion_commands import parse_motion
        self.motion.start(parse_motion('xoay trái đến khi tôi bảo dừng'))
        self.pump_until(lambda: any(z > 0 for _, z in self.velocities))
        self.motion.stop()
        self.pump_until(lambda: self.arbiter.core.owner == '' and self.velocities[-1] == (0., 0.))
        self.assertFalse(self.motion.controller.active)

    def test_stationary_nav_action_rejects_ui(self):
        from action_msgs.msg import GoalStatus, GoalStatusArray
        from rclpy.qos import QoSProfile, DurabilityPolicy
        from motion_commands import parse_motion
        status_pub = self.probe.create_publisher(GoalStatusArray, '/navigate_to_pose/_action/status',
                      QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        msg = GoalStatusArray(status_list=[GoalStatus(status=2)])
        self.pump_until(lambda: self.arbiter.core.navigation_busy, send=lambda: status_pub.publish(msg))
        self.motion.start(parse_motion('đi lùi 2 giây'))
        self.pump_until(lambda: any('Nav2 đang' in msg for msg in self.messages))
        self.assertTrue(all(value == (0., 0.) for value in self.velocities))

    def test_ui_packet_loss_revokes_lease(self):
        from motion_commands import parse_motion
        self.motion.start(parse_motion('đi lùi 3 giây'))
        self.pump_until(lambda: any(x < 0 for x, _ in self.velocities))
        # Drop outbound UI velocity samples while the arbiter stays alive.
        self.motion.controller.publish = lambda *args: None
        self.pump_until(lambda: not self.motion.controller.active and self.arbiter.core.owner == '')
        self.assertEqual(self.velocities[-1], (0., 0.))

    def test_emergency_stop_hook_zeros_output_and_cancels_ui(self):
        from std_msgs.msg import Bool
        from motion_commands import parse_motion
        stop_pub = self.probe.create_publisher(Bool, '/velocity_arbiter/emergency_stop', 1)
        self.motion.start(parse_motion('đi lùi 3 giây'))
        self.pump_until(lambda: any(x < 0 for x, _ in self.velocities))
        self.pump_until(lambda: not self.motion.controller.active and self.arbiter.core.emergency_stop,
                        send=lambda: stop_pub.publish(Bool(data=True)))
        self.assertEqual(self.velocities[-1], (0., 0.))

    def test_unremapped_publisher_rejected(self):
        from geometry_msgs.msg import Twist
        rogue = self.probe.create_publisher(Twist, '/cmd_vel', 1)
        self.pump_until(lambda: not self.arbiter.core.healthy)
        self.assertIsNotNone(self.motion._ownership_error())
        self.probe.destroy_publisher(rogue)


if __name__ == '__main__':
    unittest.main()
