import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src/view_robot'))
from view_robot_pkg.velocity_arbiter_core import VelocityArbiterCore


class ArbiterTests(unittest.TestCase):
    def setUp(self):
        self.now = 0.0
        self.core = VelocityArbiterCore(clock=lambda: self.now)
        self.core.set_health(True)

    def test_idle_publishers_and_idle_zeroes_do_not_block_ui(self):
        for _ in range(6):
            self.core.receive('navigation', 0., 0.)
        self.assertTrue(self.core.claim_ui()[0])
        self.core.receive('ui', -0.1, 0.)
        self.assertEqual(self.core.output(), (-0.1, 0.))

    def test_active_nav_and_stationary_nav_action_reject_ui(self):
        self.core.receive('navigation', 0.2, 0.)
        self.assertFalse(self.core.claim_ui()[0])
        self.core.clear()
        self.core.navigation_busy = True
        self.assertFalse(self.core.claim_ui()[0])

    def test_teleop_preempts_and_revokes_ui_without_replaying_old_commands(self):
        self.assertTrue(self.core.claim_ui()[0])
        self.core.receive('ui', -0.1, 0.)
        self.core.receive('teleop', 0.3, 0.)
        self.assertFalse(self.core.ui_granted)
        self.core.receive('ui', -0.1, 0.)
        self.assertEqual(self.core.output(), (0.3, 0.))
        self.core.release('teleop')
        self.core.receive('ui', -0.1, 0.)
        self.assertEqual(self.core.output(), (0., 0.))

    def test_centered_deadman_owns_teleop(self):
        self.core.receive('navigation', 0.2, 0.)
        self.core.receive('teleop', 0., 0.)
        self.core.receive('navigation', 0.2, 0.)
        self.assertEqual(self.core.owner, 'teleop')
        self.assertEqual(self.core.output(), (0., 0.))

    def test_timeout_zeroes_without_fallback_to_stale_lower_priority(self):
        self.core.receive('navigation', 0.2, 0.)
        self.core.receive('teleop', 0.3, 0.)
        self.core.receive('navigation', 0.2, 0.)
        self.now = 0.51
        self.assertEqual(self.core.output(), (0., 0.))
        self.assertEqual(self.core.owner, '')
        self.core.receive('navigation', 0.1, 0.)
        self.assertEqual(self.core.output(), (0.1, 0.))

    def test_ui_requires_claim_and_claim_expires_without_commands(self):
        self.core.receive('ui', -0.1, 0.)
        self.assertEqual(self.core.output(), (0., 0.))
        self.assertTrue(self.core.claim_ui()[0])
        self.now = 0.51
        self.core.receive('ui', -0.1, 0.)
        self.assertFalse(self.core.ui_granted)
        self.assertEqual(self.core.output(), (0., 0.))

    def test_release_and_estop_clear_lease_and_reject_delayed_packets(self):
        self.assertTrue(self.core.claim_ui()[0])
        self.core.receive('ui', -0.1, 0.)
        self.core.release('ui')
        self.core.receive('ui', -0.1, 0.)
        self.assertEqual(self.core.output(), (0., 0.))
        self.core.receive('navigation', 0.2, 0.)
        self.core.set_emergency_stop(True)
        self.core.receive('teleop', 0.3, 0.)
        self.assertFalse(self.core.claim_ui()[0])
        self.assertEqual(self.core.output(), (0., 0.))
        self.core.set_emergency_stop(False)
        self.assertEqual(self.core.output(), (0., 0.))

    def test_invalid_input_and_bad_graph_fail_closed(self):
        self.core.receive('navigation', 0.2, 0.)
        self.core.receive('navigation', math.nan, 0.)
        self.assertEqual(self.core.output(), (0., 0.))
        self.core.set_health(False)
        self.assertFalse(self.core.claim_ui()[0])
        self.core.receive('teleop', 0.2, 0.)
        self.assertEqual(self.core.output(), (0., 0.))

    def test_priorities_for_all_modes(self):
        for source in ('navigation', 'following', 'docking', 'localization', 'teleop'):
            self.core.receive(source, 0.1, 0.)
            self.assertEqual(self.core.owner, source)
        self.core.receive('following', 0.3, 0.)
        self.assertEqual(self.core.owner, 'teleop')


if __name__ == '__main__':
    unittest.main()
