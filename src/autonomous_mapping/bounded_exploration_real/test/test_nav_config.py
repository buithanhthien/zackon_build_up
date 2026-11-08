import unittest
from pathlib import Path

from bounded_exploration_real.nav_config import adapt_nav2_config


class RealNavConfigTest(unittest.TestCase):
    def test_hardware_config_routes_live_mask_and_velocity_through_monitor(self):
        hardware = (Path(__file__).resolve().parents[3] / 'view_robot' /
                    'config' / 'nav2_params.yaml')
        config = adapt_nav2_config(hardware)
        for name in ('local_costmap', 'global_costmap'):
            params = config[name][name]['ros__parameters']
            self.assertEqual(params['filters'], ['keepout_filter'])
            self.assertEqual(params['keepout_filter']['filter_info_topic'],
                             '/bounded_exploration/filter_info')
            self.assertEqual(params['obstacle_layer']['scan']['topic'], '/merged')
        monitor = config['collision_monitor']['ros__parameters']
        self.assertEqual(monitor['cmd_vel_in_topic'], '/cmd_vel_smoothed')
        self.assertEqual(monitor['cmd_vel_out_topic'], '/cmd_vel_sources/navigation')


if __name__ == '__main__':
    unittest.main()
