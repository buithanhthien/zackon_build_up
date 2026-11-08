"""Adapt the installed Zackon Nav2 parameters for a live SLAM mission."""

import tempfile
from pathlib import Path

import yaml


FILTER_INFO_TOPIC = '/bounded_exploration/filter_info'


def adapt_nav2_config(source):
    """Keep hardware tuning and replace its static-map filter with the live ROI."""
    with open(source, encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict):
        raise ValueError('Invalid hardware Nav2 parameters')

    for costmap_name in ('local_costmap', 'global_costmap'):
        params = config[costmap_name][costmap_name]['ros__parameters']
        if params.get('robot_base_frame') != 'base_link':
            raise ValueError(f'{costmap_name} robot frame must be base_link')
        if 'obstacle_layer' not in params.get('plugins', []):
            raise ValueError(f'{costmap_name} must use an obstacle layer')
        scan = params['obstacle_layer']['scan']
        if scan.get('topic') != '/merged':
            raise ValueError(f'{costmap_name} must use the real /merged LiDAR')
        params['use_sim_time'] = False
        params['filters'] = ['keepout_filter']
        params['keepout_filter'] = {
            'plugin': 'nav2_costmap_2d::KeepoutFilter',
            'enabled': True,
            'filter_info_topic': FILTER_INFO_TOPIC,
            'override_lethal_cost': False,
        }

    # All Nav2 motions pass through the smoother and collision monitor before
    # the existing velocity arbiter. The real hardware launch owns that arbiter.
    smoother = config['velocity_smoother']['ros__parameters']
    monitor = config['collision_monitor']['ros__parameters']
    smoother['use_sim_time'] = False
    monitor['use_sim_time'] = False
    monitor['cmd_vel_in_topic'] = '/cmd_vel_smoothed'
    monitor['cmd_vel_out_topic'] = '/cmd_vel_sources/navigation'
    if monitor.get('scan', {}).get('topic') != '/merged':
        raise ValueError('Collision monitor must use the real /merged LiDAR')
    for name in ('controller_server', 'planner_server', 'behavior_server', 'bt_navigator'):
        config[name]['ros__parameters']['use_sim_time'] = False
    return config


def write_session_config(source):
    """Write one session copy; never change view_robot's shared config."""
    config = adapt_nav2_config(source)
    with tempfile.NamedTemporaryFile(mode='w', prefix='bounded_exploration_real_nav2_',
                                     suffix='.yaml', dir='/tmp', delete=False,
                                     encoding='utf-8') as stream:
        yaml.safe_dump(config, stream, sort_keys=False)
        return str(Path(stream.name))
