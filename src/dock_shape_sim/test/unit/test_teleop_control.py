"""Check combined controls and bounded acceleration before ROS publishing."""

import pytest

from dock_shape_sim.control.teleop import TeleopControl


@pytest.mark.parametrize('keys,expected', [
    ('W', (0.15, 0.0)), ('S', (-0.15, 0.0)),
    ('A', (0.0, 0.5)), ('D', (0.0, -0.5)),
    ('WA', (0.15, 0.5)), ('WD', (0.15, -0.5)),
    ('SA', (-0.15, 0.5)), ('SD', (-0.15, -0.5)),
    ('WS', (0.0, 0.0)), ('AD', (0.0, 0.0)), ('WSAD', (0.0, 0.0)),
])
def test_key_mixing(keys, expected):
    assert TeleopControl(keys=set(keys)).target == expected


def test_ramp_release_and_direction_change():
    control = TeleopControl(keys=set('WA'))
    for _ in range(20):
        previous = (control.linear, control.angular)
        actual = control.step(0.05)
        assert abs(actual[0] - previous[0]) <= 0.30 * 0.05 + 1e-10
        assert abs(actual[1] - previous[1]) <= 1.0 * 0.05 + 1e-10
    assert actual == pytest.approx((0.15, 0.5))
    control.keys = set('SD')
    assert control.step(0.05) == pytest.approx((0.135, 0.45))
    control.keys.clear()
    for _ in range(20):
        control.step(0.05)
    assert (control.linear, control.angular) == (0.0, 0.0)


def test_stop_and_stalled_event_loop_clear_input():
    control = TeleopControl(keys=set('WA'), linear=0.15, angular=0.5)
    assert control.step(1.0) == (0.0, 0.0)
    assert not control.keys
    control.keys.add('S')
    control.step(0.05)
    control.stop()
    assert (control.linear, control.angular) == (0.0, 0.0)
    assert not control.keys
