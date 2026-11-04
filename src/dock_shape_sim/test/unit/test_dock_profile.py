import math

import pytest

from dock_shape_sim.mission.profile import load_profile, save_profile, update_reference


def test_profile_is_tied_to_map_contents(tmp_path):
    image = tmp_path / 'room.pgm'
    image.write_bytes(b'P5\n2 2\n255\n\xff\xff\xff\xff')
    filename = tmp_path / 'room.yaml'
    filename.write_text('image: room.pgm\nresolution: 0.05\n')
    save_profile(filename, {'terminal_sample': {'stamp': 10.}})
    profile = load_profile(filename)
    assert profile['calibration_status'] == 'terminal_saved_pre_docking_required'
    image.write_bytes(b'P5\n2 2\n255\n\x00\xff\xff\xff')
    with pytest.raises(ValueError, match='Map'):
        load_profile(filename)


def teaching():
    return dict(pre_docking=dict(stamp=30., session_id='session',
                                marker_in_odom=dict(x=1.4, y=0., yaw=0.)),
                terminal_sample=dict(stamp=20., session_id='session',
                                     odom_pose=dict(x=1.04, y=0., yaw=math.pi),
                                     near_points=[[.36, -.1], [.36, 0.], [.36, .1]],
                                     full_shape_visible=False))


def test_terminal_first_partial_candidate_must_match_full_v_anchor():
    data = teaching()
    data['terminal_sample']['candidate_profile'] = dict(x=.37, y=.002, yaw=.01)
    update_reference(data)
    assert data['docked_reference']['x'] == pytest.approx(.37)
    assert data['docked_reference']['blind_finish']
    data = teaching()
    data['terminal_sample']['candidate_profile'] = dict(x=.7, y=.002, yaw=.01)
    update_reference(data)
    assert data['docked_reference']['x'] == pytest.approx(.36)


def test_legacy_capture_cannot_be_reused_after_resetting_clock():
    data = teaching()
    data['terminal_sample'].update(session_id='', stamp=37538.)
    with pytest.raises(ValueError, match='reset'):
        update_reference(data)


def test_blind_terminal_needs_an_observable_near_field_signature():
    data = teaching()
    data['terminal_sample']['near_points'] = []
    with pytest.raises(ValueError, match='LiDAR'):
        update_reference(data)
