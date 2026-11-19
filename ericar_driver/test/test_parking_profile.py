import copy
import pytest
from ericar_driver.params import ANSWER_CONFIG, apply_parking_profile


def test_profile_a_keeps_reverted_values():
    parking = apply_parking_profile(copy.deepcopy(ANSWER_CONFIG['parking']), 'a')
    assert parking['template_start_to_row'] == .45
    assert parking['template_fusion_max_offset'] == .20
    assert parking['profile'] == 'a'


def test_profile_b_restores_3283a8d_numbers_only():
    base = copy.deepcopy(ANSWER_CONFIG['parking'])
    parking = apply_parking_profile(copy.deepcopy(base), 'B')
    assert parking['template_start_to_row'] == .54
    assert parking['template_fusion_max_offset'] == .05
    changed = {k for k in parking if parking[k] != base.get(k)}
    assert changed == {'template_start_to_row', 'template_fusion_max_offset', 'profile'}


def test_unknown_profile_exits():
    with pytest.raises(SystemExit):
        apply_parking_profile({}, 'c')


def test_parse_args_profile_default_and_flag():
    pytest.importorskip('rclpy')
    from ericar_driver.node import parse_args
    assert parse_args(['driver'])[4] == 'a'
    assert parse_args(['driver', 'cfg.yaml', '--parking-profile', 'b'])[4] == 'b'
