"""이동 중 재판단이 갈림길 표지판 목적 방향을 지키는지 (11/18 redecide_1118_06 81.3 s 재현)
python ericar_driver/test/test_redecide_fork.py"""
import copy
import sys
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
from ericar_driver.lane_change import LaneChange
from ericar_driver.params import ANSWER_CONFIG

ROBOT = yaml.safe_load((HERE.parent/'config/robot.yaml').read_text(encoding='utf-8'))['robot']
LANE = dict(valid=True, heading=0.0, error=0.0, stamp=1.0, forward=0.65,
            colors={'left': 'white', 'right': 'white'}, solid={'left': False, 'right': False})


def moving_right(fork_lanes, dashed=False):
    """왼쪽 끝 차로(2) 0.7 m 앞 장애물을 오른쪽으로 피해 가운데 차로로 가는 중, 가운데 차로 0.9 m 앞에 기둥이 나타남
    반환: 재판단 뒤 (이동 칸 수, debug)"""
    config = copy.deepcopy(ANSWER_CONFIG)
    config['lane_change']['redecide'] = True
    change = LaneChange(config, ROBOT)
    change.index = 2
    x = ROBOT['lidar']['x']
    ahead = dict(points=[[0.7-x, 0.0]], wide_objects=0)
    lane = dict(LANE, colors={'left': 'yellow', 'right': 'white'}) if dashed else LANE
    for _ in range(3):
        change.update(lane, ahead, 0.15, 0.0, 0.05, True, road_heading=0.0, fork_lanes=fork_lanes)
    assert change.active and change.direction == -1, change.debug
    if change.span == 2:
        return change.span, change.debug     # 목적이 두 칸 옆이면 try_extend가 먼저 넓힘 (기존 동작)
    if dashed:
        assert change.steering_shift() == config['lane_change']['intermediate_dash_bias'], change.debug
    # 옆으로 0.25 m 옮긴 시점: 원래 차로 장애물은 지나쳤고, 도착할 가운데 차로(지금 위치에서 오른쪽 0.27 m) 0.9 m 앞에 기둥
    change.offset, change.heading = -0.25, 0.0
    pole = dict(points=[[0.9-x, -(config['lane_change']['lane_width']-0.25)]], wide_objects=0)
    change.update(LANE, pole, 0.15, 0.0, 0.05, True, road_heading=0.0, fork_lanes=fork_lanes)
    if change.span == 2:
        assert change.steering_shift() == 0.0
        assert change.target == -2 * config['lane_change']['lane_width']
    return change.span, change.debug


def test_no_sign_extends():
    span, debug = moving_right(None)
    assert span == 2 and 'redecide' in debug, debug
    print('  표지판 모름: 기존대로 두 칸으로 넓힘 통과')


def test_left_sign_keeps_one_lane():
    span, debug = moving_right(0)       # 목적 = 떠난 왼쪽 차로
    assert span == 1 and debug.get('redecide_blocked', {}).get('reason') == 'away_from_fork', debug
    print('  left 확정(목적 = 떠난 차로): 오른쪽으로 더 넓히지 않음 통과')


def test_right_sign_extends():
    span, debug = moving_right(-2)      # 목적 = 오른쪽 끝 차로
    assert span == 2, debug
    print('  right 확정(목적 = 오른쪽 끝): 두 칸 이동 그대로 통과')


def test_dash_bias_does_not_shorten_double_crossing():
    span, debug = moving_right(None, dashed=True)
    assert span == 2 and 'redecide' in debug, debug


if __name__ == '__main__':
    test_no_sign_extends()
    test_left_sign_keeps_one_lane()
    test_right_sign_extends()
    test_dash_bias_does_not_shorten_double_crossing()
    print('test_redecide_fork 통과')
