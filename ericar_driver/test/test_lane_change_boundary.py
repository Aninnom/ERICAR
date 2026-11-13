"""노란 경계 횡단 차단 시험. python ericar_driver/test/test_lane_change_boundary.py"""
import copy
import sys
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
from ericar_driver.lane_change import LaneChange
from ericar_driver.params import ANSWER_CONFIG


def test_boundary():
    robot = yaml.safe_load((HERE.parent/'config/robot.yaml').read_text(encoding='utf-8'))['robot']
    for direction, side in [(1, 'left'), (-1, 'right')]:
        change = LaneChange(copy.deepcopy(ANSWER_CONFIG), robot)
        free = {0: 3., 1: 3., -1: 3.}
        yellow = dict(valid=True, colors={side: 'yellow'}, solid={side: True})
        assert change.choose_direction(free, yellow, requested=direction) == 0
        assert change.choose_direction(free, yellow) != direction
        dashed = dict(valid=True, colors={side: 'white'}, solid={side: False})
        assert change.choose_direction(free, dashed, requested=direction) == direction
        lane = dict(yellow, heading=0., error=0., stamp=1.)
        result = change.update(lane, dict(points=[], wide_objects=0), 0., 0., .05,
                               True, requested=direction, preferred=direction)
        assert not change.active and result is None, '경계 차단 후 이동 재시작'


def test_avoid_only():
    """멈춤 방지가 차선 변경을 끈 동안(avoid_only)에도 앞 장애물은 피함. 갈림길 요청은 무시, 갈 곳 없으면 대기 대신 None"""
    robot = yaml.safe_load((HERE.parent/'config/robot.yaml').read_text(encoding='utf-8'))['robot']
    lane = dict(valid=True, heading=0., error=0., stamp=1., colors={'left': 'white', 'right': 'white'},
                solid={'left': False, 'right': False})
    lidar_x = robot['lidar']['x']
    ahead = dict(points=[[0.9-lidar_x, 0.0]], wide_objects=0)          # 가운데 차로 0.9 m 앞 장애물
    change = LaneChange(copy.deepcopy(ANSWER_CONFIG), robot)
    assert change.update(lane, ahead, 0., 0., .05, True, avoid_only=True) is not None and change.active, '꺼진 동안 회피 안 함'
    change = LaneChange(copy.deepcopy(ANSWER_CONFIG), robot)
    clear = dict(points=[], wide_objects=0)
    assert change.update(lane, clear, 0., 0., .05, True, requested=1, avoid_only=True) is None and not change.active, \
        '꺼진 동안 갈림길 요청으로 이동'
    blocked = dict(points=[[0.9-lidar_x, 0.0], [0.5-lidar_x, 0.52], [0.5-lidar_x, -0.52]], wide_objects=0)
    change = LaneChange(copy.deepcopy(ANSWER_CONFIG), robot)
    assert change.update(lane, blocked, 0., 0., .05, True, avoid_only=True) is None and not change.waiting, \
        '꺼진 동안 갈 곳 없을 때 대기 (멈춤 방지 반복)'


if __name__ == '__main__':
    test_boundary()
    test_avoid_only()
    print('노란 경계 횡단 차단 시험 통과')
