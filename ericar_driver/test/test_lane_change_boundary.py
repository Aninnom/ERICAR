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


if __name__ == '__main__':
    test_boundary()
    print('노란 경계 횡단 차단 시험 통과')
