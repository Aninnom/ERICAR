"""YOLO 분기 상태 시험. ROS 불필요, python ericar_driver/test/test_fork.py"""
import copy
import sys
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
from ericar_driver.driver import Driver
from ericar_driver.params import ANSWER_CONFIG


def driver():
    c = copy.deepcopy(ANSWER_CONFIG)
    c['missions']['start_stage'] = 'fork'
    robot = yaml.safe_load((HERE.parent / 'config/robot.yaml').read_text(encoding='utf-8'))['robot']
    return Driver({'answer': c, 'robot': robot})


def signs(d, names, t, wide=0):
    detections = [dict(name=n, area=1.0) for n in names]
    d.update_signs({'wide_objects': wide}, detections, t, .2, t, .05)


def test_conflicting_arrows():
    d = driver()
    for t in (0., .1, .2):
        signs(d, ['left', 'right'], t)
    assert 'fork' not in d.done, '동시 화살표로 방향 확정'


def test_both_directions():
    for name, start, destination, direction in [('left', 0, 2, 1), ('right', 2, 0, -1)]:
        d = driver()
        d.lane_change.index = start
        for t in (0., .1):
            signs(d, [name], t, wide=1)
        assert d.fork_pending and d.fork_request == 0
        for i in range(15):
            signs(d, [], .2+i*.05)
        assert d.fork_request == direction
        d.lane_change.active = True
        signs(d, [], 1.)
        assert d.fork_request == direction, '이동 중 방향 요청 소실'
        d.lane_change.active = False
        d.lane_change.index = start+direction
        signs(d, [], 1.1)
        assert d.fork_request == direction, '두 번째 차로 이동 누락'
        # 카메라 확인 전 거리 제한 만료 시 이전 요청 제거 확인
        d.fork_remaining = 0
        signs(d, [], 1.15)
        assert d.fork_request == 0
        d.fork_remaining = 1
        d.fork_target = direction*.55
        d.lane_change.index = destination
        signs(d, [], 1.2)
        assert d.fork_request == 0
        d.fork_remaining = 0
        signs(d, [], 1.3)
        assert d.fork_request == 0 and not d.fork_pending


if __name__ == '__main__':
    test_conflicting_arrows()
    test_both_directions()
    print('YOLO 좌·우 분기 상태 시험 통과')
