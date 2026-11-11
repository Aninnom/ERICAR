"""원거리 화살표 확인·방향 고정 시험. ROS 불필요"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_fork import driver


def detect(d, names, stamp, area=.002):
    d.update_signs({'wide_objects': 0}, [dict(name=n, area=area) for n in names], stamp, .2, stamp, .05)


def test_prepare():
    for name, direction in [('left', 1), ('right', -1)]:
        d = driver()
        detect(d, [name], 1.)
        for _ in range(10):
            detect(d, [name], 1.)
        assert d.fork_goal == 0, '같은 프레임 반복으로 방향 확정'
        detect(d, [name], 1.2)
        assert d.fork_goal == 0, '두 프레임만으로 방향 확정'
        detect(d, [name], 1.4)
        assert d.fork_goal == direction and d.fork_request == direction, '작은 표지판 조기 이동 요청 누락'
        for i in range(10):
            detect(d, ['right' if name == 'left' else 'left'], 2+i*.2, area=.1)
        assert d.fork_goal == direction and d.fork_sign == name
    d = driver()
    detect(d, ['right'], 1.)
    detect(d, ['right'], 1.2)
    detect(d, ['left', 'right'], 1.4)
    assert d.fork_goal == 0, '동시 화살표에서 기존 후보 확정'
    for stamp in (1.6, 1.8, 2.):
        detect(d, ['left'], stamp)
    assert d.fork_goal == 1
    d = driver()
    detect(d, ['right'], 1.)
    detect(d, ['right'], 1.2)
    for stamp in (1.4, 1.6, 1.8):
        detect(d, ['left'], stamp)
    assert d.fork_goal == 1, '짧은 반대 검출에 먼저 고정'


def test_prepare_speed():
    d = driver()
    d.lane_driver.update = lambda *args, **kw: (.4, 0.)
    d.cones.update = lambda *args, **kw: None
    lane = dict(valid=True, error=0., heading=0., stamp=1., stop_distance=None, colors={}, solid={})
    scan = dict(points=[], wide_objects=0, front=3., rear=3., valid_fraction=1.)
    speed, _, _ = d.update(lane, scan, [], 1., .2, 1.)
    assert speed == d.c['missions']['fork_prepare_speed']


if __name__ == '__main__':
    test_prepare()
    test_prepare_speed()
    print('조기 화살표 확인·목표 방향 고정 시험 통과')
