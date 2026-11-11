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


def test_front_points_without_wide_cluster():
    d = driver()
    lane = dict(valid=True, heading=0., error=0., stamp=1., colors={}, solid={})
    scan = dict(points=[[.8-d.robot['lidar']['x'], 0.]], wide_objects=0)
    result = d.lane_change.update(lane, scan, 0., 0., .05, True)
    assert d.lane_change.active and result is not None and result[0] > 0, '넓은 군집 아님 → 전방 회피 누락'


def test_short_approach_before_alignment():
    for name, direction in [('left', 1), ('right', -1)]:
        d = driver(prepare_distance=.15)
        for stamp in (1., 1.2, 1.4):
            detect(d, [name], stamp)
        assert d.fork_goal == direction and d.fork_pending and d.fork_request == 0
        # 멈춘 채 시간만 흐르면 출구 정렬 시작 금지
        for i in range(20):
            d.update_signs({'wide_objects': 0}, [], 2+i*.05, 0., 2+i*.05, .05)
        assert d.fork_request == 0
        # 표지판을 놓쳐도 거리만큼 접근한 뒤 기억한 방향으로 요청
        for i in range(20):
            d.update_signs({'wide_objects': 0}, [], 4+i*.05, .2, 4+i*.05, .05)
        assert not d.fork_pending and d.fork_request == direction
    d = driver(prepare_distance=.15)
    for stamp in (1., 1.2, 1.4):
        detect(d, ['right'], stamp)
    lane = dict(valid=True, heading=0., error=0., stamp=2., colors={}, solid={})
    scan = dict(points=[[.8-d.robot['lidar']['x'], 0.]], wide_objects=0)
    result = d.lane_change.update(lane, scan, 0., 0., .05, True, requested=d.fork_request,
                                  preferred=d.fork_goal)
    assert d.fork_pending and d.lane_change.active and result[0] > 0, '출구 준비 대기 중 장애물 회피 지연'


if __name__ == '__main__':
    test_prepare()
    test_prepare_speed()
    test_front_points_without_wide_cluster()
    test_short_approach_before_alignment()
    print('조기 화살표 확인·목표 방향 고정 시험 통과')
