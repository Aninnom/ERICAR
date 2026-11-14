"""갈림길 상태 전환·목적지 고정 시험. ROS 불필요"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ericar_driver.fork import ForkMission


def test_states():
    for sign, start, destination in [('left', 0, 2), ('right', 2, 0)]:
        f = ForkMission(3, 3.)
        assert f.phase == 'lane'
        assert not f.lock({'left', 'right'}, 0.)
        assert f.lock({sign}, 1.) and f.phase == 'locked'
        direction = 1 if destination > start else -1
        assert f.destination == destination and f.request(start, False) == direction
        assert not f.lock({'right' if sign == 'left' else 'left'}, 2.)
        f.step(start, True, .2, .1)
        assert f.phase == 'align' and f.request(start, True) == 0
        # 목표 반대쪽으로 회피해도 기억 유지, 장시간 대기해도 완료 처리 금지
        for _ in range(1000):
            f.step(start, False, 0., .1)
        assert f.phase == 'align' and f.destination == destination
        assert f.request(1, False) == direction
        f.step(destination, False, .2, .1)
        assert f.phase == 'follow' and f.vision_direction == direction
        assert f.request(destination, False) == 0
        f.step(destination, False, .2, 1.)
        assert f.phase == 'follow'
        f.step(1, True, .2, .1)
        assert f.phase == 'align' and f.destination == destination
        f.step(destination, False, .2, .1)
        for _ in range(160):
            f.step(destination, False, .2, .1)
        assert f.phase == 'done' and f.vision_direction == 0
        assert f.sign == sign and not f.lock({'left', 'right'}, 10.)


def test_center_and_exit():
    f = ForkMission(3, 3.)
    lane = dict(valid=True, colors=dict(left='white', right='white'))
    assert f.center_request(0, False, lane) == 1
    assert f.center_request(2, False, lane) == -1
    assert f.center_request(1, False, lane) == 0
    assert f.center_request(0, False, lane) == 0  # 회피 후 중앙으로 무조건 되돌아가지 않음
    assert f.center_request(0, True, lane) == 0
    assert f.center_request(0, False, dict(valid=True, colors={'left': 'yellow'})) == 0
    f.lock({'left'}, 1.)
    assert f.center_request(0, False, lane) == 0
    f.step(0, True, .2, .1, left_section=True)
    assert f.phase == 'done' and f.request(0, False) == 0


def test_driver_priority():
    from test_fork import driver
    for name, direction, destination in [('left', 1, 2), ('right', -1, 0)]:
        d = driver()
        d.lane_driver.update = lambda *args, **kw: (.4, 0.)
        d.cones.update = lambda *args, **kw: None
        scan = dict(points=[], wide_objects=0, front=3., rear=3., valid_fraction=1.)
        lane = dict(valid=True, error=0., heading=0., stamp=1., stop_distance=None,
                    colors=dict(left='white', right='white'), solid={})
        for t in (1., 1.2, 1.4):
            lane['stamp'] = t
            speed, steering, state = d.update(lane, scan, [dict(name=name, area=.01)], t, 0., t)
        assert d.fork.phase == 'locked' and d.fork.destination == destination
        assert state == 'lane_change' and d.lane_change.direction == direction
        assert direction*steering > 0 and speed > 0
        # 현재 이동이 끝났다는 입력 후 선택 출구 추종으로 전환
        d.lane_change.active = False
        d.lane_change.index = destination
        lane['colors'] = dict(left='yellow', right='white') if direction > 0 else dict(left='white', right='yellow')
        lane['stamp'] = 1.6
        d.update(lane, scan, [], 1.6, 0., 1.6)
        assert d.fork.phase == 'follow' and d.fork_direction() == direction
        # 구간 전환 시 이후 미션과 출구 정렬 요청 분리
        d.sections.index = 3
        lane['stamp'] = 1.8
        d.update(lane, scan, [], 1.8, 0., 1.8)
        assert d.fork.phase == 'done' and d.fork_request == 0 and d.fork.sign == name


if __name__ == '__main__':
    test_states()
    test_center_and_exit()
    test_driver_priority()
    print('갈림길 상태 전환 시험 통과')
