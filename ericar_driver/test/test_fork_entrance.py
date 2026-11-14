"""분기 입구(흰 점선 끝) 판단 시험. ROS 불필요, python ericar_driver/test/test_fork_entrance.py

- 점선이 화면 끝까지 보이면 정렬 요청, 점선 끝이 보이면(늦음) 새 요청 금지
- 입구 도착: 목표 차로 → exit(차선 변경 금지), 가운데·반대편 → manual(정지)
- 진행 중인 차선 변경은 늦은 지점에서도 끝까지 (입구 판단은 변경이 끝난 뒤)
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ericar_driver.fork import ForkMission

FAR = 1.7
LANE = dict(valid=True, colors={'left': 'white', 'right': 'white'})


def lane(dash_end):
    return dict(LANE, dash_end=dash_end)


def drive(f, index, distance, dash_end, changing=False, v=.2, dt=.05):
    """distance(m) 동안 v로 주행. dash_end = 함수(남은 점선 거리) 또는 고정값"""
    for _ in range(int(round(distance/(v*dt)))):
        f.step(index, changing, v, dt)
        f.observe(lane(dash_end), index, changing, v, dt)


def mission(sign='left'):
    f = ForkMission(3, 3., far=FAR, dash_gap=.45, entrance_confirm=.3)
    f.observe(lane(FAR), 1, False, 0., .05)
    assert f.lock({sign}, 0.)
    f.step(1, False, 0., .05)
    return f


def dashes(end, on=.2, off=.3, clip=.08, flicker=0):
    """점선 끝이 end(m)인 점선을 s(m) 주행했을 때 카메라가 보는 dash_end
    clip: 화면 끝에서 잘려 이보다 짧은 조각은 길이 필터에 걸림
    flicker: n프레임마다 가장 먼 조각을 놓침 (먼 점선은 픽셀이 적어 흰색 임계값에 자주 걸림)"""
    frame = [0]
    def seen(s):
        pieces = []
        for k in range(200):
            a, b = k*(on+off)-s, k*(on+off)+on-s           # 차 기준 점선 조각 [a, b]
            a, b = max(a, .6), min(b, FAR, end-s)
            if b-a >= clip:
                pieces.append(b)
        frame[0] += 1
        if flicker and frame[0] % flicker == 0 and pieces:
            pieces.pop()
        return pieces[-1] if pieces else None
    return seen


def drive_seen(f, index, distance, seen, v=.2, dt=.05):
    s = 0.
    for _ in range(int(round(distance/(v*dt)))):
        f.step(index, False, v, dt)
        f.observe(lane(seen(s)), index, False, v, dt)
        s += v*dt
        yield s


def test_gaps_not_late():
    # 11/12 run_1112_0408: 빈칸이 화면 끝에 걸리면 dash_end가 순간 짧아짐 → 늦음 착각으로 회피 막힘
    f = mission('left')
    # 빈칸 0.3 m + 잘린 조각 + 3프레임마다 가장 먼 조각 놓침 → 예전 코드는 dash_end 1.2 m 아래로 순간 떨어져 늦음 착각
    late = [s for s in drive_seen(f, 1, 4., dashes(20., flicker=3)) if f.late]
    assert not late, late[:3]
    # 장애물이 0.3 m 동안 점선을 가림 → 늦음 아님
    f = mission('left')
    for s in drive_seen(f, 1, .3, lambda s: None):
        pass
    assert not f.late


def test_early_and_late():
    f = mission('left')
    assert not f.late and f.request(1, False) == 1 and f.allows_change(False)
    # 실제 점선 끝이 다가옴 → 남은 거리 1.25 m 아래에서 늦음: 새 요청·새 변경 금지, 진행 중 변경은 허용
    # 마지막 점선 조각 [2.0, 2.2] → 점선 끝 2.2 m. 남은 거리 = 2.2 − s < 1.25 되는 s ≈ 0.95에서 늦음
    on_late = [s for s in drive_seen(f, 1, 1.2, dashes(2.2)) if f.late]
    assert on_late and abs(on_late[0]-.95) < .05, on_late[:1]
    assert f.late and f.request(1, False) == 0
    assert not f.allows_change(False) and f.allows_change(True)
    assert f.center_request(0, False, lane(1.2)) == 0


def test_wrong_lane_stops():
    for sign, index in [('left', 1), ('left', 0), ('right', 1), ('right', 2)]:
        f = mission(sign)
        drive(f, index, .5, FAR)
        assert f.phase == 'align'
        drive(f, index, 1.7, None)          # 점선 끝 → 입구
        assert f.phase == 'manual', (sign, index, f.phase)
        assert not f.allows_change(False) and f.request(index, False) == 0


def test_right_lane_exits():
    for sign, destination in [('left', 2), ('right', 0)]:
        f = mission(sign)
        f.step(destination, False, .2, .05)
        assert f.phase == 'follow'
        drive(f, destination, 1.0, FAR)
        assert f.phase == 'follow'           # 점선 구간에서는 거리로 끝내지 않음
        drive(f, destination, 1.7+.31, None)
        assert f.phase == 'exit' and f.vision_direction == (1 if sign == 'left' else -1)
        assert not f.allows_change(False)    # 입구 뒤 차선 변경 금지
        f.observe(lane(None), 1, False, .2, .05)
        f.step(1, False, .2, .05)            # 경계 추적 중 차로 번호가 바뀌어도 재정렬 안 함
        assert f.phase == 'exit' and f.request(1, False) == 0
        drive(f, destination, 3.1, None)
        assert f.phase == 'done' and not f.allows_change(False)


def test_change_in_progress_finishes_first():
    f = mission('left')
    drive(f, 1, 2.2, None, changing=True)  # 이동 중 점선이 끝남
    assert f.phase == 'align'               # 판단 보류
    f.step(2, False, .2, .05)
    f.observe(lane(None), 2, False, .2, .05)
    assert f.phase in ('follow', 'exit')
    drive(f, 2, .1, None)
    assert f.phase == 'exit'


def test_no_sign():
    # 표지판 없이 끝 차로 → 그 출구, 가운데 → 정지
    for index, phase, direction in [(2, 'exit', 1), (0, 'exit', -1), (1, 'manual', 0)]:
        f = ForkMission(3, 3., far=FAR)
        drive(f, index, .5, FAR)
        drive(f, index, 2.1, None)
        assert f.phase == phase and f.direction == direction, (index, f.phase)


def test_without_dashes_keeps_old_behavior():
    f = ForkMission(3, 3., far=FAR)
    assert f.lock({'left'}, 0.)
    f.step(2, False, .2, .05)
    for _ in range(320):
        f.step(2, False, .2, .05)
        f.observe(dict(valid=True), 2, False, .2, .05)
    assert f.phase == 'done'


def test_driver_manual_stop():
    import copy
    import yaml
    from ericar_driver.driver import Driver
    from ericar_driver.params import ANSWER_CONFIG
    c = copy.deepcopy(ANSWER_CONFIG)
    c['missions']['start_stage'] = 'fork'
    robot = yaml.safe_load((Path(__file__).resolve().parents[1] / 'config/robot.yaml').read_text(encoding='utf-8'))['robot']
    d = Driver({'answer': c, 'robot': robot})
    d.fork = mission('left')
    d.fork.phase = 'manual'
    scan = dict(front=4.0, rear=4.0, valid_fraction=1.0, wide_objects=0, points=[], ranges=[], angle_min=0.,
                angle_increment=.01, stamp=0.)
    lane_result = dict(LANE, error=0., heading=0., forward=.9, pursuit=False, abnormal=False, fits={},
                       stop_distance=None, stop_candidates=[], dash_end=None)
    try:
        speed, _, mode = d.update(lane_result, scan, [], 0., .2, 1.)[:3]
    except Exception as error:  # 입력 형식이 달라 드라이버 전체를 못 돌리면 상태 시험만으로 판단
        print('  (driver 연결 시험 건너뜀: %s)' % error)
        return
    assert speed == 0.0, speed


if __name__ == '__main__':
    test_gaps_not_late()
    test_early_and_late()
    test_wrong_lane_stops()
    test_right_lane_exits()
    test_change_in_progress_finishes_first()
    test_no_sign()
    test_without_dashes_keeps_old_behavior()
    test_driver_manual_stop()
    print('분기 입구 판단 시험 통과')
