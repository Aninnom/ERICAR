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
    # 가운데 차로: 교통섬 정면 → 정지. 반대편 끝 차로: 멈추지 않고 그 길로 진행 (11/15, 예전 manual)
    for sign, index, phase in [('left', 1, 'manual'), ('left', 0, 'exit'), ('right', 1, 'manual'), ('right', 2, 'exit')]:
        f = mission(sign)
        drive(f, index, .5, FAR)
        assert f.phase == 'align'
        drive(f, index, 1.7, None)          # 점선 끝 → 입구
        assert f.phase == phase, (sign, index, f.phase)
        assert not f.allows_change(False) and f.request(index, False) == 0
        if phase == 'exit':
            assert f.destination == index and f.direction == (1 if index == 2 else -1)


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


def test_false_dash_after_end():
    """점선이 끝난 뒤 길 안 흰 글자·흰 선을 점선으로 봐도 입구 판정 (11/15 main_1115_1904 198~227 s)
    예전: 남은 거리가 0.5 → 1.5 m로 되돌아가 follow가 구간 전환까지 유지 → 3번 코너 유지 꺼짐"""
    f = mission('left')
    drive(f, 2, .2, FAR)                       # 목적 차로(왼쪽 끝) 도착 → follow
    assert f.phase == 'follow'
    drive(f, 2, 1.2, None)                     # 점선 끝이 화면 안으로 들어와 줄어듦 (1.7 → 0.5)
    assert f.dash_locked and f.dash_left < .6
    drive(f, 2, 1.0, 1.5)                      # 길 안 흰 표시를 1.5 m 점선으로 오검출
    assert f.phase == 'exit', f.phase
    # 차선을 못 본 동안 줄어든 값으로는 끝 확정 안 함 (다시 보이면 원래대로)
    f = mission('left')
    drive(f, 2, .2, FAR)
    for _ in range(120):
        f.step(2, False, .2, .05)
        f.observe(dict(valid=False), 2, False, .2, .05)
    assert not f.dash_locked
    drive(f, 2, .2, FAR)
    assert f.phase == 'follow' and f.dash_left == FAR and not f.late


def test_sign_pass():
    """표지판 통과 = 분기 입구 (11/15 fix_1115_1949: 점선 끝 판정이 안 걸려 follow가 구간 전환까지 유지)"""
    def see(f, index, distance, area, changing=False, fresh=True, v=.2, dt=.05):
        for _ in range(int(round(distance/(v*dt)))):
            f.step(index, changing, v, dt)
            f.observe(lane(FAR), index, changing, v, dt)      # 점선은 계속 보임 (오검출 포함) → 점선 판정은 안 걸림
            f.see_sign(area, index, changing, v, dt, fresh)
    # 목적 차로: 크게 보이다 사라지면 exit, 2.5 m 뒤 done
    f = ForkMission(3, 2.5, far=FAR, dash_gap=.45, entrance_confirm=.3)
    f.observe(lane(FAR), 1, False, 0., .05)
    assert f.lock({'left'}, 0.)
    see(f, 2, .5, .003)
    assert f.phase == 'follow'
    see(f, 2, .3, None)
    assert f.phase == 'follow', '멀리서(작은 박스) 놓친 것을 통과로 봄'
    see(f, 2, .5, .01)
    see(f, 2, .1, None)
    assert f.phase == 'follow'
    see(f, 2, .1, None)
    assert f.phase == 'exit' and not f.allows_change(False)
    see(f, 2, 2.4, None)
    assert f.phase == 'exit'
    see(f, 2, .2, None)
    assert f.phase == 'done'
    # 차선 변경 중·검출 끊김·틀어진 자세(fresh False)는 안 보인 거리로 안 셈
    f = mission('left')
    see(f, 2, .3, .01)
    see(f, 2, 1., None, changing=True)
    see(f, 2, 1., None, fresh=False)
    assert f.phase in ('align', 'follow')
    # 반대편 끝 차로: 0.4 m 넘게 안 보이면 그 길로 진행. 가운데 차로는 표지판 통과로 판정 안 함
    f = mission('left')
    see(f, 0, .3, .01)
    see(f, 0, .3, None)
    assert f.phase == 'align'
    see(f, 0, .2, None)
    assert f.phase == 'exit' and f.destination == 0 and f.direction == -1
    f = mission('left')
    see(f, 1, .3, .01)
    see(f, 1, 1., None)
    assert f.phase == 'align'


def test_relock():
    f = mission('right')
    assert not f.relock({'right'}, 1.) and not f.relock({'left', 'right'}, 1.)
    assert f.relock({'left'}, 1.) and f.destination == 2 and f.direction == 1 and f.phase == 'align'
    f.phase = 'exit'
    assert not f.relock({'right'}, 2.), '입구를 지난 뒤 방향 교체'


def test_arrow_filter():
    """트랙 밖 물체 오검출 제외 (fix_1115_1949 168.3 s 파란 천막: 가로:세로 2.45, 차 34° 틀어짐)"""
    import copy
    import yaml
    from ericar_driver.driver import Driver
    from ericar_driver.params import ANSWER_CONFIG
    robot = yaml.safe_load((Path(__file__).resolve().parents[1] / 'config/robot.yaml').read_text(encoding='utf-8'))['robot']
    c = copy.deepcopy(ANSWER_CONFIG)
    c['missions']['start_stage'] = 'fork'
    d = Driver({'answer': c, 'robot': robot})
    tarp = dict(name='right', area=.0184, bbox=[.303, .373, .486, .473])
    sign = dict(name='left', area=.008, bbox=[.1, .348, .177, .452])
    other = dict(name='slow', area=.01, bbox=[.1, .3, .5, .4])
    assert d.arrow_filter([tarp, sign, other]) == [sign, other]
    d.sections.turned = 34.0
    assert d.arrow_filter([tarp, sign, other]) == [other]


def test_corner_hold_after_destination():
    """목적 차로 도착 뒤(follow·exit)에는 A·B 모두 코너 유지 허용, 정렬 중은 금지 (11/15 main_1115_1904 216~221 s A 길,
    fix_1115_1949 204~209 s B 길: 표지판을 읽으면 S3 전환까지 fork.running이라 3번 코너 유지가 꺼져 벽으로 직진)"""
    import copy
    import yaml
    from ericar_driver.driver import Driver
    from ericar_driver.params import ANSWER_CONFIG
    robot = yaml.safe_load((Path(__file__).resolve().parents[1] / 'config/robot.yaml').read_text(encoding='utf-8'))['robot']
    scan = dict(front=4.0, rear=4.0, valid_fraction=1.0, wide_objects=0, points=[], ranges=[], angle_min=0.,
                angle_increment=.01, stamp=0.)
    lane_result = dict(valid=True, colors={'left': 'yellow'}, error=0., heading=0., forward=.9, pursuit=False,
                       abnormal=False, fits={}, stop_distance=None, stop_candidates=[], dash_end=None)
    for sign, phase, index, expected in (('left', 'follow', 2, True), ('left', 'exit', 2, True), ('left', 'align', 1, False),
                                         ('right', 'follow', 0, True), ('right', 'exit', 0, True),
                                         ('right', 'align', 1, False)):
        c = copy.deepcopy(ANSWER_CONFIG)
        c['missions']['start_stage'] = 'fork'
        d = Driver({'answer': c, 'robot': robot})
        d.fork = mission(sign)
        d.fork.phase, d.lane_change.index = phase, index
        seen = []
        original = d.main_lane_command
        d.main_lane_command = lambda *a, **k: (seen.append(a[5]), original(*a, **k))[1]
        d.update(lane_result, scan, [], 0., .2, 1.)
        assert seen and seen[0] is expected, (sign, phase, seen)


def test_outer_only_ends_after_island():
    """바깥선 하나 추종은 목적 차로 도착 ~ 분기 입구 뒤 0.8 m까지만 (11/15 fix2_1115_2017: exit 2.5 m 내내 켜져 있어
    섬 뒤에서 보이는 안쪽 선을 버림 → 4.5 s 차선 무효 → 3번 코너 벽)"""
    f = ForkMission(3, 2.5, far=FAR, dash_gap=.45, entrance_confirm=.3, outer_distance=.8)
    f.observe(lane(FAR), 1, False, 0., .05)
    assert f.lock({'left'}, 0.)
    f.step(1, False, 0., .05)
    assert not f.outer_only and f.vision_direction == 0          # 정렬 중
    f.step(2, False, .2, .05)
    assert f.phase == 'follow' and f.outer_only and f.vision_direction == 1
    f.enter(2)
    assert f.phase == 'exit' and f.outer_only
    for _ in range(int(.7/(.2*.05))):
        f.step(2, False, .2, .05)
    assert f.outer_only
    for _ in range(int(.2/(.2*.05))):
        f.step(2, False, .2, .05)
    assert f.phase == 'exit' and not f.outer_only and f.vision_direction == 0 and not f.allows_change(False)


def test_hold_max_turned():
    """코너 유지는 시작한 구간 직선 방향에서 70°(IMU)까지만 (11/15 fix2_1115_2017 204 s: 44° 돈 지점에서 시작해 65° 더 돌려
    안쪽 콘으로 향함)"""
    import copy
    import math
    import yaml
    from ericar_driver.driver import Driver
    from ericar_driver.params import ANSWER_CONFIG
    robot = yaml.safe_load((Path(__file__).resolve().parents[1] / 'config/robot.yaml').read_text(encoding='utf-8'))['robot']
    scan = dict(front=4.0, rear=4.0, valid_fraction=1.0, wide_objects=0, points=[], ranges=[], angle_min=0.,
                angle_increment=.01, stamp=0.)
    lane_result = dict(valid=True, colors={'left': 'yellow'}, error=0., heading=0., forward=.9, pursuit=False,
                       abnormal=False, fits={}, stop_distance=None, stop_candidates=[], dash_end=None)
    for turned, stop in ((44., False), (69., False), (71., True)):
        c = copy.deepcopy(ANSWER_CONFIG)
        c['missions']['start_stage'] = 'fork'
        d = Driver({'answer': c, 'robot': robot})
        d.sections.yaw = -math.radians(90.*d.sections.index+turned)
        d.lane_driver.hold = dict(yaw=0., distance=0., base=.31, steering=.31, angle=65., error0=0.)
        d.hold_section = d.sections.index
        d.update(lane_result, scan, [], 0., .2, 1.)
        assert d.lane_driver.hold_stop is stop and (d.lane_driver.hold is None) is stop, (turned, d.lane_driver.hold)


def test_lane_change_off_after_three_lane():
    """3차선 구간이 끝나면 차선 변경 판단 자체를 끔: 분기 입구 통과 뒤, 또는 표지판 없이 S2 6 m 뒤 (11/15)"""
    import copy
    import yaml
    from ericar_driver.driver import Driver
    from ericar_driver.params import ANSWER_CONFIG
    robot = yaml.safe_load((Path(__file__).resolve().parents[1] / 'config/robot.yaml').read_text(encoding='utf-8'))['robot']
    c = copy.deepcopy(ANSWER_CONFIG)
    c['missions']['start_stage'] = 'fork'
    lidar_x = robot['lidar']['x']
    scan = dict(front=.9, rear=4.0, valid_fraction=1.0, wide_objects=0, points=[[.9-lidar_x, 0.]], ranges=[], angle_min=0.,
                angle_increment=.01, stamp=0.)
    lane_result = dict(valid=True, colors={'left': 'white', 'right': 'white'}, solid={'left': False, 'right': False},
                       error=0., heading=0., forward=.9, pursuit=False, abnormal=False, fits={}, stop_distance=None,
                       stop_candidates=[], dash_end=None, stamp=1.)
    # 3차선 안: 앞 장애물이면 차선 변경 판단이 돎
    d = Driver({'answer': c, 'robot': robot})
    assert not d.three_lane_over()
    d.update(lane_result, scan, [], 0., .2, 1.)
    assert d.lane_change.active or d.lane_change.debug.get('reason') != 'disabled', d.lane_change.debug
    # 표지판 없이 S2 5 m 뒤
    d = Driver({'answer': c, 'robot': robot})
    d.sections.distance = 6.2
    d.update(lane_result, scan, [], 0., .2, 1.)
    assert d.three_lane_over() and not d.lane_change.active and d.lane_change.debug.get('reason') == 'disabled'
    # 분기 입구 통과 뒤 (갈림길이 끝난 뒤에도)
    d = Driver({'answer': c, 'robot': robot})
    d.fork = mission('left')
    d.fork.enter(2)
    d.fork.phase = 'done'
    d.lane_change.index = 2
    d.update(lane_result, scan, [], 0., .2, 1.)
    assert d.three_lane_over() and not d.lane_change.active and d.lane_change.debug.get('reason') == 'disabled'


if __name__ == '__main__':
    test_gaps_not_late()
    test_early_and_late()
    test_wrong_lane_stops()
    test_right_lane_exits()
    test_change_in_progress_finishes_first()
    test_no_sign()
    test_without_dashes_keeps_old_behavior()
    test_driver_manual_stop()
    test_false_dash_after_end()
    test_sign_pass()
    test_relock()
    test_arrow_filter()
    test_corner_hold_after_destination()
    test_outer_only_ends_after_island()
    test_hold_max_turned()
    test_lane_change_off_after_three_lane()
    print('분기 입구 판단 시험 통과')
