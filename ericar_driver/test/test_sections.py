"""구간 관리자·멈춤 방지 시험 (맥·차, ROS 불필요)

1) section.py 단독: 코너 4개, 갈림길 꺾임, 차선 변경 흔들림, 자이로 치우침
2) 노드 전체 (scenario.py): 흰색 정지선 + 가짜 코너(IMU)로 보행자 → 갈림길 → 신호등 → 주차 진행,
   표지판 하나를 빼도 이후 미션이 동작하는지 (예전 미션 사슬은 이후 전부 미동작)
3) 멈춤 방지: 표지판 검출이 끊겨도 주행 계속, 신호등 대기 중 끊기면 포기 후 진행,
   차선 변경 대기·전방 비상 정지가 stall.seconds 넘으면 강제 전환
4) 서행·방지턱 (slow_zone.py): 표지판·거리 진입, 방지턱 감지 뒤 해제, 감지 실패 대비, 턱에서 멈춤

실행: python3 test/test_sections.py   (약 2분, numpy·opencv·pyyaml 필요)
"""
import copy
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent))

import scenario  # noqa: E402
from ericar_driver.params import ANSWER_CONFIG  # noqa: E402
from ericar_driver.section import SectionManager  # noqa: E402

CONFIG = str(HERE.parent / 'config' / 'robot.yaml')
WHITE = (225, 225, 225)
# 가짜 코너 위치(s): 서행 표지판 뒤, 보행자 정지선 뒤, 3차선 장애물 뒤 2개 (라바콘 구간은 시나리오에 없음)
CORNERS = [1.25, 3.5, 10.9, 11.15]
DT = 0.05
# 3차선 구간(안쪽 선 = 흰 점선): 2번 코너 뒤 ~ 3번 코너 앞. 그 밖은 선 4개 모두 노란색 (예전 장면)
THREE_LANE = (4.5, 10.8)
# 이 장면의 3차선은 S2 시작(3.5) 뒤 7.3 m까지 → 실제 트랙용 거리 기준(lane_change_max_distance)은 끔
ANSWER_CONFIG['sections']['lane_change_max_distance'] = None
ANSWER_CONFIG['missions']['fork_enter_distance'] = None   # 같은 이유: 입구 거리 안전장치도 끔
# 주행 시간(s): S2 방향 확정 전 속도 상한(fork_prepare_speed 0.18, PR #37)으로 주차 완료가 약 111 s → 기본 110 s로는 모자람
DURATION = 130.0


# ---------- 1. 구간 관리자 단독 ----------

def drive(manager, segments, speed=0.3, bias=0.0, scale=1.0):
    """segments: (거리 m, 회전 °, + 왼쪽[, 전환 보류]) 목록을 speed로 주행. 구간 번호 기록 반환"""
    seen = [manager.index]
    for length, turn, *hold in segments:
        steps = max(1, int(round(length / speed / DT)))
        rate = math.radians(turn) / (steps * DT)
        for _ in range(steps):
            manager.update(rate * scale + bias, speed, DT, hold=bool(hold and hold[0]))
            if manager.index != seen[-1]:
                seen.append(manager.index)
    return seen


def lap(fork=0.0):
    """한 바퀴 (도면 대략): 직선·우회전 코너 4개. fork = 하단 직선 갈림길 꺾임 각(°, + 왼쪽 갈림길)"""
    corner = (1.0, -90)
    wiggle = [(0.5, -20), (0.5, 20), (0.5, 20), (0.5, -20)]   # 3차선 차선 변경 2회
    bottom = wiggle + ([(0.4, fork), (1.5, 0), (0.4, -fork)] if fork else [(2.3, 0)]) + [(1.0, 0)]
    return [(7.0, 0), corner, (4.0, 0), corner] + bottom + [corner, (4.0, 0), corner, (5.0, 0)]


def test_section_manager():
    c = ANSWER_CONFIG['sections']
    assert drive(SectionManager(c), lap()) == [0, 1, 2, 3, 4]
    assert drive(SectionManager(c), lap(fork=45)) == [0, 1, 2, 3, 4], '왼쪽 갈림길 꺾임을 코너로 셈'
    assert drive(SectionManager(c), lap(fork=-45)) == [0, 1, 2, 3, 4], '오른쪽 갈림길 꺾임을 코너로 셈'
    # 자이로 치우침: 실측 0.3°/분의 10배(3°/분)에서도 같음
    for bias in (math.radians(3) / 60, -math.radians(3) / 60):
        assert drive(SectionManager(c), lap(fork=-45), bias=bias) == [0, 1, 2, 3, 4], bias
        assert drive(SectionManager(c), lap(fork=45), bias=bias) == [0, 1, 2, 3, 4], bias
    assert drive(SectionManager(c), lap(), speed=0.15) == [0, 1, 2, 3, 4]
    # 구간 안에서 좌우로 크게 흔들림 (3차선 연속 차선 변경 ±40°, 수동 개입 뒤 복귀 70°): 코너로 세지 않음
    # 11/11 segA_1111_lap2b: 수동 개입 뒤 자동 복귀 회전을 "최근 2 m 안 65°" 방식이 코너로 셈
    slalom = [(0.5, 40), (0.5, -40), (0.5, -40), (0.5, 40)] * 3
    assert drive(SectionManager(c, 2), [(1, 0)] + slalom + [(1, 0)]) == [2]
    assert drive(SectionManager(c, 2), [(1, 0), (0.6, 40), (1.2, -70), (0.6, 30), (1, 0)]) == [2]
    # 코너 유지 실측 58°(IMU) 뒤 나머지를 pursuit로 천천히 돌아도 1개
    assert drive(SectionManager(c), [(2, 0), (0.6, -58), (1.0, -32), (2, 0)], speed=0.2) == [0, 1]
    # 재출발 구간 지정 (그 구간 직선 방향에서 시작), 마지막 구간 뒤에는 더 올라가지 않음
    assert drive(SectionManager(c, 3), [(1, 0), (1, -90), (2, 0), (1, -90), (1, 0)]) == [3, 4]
    # 라바콘이 우회전을 강제하는 배치 (11/13 유경 영상): 통과 중(보류) 오른쪽 80° → 되돌아옴. S3 유지, 뒤 4번 코너에서만 S4
    cones_right = [(1, 0), (0.8, -80, True), (0.5, 0, True), (0.8, 80, True)]
    assert drive(SectionManager(c, 3), cones_right + [(0.5, 0)]) == [3]
    assert drive(SectionManager(c, 3), cones_right + [(0.3, 0), (1, -90), (2, 0)]) == [3, 4]
    assert drive(SectionManager(c, 3), [(1, 0), (0.8, -80), (0.5, 0)]) == [3, 4], '보류 없으면 예전처럼 전환 (문제 재현)'
    # 라바콘이 4번 코너 안에서 끝남: 보류가 풀리는 순간 전환
    late = SectionManager(c, 3)
    assert drive(late, [(1, 0), (0.9, -80, True)]) == [3] and late.held
    assert drive(late, [(0.1, -10), (1, 0)]) == [3, 4] and not late.held and abs(late.turned) < 1
    # 보류가 끝내 안 풀림: 기준 넘은 채 hold_distance 가면 전환
    stuck = SectionManager(c, 3)
    assert drive(stuck, [(1, 0), (1, -90, True), (c['hold_distance']-0.5, 0, True)]) == [3]
    assert drive(stuck, [(0.6, 0, True)]) == [3, 4]
    disabled = SectionManager(dict(c, enabled=False))
    assert disabled.allows('parking') and disabled.allows('lane_change')
    manager = SectionManager(c)
    assert manager.allows('slow') and not manager.allows('pedestrian') and not manager.allows('cones')
    # 라바콘: 기본값에서 S3에서만 켜짐 (11/12 sections.off에서 뺌). 목록에 넣으면 구간과 무관하게 꺼짐
    assert 'cones' not in c['off'] and SectionManager(c, 3).allows('cones') and not SectionManager(c, 2).allows('cones')
    off = dict(c, off=['cones'])
    assert not SectionManager(off, 3).allows('cones') and not SectionManager(dict(off, enabled=False)).allows('cones')
    print('구간 관리자 단독 통과')


# ---------- 2. 노드 전체: 표지판 누락 ----------

def full(drop=(), **kw):
    sections = []

    reset = set()

    def on_tick(node, sim, t):
        driver = node.loop.driver
        # 차선 변경기(lane_change.heading)는 차선 변경이 없는 동안에도 회전을 계속 적분함 → 코너를 돌면 도로 방향 기억이
        # 90°씩 틀어지고, 그 값이 갈림길 요청에 쓰임 (기존 문제, 3차선·갈림길 담당 범위, 보드 42번)
        # 이 시험은 구간 관리자만 보므로 가짜 코너가 끝날 때마다 0으로 되돌림
        for corner in CORNERS:
            if corner not in reset and sim['s'] >= corner + scenario.CORNER_LENGTH:
                reset.add(corner)
                driver.lane_change.heading = 0.0
        index = driver.sections.index
        if not sections or sections[-1][0] != index:
            sections.append((index, round(sim['s'], 2)))
    records = scenario.run(CONFIG, stop_paint=WHITE, drop=drop, corners=CORNERS, on_tick=on_tick,
                           three_lane=THREE_LANE, **dict(dict(duration=DURATION), **kw))
    states = [r[2] for r in records if r[0] == 'status']
    return states, sections, records


def has(states, name):
    return any(s == name or s.startswith(name+':') for s in states)


def test_missing_signs():
    # 일반 코스는 주차 진입까지만 검증. FREE·흰 선 소실 완료는 test_parking_2에서 검증
    states, sections, _ = full()
    print('  전체 표지판:', states)
    print('  구간 전환 (번호, s):', sections)
    assert [i for i, _ in sections] == [0, 1, 2, 3, 4]
    for name in ('pedestrian_hold', 'lane_change', 'traffic_hold', 'parking_scan'):
        assert has(states, name), f'{name} 없음: {states}'
    for drop, later in ((('slow',), ('pedestrian_hold', 'traffic_hold', 'parking_scan')),
                        (('pedestrian',), ('lane_change', 'traffic_hold', 'parking_scan')),
                        (('left',), ('traffic_hold', 'parking_scan')),
                        (('traffic_red', 'traffic_green'), ('parking_scan',))):
        states, sections, _ = full(drop)
        print(f'  {drop} 누락:', states)
        assert [i for i, _ in sections] == [0, 1, 2, 3, 4]
        for name in later:
            assert has(states, name), f'{drop} 누락 시 {name} 없음: {states}'
    print('표지판 누락 시험 통과')


# ---------- 3. 멈춤 방지 ----------

def test_detector_loss():
    # 주행 중 검출이 끊김 (s 4~9, 갈림길 표지판 구간 포함): 서지 않고 지나가고, 돌아온 뒤 신호등·주차 동작
    states, _, records = full(detector=True, detection_gap=(4.0, 9.0))
    print('  검출 끊김 4~9 m:', states)
    assert not has(states, 'waiting for fresh sensors / detector')
    assert has(states, 'traffic_hold') and has(states, 'parking_scan')
    # 신호등 접근 중 검출이 끊겨 돌아오지 않음 (녹색을 볼 수 없음): 정지 후 stall.seconds 뒤 포기하고 주차까지 진행
    states, _, records = full(detector=True, detection_gap=(12.0, 16.0))
    print('  신호등 대기 중 검출 끊김:', states)
    status = [r for r in records if r[0] == 'status']
    hold = next(i for i, r in enumerate(status) if r[2] == 'traffic_hold')
    # 11/14: 합성 장면에는 횡단보도 선이 없어 큰 빨강 박스에서 예비 정지 → 녹색 뒤 traffic_approach로 풀림 (PR #42)
    assert status[hold + 1][2] in ('lane', 'traffic_approach'), states
    waited = status[hold + 1][1] - status[hold][1]
    print(f'  신호등 정지 {waited:.1f} s 뒤 포기')
    assert waited < ANSWER_CONFIG['stall']['seconds'] + 1.0, waited
    assert has(states, 'parking_scan'), states
    # 시작부터 검출이 한 번도 안 옴: startup_wait 뒤 차선 주행 시작
    states, _, records = full(detector=True, detection_gap=(-1.0, 99.0), duration=40.0)
    print('  검출 없음:', states)
    moving = [r[1] for r in records if r[0] == 'drive' and r[2] > 0.05]
    assert moving and abs(moving[0] - ANSWER_CONFIG['detector']['startup_wait']) < 1.0, moving[:1]
    print('검출 끊김 시험 통과')


def scan_with(front=None, wide=0, narrow=(), points=()):
    return dict(front=front if front is not None else 4.0, rear=4.0, valid_fraction=1.0,
                wide_objects=wide, narrow_centers=list(narrow), points=list(points))


def lane_ok():
    return dict(valid=True, abnormal=False, error=0.0, heading=0.0, stop_distance=None, forward=0.65,
                pursuit=True, curvature=0.0, solid={'left': True, 'right': True}, colors={})


def test_stall_guard():
    import yaml
    from ericar_driver.driver import Driver
    robot = yaml.safe_load(open(CONFIG))['robot']
    c = copy.deepcopy(ANSWER_CONFIG)
    limit = c['stall']['seconds']

    # (가) 차선 변경 대기: 앞이 막혔고 양옆이 실선 → 예전에는 끝까지 정지
    driver = Driver({'answer': c, 'robot': robot})
    driver.sections.index = 2
    # 라이다 기준 0.35 m 앞 물체 (뒷축 기준 약 0.56 m). lane_change.stop_distance(0.6) 안쪽이라야 접근 없이 바로 대기
    wall = [(x, y) for x in (0.35,) for y in np.linspace(-0.3, 0.3, 13)]
    t, v, resumed = 0.0, 0.0, None
    while t < limit + 6.0:
        speed, steering, state = driver.update(lane_ok(), scan_with(wide=1, points=wall), [], t, v, t)
        if t < limit - 0.5:
            assert speed == 0.0 and 'waiting' in state, (t, speed, state)
        if resumed is None and speed > 0:
            resumed, off = t, driver.lane_change_off
        v = speed
        t += DT
    assert resumed is not None and resumed < limit + 0.5, resumed
    print(f'  차선 변경 대기 {resumed:.1f} s 뒤 차선 주행 복귀 (차선 변경 {off:.2f} m 더 꺼 둠)')

    # (나) 전방 비상 정지: 오른쪽 앞 물체 → 왼쪽으로 비켜 가는 조향이 있으면 서행 회피
    # 치수는 실측값(PR #23: 라이다 x 0.214, 길이 0.33). 지금 robot.yaml(시뮬 값)은 비상 정지 거리 0.18 m가
    # 범퍼 앞 2 cm라 비켜 갈 원호가 없음 → 실측값 반영 전에는 (다)처럼 계속 정지
    robot = copy.deepcopy(robot)
    robot['lidar']['x'], robot['length'] = 0.214, 0.33
    driver = Driver({'answer': c, 'robot': robot})
    block = [(0.17, y) for y in np.linspace(-0.25, -0.085, 8)]
    t, v, crept = 0.0, 0.0, None
    while t < limit + 2.0:
        speed, steering, state = driver.update(lane_ok(), scan_with(front=0.17, points=block), [], t, v, t)
        if t < limit - 0.5:
            assert speed == 0.0, (t, speed)
        if crept is None and speed > 0:
            crept = (t, speed, steering)
        v = speed
        t += DT
    assert crept is not None and crept[1] == c['stall']['creep_speed'] and crept[2] > 0.05, crept
    print(f'  비상 정지 {crept[0]:.1f} s 뒤 {crept[1]} m/s·조향 {crept[2]:+.2f}로 회피')

    # (다) 정면 전체가 막힘: 비켜 갈 조향이 없으면 계속 정지 (밀고 가지 않음)
    driver = Driver({'answer': c, 'robot': robot})
    block = [(0.10, y) for y in np.linspace(-0.6, 0.6, 41)]
    t = 0.0
    while t < limit + 4.0:
        speed, _, _ = driver.update(lane_ok(), scan_with(front=0.10, points=block), [], t, 0.0, t)
        assert speed == 0.0, (t, speed)
        t += DT
    print('  정면 전체 막힘: 계속 정지')

    # (라) 의도한 정지(보행자 대기)는 대상 아님
    driver = Driver({'answer': c, 'robot': robot})
    driver.state = 'pedestrian_hold'
    for k in range(40):
        driver.update(lane_ok(), scan_with(), [], k * DT, 0.0, k * DT)
    assert driver.stall_time == 0.0
    print('멈춤 방지 시험 통과')


# ---------- 4. 서행·방지턱 ----------

def slow_run(sign=(0.8, 1.4), bump=None, stuck=None, corner=None, length=7.0, tilt=True, line=0.9, **override):
    """S0 직선 주행. sign: slow 표지판이 보이는 s 구간(None = 누락), line: 서행 시작 흰 가로선 위치 s(앞축 기준, None = 없음), bump: 앞바퀴가 턱에 닿는 s (자이로 y에 기울기 주입),
    stuck: (s, 초) 그 지점에서 명령 0.2 이하로는 구르지 않음, corner: 1번 코너 시작 s. 반환 [(s, 명령 속도, 조향, mode)]"""
    import yaml
    from ericar_driver.driver import Driver
    c = copy.deepcopy(ANSWER_CONFIG)
    c['slow_zone'].update(override)
    driver = Driver({'answer': c, 'robot': yaml.safe_load(open(CONFIG))['robot']})
    s = v = t = 0.0
    log, stuck_left = [], stuck[1] if stuck else 0.0
    while s < length and t < 120.0:
        names = ['slow'] if sign and sign[0] <= s < sign[1] else []
        boxes = [dict(name=n, truncated=False, area=0.05) for n in names]
        # 앞바퀴·뒷바퀴(축거 0.2 m)가 턱을 오르내림: 6°를 0.05 m 동안 올랐다가 0.05 m 동안 내려옴
        rate = 0.0
        for start, sign_ in ((bump, 1), (bump + 0.05, -1), (bump + 0.2, -1), (bump + 0.25, 1)) if bump is not None else ():
            if start <= s < start + 0.05:
                rate = sign_ * math.radians(6.0) / 0.05 * v
        yaw = -(math.pi / 2) / 1.0 * v if corner is not None and corner <= s < corner + 1.0 else 0.0
        lane = lane_ok()
        lane['stamp'] = t
        if line is not None and 0.4 <= line - s <= 1.4:   # 카메라가 보는 범위에서만 검출, 그 안쪽은 odom 추정
            lane['stop_distance'] = line - s
        if rate:
            lane['error'] = 0.3    # 턱 위 카메라 흔들림 → 차선 오차 튐
        speed, steering, mode = driver.update(lane, scan_with(), boxes, t, v, t, yaw_rate=yaw,
                                              tilt_rate=(0.0, rate) if tilt else None)
        log.append((s, speed, steering, mode))
        target = speed
        if stuck and s >= stuck[0] and stuck_left > 0 and speed <= 0.2:
            target = 0.0
            stuck_left -= DT
        v += (target - v) * 0.4
        s += v * DT
        t += DT
    return log, driver


def speed_at(log, s):
    return next(row[1] for row in log if row[0] >= s)


def test_slow_zone():
    z = ANSWER_CONFIG['slow_zone']
    base = ANSWER_CONFIG['base_speed']
    # (가) 표지판 → 감속, 방지턱(2.0 m) 감지 뒤 0.5 m에서 해제. 출발 직후는 기본 속도 (감속이 눈에 보여야 함)
    log, driver = slow_run(bump=2.0)
    assert speed_at(log, 0.5) > 0.3 and speed_at(log, 1.0) == z['speed'] and speed_at(log, 2.3) == z['speed']
    assert 2.0 <= driver.slow_zone.bump_at + 0.8 <= 2.1, driver.slow_zone.bump_at
    assert speed_at(log, 2.7) > 0.3 and 'slow' in driver.done
    assert max(row[1] for row in log if 0.85 <= row[0] < 2.45) <= 0.2
    # 턱 위 조향 고정: 차선 오차가 튀어도 조향은 직전 값(직선 0)
    assert max(abs(row[2]) for row in log if 2.0 <= row[0] < 2.45) < 0.02
    print('  표지판 + 방지턱 감지: 0.8 m 감속, %.2f m 해제' % next(r[0] for r in log if r[0] > 1.0 and r[1] > 0.3))
    # (나) 표지판 누락 → entry_distance에서 감속
    log, driver = slow_run(sign=None, bump=2.0)
    assert speed_at(log, 1.3) > 0.3 and speed_at(log, 1.6) == z['speed'] and speed_at(log, 2.7) > 0.3
    # (다) 방지턱 감지 실패(IMU 없음) → 1번 코너(S1)에서 해제, 그 전에는 서행 유지
    log, driver = slow_run(tilt=False, corner=3.0, length=5.0)
    assert speed_at(log, 2.9) == z['speed'] and driver.slow_zone.state == 'done' and 'slow' in driver.done
    assert driver.sections.index == 1
    # (라) 감지 실패 + 현장 실측 exit_distance → 그 거리에서 해제
    log, driver = slow_run(tilt=False, exit_distance=2.6)
    assert speed_at(log, 2.5) == z['speed'] and speed_at(log, 2.8) > 0.3
    # (마) 감지 실패 + 코너 없음 → max_distance에서 해제 (끝없이 서행하지 않음)
    log, driver = slow_run(tilt=False)
    assert speed_at(log, 0.8 + z['max_distance'] - 0.2) == z['speed'] and speed_at(log, 0.8 + z['max_distance'] + 0.3) > 0.3
    # (바) 턱에서 멈춤: 0.2 이하 명령으로는 3초 동안 안 구름 → 단계적으로 올려 넘어감, 넘은 뒤 서행 속도 복귀
    log, driver = slow_run(bump=2.0, stuck=(2.0, 3.0))
    peak = max(row[1] for row in log if 1.95 <= row[0] < 2.2)
    assert 0.2 < peak <= z['boost_speeds'][-1], peak   # 차선 주행 속도(턱 위 오차로 감속)보다 높이지는 않음
    assert log[-1][0] >= 6.9 and speed_at(log, 2.35) == z['speed'] and 'slow' in driver.done
    # (사) 감속 직후 차체 숙임(3°)은 방지턱으로 보지 않음: arm_distance 안
    log, driver = slow_run(bump=0.75, tilt_threshold=2.0, length=2.0)
    assert driver.slow_zone.bump_at is None or driver.slow_zone.bump_at >= z['arm_distance']
    # (아) entry_distance 없음(None) + 표지판 누락 → 서행 안 함 (거리 진입을 끈 설정 확인)
    log, driver = slow_run(sign=None, length=3.0, entry_distance=None)
    assert min(row[1] for row in log if row[0] > 0.5) > 0.3
    # (자) 출발 위치에서 표지판이 이미 보임 + 흰 선 1.5 m: 선 전에는 기본 속도, 선부터 서행 (11/16 main_1116_1605·1614)
    log, driver = slow_run(sign=(0.0, 2.0), line=1.5, length=3.0)
    assert speed_at(log, 1.0) > 0.3 and speed_at(log, 1.2) > 0.3 and speed_at(log, 1.7) == z['speed']
    # (차) 표지판 확정이 선보다 늦음: 확정 즉시 서행
    log, driver = slow_run(sign=(1.2, 2.0), line=0.9, length=3.0, entry_distance=None)
    assert speed_at(log, 1.1) > 0.3 and speed_at(log, 1.45) == z['speed']
    # (카) entry_at_line 끔: 예전처럼 표지판 확정만으로 서행
    log, driver = slow_run(sign=(0.0, 2.0), line=1.5, length=3.0, entry_at_line=False)
    assert speed_at(log, 0.6) == z['speed']
    assert base > 2 * z['speed'] - 1e-9 and z['speed'] <= 0.2
    print('서행·방지턱 시험 통과')


def test_cone_hold():
    """라바콘 통과 중 오른쪽으로 80° 돌아도 S3·라바콘 모드 유지, 끝난 뒤 4번 코너에서 S4 (11/13)"""
    import yaml
    from ericar_driver.driver import Driver
    robot = yaml.safe_load(open(CONFIG))['robot']

    def run(hold):
        c = copy.deepcopy(ANSWER_CONFIG)
        c['sections']['off'] = []
        c['missions']['start_stage'] = 'cones'
        driver = Driver({'answer': c, 'robot': robot})
        cones = driver.cones
        # 라바콘 담당 코드 대신 가짜: 켜져 있는 동안 고정 명령, 구간이 허용하지 않으면 꺼짐 (실제 update와 같은 규칙)
        def fake(lane, scan, velocity, yaw_rate, dt, enabled, red_only=False, holding=False):
            if not enabled or cones.done:
                cones.active = False
                return None
            return (0.2, -0.3) if cones.active else None
        cones.update = fake
        cones.active = hold
        t, log = 0.0, []
        # 1.6 s 동안 오른쪽 80° → 0.5 s 직진 → 1.6 s 동안 왼쪽 80° (콘 통로), 그 뒤 라바콘 끝 → 직진 1 s → 4번 코너 90° → 직진
        for seconds, turn, in_cones in [(1.6, -80, True), (0.5, 0, True), (1.6, 80, True),
                                        (1.0, 0, False), (3.0, -90, False), (1.0, 0, False)]:
            if not in_cones and cones.active:
                cones.active, cones.done = False, True
            for _ in range(int(round(seconds/DT))):
                _, _, mode = driver.update(lane_ok(), scan_with(), [], t, 0.3, t,
                                           yaw_rate=math.radians(turn)/seconds)
                log.append((in_cones, driver.sections.index, mode))
                t += DT
        return log

    log = run(True)
    assert all(index == 3 and mode == 'cones' for inside, index, mode in log if inside), '라바콘 도중 구간 전환'
    assert log[-1][1] == 4 and [i for _, i, _ in log].count(4) < 2.5/DT, '라바콘 뒤 4번 코너에서 S4 전환 안 됨'
    print('라바콘 중 구간 전환 보류 통과')


if __name__ == '__main__':
    test_section_manager()
    test_cone_hold()
    test_stall_guard()
    test_slow_zone()
    test_missing_signs()
    test_detector_loss()
    print('구간 관리자·멈춤 방지 시험 통과')
