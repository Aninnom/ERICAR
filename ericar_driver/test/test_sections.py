"""구간 관리자·멈춤 방지 시험 (맥·차, ROS 불필요)

1) section.py 단독: 코너 4개, 갈림길 꺾임, 차선 변경 흔들림, 자이로 치우침
2) 노드 전체 (scenario.py): 흰색 정지선 + 가짜 코너(IMU)로 보행자 → 갈림길 → 신호등 → 주차 진행,
   표지판 하나를 빼도 이후 미션이 동작하는지 (예전 미션 사슬은 이후 전부 미동작)
3) 멈춤 방지: 표지판 검출이 끊겨도 주행 계속, 신호등 대기 중 끊기면 포기 후 진행,
   차선 변경 대기·전방 비상 정지가 stall.seconds 넘으면 강제 전환

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


# ---------- 1. 구간 관리자 단독 ----------

def drive(manager, segments, speed=0.3, bias=0.0, scale=1.0):
    """segments: (거리 m, 회전 °, + 왼쪽) 목록을 speed로 주행. 구간 번호 기록 반환"""
    seen = [manager.index]
    for length, turn in segments:
        steps = max(1, int(round(length / speed / DT)))
        rate = math.radians(turn) / (steps * DT)
        for _ in range(steps):
            manager.update(rate * scale + bias, speed, DT)
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
    records = scenario.run(CONFIG, stop_paint=WHITE, drop=drop, corners=CORNERS, on_tick=on_tick, **kw)
    states = [r[2] for r in records if r[0] == 'status']
    return states, sections, records


def has(states, name):
    return any(s == name for s in states)


def test_missing_signs():
    states, sections, _ = full()
    print('  전체 표지판:', states)
    print('  구간 전환 (번호, s):', sections)
    assert [i for i, _ in sections] == [0, 1, 2, 3, 4]
    for name in ('pedestrian_hold', 'lane_change', 'traffic_hold', 'parking_sequence', 'parked'):
        assert has(states, name), f'{name} 없음: {states}'
    for drop, later in ((('slow',), ('pedestrian_hold', 'traffic_hold', 'parked')),
                        (('pedestrian',), ('lane_change', 'traffic_hold', 'parked')),
                        (('left',), ('traffic_hold', 'parked')),
                        (('traffic_red', 'traffic_green'), ('parked',))):
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
    assert has(states, 'traffic_hold') and has(states, 'parked')
    # 신호등 접근 중 검출이 끊겨 돌아오지 않음 (녹색을 볼 수 없음): 정지 후 stall.seconds 뒤 포기하고 주차까지 진행
    states, _, records = full(detector=True, detection_gap=(12.0, 16.0))
    print('  신호등 대기 중 검출 끊김:', states)
    status = [r for r in records if r[0] == 'status']
    hold = next(i for i, r in enumerate(status) if r[2] == 'traffic_hold')
    assert status[hold + 1][2] == 'lane', states
    waited = status[hold + 1][1] - status[hold][1]
    print(f'  신호등 정지 {waited:.1f} s 뒤 포기')
    assert waited < ANSWER_CONFIG['stall']['seconds'] + 1.0, waited
    assert has(states, 'parked'), states
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


if __name__ == '__main__':
    test_section_manager()
    test_stall_guard()
    test_missing_signs()
    test_detector_loss()
    print('구간 관리자·멈춤 방지 시험 통과')
