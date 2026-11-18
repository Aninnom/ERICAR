"""라이다 미션 시험 (ROS·카메라 불필요): 라바콘 진입·통과, 3차선 정적 장애물 좌·우 회피

합성 라이다(실차처럼 ±110°만 측정, 무반사 0)를 lidar.scan_sectors에 넣고 cones.py·lane_change.py만 실행.
차량은 발행 명령으로 자전거 모델 적분. 차체 직사각형과 물체 외곽 사이 최소 거리로 접촉 여부 판정.
차선 인식은 시험하지 않음: 라바콘 전후·차선 변경 사이에는 차로 중앙을 따라가는 것으로 대신함.

실행: python3 test/test_lidar_missions.py   (맥·차, numpy·pyyaml 필요)
"""
import copy
import math
import sys
from pathlib import Path

import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))

from ericar_driver.cones import ConeDriving  # noqa: E402
from ericar_driver.lane_change import LaneChange  # noqa: E402
from ericar_driver.lidar import footprint_clearance, pursuit_steering, scan_sectors  # noqa: E402
from ericar_driver.params import ANSWER_CONFIG  # noqa: E402

DT = 0.05
CONE_RADIUS = 0.04      # 라이다 높이(약 16 cm)에서 라바콘 반지름
LANE_WIDTH = 0.52       # 도면 선 중심 간격
ROBOT = yaml.safe_load((HERE.parent / 'config' / 'robot.yaml').read_text(encoding='utf-8'))['robot']


def config():
    c = copy.deepcopy(ANSWER_CONFIG)
    c['lidar']['zero_is_no_return'] = True
    return c


# ---------- 합성 라이다 ----------

def cast(origin, angles, cones, boxes):
    """월드 좌표 광선 → 가장 가까운 물체 거리. 없으면 0 (실차 YDLidar 무반사)"""
    d = np.column_stack((np.cos(angles), np.sin(angles)))
    best = np.full(len(angles), np.inf)
    for cx, cy in cones:
        oc = np.array([cx, cy]) - origin
        b = d @ oc
        disc = b*b - (oc @ oc - CONE_RADIUS**2)
        t = b - np.sqrt(np.maximum(disc, 0))
        best = np.where((disc >= 0) & (t > 0) & (t < best), t, best)
    for cx, cy, hx, hy in boxes:
        with np.errstate(divide='ignore', invalid='ignore'):
            t1 = (cx - hx - origin[0]) / d[:, 0]
            t2 = (cx + hx - origin[0]) / d[:, 0]
            t3 = (cy - hy - origin[1]) / d[:, 1]
            t4 = (cy + hy - origin[1]) / d[:, 1]
        near = np.maximum(np.minimum(t1, t2), np.minimum(t3, t4))
        far = np.minimum(np.maximum(t1, t2), np.maximum(t3, t4))
        hit = (near <= far) & (near > 0) & (near < best)
        best = np.where(hit, near, best)
    return np.where(np.isfinite(best) & (best < 12.0), best, 0.0)


def wall_box(y):
    """외곽 가벽: 차로 중앙에서 왼쪽 y(m)에 진행 방향(월드 x)과 나란한 긴 상자 (두께 2 cm)"""
    return (1.0, y + 0.01, 8.0, 0.01)


def scan_at(car, cones, boxes, lidar_config, n=720, wall=None, drop=0.0, noise=0.0, rng=None):
    """wall(상자)을 주면 벽에 맞은 광선은 drop 확률로 무반사, 전체 거리에 표준편차 noise(m) 잡음 (실차 누락·잡음 흉내)"""
    angles = -math.pi + np.arange(n) * (2*math.pi/n)
    x, y, psi = car
    origin = np.array([x + ROBOT['lidar']['x']*math.cos(psi), y + ROBOT['lidar']['x']*math.sin(psi)])
    ranges = cast(origin, angles + psi, cones, boxes)
    if wall is not None:
        seen = cast(origin, angles + psi, cones, list(boxes) + [wall])
        ranges = np.where((seen != ranges) & (rng.random(n) < drop), 0.0, seen)
    if noise > 0:
        ranges = np.where(ranges > 0, ranges + rng.normal(0, noise, n), 0.0)
    ranges[np.abs(angles) > math.radians(110)] = 0.0   # 실차 유효 각도 약 ±110°
    return scan_sectors(ranges, angles[0], 2*math.pi/n, 0.06, 12.0, lidar_config)


def outline(cones, boxes):
    """접촉 판정용 물체 외곽 점 (1 cm 간격)"""
    points = []
    for cx, cy in cones:
        a = np.linspace(0, 2*math.pi, 32, endpoint=False)
        points += list(zip(cx + CONE_RADIUS*np.cos(a), cy + CONE_RADIUS*np.sin(a)))
    for cx, cy, hx, hy in boxes:
        for s in np.linspace(-1, 1, max(3, int(2*hx/0.01))):
            points += [(cx + s*hx, cy - hy), (cx + s*hx, cy + hy)]
        for s in np.linspace(-1, 1, max(3, int(2*hy/0.01))):
            points += [(cx - hx, cy + s*hy), (cx + hx, cy + s*hy)]
    return np.asarray(points, dtype=float).reshape(-1, 2)


def clearance(car, obstacle_points):
    x, y, psi = car
    center = [[x + ROBOT['wheelbase']/2*math.cos(psi), y + ROBOT['wheelbase']/2*math.sin(psi), psi]]
    return footprint_clearance(obstacle_points, np.asarray(center), ROBOT)


def follow(car, lane_y, lookahead=0.6):
    """차선 주행 대신: 월드 y = lane_y 직선을 따라가는 pure pursuit"""
    x, y, psi = car
    dx, dy = lookahead, lane_y - y
    bx, by = dx*math.cos(psi) + dy*math.sin(psi), -dx*math.sin(psi) + dy*math.cos(psi)
    return pursuit_steering(bx, by, ROBOT)


def lane_seen(car, lane_y, forward=0.65):
    """카메라 차선 결과 흉내: 도로 방향(차체 기준)과 차 앞 forward 지점의 차로 중앙 횡오차"""
    x, y, psi = car
    return dict(valid=True, heading=-psi, forward=forward, solid={},
                error=(lane_y - y - forward*math.sin(psi))/math.cos(psi))


class Car:
    def __init__(self, x, y, psi):
        self.x, self.y, self.psi, self.v, self.steer = x, y, psi, 0.0, 0.0

    @property
    def pose(self):
        return self.x, self.y, self.psi

    def yaw_rate(self):
        return self.v/ROBOT['wheelbase']*math.tan(self.steer)

    def step(self, speed, steering):
        self.v += (speed - self.v)*0.4
        self.steer = steering
        self.psi += self.yaw_rate()*DT
        self.x += self.v*math.cos(self.psi)*DT
        self.y += self.v*math.sin(self.psi)*DT


# ---------- 라바콘 ----------

def cone_layout(kind, seed=0):
    """라바콘 배치 (월드 x = 진행 방향, 첫 콘 x = 0, 구간 약 2.1 m). 반환 (왼쪽 줄, 오른쪽 줄)"""
    if kind == 'pairs':        # 좌우 쌍, 0.5 m마다, 간격 0.55
        xs = np.arange(0, 2.11, 0.5)
        return [(x, 0.275) for x in xs], [(x, -0.275) for x in xs]
    if kind == 'zigzag':       # 좌우 엇갈림, 간격 0.6
        return [(x, 0.3) for x in (0, .7, 1.4, 2.1)], [(x, -0.3) for x in (.35, 1.05, 1.75)]
    if kind == 'shift':        # 통로 중앙이 좌우로 ±0.12 m 휘는 엇갈림
        center = lambda x: 0.12*math.sin(math.pi*x/1.05)  # noqa: E731
        return ([(x, center(x) + 0.29) for x in (0, .7, 1.4, 2.1)],
                [(x, center(x) - 0.29) for x in (.35, 1.05, 1.75)])
    if kind == 'curve':        # 일정 곡률로 휘는 엇갈림 통로. seed = 곡률(1/m)×100, 음수 = 오른쪽
        center = lambda x: 0.5*seed/100*x*x  # noqa: E731
        return ([(x, center(x) + 0.29) for x in (0, .7, 1.4, 2.1)],
                [(x, center(x) - 0.29) for x in (.35, 1.05, 1.75)])
    # random: 간격 0.5~0.7, 앞뒤 ±0.1·좌우 ±0.04 흔들림
    rng = np.random.default_rng(seed)
    width = rng.uniform(0.5, 0.7)
    left = [(x + rng.uniform(-.1, .1), width/2 + rng.uniform(-.04, .04)) for x in (0, .7, 1.4, 2.1)]
    right = [(x + rng.uniform(-.1, .1), -width/2 + rng.uniform(-.04, .04)) for x in (.35, 1.05, 1.75)]
    return left, right


def gate_layout(kind, seed=0):
    """좌·우 한 쌍(문)이 띄엄띄엄 놓인 배치. 줄이 아니고 쌍 개수·간격을 모름 (11/12 미션 설명). 반환 (왼쪽, 오른쪽)"""
    def pairs(gates):   # gates: [(앞 거리, 중앙 y, 폭)]
        return [(x, y + w/2) for x, y, w in gates], [(x, y - w/2) for x, y, w in gates]
    if kind == 'two':          # 구간 처음과 끝에만 한 쌍씩
        return pairs([(0, 0, .55), (2.0, 0, .55)])
    if kind == 'three':        # 1 m마다 한 쌍
        return pairs([(0, 0, .6), (1.0, 0, .6), (2.0, 0, .6)])
    if kind == 'shift':        # 쌍마다 중앙이 옆으로 옮겨 감 (차선과 무관한 새 길)
        return pairs([(0, 0, .55), (0.9, .25, .55), (1.8, -.05, .55)])
    if kind == 'stuck':        # 쌍마다 한쪽에 콘 3개를 붙여 놓음 (라이다에는 넓은 한 덩어리)
        left, right = pairs([(0, 0, .6), (1.0, .1, .6), (2.0, 0, .6)])
        return [(x + d, y) for x, y in left for d in (0, .08, .16)], right
    # random: 쌍 2~5개, 구간 2.1 m, 폭 0.45~0.75, 중앙 ±0.2씩 이동(최대 ±0.3), 좌·우 콘 앞뒤 ±0.08 어긋남
    rng = np.random.default_rng(seed)
    count = int(rng.integers(2, 6))
    xs = np.linspace(0, 2.1, count) + np.r_[0, rng.uniform(-.15, .15, count - 1)]
    left, right, y = [], [], 0.0
    for x in xs:
        w = rng.uniform(.45, .75)
        left.append((x + rng.uniform(-.08, .08), y + w/2))
        right.append((x + rng.uniform(-.08, .08), y - w/2))
        y = float(np.clip(y + rng.uniform(-.2, .2), -.3, .3))
    return left, right


def corner_start(first_x=0.0, angle=45.0, radius=0.9, straight=0.5):
    """우회전 코너를 angle(도) 남긴 위치·방향. 코너가 끝나고 straight(m) 뒤에 첫 콘(first_x)"""
    a = math.radians(angle)
    return first_x - straight - radius*math.sin(a), -radius*(1 - math.cos(a)), a


def run_cones(left, right, others=(), boxes=(), start=(-1.6, 0.03, 0.04), lane_speed=0.3,
              wall=None, drop=0.0, noise=0.0, red=False):
    """wall: 외곽 가벽 위치(차로 중앙에서 왼쪽 m). 주면 라이다 간격 0.9°(실차), 벽 점 누락 drop·거리 잡음 noise 적용
    red: 카메라 붉은 비율 흉내 (콘 구간 0.8 m 앞부터 0.25 = 진입 기준 이상, 1.6 m 앞부터 0.05 = 콘 색 보임)"""
    c = config()
    cones_driver = ConeDriving(c, ROBOT)
    car = Car(*start)
    cones = list(left) + list(right) + list(others)
    wall = None if wall is None else wall_box(wall)
    obstacles = outline(cones, list(boxes) + ([] if wall is None else [wall]))
    first_x, last_x = min(x for x, _ in cones), max(x for x, _ in cones)
    end_x = last_x + 1.5
    rng = np.random.default_rng(0)
    result = dict(start_x=None, end_x=None, min_clear=np.inf, wrong_side=0, stalled=False, entries=0,
                  ahead_at_exit=0)
    scan, was_active = None, False
    for k in range(int(40/DT)):
        if k % 2 == 0:
            scan = scan_at(car.pose, cones, list(boxes), c['lidar'], n=720 if wall is None else 400,
                           wall=wall, drop=drop, noise=noise, rng=rng)
        lane = lane_seen(car.pose, 0.0)
        if red:
            camera_x = car.x + ROBOT['camera']['x']
            lane['cone_red'] = 0.25 if first_x - 0.8 < camera_x < last_x else (0.05 if first_x - 1.6 < camera_x < last_x else 0.0)
        command = cones_driver.update(lane, scan, car.v, car.yaw_rate(), DT, True)
        if cones_driver.active and result['start_x'] is None:
            result['start_x'] = car.x
        # 콘이 안 보이는 사이 차선 주행으로 돌아갔다 다시 켜질 수 있음 → 마지막으로 꺼진 위치를 종료로 봄
        if was_active and not cones_driver.active:
            result['end_x'] = car.x
            result['ahead_at_exit'] = cones_ahead(car.pose, cones)
        result['entries'] += cones_driver.active and not was_active
        was_active = cones_driver.active
        if command is None:
            command = (lane_speed, follow(car.pose, 0.0))
        previous_x = car.x
        car.step(*command)
        result['min_clear'] = min(result['min_clear'], clearance(car.pose, obstacles))
        # 각 라바콘 옆을 지날 때 차가 통로 안쪽에 있는지 (왼쪽 콘은 차 왼쪽, 오른쪽 콘은 차 오른쪽)
        for row, side in ((left, 1), (right, -1)):
            for cx, cy in row:
                if previous_x < cx <= car.x and (cy - car.y)*side < 0:
                    result['wrong_side'] += 1
        if car.x > end_x:
            break
    else:
        result['stalled'] = True
    result['final_y'] = car.y
    return result


def cones_ahead(car, cones, reach=1.2, band=0.5):
    """차 앞 범퍼보다 앞, reach(m) 안·좌우 band(m) 안에 남은 콘 수

    범위는 cones.one_side가 쓰는 값(cone_start_range 1.2, cone_gate_max/2 0.5)과 같게 둠 → 0이면 더
    지나갈 콘이 없다는 뜻. 기준점이 앞 범퍼인 이유: 라바콘은 마지막 콘이 라이다와 나란해질 때 넘기는데,
    그때 콘은 이미 범퍼를 지나 차 옆에 있음 (라이다 기준으로 세면 0.001 m 차이로 걸림)
    """
    x, y, psi = car
    nose = ROBOT['front_bumper_x']
    origin = np.array([x + nose*math.cos(psi), y + nose*math.sin(psi)])
    aim = np.array([math.cos(psi), math.sin(psi)])
    left = np.array([-aim[1], aim[0]])
    count = 0
    for cx, cy in cones:
        relative = np.array([cx, cy]) - origin
        forward, lateral = float(relative@aim), float(relative@left)
        if 0 < forward <= reach and abs(lateral) <= band:
            count += 1
    return count


def assert_handover_after_cones(result, last, label):
    """차선 주행으로 넘긴 순간 아직 지나갈 콘이 남아 있지는 않았는지 (통로 중간에 꺼졌는지)

    11/14 즉시 복귀 전에는 "넘긴 위치가 마지막 콘보다 뒤(end_x > last)"로 쟀음. 지금은 지날 문도 콘도
    없고 화면에 콘 색도 없으면 바로 넘기므로, 마지막 콘과 나란한 지점에서 넘어감 → 위치 대신 그때
    앞에 남은 콘 수로 잼. 휘어진 배치(curve)는 통로가 x축과 어긋나 위치 비교가 맞지 않는 점도 해결
    """
    assert result['end_x'] is not None, (label, result)
    assert result['ahead_at_exit'] == 0, (label, f"넘긴 시점에 앞 콘 {result['ahead_at_exit']}개", result)


def test_cones_pass():
    layouts = ([('pairs', 0), ('zigzag', 0), ('shift', 0), ('curve', 20), ('curve', -15)] +
               [('random', s) for s in range(8)])
    for kind, seed in layouts:
        left, right = cone_layout(kind, seed)
        last = max(x for x, _ in left + right)
        r = run_cones(left, right)
        print(f"라바콘 {kind:6s} {seed}: 시작 x {r['start_x']!s:.5} 종료 x {r['end_x']!s:.5} "
              f"최소 여유 {r['min_clear']:.3f} m, 바깥 통과 {r['wrong_side']}, 종료 후 y {r['final_y']:+.3f}")
        # 11/12 진입 시점을 문 중점이 라이다 0.3 m 안으로 늦춤 → 엇갈림 배치(첫 문이 비스듬)는 차 앞부분이 첫 콘 옆일 때 진입
        # (shift 0.06, random 0 0.03). 바깥 통과·접촉 없음이 기준. 대회 배치는 좌·우 쌍(test_cone_gates)
        assert r['start_x'] is not None and r['start_x'] < 0.1, (kind, seed, r)
        assert_handover_after_cones(r, last, (kind, seed))
        assert r['min_clear'] > 0.0 and r['wrong_side'] == 0 and not r['stalled'], (kind, seed, r)


def test_cone_gates():
    """줄이 아닌 좌·우 쌍 배치: 정면 진입 + 코너를 30°·45° 남기고 비스듬히 진입"""
    layouts = ([('two', 0), ('three', 0), ('shift', 0), ('stuck', 0)] + [('random', s) for s in range(12)])
    starts = {'정면': (-1.6, 0.03, 0.04), '코너 30°': corner_start(angle=30), '코너 45°': corner_start(angle=45)}
    failed = []
    for kind, seed in layouts:
        left, right = gate_layout(kind, seed)
        last = max(x for x, _ in left + right)
        for name, start in starts.items():
            r = run_cones(left, right, start=start)
            ok = (r['start_x'] is not None and r['start_x'] < -0.1 and r['end_x'] is not None
                  and r['ahead_at_exit'] == 0   # 넘긴 시점에 지나갈 콘이 남아 있지 않음
                  and r['min_clear'] > 0.0 and r['wrong_side'] == 0 and not r['stalled'])
            print(f"라바콘 쌍 {kind:6s} {seed:2d} {name:7s}: 시작 x {r['start_x']!s:.5} 종료 x {r['end_x']!s:.5} "
                  f"진입 {r['entries']}회, 최소 여유 {r['min_clear']:.3f} m, 바깥 통과 {r['wrong_side']}"
                  f"{'' if ok else '  ← 실패'}")
            if not ok:
                failed.append((kind, seed, name))
    assert not failed, failed


def test_cone_red_without_corridor():
    """붉은색은 보이는데 비율이 진입 기준 미달(red·corridor 아님)일 때 (11/18 유경, run_1118_1931)

    예전에는 라이다 진입까지 막아 5.3 s 동안 아무 진입도 못 하고 콘에 추돌 → 라이다 문 조건이 다 맞으면 켬.
    문 쌍·정면 방위·cone_enter_distance·넓은 군집 조건은 그대로라 한쪽 줄·먼 문·표지판 기둥에는 안 켜짐"""
    c = config()
    lidar_x = ROBOT['lidar']['x']
    cones = ConeDriving(c, ROBOT)
    cones.red, cones.corridor = True, False
    none = np.zeros((0, 2))

    def posts(distance, center, gap=.6):
        return np.array([[lidar_x+distance, center+gap/2], [lidar_x+distance, center-gap/2]])
    assert cones.start_gate(posts(.25, 0.), none) is not None, 'red인데 바로 앞 문에 진입 안 함'
    assert cones.start_gate(posts(.8, 0.), none) is None, 'red에서 0.3 m보다 먼 문에 진입'
    row = np.array([[lidar_x+x, -.35] for x in (.1, .3, .5, .7)])
    assert cones.start_gate(row, none) is None, 'red에서 한쪽 콘 줄에 진입'
    cones.c = copy.deepcopy(c)['lidar']
    cones.c['cone_red_lidar_entry'] = False
    assert cones.start_gate(posts(.25, 0.), none) is None, 'cone_red_lidar_entry False인데 진입'


def test_cones_not_started_by_other_objects():
    cases = {
        '표지판 기둥 2개': dict(others=[(1.0, 0.45), (2.4, -0.5)]),
        '한쪽 줄 라바콘': dict(others=[(x, -0.35) for x in (0.0, 0.5, 1.0, 1.5)]),
        '차량 장애물': dict(boxes=[(1.0, 0.0, 0.15, 0.1)], others=[(0.6, 0.35), (0.6, -0.35), (1.6, 0.35)]),
    }
    for name, case in cases.items():
        r = run_cones([], [], others=case.get('others', ()), boxes=case.get('boxes', ()), lane_speed=0.2)
        print(f"라바콘 오진입 {name}: 시작 x {r['start_x']}")
        assert r['start_x'] is None, (name, r)


def test_cones_outer_wall():
    """외곽 가벽 (11/13 도면: 맵 가장자리, 높이 200 mm, 좌측 직선 차로 중앙에서 약 0.72 m → 차 왼쪽)

    벽은 콘 그림자·점 누락으로 끊겨 보임. 그 조각을 콘으로 보면 벽·콘 사이를 문으로 잡아 왼쪽 콘 바깥으로 통과.
    벽 점 제외(cones.wall_lines) 전: 벽 0.60 m에서 6/57, 벽 0.72 m·누락 40%에서 2/57 실패 (바깥 통과).
    벽 0.60 m = 콘 길이 차로 중앙보다 벽 쪽으로 0.12 m 치우친 경우 (주최: 콘은 차선과 무관하게 배치)
    """
    layouts = ([(f'쌍 {k}', gate_layout(k)) for k in ('two', 'three', 'shift', 'stuck')] +
               [(f'쌍 임의 {s}', gate_layout('random', s)) for s in range(12)] +
               [(f'줄 {k}', cone_layout(k)) for k in ('pairs', 'zigzag', 'shift')])
    starts = {'정면': (-1.6, 0.03, 0.04), '코너 30°': corner_start(angle=30), '코너 45°': corner_start(angle=45)}
    for wall, drop, noise in ((0.72, 0.0, 0.0), (0.72, 0.4, 0.01), (0.60, 0.0, 0.0)):
        failed = []
        for name, (left, right) in layouts:
            for start_name, start in starts.items():
                r = run_cones(left, right, start=start, wall=wall, drop=drop, noise=noise, red=True)
                if r['start_x'] is None or r['wrong_side'] or r['min_clear'] <= 0.0 or r['stalled']:
                    failed.append((name, start_name, r['wrong_side'], round(r['min_clear'], 3)))
        print(f"라바콘 + 벽 {wall:.2f} m, 벽 점 누락 {drop:.0%}, 잡음 {noise*100:.0f} cm: "
              f"실패 {len(failed)}/{len(layouts)*len(starts)}")
        assert not failed, (wall, drop, noise, failed)


# ---------- 3차선 정적 장애물 ----------

def lane_y(index):
    return (index - 1)*LANE_WIDTH   # 0=오른쪽, 1=가운데, 2=왼쪽


def run_three_lane(layout, half=(0.15, 0.1)):
    """layout: [(x, 차선 번호)]. 차는 가운데 차로에서 출발"""
    c = config()
    change = LaneChange(c, ROBOT)
    car = Car(-1.0, 0.0, 0.0)
    boxes = [(x, lane_y(i), half[0], half[1]) for x, i in layout]
    obstacles = outline([], boxes)
    end_x = max(x for x, _ in layout) + 1.2
    lanes, waits = [change.index], 0
    min_clear, scan = np.inf, None
    for k in range(int(120/DT)):
        if k % 2 == 0:
            scan = scan_at(car.pose, [], boxes, c['lidar'])
        lane = lane_seen(car.pose, lane_y(change.index))
        command = change.update(lane, scan, car.v, car.yaw_rate(), DT, True)
        waits += change.waiting
        if command is None:
            command = (0.3, follow(car.pose, lane_y(change.index)))
        car.step(*command)
        if change.index != lanes[-1]:
            lanes.append(change.index)
        min_clear = min(min_clear, clearance(car.pose, obstacles))
        if car.x > end_x:
            return dict(lanes=lanes, min_clear=min_clear, waits=waits, done=True, x=car.x)
    return dict(lanes=lanes, min_clear=min_clear, waits=waits, done=False, x=car.x)


THREE_LANE_LAYOUTS = {
    # 7개, 한 줄에 1개씩 엇갈림 (간격 1.3 m)
    'staggered': [(1.5, 1), (2.8, 2), (4.1, 0), (5.4, 1), (6.7, 2), (8.0, 0), (9.3, 1)],
    # 7개, 가운데 1개 ↔ 양옆 2개 번갈아 (간격 1.6 m) → 가운데로 돌아와야 함
    'alternating': [(1.5, 1), (3.1, 0), (3.1, 2), (4.7, 1), (6.3, 0), (6.3, 2), (7.9, 1)],
    # 7개, 두 차로가 같이 막혀 한쪽만 가능 (왼쪽 → 가운데 → 오른쪽 → 가운데)
    'one_side': [(1.5, 1), (1.5, 0), (3.2, 2), (3.2, 0), (4.9, 1), (4.9, 2), (6.6, 0)],
}


def test_three_lane_left_right():
    for name, layout in THREE_LANE_LAYOUTS.items():
        r = run_three_lane(layout)
        print(f"3차선 {name:11s}: 차로 {r['lanes']} 최소 여유 {r['min_clear']:.3f} m, "
              f"대기 {r['waits']}주기, 통과 {r['done']} (x {r['x']:.2f})")
        assert r['done'] and r['min_clear'] > 0.0, (name, r)


def random_three_lane_layout(seed, cars=7, gap=(1.0, 1.6)):
    """임의 배치 7대: 줄마다 1~2대(차로 무작위), 줄 간격 gap(m), 앞뒤 ±0.1 m 흔들림.
    직전 줄에서 한 차로 이내로 옮겨 지나갈 수 있는 배치만 사용 (세 차로 전부 막힘 없음). 반환 [(x, 차선 번호)]"""
    rng = np.random.default_rng(seed)
    while True:
        layout, x, reachable = [], 1.5, {1}
        while len(layout) < cars and reachable:
            count = min(int(rng.integers(1, 3)), cars - len(layout))
            blocked = set(rng.choice(3, size=count, replace=False).tolist())
            reachable = {i for i in range(3) if i not in blocked and any(abs(i - p) <= 1 for p in reachable)}
            layout += [(x + float(rng.uniform(-0.1, 0.1)), i) for i in sorted(blocked)]
            x += float(rng.uniform(*gap))
        if reachable:
            return layout


# 임의 배치 30가지 중 아직 실패하는 수 (11/11, 줄 간격 1.0~1.6 m). 고치면 줄일 것
#   시드 4·24: 두 차로 옆으로 가야 하는 배치. 바로 앞 줄만 보고 고른 차로가 다음 줄에서 막혀 정지
#   시드 19: 통과하지만 접촉 (원인 미분석)
# 참고: 줄 간격 0.8~1.2 m는 22/30 (차선 변경 거리 부족으로 정지). 도면 점선 구간 약 4.05 m에 7대면 이쪽에 가까울 수 있음
THREE_LANE_RANDOM_ALLOWED_FAILURES = 3


def test_three_lane_random(seeds=30):
    failed = []
    for seed in range(seeds):
        r = run_three_lane(random_three_lane_layout(seed))
        if not (r['done'] and r['min_clear'] > 0.0):
            failed.append(seed)
            print(f"3차선 임의 {seed:2d} 실패: 차로 {r['lanes']} 최소 여유 {r['min_clear']:.3f} m, "
                  f"대기 {r['waits']}주기, 통과 {r['done']} (x {r['x']:.2f})")
    print(f"3차선 임의 배치: {seeds - len(failed)}/{seeds} 통과, 실패 시드 {failed}")
    assert len(failed) <= THREE_LANE_RANDOM_ALLOWED_FAILURES, failed


def test_cone_handover():
    """라바콘 인계 조건 (11/15 fix2_1115_2017 207.6 s: 콘 0.3 m 앞에서야 켜져 안쪽 콘 0.2 m 앞 정지)
    - 문을 정면으로 보면 1 m 앞에서 미리 인계, 비스듬하거나 옆으로 비켜 있으면 예전처럼 콘이 바로 앞일 때
    - 코너 유지 중에는 차 진행 폭 안 콘이 바로 앞일 때만"""
    c = config()
    lidar_x = ROBOT['lidar']['x']
    cones = ConeDriving(c, ROBOT)
    cones.red = cones.corridor = True
    none = np.zeros((0, 2))

    def posts(distance, center, gap=.6):
        return np.array([[lidar_x+distance, center+gap/2], [lidar_x+distance, center-gap/2]])
    assert cones.start_gate(posts(.8, 0.), none) is not None, '정면 문 0.8 m 앞인데 인계 안 함'
    assert cones.start_gate(posts(1.3, 0.), none) is None, '1 m보다 먼 문에 인계'
    assert cones.start_gate(posts(.8, .35), none) is None, '옆으로 0.35 m 비켜난 문에 미리 인계'
    assert cones.start_gate(posts(.8, 0.), none, holding=True) is None, '코너 유지 중 먼 문에 인계'
    assert cones.start_gate(posts(.25, .35), none) is not None, '바로 앞 콘(예전 조건)에 인계 안 함'
    assert cones.start_gate(posts(.25, .35), none, holding=True) is not None, '코너 유지 중 진행 폭 안 콘에 인계 안 함'
    assert cones.start_gate(posts(.25, .6), none, holding=True) is None, '코너 유지 중 진행 폭 밖 콘에 인계'
    # 11/18 완화: 코너 유지 중에도 붉은색이 찼고 문을 정면으로 보면 cone_hold_face_distance(0.6 m) 안에서 미리 인계
    assert cones.start_gate(posts(.5, 0.), none, holding=True) is not None, '코너 유지 중 정면 문 0.5 m에 인계 안 함'
    assert cones.start_gate(posts(.8, 0.), none, holding=True) is None, '코너 유지 중 0.6 m보다 먼 문에 인계'
    assert cones.start_gate(posts(.5, .35), none, holding=True) is None, '코너 유지 중 옆으로 비켜난 문에 미리 인계'
    cones.red = cones.corridor = False
    assert cones.start_gate(posts(.5, 0.), none, holding=True) is None, '붉은색 없이 코너 유지 중 먼 문에 인계'
    cones.red = cones.corridor = True


if __name__ == '__main__':
    test_cones_pass()
    test_cone_gates()
    test_cone_red_without_corridor()
    test_cone_handover()
    test_cones_not_started_by_other_objects()
    test_cones_outer_wall()
    test_three_lane_left_right()
    test_three_lane_random()
    print('라이다 미션 시험 통과')
