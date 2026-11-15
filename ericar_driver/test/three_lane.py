"""3차선 정적 장애물·갈림길 합성 시험 (차 없이 판단 로직 확인용)

직선 도로: s < ZONE_START는 1차로(노란 선 ±0.26), ZONE_START 이후는 3차로 (ZONE_END 지나면 시험 끝)
(노란 바깥선 ±0.78, 흰 점선 ±0.26, 노란 선 바깥 흰 테두리 ±0.95). y + = 왼쪽, 차로 번호 0=오른쪽·1=가운데·2=왼쪽
장애물 = LIMO 크기 상자(0.22×0.32). 라이다는 상자에 광선 투사. 표지판 검출은 거리 구간으로 주입
차량은 발행 명령으로 자전거 모델 적분. /imu는 실제 회전, odom 회전은 실측 오차를 흉내 내도록 odom_yaw_scale배 (11/9 측정 1.07)

구간 관리자 기준 S2(3차선·갈림길)에서 출발 (start_stage = fork)
실행: python3 test/three_lane.py [사례 이름 ...] [-v]   (생략 시 전체, -v는 0.5초마다 상태 출력)
판정: 충돌 없음, 3차로 바깥선(±0.78) 밖으로 차체 중심이 나가지 않음, 3차로 끝까지 도달
"""
import json
import math
import sys
import tempfile
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent))

import fake_ros  # noqa: E402
import scenario  # noqa: E402

DT = 0.05
W, H = scenario.W, scenario.H
ZONE_START, ZONE_END = 3.5, 11.0
DASH_ON, DASH_OFF = 0.2, 0.2
LINE = 0.02
CAR_W, CAR_L = 0.22, 0.32
STOP_LINE = 2.0          # 보행자 정지선 (사례에서 pedestrian 표지판을 줄 때만 의미)
WHITE_BGR = np.uint8([235, 235, 235])

CASES = {
    # 장애물 3개 → 갈림길 왼쪽
    'normal': dict(signs={'left': (7.6, 8.4)}, obstacles=[(5.0, 0.0), (6.9, 0.52), (8.6, 0.0)]),
    # 장애물 피하기 전에 갈림길 표지판을 먼저 읽음 → 표지판 쪽(오른쪽)으로 피해야 함
    'sign_first': dict(signs={'right': (3.0, 3.8)}, obstacles=[(5.0, 0.0)]),
    # YOLO 끔(--yolo false)과 같은 조건: 표지판 없음
    'no_signs': dict(signs={}, obstacles=[(5.0, 0.0), (6.9, 0.52), (8.6, 0.0)]),
    # 오른쪽 갈림길: 왼쪽으로 피한 뒤 오른쪽 끝 차로까지 두 칸 이동 필요
    'fork_right': dict(signs={'right': (7.6, 8.4)},
                       obstacles=[(5.0, 0.0)]),
    # 가운데 막힘 + 왼쪽 막힘: 오른쪽으로 피해야 함
    'left_blocked': dict(signs={}, obstacles=[(5.0, 0.0), (5.6, 0.52)]),
    # 가운데·왼쪽 막힘, 오른쪽은 2 m 앞에 또 장애물 → 오른쪽으로 피한 뒤 가운데로 복귀
    # (장애물 사이 1.1 m는 실패: 한 차로 이동에 약 1.6 m 필요 → 변경 취소 후 대기. 실제 배치 확인 필요)
    'tight': dict(signs={}, obstacles=[(5.0, 0.0), (5.4, 0.52), (7.0, -0.52)]),
    # 처음부터 왼쪽 차로에서 출발 (차로 번호 초기값 1과 다름) → 색으로 보정해 바깥으로 안 나가야 함
    'start_left': dict(signs={}, obstacles=[(5.3, 0.52)], start_y=0.52, start_zone=True),
    # IMU 없음(끊김): odom 회전(실측 1.07배)만으로 차선 변경 → 카메라 종료·지나침 방지로 바깥선 밖으로 안 나가야 함
    # 11/12 실차: 가운데·왼쪽 막힘 → 오른쪽 끝(0)으로 피함 → 왼쪽 표지판 → 두 칸 연속 이동(0→1→2) 뒤 노란 바깥선 이탈
    'right_then_left': dict(signs={'left': (6.6, 7.4)}, obstacles=[(5.0, 0.0), (5.6, 0.52)]),
    # 반대: 가운데 막힘 → 왼쪽(2)으로 피함 → 오른쪽 표지판 → 2→1→0
    'left_then_right': dict(signs={'right': (6.6, 7.4)}, obstacles=[(5.0, 0.0)]),
    'no_imu': dict(signs={}, obstacles=[(5.0, 0.0), (6.9, 0.52)], imu=False),
    # 벽 없음 + 왼쪽 차로에 바깥선 쪽으로 붙은 차 3대 줄: 줄을 벽으로 보고 지우면 막힌 왼쪽으로 변경 → 정지 (11/14)
    'obstacle_row': dict(signs={}, obstacles=[(5.0, 0.0), (5.2, 0.65), (6.0, 0.65), (6.8, 0.65)]),
}


def paint_lines(ws, wl, finite):
    """반환: (노랑 마스크, 흰 마스크)"""
    zone = ws >= ZONE_START
    yellow = np.zeros(ws.shape, bool)
    white = np.zeros(ws.shape, bool)
    yellow |= ~zone & (np.abs(np.abs(wl) - 0.26) < LINE / 2)
    yellow |= zone & (np.abs(np.abs(wl) - 0.78) < LINE / 2)
    dash = np.mod(ws - ZONE_START, DASH_ON + DASH_OFF) < DASH_ON
    white |= zone & dash & (np.abs(np.abs(wl) - 0.26) < LINE / 2)
    white |= np.abs(np.abs(wl) - 0.95) < LINE                      # 트랙 밖 흰 테두리 (실선)
    yellow &= finite
    white &= finite
    return yellow, white


def render(camera, s, y, psi, case):
    v, u = np.indices((H, W), dtype=np.float64)
    focal = W / (2 * math.tan(math.radians(camera['hfov_deg']) / 2))
    p, z, cx = camera['pitch'], camera['z'], camera['x']
    t = (H / 2 - v) / focal
    denom = math.sin(p) - t * math.cos(p)
    with np.errstate(divide='ignore', invalid='ignore'):
        dx = np.where(denom > 1e-6, z * (math.cos(p) + t * math.sin(p)) / denom, np.nan)
    dx = np.where(np.isfinite(dx) & (dx > 0), dx, np.nan)
    forward = cx + dx
    depth = math.cos(p) * dx + math.sin(p) * z
    left = (W / 2 - u) * depth / focal - camera.get('y', 0.0)
    ws = s + forward * math.cos(psi) - left * math.sin(psi)
    wl = y + forward * math.sin(psi) + left * math.cos(psi)
    finite = np.isfinite(forward) & (forward < 4.0)
    yellow, white = paint_lines(ws, wl, finite)
    if case.get('stop_line', 'pedestrian' in case['signs']):
        white |= finite & (ws >= STOP_LINE) & (ws < STOP_LINE + 0.03) & (np.abs(wl) < 0.26)
    image = np.full((H, W, 3), 60, np.uint8)
    image[white] = WHITE_BGR
    image[yellow] = scenario.paint_bgr()
    return image


WALL_T = 0.02   # 벽 두께 (m)


def walls():
    """대회장 벽 (11/12 소민: 트랙 가장자리 = 노란 바깥선 위에 벽). (중심 s, 중심 y, 길이, 폭)
    1차로 구간 ±0.26, 3차로 구간 ±0.78, 넓어지는 곳(ZONE_START)은 가로 벽"""
    out = []
    for side in (1, -1):
        out.append((ZONE_START / 2, side * (0.26 + WALL_T / 2), ZONE_START, WALL_T))
        out.append(((ZONE_START + ZONE_END + 3) / 2, side * (0.78 + WALL_T / 2), ZONE_END + 3 - ZONE_START, WALL_T))
        out.append((ZONE_START, side * 0.52, WALL_T, 0.52))
    return out


def boxes(case):
    """장애물 차 (s, y) + 벽 → (s, y, 길이, 폭)"""
    out = [(os_, oy, CAR_L, CAR_W) for os_, oy in case['obstacles']]
    return out + (walls() if case.get('walls') else [])


def scan_ranges(s, y, psi, obstacles):
    n = 720
    angles = -math.pi + np.arange(n) * (2 * math.pi / n)
    ranges = np.full(n, 4.0)
    world = angles + psi
    dirs = np.column_stack((np.cos(world), np.sin(world)))
    for os_, oy, length, width in obstacles:
        # 축 정렬 상자와 광선 교차 (slab)
        lo = np.array([os_ - length / 2 - s, oy - width / 2 - y])
        hi = np.array([os_ + length / 2 - s, oy + width / 2 - y])
        with np.errstate(divide='ignore', invalid='ignore'):
            t1, t2 = lo / dirs, hi / dirs
        tmin = np.nanmax(np.minimum(t1, t2), axis=1)
        tmax = np.nanmin(np.maximum(t1, t2), axis=1)
        hit = (tmax >= np.maximum(tmin, 0)) & (tmin > 0)
        ranges = np.where(hit, np.minimum(ranges, tmin), ranges)
    return n, angles[0], 2 * math.pi / n, ranges


def collides(s, y, psi, obstacles):
    # 시뮬 (s, y) = 라이다 위치. 차체: 라이다 앞 0.044 m(앞 범퍼 0.258 − 라이다 0.214) ~ 뒤 0.286 m
    corners = [(dx, dy) for dx in (-0.286, 0.044) for dy in (-CAR_W / 2, CAR_W / 2)]
    for os_, oy, length, width in obstacles:
        for dx, dy in corners:
            px = s + dx * math.cos(psi) - dy * math.sin(psi)
            py = y + dx * math.sin(psi) + dy * math.cos(psi)
            if abs(px - os_) < length / 2 and abs(py - oy) < width / 2:
                return True
    return False


def run(name, case, odom_yaw_scale=1.07, duration=70.0, verbose=False):
    import os
    import yaml
    out = dict(name=name, collision=False, max_abs_y=0.0, timeline=[], stuck=0.0, end_lane=None)
    sim = dict(s=case.get('start_s', ZONE_START + 0.3 if case.get('start_zone') else 0.0),
               y=case.get('start_y', 0.0), psi=0.0, v=0.0, steer=0.0)
    obstacles = boxes(case)

    def detections_at(s):
        def box(n):
            return dict(name=n, bbox=[0.4, 0.3, 0.6, 0.5], truncated=False, confidence=0.9, area=0.05, center=(0.5, 0.4))
        return [box(n) for n, (a, b) in case['signs'].items() if a < s < b]

    def spin(node):
        config = yaml.safe_load(open(cfg_path))
        camera, wheelbase = scenario.REFERENCE_CAMERA, config['robot']['wheelbase']
        last_print, zero_since, mode = -1.0, None, None
        for k in range(int(duration / DT)):
            t = k * DT
            fake_ros.Clock.now_s = 100.0 + t
            stamp = fake_ros.Header(100.0 + t)
            image = render(camera, sim['s'], sim['y'], sim['psi'], case)
            node.subscriptions[config['topics']['image']](fake_ros.Msg(header=stamp, array=image))
            if k % 2 == 0:
                n, amin, inc, ranges = scan_ranges(sim['s'], sim['y'], sim['psi'], obstacles)
                node.subscriptions[config['topics']['scan']](fake_ros.Msg(
                    header=stamp, ranges=ranges.tolist(), angle_min=amin, angle_increment=inc,
                    range_min=0.06, range_max=12.0))
            yaw_rate = sim['v'] / wheelbase * math.tan(sim['steer'])
            # odom_speed_scale: odom 속도 오차 (1보다 작으면 실제보다 덜 간 것으로 보고 → 추측 항법이 더 감), imu_scale: IMU 회전 오차
            twist = fake_ros.Msg(linear=fake_ros.Msg(x=sim['v'] * case.get('odom_speed_scale', 1.0)),
                                 angular=fake_ros.Msg(z=yaw_rate * odom_yaw_scale))
            node.subscriptions[config['topics']['odometry']](fake_ros.Msg(header=stamp, twist=fake_ros.Msg(twist=twist)))
            if case.get('imu', True):
                node.subscriptions[config['topics'].get('imu', '/imu')](
                    fake_ros.Msg(header=stamp, angular_velocity=fake_ros.Msg(z=yaw_rate * case.get('imu_scale', 1.0))))
            if k % 4 == 0:
                with node.lock:
                    node.detections = detections_at(sim['s'])
                    node.detection_stamp = 100.0 + t
            before = len(node.published)
            node.timers[0]()
            command = None
            for topic, message in node.published[before:]:
                if topic == config['topics']['drive']:
                    command = (message.speed, message.steering_angle)
                elif topic == config['topics']['status']:
                    mode = message.data
            if command is not None:
                sim['v'] += (command[0] - sim['v']) * 0.4
                # 실차 흉내: steer_scale배 과·저조향, steer_lag(0~1) 조향 지연 (기본 1, 0 = 즉시)
                lag = case.get('steer_lag', 0.0)
                sim['steer'] = lag*sim['steer'] + (1-lag)*command[1]*case.get('steer_scale', 1.0)
            sim['psi'] += sim['v'] / wheelbase * math.tan(sim['steer']) * DT
            sim['s'] += sim['v'] * math.cos(sim['psi']) * DT
            sim['y'] += sim['v'] * math.sin(sim['psi']) * DT
            d = node.driver.debug if hasattr(node, 'driver') else getattr(node.loop.driver, 'debug', {})
            if ZONE_START <= sim['s'] < ZONE_END:
                out['max_abs_y'] = max(out['max_abs_y'], abs(sim['y']))
                out['max_y_signed'] = max(out.get('max_y_signed', 0.0), sim['y'], key=abs)
            if collides(sim['s'], sim['y'], sim['psi'], obstacles):
                out['collision'] = True
            if abs(sim['v']) < 0.01 and t > 2:
                zero_since = t if zero_since is None else zero_since
                out['stuck'] = max(out['stuck'], t - zero_since)
            else:
                zero_since = None
            if t - last_print >= 0.5:
                last_print = t
                out['timeline'].append('%5.1f s=%5.2f y=%+.2f psi=%+.2f v=%.2f idx=%s stage=%s front=%.2f lane=%s mode=%s' % (
                    t, sim['s'], sim['y'], sim['psi'], sim['v'], d.get('lane_index'), d.get('stage'), d.get('front', -1) or -1, d.get('lane_valid'), mode))
            if sim['s'] > ZONE_END + 0.5:
                break
        out['final'] = (round(sim['s'], 2), round(sim['y'], 2))
        out['end_lane'] = int(round(sim['y'] / 0.52)) + 1

    fake_ros.install(spin)
    from ericar_driver import node as node_module, params
    params.ANSWER_CONFIG['missions']['start_stage'] = 'fork'   # S2에서 출발
    params.ANSWER_CONFIG['sections']['lane_change_max_distance'] = None   # 합성 장면은 3차선이 실제(약 5 m)보다 김
    if os.environ.get('LC_LOOKAHEAD'):   # 값 비교 실험용
        params.ANSWER_CONFIG['lane_change']['lookahead'] = float(os.environ['LC_LOOKAHEAD'])
    sys.argv = [sys.argv[0]]
    config = yaml.safe_load(open(HERE.parent / 'config' / 'robot.yaml'))
    config['robot']['steering_offset'] = 0.0
    config['robot']['camera'] = dict(scenario.REFERENCE_CAMERA)
    with tempfile.NamedTemporaryFile('w', suffix='.yaml', delete=False) as f:
        yaml.safe_dump(config, f, allow_unicode=True)
        cfg_path = f.name
    node_module.main(config_path=cfg_path)
    return out


def judge(case_name, r):
    reached = r['final'][0] > ZONE_END
    off = r['max_abs_y'] + CAR_W/2 > 0.78   # 차체 가장자리가 노란 바깥선을 넘으면 이탈 (예전: 차체 중심 기준)
    fork = CASES[case_name]['signs']
    want = 2 if 'left' in fork else 0 if 'right' in fork else None
    fork_ok = want is None or r['end_lane'] == want
    ok = reached and not off and not r['collision'] and fork_ok
    return ok, dict(도달=reached, 이탈=off, 충돌=r['collision'], 최대정지_s=round(r['stuck'], 1),
                    끝차로=r['end_lane'], 갈림길=('-' if want is None else fork_ok), 끝=r['final'], 최대횡=round(r['max_abs_y'], 2))


if __name__ == '__main__':
    names = [a for a in sys.argv[1:] if not a.startswith('-')] or list(CASES)
    verbose = '-v' in sys.argv
    if len(names) > 1:
        # 노드 모듈이 가짜 ROS를 한 번만 잡으므로 사례마다 새 프로세스
        import subprocess
        passed = sum(subprocess.run([sys.executable, __file__, n] + (['-v'] if verbose else [])).returncode == 0
                     for n in names)
        print('%d/%d 통과' % (passed, len(names)))
        sys.exit(0 if passed == len(names) else 1)
    from ericar_driver import params
    params.ANSWER_CONFIG['detector']['enabled'] = False   # 모델 대신 검출 주입
    name = names[0]
    r = run(name, CASES[name], odom_yaw_scale=CASES[name].get('odom_yaw_scale', 1.07))
    ok, info = judge(name, r)
    print(('통과 ' if ok else '실패 ') + name, json.dumps(info, ensure_ascii=False))
    if verbose or not ok:
        print('\n'.join('    ' + line for line in r['timeline']))
    sys.exit(0 if ok else 1)
