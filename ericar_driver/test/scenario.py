"""회귀 시험 시나리오: 합성 카메라·라이다·odom으로 노드 전체를 결정적으로 실행

도면 치수 기준 직선 차로(선 중심 간격 0.52 m, 선 굵기 20 mm)에 정지선을 두고
서행 → 보행자 → 갈림길 → 신호등 → 주차 순서로 표지판 검출을 주입함.
차량은 발행된 명령으로 자전거 모델 적분. 결과는 발행 메시지 기록.

실행: python3 test/scenario.py --write   (기준값 갱신)
노면 표시 색은 params.py 차선 색 범위 중앙값 → 차선 색 변경 PR은 기준값 갱신 필요
"""
import hashlib
import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent))

import fake_ros  # noqa: E402

GOLDEN = HERE / 'golden_regression.json'
DT = 0.05
DURATION = 110.0
W, H = 640, 480
HALF_LANE = 0.26
LANE_LINES = [-0.78, -0.26, 0.26, 0.78]  # 3차로 (도면 선 중심 간격 0.52 m)
LINE = 0.02
STOP_LINES = [3.0, 7.0, 13.0, 18.0]  # 보행자, (갈림길 구간 통과용), 신호등, 주차 기준
OBSTACLE = (10.0, 10.3)  # 차로 중앙 정적 장애물 구간 s (차선 변경 경로 실행용)


def paint_bgr():
    """노면 표시 색: params.py 차선 색 범위(HSV)의 중앙값. 차선 색 설정이 바뀌어도 시험 장면이 따라감"""
    import cv2
    from ericar_driver.params import ANSWER_CONFIG
    lane = ANSWER_CONFIG['lane']
    hsv = [(a + b) // 2 for a, b in zip(lane['white_min'], lane['white_max'])]
    return cv2.cvtColor(np.uint8([[hsv]]), cv2.COLOR_HSV2BGR)[0, 0]


def render(camera, s, y, psi, curve, paint):
    """카메라 영상 합성: 각 픽셀의 바닥 좌표 → 차선·정지선 색칠"""
    v, u = np.indices((H, W), dtype=np.float64)
    focal = W / (2 * math.tan(math.radians(camera['hfov_deg']) / 2))
    p, z, cx = camera['pitch'], camera['z'], camera['x']
    t = (H / 2 - v) / focal
    denom = math.sin(p) - t * math.cos(p)
    with np.errstate(divide='ignore', invalid='ignore'):
        dx = np.where(denom > 1e-6, z * (math.cos(p) + t * math.sin(p)) / denom, np.nan)
    dx = np.where(np.isfinite(dx) & (dx > 0), dx, np.nan)  # 지평선 아래 화소만 바닥과 만남
    forward = cx + dx
    depth = math.cos(p) * dx + math.sin(p) * z
    left = (W / 2 - u) * depth / focal - camera.get('y', 0.0)
    ws = s + forward * math.cos(psi) - left * math.sin(psi)
    wl = y + forward * math.sin(psi) + left * math.cos(psi) - curve(ws)
    white = np.zeros((H, W), bool)
    for offset in LANE_LINES:
        white |= np.abs(wl - offset) < LINE / 2
    for line in STOP_LINES:
        white |= (ws >= line) & (ws < line + 0.03) & (np.abs(wl) < LANE_LINES[-1])
    white &= np.isfinite(forward) & (forward < 4.0)
    image = np.full((H, W, 3), 60, np.uint8)
    image[white] = paint
    return image


def scan_ranges(s, y):
    """360° 라이다 합성. 정적 장애물 구간에서 전방 차로 중앙에 물체"""
    n = 720
    angles = -math.pi + np.arange(n) * (2 * math.pi / n)
    ranges = np.full(n, 4.0)
    near, far = OBSTACLE
    if near - 1.8 < s < far:
        distance = max(0.25, near - s)
        width = 0.11
        half = math.atan2(width, distance)
        hit = np.abs(angles - math.atan2(-y, distance)) < half
        ranges[hit] = distance / np.maximum(np.cos(angles[hit]), 0.2)
    return n, angles[0], 2 * math.pi / n, ranges


def detections_at(s, t, phase):
    """미션 순서대로 표지판 검출 주입 (면적은 클래스별 하한 위)"""
    def box(name):
        return dict(name=name, bbox=[0.4, 0.3, 0.6, 0.5], truncated=False,
                    confidence=0.9, area=0.05, center=(0.5, 0.4))
    names = []
    if 0.5 < s < 1.2:
        names.append('slow')
    if 1.6 < s < 2.9:
        names.append('pedestrian')
    if 5.0 < s < 6.0:
        names.append('left')
    if 11.5 < s < 13.2:
        names.append('traffic_green' if phase.get('green_after') and t > phase['green_after'] else 'traffic_red')
    if 16.5 < s < 18.0:
        names.append('parking')
    return [box(n) for n in names]


def run(config_path):
    import yaml
    records = []
    sim = dict(s=0.0, y=0.0, psi=0.0, v=0.0, steer=0.0)
    phase = {}

    def curve(ws):
        return 0.08 * np.sin(np.asarray(ws) * 0.6)

    def spin(node):
        config = yaml.safe_load(open(config_path))
        camera, wheelbase = config['robot']['camera'], config['robot']['wheelbase']
        steps = int(DURATION / DT)
        paint = paint_bgr()
        for k in range(steps):
            t = k * DT
            fake_ros.Clock.now_s = 100.0 + t
            stamp = fake_ros.Header(100.0 + t)
            image = render(camera, sim['s'], sim['y'], sim['psi'], curve, paint)
            node.subscriptions[config['topics']['image']](fake_ros.Msg(header=stamp, array=image))
            if k % 2 == 0:
                n, amin, inc, ranges = scan_ranges(sim['s'], sim['y'])
                node.subscriptions[config['topics']['scan']](fake_ros.Msg(
                    header=stamp, ranges=ranges.tolist(), angle_min=amin, angle_increment=inc,
                    range_min=0.06, range_max=12.0))
            twist = fake_ros.Msg(linear=fake_ros.Msg(x=sim['v']), angular=fake_ros.Msg(
                z=sim['v'] / wheelbase * math.tan(sim['steer'])))
            node.subscriptions[config['topics']['odometry']](
                fake_ros.Msg(header=stamp, twist=fake_ros.Msg(twist=twist)))
            if k % 4 == 0:
                with node.lock:
                    node.detections = detections_at(sim['s'], t, phase)
                    node.detection_stamp = 100.0 + t
            before = len(node.published)
            node.timers[0]()
            command = None
            for topic, message in node.published[before:]:
                if topic == config['topics']['drive']:
                    command = (message.speed, message.steering_angle)
                elif topic == config['topics']['status']:
                    records.append(['status', round(t, 2), message.data])
                    if message.data == 'traffic_hold' and 'green_after' not in phase:
                        phase['green_after'] = t + 2.0
                elif topic == config['topics']['detections']:
                    records.append(['detections', round(t, 2), hashlib.sha1(message.data.encode()).hexdigest()])
                elif topic == config['topics']['debug_image']:
                    records.append(['debug_image', round(t, 2),
                                    hashlib.sha1(np.ascontiguousarray(message.array).tobytes()).hexdigest()])
            if command is not None:
                records.append(['drive', round(t, 2), command[0], command[1]])
                sim['v'] += (command[0] - sim['v']) * 0.4
                sim['steer'] = command[1]
            sim['psi'] += sim['v'] / wheelbase * math.tan(sim['steer']) * DT
            sim['s'] += sim['v'] * math.cos(sim['psi']) * DT
            sim['y'] += sim['v'] * math.sin(sim['psi']) * DT
        records.append(['final', round(sim['s'], 6), round(sim['y'], 6), round(sim['psi'], 6)])

    fake_ros.install(spin)
    from ericar_driver import params
    params.ANSWER_CONFIG['detector']['enabled'] = False
    from ericar_driver import node as node_module
    sys.argv = [sys.argv[0]]
    node_module.main(config_path=str(HERE.parent / 'config' / 'robot.yaml'))
    return records


def summary(records):
    states = [r[2] for r in records if r[0] == 'status']
    drives = [r for r in records if r[0] == 'drive']
    return dict(states=states, drives=len(drives), final=records[-1])


if __name__ == '__main__':
    write = '--write' in sys.argv
    result = run(str(HERE.parent / 'config' / 'robot.yaml'))
    print(json.dumps(summary(result), ensure_ascii=False))
    if write:
        GOLDEN.write_text(json.dumps(result))
        print('기준값 저장:', GOLDEN.name, len(result), '항목')
