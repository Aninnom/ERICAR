#!/usr/bin/env python3
"""카메라 높이·기울기 보정: 직선 차로 사진에서 차선 두 줄이 평행·간격 0.52 m가 되는 값 탐색

원리: BEV 변환(lane_vision.ground_maps와 같은 식)으로 차선 화소를 바닥 좌표로 바꾸면
- 기울기(pitch)가 틀리면 두 줄이 V자로 벌어지거나 좁아짐 → 평행 조건으로 pitch 결정
- 높이(z)가 틀리면 간격 전체가 비례해서 커지거나 작아짐 → 간격 0.52 m로 z 결정
차량이 차로와 비스듬하면 두 줄이 같이 기울 뿐 평행·간격은 유지 → yaw는 결과로만 표시

사용 (차, 직선 차로 가운데에 똑바로 세운 상태):
  python3 calibrate_camera.py 사진1.jpg [사진2.jpg ...] [--config robot.yaml] [--spacing 0.52]
사진 없이 카메라 토픽에서 바로 (차, 터미널 환경 1, 카메라 실행 중):
  python3 calibrate_camera.py --capture 5

줄자 검증: 앞바퀴 축 중심 → 정지선 가까운 쪽 가장자리 거리를 줄자로 재고
  python3 calibrate_camera.py --capture 3 --verify 0.30
  → robot.yaml의 카메라 값으로 lane_vision이 계산한 정지선 거리와 비교 (미션과 같은 코드)
차선 색 설정을 임시로 바꿀 때: --hsv 15,80,100,35,255,255
"""
import argparse
import math
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))


def lane_pixels(bgr, lane):
    """차선 색 마스크에서 세로로 긴 덩어리 2개(좌·우 차선)의 화소 좌표"""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(lane['white_min']), np.array(lane['white_max']))
    mask[: bgr.shape[0] // 3] = 0  # 위쪽 1/3(먼 곳·배경) 제외
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
    blobs = sorted(range(1, count), key=lambda i: -stats[i, cv2.CC_STAT_HEIGHT])[:2]
    if len(blobs) < 2:
        return None
    lines = []
    for i in blobs:
        v, u = np.nonzero(labels == i)
        lines.append((u.astype(float), v.astype(float)))
    return sorted(lines, key=lambda line: line[0].mean())  # 왼쪽 먼저


def to_ground(u, v, w, h, camera, pitch, z):
    """화소 → 바닥 좌표 (전방 m, 왼쪽 m). lane_vision.ground_maps의 역변환"""
    focal = w / (2 * math.tan(math.radians(camera['hfov_deg']) / 2))
    t = (h / 2 - v) / focal
    denom = math.sin(pitch) - t * math.cos(pitch)
    ok = denom > 1e-6
    dx = np.where(ok, z * (math.cos(pitch) + t * math.sin(pitch)) / np.where(ok, denom, 1), np.nan)
    depth = math.cos(pitch) * dx + math.sin(pitch) * z
    left = (w / 2 - u) * depth / focal - camera.get('y', 0.0)
    return camera['x'] + dx, left


def score(lines, w, h, camera, pitch, z, spacing, far):
    fits = []
    for u, v in lines:
        forward, left = to_ground(u, v, w, h, camera, pitch, z)
        keep = np.isfinite(forward) & (forward > 0) & (forward < far)
        if keep.sum() < 30:
            return None
        fits.append(np.polyfit(forward[keep], left[keep], 1))  # left = a*forward + b
    (al, bl), (ar, br) = fits
    slope_gap = al - ar  # 평행이면 0
    middle = 0.6  # 간격 평가 지점 (전방 m)
    gap = (al * middle + bl) - (ar * middle + br)
    return slope_gap, gap, (al + ar) / 2, ((al * middle + bl) + (ar * middle + br)) / 2


def verify(images, config, expected):
    """미션 코드(LaneVision)로 정지선 거리 계산 후 줄자 값과 비교"""
    from ericar_driver.lane_vision import LaneVision
    from ericar_driver.params import ANSWER_CONFIG
    results = []
    for image in images:
        vision = LaneVision(ANSWER_CONFIG['lane'], config['robot']['camera'], config['robot']['front_axle_x'])
        lane = vision.analyze(image)
        distance = lane['stop_distance']
        if distance is None and lane['stop_candidates']:
            distance = lane['stop_candidates'][0]['distance']  # 두께·잘림 조건 미통과 후보
        results.append(distance)
    found = [d for d in results if d is not None]
    print(f'줄자 {expected:.3f} m / 계산 {[round(d, 3) for d in found] or "정지선 미검출"}')
    if found:
        error = float(np.mean(found)) - expected
        verdict = '통과' if abs(error) <= 0.02 else '재보정 필요'
        print(f'평균 오차 {error * 100:+.1f} cm → {verdict} (기준 ±2 cm, 규정 정지 허용 ±5 cm)')


def calibrate(images, config, spacing, far=1.2):
    from ericar_driver.params import ANSWER_CONFIG
    camera = dict(config['robot']['camera'])
    lane = ANSWER_CONFIG['lane']
    samples = []
    for image in images:
        lines = lane_pixels(image, lane)
        if lines is None:
            print('  차선 두 줄을 찾지 못함 (색 범위·화면 위치 확인)')
            continue
        samples.append((lines, image.shape[1], image.shape[0]))
    if not samples:
        raise SystemExit('사용 가능한 사진 없음')
    best = None
    for pitch in np.arange(0.0, 0.9, 0.002):
        # 평행 조건은 높이와 무관 → pitch만으로 판정
        gaps = [score(lines, w, h, camera, pitch, 0.2, spacing, far * 3) for lines, w, h in samples]
        if any(g is None for g in gaps):
            continue
        err = float(np.mean([abs(g[0]) for g in gaps]))
        if best is None or err < best[0]:
            best = (err, pitch)
    if best is None:
        raise SystemExit('평행 조건을 만족하는 기울기 없음')
    pitch = best[1]
    # 간격은 높이에 비례 → z=0.2 기준 간격으로 비례 환산
    gaps = [score(lines, w, h, camera, pitch, 0.2, spacing, far * 3) for lines, w, h in samples]
    z = 0.2 * spacing / float(np.mean([g[1] for g in gaps]))
    final = [score(lines, w, h, camera, pitch, z, spacing, far) for lines, w, h in samples]
    return dict(pitch=round(float(pitch), 4), z=round(float(z), 4),
                parallel_error=round(best[0], 4),
                spacing=[round(float(f[1]), 4) for f in final if f],
                yaw_deg=[round(-math.degrees(math.atan(f[2])), 2) for f in final if f],  # 차로 대비 차량 방향, + = 왼쪽
                center_offset=[round(float(f[3]), 4) for f in final if f])


def capture(topic, count):
    """카메라 토픽에서 0.5초 간격으로 count장 수집, 확인용으로 /tmp/calib_N.jpg 저장"""
    import time
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    rclpy.init()
    node = rclpy.create_node('calibrate_camera')
    latest = {}
    node.create_subscription(Image, topic, lambda m: latest.update(m=m), qos_profile_sensor_data)
    images, last = [], 0.0
    deadline = time.time() + count * 0.5 + 10
    while len(images) < count and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if 'm' in latest and time.time() - last >= 0.5:
            m = latest.pop('m')
            a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, -1)
            a = a[:, :, ::-1].copy() if m.encoding == 'rgb8' else a.copy()
            cv2.imwrite(f'/tmp/calib_{len(images)}.jpg', a)
            images.append(a)
            last = time.time()
    rclpy.shutdown()
    if not images:
        raise SystemExit(f'{topic} 수신 없음 (카메라 실행, ROS_LOCALHOST_ONLY 확인)')
    return images


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('images', nargs='*')
    p.add_argument('--config', default=str(HERE.parent / 'config' / 'robot.yaml'))
    p.add_argument('--spacing', type=float, default=0.52, help='차선 중심 간격 m (도면 500+20)')
    p.add_argument('--capture', type=int, default=0, help='카메라 토픽에서 N장 촬영 (0.5초 간격)')
    p.add_argument('--verify', type=float, help='줄자로 잰 앞바퀴 축 → 정지선 거리 m. 보정 대신 검증 실행')
    p.add_argument('--hsv', help='차선 색 범위 임시 지정 h,s,v,h,s,v (예: 15,80,100,35,255,255)')
    args = p.parse_args()
    config = yaml.safe_load(open(args.config))
    if args.hsv:
        from ericar_driver.params import ANSWER_CONFIG
        values = [int(x) for x in args.hsv.split(',')]
        ANSWER_CONFIG['lane']['white_min'], ANSWER_CONFIG['lane']['white_max'] = values[:3], values[3:]
    images = capture(config['topics']['image'], args.capture) if args.capture else [cv2.imread(f) for f in args.images]
    if not images or any(i is None for i in images):
        raise SystemExit('사진 경로 확인')
    if args.verify is not None:
        verify(images, config, args.verify)
        return
    r = calibrate(images, config, args.spacing)
    cam = config['robot']['camera']
    print(f"기울기 pitch {r['pitch']} rad ({math.degrees(r['pitch']):.1f}°)   현재 설정 {cam['pitch']}")
    print(f"높이 z {r['z']} m   현재 설정 {cam['z']}   ← 줄자 측정값과 비교")
    print(f"평행 오차 {r['parallel_error']} (0에 가까울수록 좋음), 보정 후 간격 {r['spacing']} m")
    print(f"차량 방향 {r['yaw_deg']}°, 차로 중앙 대비 {r['center_offset']} m (+ = 차로 중앙이 왼쪽)")
    print('robot.yaml 반영 예시:')
    print(f"    z: {r['z']}\n    pitch: {r['pitch']}")


if __name__ == '__main__':
    main()
