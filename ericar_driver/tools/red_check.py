#!/usr/bin/env python3
"""라바콘 붉은색 판정 확인: 카메라 → 붉은색 비율·색 분포 출력 + 판정 결과 사진 저장

bringup을 driver:=false로 켜 둔 상태에서 실행. 명령을 보내지 않음 (차는 손으로 밀거나 조종기로 움직임).
    /usr/bin/python3 ~/try_yang_ws/src/ericar/ericar_driver/tools/red_check.py            # 1장
    /usr/bin/python3 ~/try_yang_ws/src/ericar/ericar_driver/tools/red_check.py --every 0.5 # 0.5초마다 연속 (Ctrl+C로 끝)
    - dl_env의 python3 금지 (영상 변환 libgdal/libtiff 충돌)
1장 모드 출력: 지금 기준 비율, 화면 전체·위쪽 비율, 채도 높은 화소의 색상(H) 분포 → ~/bags/red_check.jpg
연속 모드: ~/bags/red_<시각>/NNN_r<붉은 비율>_c<라이다 앞 거리>.jpg 저장, 한 줄씩 출력
  - r: 붉은 화소 비율 (판정 기준 cone_red_ratio). c: 라이다 앞 방향으로 가장 가까운 콘 후보 거리(m, 좌우 cone_red_half_width 안,
    없으면 x). 진입 조건 = r ≥ 기준 이고 c ≤ cone_enter_distance
  - 사진: 위 = 원본(값 글자), 아래 = 붉은색 판정 화소(흰색). 장당 약 50 KB라 노트북으로 복사 가능
"""
import argparse
import os
import sys
import time
from pathlib import Path

import numpy as np

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))
from ericar_driver.cones import find_posts, red_ratio  # noqa: E402
from ericar_driver.lidar import scan_sectors  # noqa: E402
from ericar_driver.loop import load_config  # noqa: E402


def red_mask(image, c):
    import cv2
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    hue, sat, val = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    return (((hue <= c['cone_red_h_low']) | (hue >= c['cone_red_h_high'])) &
            (sat >= c['cone_red_s_min']) & (val >= c['cone_red_v_min']))


def report(image, c):
    import cv2
    h = image.shape[0]
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    hue, sat, val = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    red = red_mask(image, c)
    top = int(h*(1-c['cone_red_rows']))
    ratio = red_ratio(image, c)
    print('판정 비율 (아래 %.0f%%, 기준 %.4f): %.4f%s' % (
        c['cone_red_rows']*100, c['cone_red_ratio'], ratio, '  → 콘 색 있음' if ratio >= c['cone_red_ratio'] else '  → 없음'))
    print('화면 전체 비율: %.4f, 위쪽 영역만: %.4f' % (red.mean(), red[:top].mean() if top else 0.0))
    colorful = sat >= 80
    print('채도 80 이상 화소 %.1f%%. 색상 분포 (H 범위: 비율%%, 평균 밝기):' % (colorful.mean()*100))
    for start in range(0, 180, 10):
        band = colorful & (hue >= start) & (hue < start+10)
        if band.mean() >= 0.001:
            print('  H %3d~%3d: %5.2f%%  V %3.0f  S %3.0f' % (start, start+9, band.mean()*100,
                                                         val[band].mean(), sat[band].mean()))
    mask = cv2.cvtColor((red*255).astype(np.uint8), cv2.COLOR_GRAY2BGR)
    cv2.line(mask, (0, top), (image.shape[1]-1, top), (0, 255, 0), 2)
    path = os.path.expanduser('~/bags/red_check.jpg')
    cv2.imwrite(path, np.vstack([image, mask]), [cv2.IMWRITE_JPEG_QUALITY, 70])
    print('사진 저장:', path)


def nearest_cone(scan, robot, c):
    """라이다 앞 방향으로 가장 가까운 콘 후보 거리(m, 좌우 cone_red_half_width 안). 없으면 None"""
    if scan is None:
        return None
    posts, _ = find_posts(scan, robot, c)
    posts = posts[np.abs(posts[:, 1]) < c['cone_red_half_width']]
    return float(posts[:, 0].min()-robot['lidar']['x']) if len(posts) else None


def save_frame(folder, index, image, c, ratio, near):
    import cv2
    shown = image.copy()
    enter = ratio >= c['cone_red_ratio'] and near is not None and near <= c['cone_enter_distance']
    text = 'red %.4f  cone %s  %s' % (ratio, '-' if near is None else '%.2f m' % near, 'ENTER' if enter else '')
    cv2.putText(shown, text, (8, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0) if enter else (255, 255, 255), 2)
    mask = cv2.cvtColor((red_mask(image, c)*255).astype(np.uint8), cv2.COLOR_GRAY2BGR)
    name = '%03d_r%.4f_c%s.jpg' % (index, ratio, 'x' if near is None else '%.2f' % near)
    cv2.imwrite(str(folder / name), np.vstack([shown, mask]), [cv2.IMWRITE_JPEG_QUALITY, 60])
    return name, enter


def main():
    parser = argparse.ArgumentParser(description='라바콘 붉은색 판정 확인')
    parser.add_argument('--every', type=float, default=0.0, help='0보다 크면 이 간격(초)으로 연속 저장 (Ctrl+C로 끝)')
    options = parser.parse_args()

    import rclpy
    from cv_bridge import CvBridge
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image, LaserScan

    config = load_config(str(PACKAGE / 'config' / 'robot.yaml'))
    c, robot, topics = config['answer']['lidar'], config['robot'], config['topics']
    rclpy.init()
    node = rclpy.create_node('red_check')
    bridge, frames, state = CvBridge(), [], {'scan': None}
    node.create_subscription(Image, topics['image'], lambda m: frames.append(m), qos_profile_sensor_data)

    def on_scan(m):
        state['scan'] = scan_sectors(m.ranges, m.angle_min, m.angle_increment, m.range_min, m.range_max, c)
    node.create_subscription(LaserScan, topics['scan'], on_scan, qos_profile_sensor_data)

    deadline = time.time()+5.0
    while not frames and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not frames:
        print('%s 영상이 5초 동안 안 들어옴 (bringup 켜져 있는지 확인)' % topics['image'])
    elif options.every <= 0:
        report(bridge.imgmsg_to_cv2(frames[-1], 'bgr8'), c)
    else:
        folder = Path(os.path.expanduser(time.strftime('~/bags/red_%m%d_%H%M%S')))
        folder.mkdir(parents=True, exist_ok=True)
        print('연속 저장: %s (기준 붉은 비율 %.4f, 진입 거리 %.2f m). Ctrl+C로 끝' % (
            folder, c['cone_red_ratio'], c['cone_enter_distance']))
        index, last = 0, 0.0
        try:
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.05)
                if time.time()-last < options.every or not frames:
                    continue
                last = time.time()
                image = bridge.imgmsg_to_cv2(frames[-1], 'bgr8')
                del frames[:-1]
                ratio = red_ratio(image, c)
                near = nearest_cone(state['scan'], robot, c)
                name, enter = save_frame(folder, index, image, c, ratio, near)
                print('%s%s' % (name, '  ← 진입' if enter else ''))
                index += 1
        except KeyboardInterrupt:
            pass
        print('저장 %d장: %s' % (index, folder))
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
