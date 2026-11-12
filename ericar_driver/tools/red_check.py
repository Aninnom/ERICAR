#!/usr/bin/env python3
"""라바콘 붉은색 판정 확인: 카메라 1장 → 붉은색 비율·색 분포 출력 + 판정 결과 사진 저장

차를 콘 앞에 두고 실행 (bringup은 driver:=false로 켜 둔 상태). 명령을 보내지 않음.
    /usr/bin/python3 ~/try_yang_ws/src/ericar/ericar_driver/tools/red_check.py
    - dl_env의 python3 금지 (영상 변환 libgdal/libtiff 충돌)
출력
  - 지금 기준(params.py cone_red_*)의 비율, 화면 전체 기준 비율
  - 채도 높은 화소의 색상(H) 분포: 콘 색이 실제로 몇 H인지 확인용
저장 ~/bags/red_check.jpg (작음, 노트북으로 복사 가능): 위 = 원본, 아래 = 붉은색 판정 화소(흰색)·판정 영역 경계선
"""
import os
import sys
import time
from pathlib import Path

import numpy as np

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))
from ericar_driver.cones import red_ratio  # noqa: E402
from ericar_driver.loop import load_config  # noqa: E402


def report(image, c):
    import cv2
    h = image.shape[0]
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    hue, sat, val = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    red = (((hue <= c['cone_red_h_low']) | (hue >= c['cone_red_h_high'])) &
           (sat >= c['cone_red_s_min']) & (val >= c['cone_red_v_min']))
    top = int(h*(1-c['cone_red_rows']))
    print('판정 비율 (아래 %.0f%%, 기준 %.4f): %.4f%s' % (
        c['cone_red_rows']*100, c['cone_red_ratio'], red_ratio(image, c),
        '  → 콘 색 있음' if red_ratio(image, c) >= c['cone_red_ratio'] else '  → 없음'))
    print('화면 전체 비율: %.4f, 위쪽 영역만: %.4f' % (red.mean(), red[:top].mean() if top else 0.0))
    # 채도 높은 화소의 색상 분포 (H 0~179, 10 단위)
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


def main():
    import rclpy
    from cv_bridge import CvBridge
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image

    config = load_config(str(PACKAGE / 'config' / 'robot.yaml'))
    c, topic = config['answer']['lidar'], config['topics']['image']
    rclpy.init()
    node = rclpy.create_node('red_check')
    bridge, frames = CvBridge(), []
    node.create_subscription(Image, topic, lambda m: frames.append(m), qos_profile_sensor_data)
    deadline = time.time()+5.0
    while not frames and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not frames:
        print('%s 영상이 5초 동안 안 들어옴 (bringup 켜져 있는지 확인)' % topic)
    else:
        report(bridge.imgmsg_to_cv2(frames[-1], 'bgr8'), c)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
