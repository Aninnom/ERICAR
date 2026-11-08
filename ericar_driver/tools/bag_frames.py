#!/usr/bin/env python3
"""녹화 프레임 추출: 지정 시각의 카메라 영상 + BEV 차선 창·곡선 + 목표점을 한 장으로

실행 (차, ROS 환경): python3 tools/bag_frames.py ~/bags/<이름> 29.0,30.6,33.0 [출력.jpg]
시각은 bag_timeline.py 첫 열 기준(s). 기본 출력 /tmp/frames.jpg → 노트북으로: scp wego@<차>:/tmp/frames.jpg .
BEV 표시는 녹화 당시 주행 노드 텔레메트리(/answer/detections) 기준 (지금 코드로 다시 계산하려면 replay_bag.py)
초록 = 왼쪽 선, 파랑 = 오른쪽 선, 빨간 원 = pure pursuit 목표점
"""
import json
import sys

import cv2
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

H, W, NEAR, FAR = 420, 480, 0.6, 1.7      # params.py lane BEV 크기·범위와 동일
SCALE = W / 2.4                             # view_width 2.4 m


def draw_bev(lane):
    bev = np.zeros((H, W, 3), np.uint8)
    for window in lane.get('windows', []):
        color = (0, 255, 0) if window['side'] == 'left' else (255, 128, 0)
        cv2.rectangle(bev, (int(window['x0']), window['y0']), (int(window['x1']), window['y1']), color, 1)
    for side, fit in (lane.get('fits') or {}).items():
        rows = np.arange(H)
        points = np.stack([np.polyval(fit, rows), rows], 1).astype(np.int32)
        cv2.polylines(bev, [points], False, (0, 255, 0) if side == 'left' else (255, 128, 0), 2)
    forward, lateral = lane.get('forward', 0.9), lane.get('error', 0.0)
    row, col = (FAR - forward) / (FAR - NEAR) * (H - 1), W / 2 - lateral * SCALE
    cv2.circle(bev, (int(col), int(min(row, H - 5))), 8, (0, 0, 255), 2)
    cv2.line(bev, (W // 2, H - 1), (int(col), int(min(row, H - 5))), (0, 0, 255), 1)
    return bev


def main(path, times, output='/tmp/frames.jpg'):
    wanted = sorted(float(x) for x in times.split(','))
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/camera/color/image_raw/compressed', '/answer/detections']))
    t0, image, panels = None, None, []
    while reader.has_next() and len(panels) < len(wanted):
        topic, data, t = reader.read_next()
        t *= 1e-9
        t0 = t0 or t
        if topic.endswith('compressed'):
            image = cv2.imdecode(np.frombuffer(deserialize_message(data, CompressedImage).data, np.uint8), 1)
            continue
        if image is None or t - t0 < wanted[len(panels)]:
            continue
        message = json.loads(deserialize_message(data, String).data)
        lane, control = message.get('lane') or {}, message['control']
        camera = cv2.resize(image, (int(image.shape[1] * H / image.shape[0]), H))
        panel = np.hstack([camera, draw_bev(lane)])
        cv2.putText(panel, 't=%.1f err=%+.3f st=%+.3f n=%s' % (
            t - t0, control['lane_error'] or 0, control['steering'], ''.join(k[0] for k in lane.get('fits') or {})),
            (10, 30), 0, 0.8, (0, 255, 255), 2)
        panels.append(panel)
    if not panels:
        sys.exit('지정 시각의 프레임 없음')
    cv2.imwrite(output, np.vstack(panels), [cv2.IMWRITE_JPEG_QUALITY, 70])
    print('저장', output, len(panels), '장')


if __name__ == '__main__':
    if len(sys.argv) < 3:
        sys.exit(__doc__)
    main(*sys.argv[1:4])
