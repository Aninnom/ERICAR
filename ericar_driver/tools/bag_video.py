#!/usr/bin/env python3
"""녹화 bag → 카메라 영상 mp4 (주행 상태·조향·붉은 비율 글자 포함). 작은 파일이라 노트북으로 복사 가능

실행 (차): /usr/bin/python3 tools/bag_video.py ~/bags/<이름> [출력.mp4] [시작 s] [끝 s]
  - 기본 출력 ~/bags/<이름>.mp4, 480×360, 녹화 속도 그대로
  - 글자: 시각(s), 상태(lane·corner_hold·cones 등), 조향(°), 속도, 붉은 비율(cone_red_ratio), 라바콘 모드, 정면 거리,
    구간(S0~S4·활성 미션)과 구간 직선 방향에서 돈 각도
    값은 녹화 당시 주행 노드 텔레메트리(/answer/detections) 기준
  - dl_env의 python3 금지 (영상 변환 libgdal/libtiff 충돌)
노트북으로: scp wego@<차 IP>:~/bags/<이름>.mp4 .
"""
import json
import math
import os
import sys

import cv2
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

SIZE = (480, 360)


def label(control):
    if not control:
        return ['no telemetry']
    return ['%s  steer %+.0f deg  v %.2f' % (control.get('state', '?'), math.degrees(control.get('steering', 0.0)),
                                             control.get('speed', 0.0)),
            'red %.3f  cone %s  front %.2f m' % (control.get('cone_red_ratio', 0.0),
                                                 'ON' if control.get('cone_active') else '-',
                                                 min(control.get('front', 9.99), 9.99)),
            '%s  turned %+.0f deg' % (control.get('stage', '?'), control.get('section_turned', 0.0))]


def main(path, output=None, start=0.0, end=1e9):
    path = os.path.expanduser(path.rstrip('/'))
    output = os.path.expanduser(output or path+'.mp4')
    start, end = float(start), float(end)
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/camera/color/image_raw/compressed', '/answer/detections']))
    frames, control, t0 = [], {}, None
    while reader.has_next():
        topic, data, t = reader.read_next()
        t *= 1e-9
        t0 = t0 if t0 is not None else t
        if topic == '/answer/detections':
            control = json.loads(deserialize_message(data, String).data).get('control', {})
            continue
        if not start <= t-t0 <= end:
            continue
        image = cv2.imdecode(np.frombuffer(deserialize_message(data, CompressedImage).data, np.uint8), 1)
        if image is None:
            continue
        image = cv2.resize(image, SIZE)
        lines = ['t=%.1f' % (t-t0)] + label(control)
        color = (0, 255, 0) if control.get('cone_active') else (0, 255, 255)
        for i, text in enumerate(lines):
            cv2.putText(image, text, (8, 22+i*22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3)
            cv2.putText(image, text, (8, 22+i*22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1)
        frames.append((t, image))
    if len(frames) < 2:
        sys.exit('영상 프레임 없음: %s' % path)
    fps = max(1.0, (len(frames)-1)/max(frames[-1][0]-frames[0][0], 1e-3))
    writer = cv2.VideoWriter(output, cv2.VideoWriter_fourcc(*'mp4v'), fps, SIZE)
    for _, image in frames:
        writer.write(image)
    writer.release()
    print('저장 %s: %d프레임, %.1f fps, %.1f s, %.1f MB' % (output, len(frames), fps, frames[-1][0]-frames[0][0],
                                                       os.path.getsize(output)/1e6))


if __name__ == '__main__':
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    main(*sys.argv[1:5])
