#!/usr/bin/env python3
"""녹화 재생: 녹화 카메라 영상을 지금 코드의 LaneVision·LaneDriving에 다시 넣어 결과 출력 (CSV)

코드 수정 전후를 같은 영상으로 비교할 때 사용. 차를 움직이지 않음 (개루프: 녹화 당시 경로 고정)
실행 (차, ROS 환경): python3 tools/replay_bag.py ~/bags/<이름> > after.csv
  수정 전과 비교: main으로 같은 명령 → before.csv, 이후 diff before.csv after.csv
열: 시각(s), 검출 여부, 목표점 옆 거리(m), 목표점 앞 거리(m), 검출 선, pursuit 사용, 조향(rad), 정지선 거리(m, 없으면 빈칸)
"""
import sys
from pathlib import Path

import cv2
import numpy as np
import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CompressedImage

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))
from ericar_driver.lane_driving import LaneDriving  # noqa: E402
from ericar_driver.lane_vision import LaneVision  # noqa: E402
from ericar_driver.params import ANSWER_CONFIG  # noqa: E402


def main(path):
    robot = yaml.safe_load(open(PACKAGE / 'config' / 'robot.yaml'))['robot']
    vision = LaneVision(ANSWER_CONFIG['lane'], robot['camera'], robot['front_axle_x'])
    driving = LaneDriving(ANSWER_CONFIG, robot)
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/camera/color/image_raw/compressed']))
    t0 = previous = None
    while reader.has_next():
        _, data, t = reader.read_next()
        t *= 1e-9
        t0 = t0 or t
        image = cv2.imdecode(np.frombuffer(deserialize_message(data, CompressedImage).data, np.uint8), 1)
        lane = vision.analyze(image)
        lane['stamp'] = t
        _, steering = driving.update(lane, t, 0.035 if previous is None else t - previous)
        previous = t
        stop = '' if lane['stop_distance'] is None else '%.3f' % lane['stop_distance']
        print('%.2f,%d,%.4f,%.3f,%s,%d,%.3f,%s' % (
            t - t0, lane['valid'], lane['error'], lane.get('forward', 0), ''.join(k[0] for k in lane['fits']),
            lane.get('pursuit', False), steering, stop))


if __name__ == '__main__':
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    main(sys.argv[1])
