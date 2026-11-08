#!/usr/bin/env python3
"""주행 녹화 시계열: 움직인 구간만 0.2초 간격으로 거리·방향·조향·오차·검출 선 출력

실행 (차, ROS 환경): python3 tools/bag_timeline.py ~/bags/<이름>
열: 시각(s) / s 누적 거리(m) / yaw 누적 회전(°, odom 각속도 적분) / v 속도 / st 조향(rad, + 왼쪽)
    / err 목표점 옆 거리(m, + 왼쪽) / hd 차로 방향(rad) / n 검출 선 (l 왼쪽, r 오른쪽)
필요 토픽: /answer/detections, /odom (bringup record:=true 기본 포함)
"""
import json
import sys

import rosbag2_py
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from std_msgs.msg import String


def main(path, step=0.2, moving=0.05):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/answer/detections', '/odom']))
    v = dist = yaw = 0.0
    t0 = previous = None
    last = -1.0
    while reader.has_next():
        topic, data, t = reader.read_next()
        t *= 1e-9
        t0 = t0 or t
        if topic == '/odom':
            odom = deserialize_message(data, Odometry)
            if previous is not None:
                dist += abs(v) * (t - previous)
                yaw += odom.twist.twist.angular.z * (t - previous)
            v, previous = odom.twist.twist.linear.x, t
            continue
        if abs(v) < moving or t - last < step:
            continue
        last = t
        message = json.loads(deserialize_message(data, String).data)
        control, lane = message['control'], message.get('lane') or {}
        print('%6.1f s=%5.2f yaw=%+4.0f v=%.2f st=%+.3f err=%+.3f hd=%+.2f n=%s' % (
            t - t0, dist, yaw * 57.3, v, control['steering'], control['lane_error'] or 0,
            lane.get('heading') or 0, ''.join(k[0] for k in lane.get('fits') or {})))


if __name__ == '__main__':
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    main(sys.argv[1])
