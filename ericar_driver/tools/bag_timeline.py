#!/usr/bin/env python3
"""주행 녹화 시계열: 움직인 구간만 0.2초 간격으로 거리·방향·조향·오차·검출 선 출력

실행 (차, ROS 환경): python3 tools/bag_timeline.py ~/bags/<이름>
열: 시각(s) / s 누적 거리(m) / yaw 누적 회전(°, odom 각속도 적분) / v 속도 / st 조향(rad, + 왼쪽)
    / err 목표점 옆 거리(m, + 왼쪽) / hd 차로 방향(rad) / n 검출 선 (l 왼쪽, r 오른쪽)
필요 토픽: /answer/detections, /odom (bringup record:=true 기본 포함)
"""
import json
import argparse

import rosbag2_py
from nav_msgs.msg import Odometry
from rclpy.serialization import deserialize_message
from std_msgs.msg import String


def main(path, step=0.2, moving=0.05, detail=False, start=0.0, end=float('inf')):
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
        if t-t0 > end:
            break
        if topic == '/odom':
            odom = deserialize_message(data, Odometry)
            if previous is not None:
                dist += abs(v) * (t - previous)
                yaw += odom.twist.twist.angular.z * (t - previous)
            v, previous = odom.twist.twist.linear.x, t
            continue
        if t-t0 < start or abs(v) < moving or t - last < step:
            continue
        last = t
        message = json.loads(deserialize_message(data, String).data)
        control, lane = message.get('control') or {}, message.get('lane') or {}
        line = '%6.1f s=%5.2f yaw=%+4.0f v=%.2f st=%+.3f err=%+.3f hd=%+.2f n=%s' % (
            t - t0, dist, yaw * 57.3, v,
            control.get('steering') if control.get('steering') is not None else float('nan'),
            control.get('lane_error') if control.get('lane_error') is not None else float('nan'),
            lane.get('heading') if lane.get('heading') is not None else float('nan'),
            ''.join(k[0] for k in lane.get('fits') or {}))
        if detail:
            # 녹화 당시 제어 상태 사용. 현재 코드로 재계산하지 않음
            keys = ['state', 'sensor_ready', 'speed', 'required_camera', 'signs_ok',
                    'front', 'rear', 'section', 'lane_index', 'avoiding', 'cone_active',
                    'lane_change_offset', 'lane_change_target', 'lane_change_heading',
                    'fork_request', 'fork_goal', 'fork_remaining',
                    'fork_sign', 'fork_sign_stamp', 'fork_sign_counts',
                    'fork_phase', 'fork_destination', 'fork_follow_travel', 'lane_change_direction',
                    'lane_change_debug']
            extra = {k: round(control[k], 3) if isinstance(control.get(k), float) else control.get(k)
                     for k in keys}
            extra['colors'] = lane.get('colors')
            for key in ('image_age', 'scan_age', 'image_stamp', 'lane_stamp', 'age'):
                extra[key] = message.get(key)
            line += ' ' + json.dumps(extra, ensure_ascii=False)
        print(line)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('path')
    parser.add_argument('--detail', action='store_true', help='차선 변경·분기 제어 상태 추가')
    parser.add_argument('--include-stopped', action='store_true', help='정지·센서 대기 구간 포함')
    parser.add_argument('--start', type=float, default=0., help='기존 시계열 기준 시작 초')
    parser.add_argument('--end', type=float, default=float('inf'), help='기존 시계열 기준 끝 초')
    args = parser.parse_args()
    main(args.path, moving=0. if args.include_stopped else .05,
         detail=args.detail, start=args.start, end=args.end)
