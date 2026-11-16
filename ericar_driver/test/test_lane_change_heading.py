"""차선 변경 방향 기준·카메라 도착 판단·라바콘 구간 전환 시험 (11/16 anchor_1116_2313 재현)
python ericar_driver/test/test_lane_change_heading.py"""
import copy
import math
import sys
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
from ericar_driver.lane_change import LaneChange
from ericar_driver.params import ANSWER_CONFIG
from ericar_driver.section import SectionManager


def robot_config():
    return yaml.safe_load((HERE.parent/'config/robot.yaml').read_text(encoding='utf-8'))['robot']


def lane_with(heading, error=0.0, stamp=1.0):
    return dict(valid=True, heading=heading, error=error, stamp=stamp, forward=0.65,
                colors={'left': 'white', 'right': 'white'}, solid={'left': False, 'right': False})


def scene(robot, yaw_deg, distance, camera_deg=None):
    """차가 도로 방향에서 yaw_deg(+ 오른쪽)만큼 틀어진 채 차로 중앙에 있고, 같은 차로 distance(m) 앞에 장애물 하나
    반환 (lane, scan). 카메라 차선 방향은 camera_deg(기본 = 실제 값), 차로 중앙은 실제 위치대로"""
    yaw = math.radians(yaw_deg)          # 도로가 차 기준 왼쪽으로 yaw만큼 보임
    point = [distance*math.cos(yaw)-robot['lidar']['x'], distance*math.sin(yaw)]
    heading = yaw if camera_deg is None else math.radians(camera_deg)
    lane = lane_with(heading, error=0.65*math.tan(yaw))
    return lane, dict(points=[point], wide_objects=0)


def test_start_alignment():
    """코너 출구: 실제는 도로보다 왼쪽 22°인데 카메라 차선 방향은 오른쪽 27° (138.9 s)
    예전: 카메라 값으로 시작 → 반대로 감. 지금: 틀어져 있는 동안 시작 안 함, 정렬되면 IMU 값으로 시작"""
    robot = robot_config()
    lane, ahead = scene(robot, -22, 1.3, camera_deg=27)      # 지금 차로 1.3 m 앞 장애물, 옆 차로는 빔
    change = LaneChange(copy.deepcopy(ANSWER_CONFIG), robot)
    result = change.update(lane, ahead, 0.2, 0.0, 0.05, True, road_heading=math.radians(-22))
    assert result is None and not change.active and change.debug.get('reason') == 'not_aligned', change.debug
    # 정렬 뒤(왼쪽 6°): 시작하고, 시작 방향은 IMU 값 (카메라가 20° 틀려도)
    lane, ahead = scene(robot, -6, 1.3, camera_deg=14)
    change = LaneChange(copy.deepcopy(ANSWER_CONFIG), robot)
    result = change.update(lane, ahead, 0.2, 0.0, 0.05, True, road_heading=math.radians(-6))
    assert change.active and result is not None, change.debug
    assert abs(change.heading-math.radians(-6)) < 1e-6, math.degrees(change.heading)
    # 앞 장애물이 stop_distance 안이면 틀어져 있어도 시작 (정렬을 기다리다 부딪히지 않게), 방향은 IMU 값
    lane, close = scene(robot, -22, 0.5, camera_deg=27)
    change = LaneChange(copy.deepcopy(ANSWER_CONFIG), robot)
    change.update(lane, close, 0.2, 0.0, 0.05, True, road_heading=math.radians(-22))
    assert change.active and abs(change.heading-math.radians(-22)) < 1e-6, change.debug
    # IMU 기준 없음(road_heading None): 예전 동작 그대로 (카메라 값으로 시작)
    lane, ahead = scene(robot, 8, 1.3)
    change = LaneChange(copy.deepcopy(ANSWER_CONFIG), robot)
    change.update(lane, ahead, 0.2, 0.0, 0.05, True)
    assert change.active and abs(change.heading-math.radians(8)) < 1e-6, change.debug
    print('  정렬 전 시작 안 함·IMU 방향으로 시작 통과')


def test_camera_guard():
    """오른쪽으로 34° 틀어진 채 흰/흰(가운데 차로)이 보여도 도착으로 보지 않음 (188.6 s)"""
    robot = robot_config()
    change = LaneChange(copy.deepcopy(ANSWER_CONFIG), robot)
    change.active, change.direction, change.span, change.index = True, -1, 1, 2
    change.target, change.offset, change.heading = -0.52, -0.40, math.radians(34)
    change.seen_lane, change.seen_count = 1, 5
    skewed = lane_with(math.radians(15), error=0.03)     # 카메라는 15°로 읽음 (실제 34°)
    assert not change.camera_correct(skewed) and not change.camera_finished(skewed)
    assert abs(change.offset+0.40) < 1e-9 and abs(change.heading-math.radians(34)) < 1e-9, '덮어쓰면 안 됨'
    # 나란해진 뒤(4°, 카메라도 5°): 예전처럼 카메라로 보정·종료
    change.heading = math.radians(4)
    aligned = lane_with(math.radians(5), error=0.03)
    assert change.camera_correct(aligned) and change.camera_finished(aligned)
    # 끄면 예전 동작
    change = LaneChange(dict(copy.deepcopy(ANSWER_CONFIG)), robot)
    change.c['camera_guard'] = False
    change.active, change.direction, change.span, change.index = True, -1, 1, 2
    change.target, change.offset, change.heading = -0.52, -0.40, math.radians(34)
    change.seen_lane, change.seen_count = 1, 5
    assert change.camera_correct(skewed)
    print('  비스듬한 자세의 카메라 도착 판정 차단 통과')


def test_cone_advance():
    """3번 코너 유지가 누적 회전 60°에서 끝나고(전환 기준 65°) 라바콘이 시작 → 보류 대신 S3로 전환 (226.3 s)"""
    sections = SectionManager(copy.deepcopy(ANSWER_CONFIG['sections']), 2)
    sections.yaw = -math.radians(180+60)
    assert not sections.update(0.0, 0.2, 0.05, hold=True) and sections.index == 2, '보류면 S2 유지 (예전 동작)'
    assert sections.update(0.0, 0.2, 0.05, hold=True, advance=True) and sections.index == 3
    assert abs(sections.turned-(60-90)) < 0.5 and sections.distance == 0.0, sections.turned
    # 4번 코너까지 65° 더 돌면 S4
    sections.yaw = -math.radians(270+66)
    assert sections.update(0.0, 0.2, 0.05) and sections.index == 4
    print('  라바콘 시작 시 구간 전환 통과')


def test_anchor_wait():
    """가운데 차로, 오른쪽 표지판 확정, 기둥 앞 1.4 m. 오른쪽 차로는 옆 장애물로 잠깐 막힘, 왼쪽은 빔 (imu_1116_2348 112.4 s)
    예전: 요청이 거부되면 일반 회피가 왼쪽으로 감 → 반대 길. 지금: 차로 유지하며 대기. 기둥이 anchor_escape 안이면 회피 허용"""
    import numpy as np
    robot = robot_config()
    lidar_x = robot['lidar']['x']
    post = [[1.4-lidar_x, y] for y in (-0.02, 0.0, 0.02)]                       # 폭 4 cm 기둥
    beside = [[x-lidar_x, -0.41] for x in np.arange(0.55, 0.90, 0.03)]          # 오른쪽 차로 장애물 옆면 (앞 0.55~0.9 m)
    scan = dict(points=post+beside, wide_objects=0)
    lane = lane_with(0.0)
    change = LaneChange(copy.deepcopy(ANSWER_CONFIG), robot)
    result = change.update(lane, scan, 0.15, 0.0, 0.05, True, requested=-1, preferred=-1, road_heading=0.0)
    assert change.anchor_front is not None and abs(change.anchor_front-1.4) < 0.05, change.debug
    assert result is None and not change.active and change.debug.get('reason') == 'request_wait_anchor', change.debug
    # 기둥이 0.8 m(anchor_escape 1.0 안)까지 왔는데도 오른쪽이 막혀 있으면 반대쪽이라도 피함
    post = [[0.8-lidar_x, y] for y in (-0.02, 0.0, 0.02)]
    beside = [[x-lidar_x, -0.41] for x in np.arange(0.50, 0.80, 0.03)]
    change = LaneChange(copy.deepcopy(ANSWER_CONFIG), robot)
    lane2 = lane_with(0.0, stamp=2.0)
    change.update(lane2, dict(points=post+beside, wide_objects=0), 0.15, 0.0, 0.05, True,
                  requested=-1, preferred=-1, road_heading=0.0)
    assert change.active and change.direction == 1, change.debug
    print('  기둥 앞에서 반대쪽 회피 대신 대기 통과')


if __name__ == '__main__':
    test_start_alignment()
    test_anchor_wait()
    test_camera_guard()
    test_cone_advance()
    print('차선 변경 방향 기준 시험 통과')
