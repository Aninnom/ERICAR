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
from ericar_driver.params import ANSWER_CONFIG as DEFAULT_CONFIG
from ericar_driver.section import SectionManager

# 11/17 정리로 기본값에서 끈 기능들. 동작 시험은 켠 설정으로 함 (기본값이 꺼져 있는지는 test_defaults)
OPTIONAL = ('request_reverse', 'request_blocked_escape', 'safe_flip_block', 'back_end', 'lane_memory', 'camera_finish_center')
ANSWER_CONFIG = copy.deepcopy(DEFAULT_CONFIG)
for _name in OPTIONAL:
    ANSWER_CONFIG['lane_change'][_name] = True


def test_defaults():
    lc = DEFAULT_CONFIG['lane_change']
    # request_reverse는 11/18 오후 켰다가 저녁에 다시 끔 (redecide_1118_33, params.py 설명). 대신 소민 줄이기(request_shrink) 켬
    off = OPTIONAL+('backup',)
    assert not any(lc[name] for name in off), {name: lc[name] for name in off}
    assert lc['request_shrink'] and lc['request_extend'] and lc['entry_align'] and lc['short_arc_margin'] == 0.08
    area = DEFAULT_CONFIG['detector']['minimum_area_by_class']
    assert area['left'] == area['right'] == 0.003
    print('  기본값(끈 기능·켠 기능) 확인 통과')


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
    # 11/17 m08_1117_0417 31.7~32.1 s: 오른쪽으로 아직 5°쯤 틀어진 채 차선 오차 −0.04 (앞보기 0.67 m) → 실제는 중앙에서 0.10 m 못 미침
    # 예전: 오차만 보고 도착 처리. 지금: 방향을 보정한 중앙값으로 판정 → 도착 아님. 나란해져 중앙이면 도착
    change = LaneChange(copy.deepcopy(ANSWER_CONFIG), robot)
    change.active, change.direction, change.span, change.index = True, -1, 1, 1
    change.target, change.offset, change.heading = -0.53, -0.48, math.radians(5.0)
    change.seen_lane, change.seen_count = 0, 5
    early = dict(lane_with(math.radians(5.0), error=-0.04), forward=0.67)
    assert not change.camera_finished(early)
    change.heading = math.radians(1.0)
    assert change.camera_finished(dict(lane_with(math.radians(1.0), error=0.02), forward=0.67))
    change.c['camera_finish_center'] = False
    change.heading = math.radians(5.0)
    assert change.camera_finished(early)        # 끄면 예전 동작
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


def test_extend():
    """왼쪽 끝 차로에서 앞 장애물을 오른쪽으로 피하기 시작한 직후 오른쪽 표지판 확정 (fork_1117_0119 62.5~63.2 s)
    예전: 한 칸으로 끝남. 지금: 경로가 비어 있으면 진행 중인 이동을 두 칸으로 넓힘"""
    robot = robot_config()
    lidar_x = robot['lidar']['x']
    lane = dict(lane_with(0.0), colors={'left': 'yellow', 'right': 'white'})
    ahead = [[0.9-lidar_x, 0.0]]

    def started(extra=()):
        change = LaneChange(copy.deepcopy(ANSWER_CONFIG), robot)
        change.index = 2
        scan = dict(points=ahead+list(extra), wide_objects=0)
        change.update(dict(lane, stamp=1.0), scan, 0.15, 0.0, 0.05, True, road_heading=0.0)
        assert change.active and change.direction == -1 and change.span == 1, change.debug
        return change, scan
    change, scan = started()
    change.update(dict(lane, stamp=2.0), scan, 0.15, 0.0, 0.05, True, road_heading=0.0, fork_lanes=-2, request_limit=2.0)
    assert change.span == 2 and abs(change.target+1.04) < 0.05 and change.move_angle == 45.0 and not change.short, change.debug
    # 목적 차로 경로 위에 장애물: 넓히지 않음
    block = [[x-lidar_x, y] for x in (1.0, 1.1, 1.2) for y in (-0.95, -1.05, -1.15)]
    change, scan = started(block)
    change.update(dict(lane, stamp=2.0), scan, 0.15, 0.0, 0.05, True, road_heading=0.0, fork_lanes=-2, request_limit=2.0)
    assert change.span == 1 and 'extend_blocked' in change.debug, change.debug
    # 남은 거리(limit)가 모자람: 넓히지 않음. 기둥을 못 본 어림값은 request_extend_slack(0.5 m)까지 넘는 것 허용
    change, scan = started()
    change.update(dict(lane, stamp=2.0), scan, 0.15, 0.0, 0.05, True, road_heading=0.0, fork_lanes=-2, request_limit=0.9)
    assert change.span == 1, change.debug
    change, scan = started()
    change.update(dict(lane, stamp=2.0), scan, 0.15, 0.0, 0.05, True, road_heading=0.0, fork_lanes=-2, request_limit=1.0)
    assert change.span == 2, change.debug
    # 기둥 거리로 잰 한계는 그대로 지킴
    change, scan = started()
    change.anchor, change.anchor_gone = 0.6, 0.0
    change.update(dict(lane, stamp=2.0), scan, 0.15, 0.0, 0.05, True, road_heading=0.0, fork_lanes=-2, request_limit=1.0)
    assert change.span == 1, change.debug
    # 목적 차로가 한 칸(이미 가는 차로)이거나 반대 방향이면 그대로
    for lanes in (-1, 2, 0):
        change, scan = started()
        change.update(dict(lane, stamp=2.0), scan, 0.15, 0.0, 0.05, True, road_heading=0.0, fork_lanes=lanes, request_limit=2.0)
        assert change.span == 1, (lanes, change.debug)
    print('  진행 중인 이동을 두 칸으로 넓힘 통과')


def test_reverse():
    """이동 시작 뒤 표지판이 반대쪽으로 확정되면 경로가 비는 순간 목적 차로 쪽으로 되돌림 (11/17 ext_1117_0144·0135)"""
    from ericar_driver.lidar import to_body
    robot = robot_config()
    lidar_x = robot['lidar']['x']
    ahead = [[0.9-lidar_x, 0.0]]
    # ① 가운데 차로, 앞 0.9 m 기둥. 왼쪽으로 피하기 시작한 직후 오른쪽 확정 → 오른쪽 끝 차로로 되돌림 (0144 좌우 반대)
    scan = dict(points=ahead, wide_objects=0)
    change = LaneChange(copy.deepcopy(ANSWER_CONFIG), robot)
    change.update(lane_with(0.0, stamp=1.0), scan, 0.15, 0.0, 0.05, True, road_heading=0.0)
    assert change.active and change.direction == 1 and change.index == 1, change.debug
    result = change.update(lane_with(0.0, stamp=2.0), scan, 0.15, 0.0, 0.05, True, road_heading=0.0,
                           fork_lanes=-1, request_limit=2.0)
    assert change.direction == -1 and change.index+change.direction*change.span == 0, change.debug
    assert change.plan.get('reversed') and abs(change.target-change.offset+0.52) < 0.05 and result[1] < 0, (change.debug, result)
    # 목적 차로를 모르면(None) 그대로
    change = LaneChange(copy.deepcopy(ANSWER_CONFIG), robot)
    change.update(lane_with(0.0, stamp=1.0), scan, 0.15, 0.0, 0.05, True, road_heading=0.0)
    change.update(lane_with(0.0, stamp=2.0), scan, 0.15, 0.0, 0.05, True, road_heading=0.0, request_limit=2.0)
    assert change.direction == 1 and change.index == 1, change.debug

    # ② 왼쪽 끝 차로, 앞 0.9 m 장애물을 오른쪽으로 피하는 중 왼쪽(지금 떠난 차로) 확정 (0135)
    lane = dict(lane_with(0.0), colors={'left': 'yellow', 'right': 'white'})
    blind = dict(valid=False, heading=0.0, error=0.0, colors={}, solid={})

    def moving(obstacle_forward):
        """오른쪽으로 0.30 m 옮겼고 40° 틀어진 자세. 떠난 차로의 장애물 뒷면~앞면이 차 기준 도로 방향 obstacle_forward(m)"""
        change = LaneChange(copy.deepcopy(ANSWER_CONFIG), robot)
        change.index = 2
        change.update(dict(lane, stamp=1.0), dict(points=ahead, wide_objects=0), 0.15, 0.0, 0.05, True, road_heading=0.0)
        assert change.active and change.direction == -1, change.debug
        # 시작 직후: 남은 횡거리 0 또는 앞 장애물 → 되돌리지 않음
        change.update(dict(lane, stamp=2.0), dict(points=ahead, wide_objects=0), 0.15, 0.0, 0.05, True,
                      road_heading=0.0, fork_lanes=0, request_limit=2.0)
        assert change.direction == -1 and change.index == 2, change.debug
        change.offset = -0.30
        heading = math.radians(40)
        points = []
        for f in obstacle_forward:
            for l in (0.20, 0.30, 0.40):
                x, y = to_body(f, l, heading)
                points.append([x-lidar_x, y])
        result = change.update(dict(blind, stamp=3.0), dict(points=points, wide_objects=0), 0.1, 0.0, 0.05, True,
                               road_heading=heading, fork_lanes=0, request_limit=2.0)
        return change, result
    change, _ = moving((0.35, 0.45, 0.55, 0.65))          # 장애물이 아직 앞: 돌아갈 경로 막힘
    assert change.direction == -1 and change.index == 2 and 'reverse_blocked' in change.debug, change.debug
    change, result = moving((-0.65, -0.55, -0.45, -0.35))  # 장애물을 지나침: 바로 되돌림
    assert change.direction == 1 and change.index == 1 and change.span == 1 and change.plan.get('reversed'), change.debug
    assert abs(change.target-0.52) < 1e-6 and abs(change.offset-0.22) < 0.02 and result[1] > 0, (change.offset, result)
    # 끄면 예전 동작
    config = copy.deepcopy(ANSWER_CONFIG)
    config['lane_change']['request_reverse'] = False
    change = LaneChange(config, robot)
    change.update(lane_with(0.0, stamp=1.0), scan, 0.15, 0.0, 0.05, True, road_heading=0.0)
    change.update(lane_with(0.0, stamp=2.0), scan, 0.15, 0.0, 0.05, True, road_heading=0.0, fork_lanes=-1, request_limit=2.0)
    assert change.direction == 1, change.debug
    print('  반대쪽 이동을 목적 차로 쪽으로 되돌림 통과')


def test_blocked_escape():
    """가운데 차로, 왼쪽 표지판, 기둥 0.8 m 앞. 왼쪽 차로 옆 장애물(뒷축 앞 0.2~0.44 m, 빈 거리 기준에서는 빠짐) 때문에
    왼쪽 경로만 막힘 (11/17 ext_1117_0135 56.4 s). 예전: request_path_blocked로 차로 유지 → 기둥 앞까지 직진. 지금: 반대쪽으로 피함"""
    import numpy as np
    robot = robot_config()
    lidar_x = robot['lidar']['x']
    post = [[0.8-lidar_x, y] for y in (-0.02, 0.0, 0.02)]
    beside = [[x-lidar_x, y] for x in np.arange(0.20, 0.45, 0.03) for y in (0.36, 0.41)]
    scan = dict(points=post+beside, wide_objects=0)
    change = LaneChange(copy.deepcopy(ANSWER_CONFIG), robot)
    change.update(lane_with(0.0), scan, 0.15, 0.0, 0.05, True, requested=1, preferred=1, road_heading=0.0)
    assert change.active and change.direction == -1 and change.debug.get('escape') == 'path', change.debug
    config = copy.deepcopy(ANSWER_CONFIG)
    config['lane_change']['request_blocked_escape'] = False
    change = LaneChange(config, robot)
    result = change.update(lane_with(0.0), scan, 0.15, 0.0, 0.05, True, requested=1, preferred=1, road_heading=0.0)
    assert result is None and not change.active and change.debug.get('reason') == 'request_path_blocked', change.debug
    print('  요청 경로만 막힌 기둥 앞에서 반대쪽 회피 통과')


def test_flip_block():
    """오른쪽 차로, 앞끝 0.33 m 장애물(차 기준 약간 왼쪽), 왼쪽으로 이동 시작 직후 (11/17 rev_1117_0212 90.9 s 스캔)
    예전: 안전 조향이 오른쪽(−0.26)을 골라 전진 → 더 붙음. 지금: 정지"""
    import numpy as np
    robot = robot_config()
    lidar_x = robot['lidar']['x']
    obstacle = [[0.56-lidar_x, y] for y in (0.05, 0.10, 0.15)]+[[0.60-lidar_x, 0.05]]

    def moving(config):
        change = LaneChange(config, robot)
        change.index = 0
        change.active, change.direction, change.span, change.short = True, 1, 1, True
        change.target, change.offset, change.heading, change.back = 0.52, 0.0, 0.0, False
        return change, change.update(lane_with(0.0), dict(points=obstacle, wide_objects=0), 0.05, 0.0, 0.05, True, road_heading=0.0)
    change, result = moving(copy.deepcopy(ANSWER_CONFIG))
    assert result[0] == 0.0 and 'flip_blocked' in change.debug and change.debug['reason'] == 'no_safe_steering', (result, change.debug)
    config = copy.deepcopy(ANSWER_CONFIG)
    config['lane_change']['safe_flip_block'] = False
    change, result = moving(config)
    assert result[0] > 0 and result[1] < -0.1, (result, change.debug)     # 예전 동작: 반대로 꺾으며 전진
    print('  목표 반대쪽 안전 조향 금지 통과')


def test_backup():
    """앞 장애물 가까이에서 서 있으면 지나온 경로를 되짚어 후진 (11/17 rev_1117_0212)"""
    from ericar_driver.lane_change import BackUp
    robot = robot_config()
    config = copy.deepcopy(ANSWER_CONFIG)
    config['lane_change']['backup'] = True      # 기본값은 끔 (11/17 실차 2회 구조 실패). 동작 시험은 켜고 함
    assert not DEFAULT_CONFIG['lane_change']['backup']
    backup = BackUp(config, robot)
    change = LaneChange(config, robot)
    # 0.8 m를 왼쪽으로 돌며 전진 (곡률 1.5 /m → 조향 약 +0.29 rad). 최근 0.6 m만 남음
    for _ in range(160):
        backup.record(0.1, 0.15, 0.05)
    assert 0.55 < sum(d for d, _ in backup.trail) <= 0.66, backup.trail
    near = [[0.55, y] for y in (-0.05, 0.0, 0.05)]       # 뒷축 앞 0.55 m
    stopped = (0.0, 0.0)
    change.active, change.direction, change.span, change.offset, change.target = True, 1, 1, 0.05, 0.52
    # backup_wait(1.0 s) 전에는 시작 안 함
    for _ in range(19):
        assert backup.observe(stopped, change, near, 0.0, 0.05) is None
    command = backup.observe(stopped, change, near, 0.0, 0.05)
    assert command is not None and command[0] < 0 and backup.active and not change.active and change.index == 1, (command, backup.debug)
    assert abs(command[1]-math.atan(robot['wheelbase']*1.5)) < 0.03, command      # 지나온 원호와 같은 바퀴 각
    travelled = 0.0
    for _ in range(200):
        command = backup.step(-0.1, 0.05)
        if command is None:
            break
        travelled += 0.1*0.05
    assert not backup.active and 0.27 < travelled < 0.33, travelled
    # 앞이 멀면(0.95 m) 후진 안 함, 움직이는 중이면 안 함, 끄면 안 함
    backup = BackUp(config, robot)
    for _ in range(80):
        backup.record(0.1, 0.0, 0.05)
    far = [[0.95, 0.0]]
    assert all(backup.observe(stopped, change, far, 0.0, 0.05) is None for _ in range(60))
    assert all(backup.observe((0.1, 0.0), change, near, 0.1, 0.05) is None for _ in range(60))
    off = copy.deepcopy(ANSWER_CONFIG)
    off['lane_change']['backup'] = False
    backup = BackUp(off, robot)
    for _ in range(80):
        backup.record(0.1, 0.0, 0.05)
    assert all(backup.observe(stopped, change, near, 0.0, 0.05) is None for _ in range(60))
    # 한 자리에서 backup_max(2)번까지
    backup = BackUp(config, robot)
    done = 0
    for _ in range(4):
        for _ in range(40):
            backup.record(0.1, 0.0, 0.05)         # 0.2 m씩만 전진 (횟수 초기화 거리 1.0 m 안)
        for _ in range(40):
            if backup.observe(stopped, change, near, 0.0, 0.05) is not None:
                done += 1
                while backup.step(-0.1, 0.05) is not None:
                    pass
                break
    assert done == 2, done
    print('  막힌 뒤 후진 재시도 통과')


def test_back_end():
    """복귀 원호 중 횡이동은 목표에 왔는데(−0.54/−0.53) 방향 4.6°가 남음. 왼쪽 앞 장애물 때문에 정렬 조향이 막힘
    (11/17 back_1117_0249 128.9 s 스캔). 예전: 안전 조향을 따라 오른쪽으로 0.53 m 더 감. 지금: 이동 종료 + 차선 없이도 판단 유지 → 정지"""
    robot = robot_config()
    lidar_x = robot['lidar']['x']
    obstacle = [[x-lidar_x, y] for x, y in ((0.55, 0.12), (0.55, 0.17), (0.70, 0.16), (0.75, 0.16))]
    blind = dict(valid=False, heading=0.0, error=0.0, colors={}, solid={})
    heading = math.radians(4.6)

    def arriving(config):
        change = LaneChange(config, robot)
        change.index = 1
        change.active, change.direction, change.span, change.short, change.back = True, -1, 1, True, True
        change.target, change.offset, change.heading, change.travel = -0.53, -0.54, heading, 1.17
        return change, change.update(dict(blind, stamp=1.0), dict(points=obstacle, wide_objects=0), 0.1, 0.0, 0.05, True,
                                     road_heading=heading)
    change, result = arriving(copy.deepcopy(ANSWER_CONFIG))
    assert not change.active and change.index == 0 and 'back_end' in change.debug, change.debug
    assert change.debug.get('lane_memory') and result is not None and result[0] == 0.0, (result, change.debug)
    off = copy.deepcopy(ANSWER_CONFIG)
    off['lane_change']['back_end'] = False
    change, result = arriving(off)
    assert change.active and result[0] > 0 and result[1] < 0.1, (result, change.debug)      # 예전 동작: 계속 전진
    # 차선 기억: 이동이 끝난 지 오래면(3 s 넘음) 예전처럼 판단 안 함
    change = LaneChange(copy.deepcopy(ANSWER_CONFIG), robot)
    change.clock, change.ended_clock = 10.0, 5.0
    assert change.update(dict(blind, stamp=1.0), dict(points=obstacle, wide_objects=0), 0.1, 0.0, 0.05, True,
                         road_heading=heading) is None and change.debug.get('reason') == 'lane_invalid'
    print('  복귀 중 목표 도달 시 이동 종료·차선 기억 통과')


def test_backup_align():
    """차로 안에 선 경우: 뒤가 빈 것을 아는 거리 안에서 도로 방향으로 정렬하며 후진 (11/17 back_1117_0249)"""
    from ericar_driver.lane_change import BackUp
    robot = robot_config()
    config = copy.deepcopy(ANSWER_CONFIG)
    config['lane_change']['backup'] = True
    near = [[0.55, y] for y in (0.10, 0.15)]
    stopped = (0.0, 0.0)
    backup, change = BackUp(config, robot), LaneChange(config, robot)
    # 차선 변경(이동 거리 1.17 m)이 끝남 → 뒤 0.65 m는 빈 것으로 봄
    change.active, change.travel = True, 1.17
    backup.observe((0.1, 0.0), change, near, 0.1, 0.05, heading=0.3)
    change.active = False
    backup.observe((0.1, 0.0), change, near, 0.1, 0.05, heading=0.1)
    assert abs(backup.known_back-0.65) < 1e-6, backup.known_back
    heading = math.radians(8.0)         # 오른쪽으로 8° 틀어진 채 정지
    command = None
    for _ in range(21):
        command = backup.observe(stopped, change, near, 0.0, 0.05, heading=heading) or command
    assert backup.active and backup.mode == 'align' and command[0] < 0 and command[1] < -0.3, (command, backup.debug)   # 바퀴 오른쪽
    travelled = 0.0
    while True:
        command = backup.step(-0.1, 0.05, heading=0.0)
        if command is None:
            break
        assert abs(command[1]) < 1e-6
        travelled += 0.005
    assert 0.27 < travelled < 0.33 and abs(backup.known_back-0.35) < 0.03, (travelled, backup.known_back)
    # 뒤를 모르면(짧은 이동 직후) 되짚기, 이동 중간에 선 경우도 되짚기
    backup = BackUp(config, robot)
    for _ in range(100):
        backup.record(0.1, 0.0, 0.05)
    backup.known_back = 0.1
    for _ in range(21):
        backup.observe(stopped, change, near, 0.0, 0.05, heading=heading)
    assert backup.active and backup.mode == 'retrace', backup.debug
    backup = BackUp(config, robot)
    for _ in range(100):
        backup.record(0.1, 0.0, 0.05)
    change.active, change.travel, change.offset, change.direction, change.span = True, 0.5, 0.2, 1, 1
    for _ in range(21):
        backup.observe(stopped, change, near, 0.0, 0.05, heading=heading)
    assert backup.active and backup.mode == 'retrace' and not change.active, backup.debug
    # 차선 변경이 꺼진 구간에서는 비상 정지로 서 있어도 후진 안 함, 켜진 구간에서는 판단이 없어도(None) 비상 정지면 후진
    backup, change = BackUp(config, robot), LaneChange(config, robot)
    backup.known_back = 0.6
    assert all(backup.observe(None, change, near, 0.0, 0.05, blocked=True, enabled=False, heading=0.0) is None for _ in range(40))
    assert any(backup.observe(None, change, near, 0.0, 0.05, blocked=True, enabled=True, heading=0.0) is not None for _ in range(40))
    print('  차로 안 정렬 후진 통과')


if __name__ == '__main__':
    test_defaults()
    test_start_alignment()
    test_flip_block()
    test_back_end()
    test_backup()
    test_backup_align()
    test_extend()
    test_reverse()
    test_blocked_escape()
    test_anchor_wait()
    test_camera_guard()
    test_cone_advance()
    print('차선 변경 방향 기준 시험 통과')
