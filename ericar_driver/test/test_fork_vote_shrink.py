"""갈림길 화살표 다수결 확정·두 칸 → 한 칸 줄이기 시험 (11/18 소민, redecide_1118_19·20 재현)
python ericar_driver/test/test_fork_vote_shrink.py"""
import copy
import math
import sys
import types
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
from ericar_driver.driver import Driver
from ericar_driver.lane_change import LaneChange
from ericar_driver.params import ANSWER_CONFIG


def robot_config():
    return yaml.safe_load((HERE.parent/'config/robot.yaml').read_text(encoding='utf-8'))['robot']


def lane_with(heading=0.0, stamp=1.0):
    return dict(valid=True, heading=heading, error=0.0, stamp=stamp, forward=0.65,
                colors={'left': 'white', 'right': 'white'}, solid={'left': False, 'right': False})


# ---------- 화살표 다수결 ----------

def fake_driver(config):
    """confirmed()·arrow_vote()만 쓰는 가짜 Driver (ROS·센서 없이)"""
    d = types.SimpleNamespace(c=config, sign_counts={}, last_detection_stamp=None, arrow_history=[])
    d.arrow_vote = types.MethodType(Driver.arrow_vote, d)
    d.confirmed = types.MethodType(Driver.confirmed, d)
    return d


def first_confirm(frames, config):
    """frames: 검출 프레임마다 'l'·'r'·'-'. 처음 확정된 (프레임 번호, 방향) 또는 None"""
    d = fake_driver(config)
    for i, f in enumerate(frames):
        dets = [] if f == '-' else [dict(name='left' if f == 'l' else 'right', area=0.005)]
        got = d.confirmed(dets, stamp=float(i)) & {'left', 'right'}
        if got:
            return i, next(iter(got))
    return None


def test_arrow_vote():
    """11/18 녹화의 검출 순서(fork_sign_counts로 복원)로 확정 시점 비교. 실제 방향: 05·19·20 left, 17 right"""
    on = copy.deepcopy(ANSWER_CONFIG)
    off = copy.deepcopy(ANSWER_CONFIG)
    off['detector']['arrow_vote']['enabled'] = False
    runs = {
        '05': ('lll-l', 'left'),
        '17': ('rrrrrrr', 'right'),
        '19': ('rlrl-llr-lllll', 'left'),
        '20': ('rlrrlllrlll', 'left'),
    }
    for name, (frames, truth) in runs.items():
        old, new = first_confirm(frames, off), first_confirm(frames, on)
        assert old is not None and old[1] == truth, (name, old)
        assert new is not None and new[1] == truth, (name, new)       # 틀린 확정 없음
        assert new[0] <= old[0], (name, old, new)                       # 늦어지지 않음
    # 19: 연속 3은 12번째 프레임, 다수결은 7번째 (l이 6중 4, r 1)
    assert first_confirm(runs['19'][0], off)[0] == 11 and first_confirm(runs['19'][0], on)[0] == 6
    # 반대 화살표가 2번 섞이면 다수결로는 확정 안 함 (20 앞부분 r l r r: 5중 3이면 right로 잘못 확정되던 경우)
    assert first_confirm('rlrrl', on) is None
    # 이번 프레임이 다른 방향이거나 비면 확정 안 함
    assert first_confirm('llrl-', on) is None
    print('  화살표 다수결 확정 통과')


# ---------- 두 칸 → 한 칸 줄이기 ----------

def moving(config, offset, heading, index=2, direction=-1, short=True):
    """index 차로에서 direction 쪽 두 칸 이동 중인 상태를 바로 만듦 (redecide_1118_20 101.8 s: 2 → 0, 횡이동 −0.06, 28°)"""
    robot = robot_config()
    change = LaneChange(config, robot)
    change.index, change.direction, change.span = index, direction, 2
    change.active, change.target, change.offset = True, direction*2*config['lane_change']['lane_width'], offset
    change.heading, change.move_angle, change.short = heading, 45.0, short
    return change, robot


def step(change, scan, fork_lanes, heading):
    return change.update(lane_with(stamp=change.clock+1.0), scan, 0.12, 0.0, 0.05, True,
                         road_heading=heading, fork_lanes=fork_lanes)


def test_shrink():
    config = copy.deepcopy(ANSWER_CONFIG)
    width = config['lane_change']['lane_width']
    empty = dict(points=[], wide_objects=0)
    # ① 20 상황: 목적 = 출발 차로(fork_lanes 0), 아직 0.06 m만 옴 → 한 칸(가운데 차로)으로 줄임
    change, _ = moving(copy.deepcopy(config), -0.06, 0.5)
    step(change, empty, 0, 0.5)
    assert change.span == 1 and change.shrunk and abs(change.target+width) < 1e-6, change.debug
    assert 'shrink' in change.debug, change.debug
    # 목적이 반대쪽 한 칸 너머(+1)여도 가운데가 더 가까움 → 줄임
    change, _ = moving(copy.deepcopy(config), -0.06, 0.5)
    step(change, empty, 1, 0.5)
    assert change.span == 1 and change.shrunk, change.debug
    # 목적 = 가운데 차로(−1)도 줄임
    change, _ = moving(copy.deepcopy(config), -0.06, 0.5)
    step(change, empty, -1, 0.5)
    assert change.span == 1, change.debug
    # ② 목적이 도착 차로(−2)이거나 모르면(None) 그대로 두 칸
    for lanes in (-2, None):
        change, _ = moving(copy.deepcopy(config), -0.06, 0.5)
        step(change, empty, lanes, 0.5)
        assert change.span == 2 and not change.shrunk, (lanes, change.debug)
    # ③ 늦음: 가운데 차로 중앙까지 남은 횡거리가 되돌리는 원호 몫보다 작음 → 그대로
    change, _ = moving(copy.deepcopy(config), -0.45, 0.7)
    step(change, empty, 0, 0.7)
    assert change.span == 2 and change.debug.get('shrink_blocked', {}).get('why') == 'late', change.debug
    # ④ 가운데 차로 쪽 경로가 막힘 → 그대로 (다음 주기에 다시 봄)
    lidar_x = robot_config()['lidar']['x']
    block = [[x-lidar_x, y] for x in (0.30, 0.35, 0.40, 0.45) for y in (-0.10, -0.20, -0.30)]
    change, _ = moving(copy.deepcopy(config), -0.06, 0.0)
    step(change, dict(points=block, wide_objects=0), 0, 0.0)
    assert change.span == 2 and change.debug.get('shrink_blocked', {}).get('why') == 'path_blocked', change.debug
    # ⑤ 끄면 예전 동작
    off = copy.deepcopy(config)
    off['lane_change']['request_shrink'] = False
    change, _ = moving(off, -0.06, 0.5)
    step(change, empty, 0, 0.5)
    assert change.span == 2, change.debug
    # ⑥ 줄인 뒤 도착 차로 앞이 막혀도 재판단(redecide)이 다시 두 칸으로 넓히지 않음
    redecide = copy.deepcopy(config)
    redecide['lane_change']['redecide'] = True
    change, _ = moving(redecide, -0.06, 0.5)
    step(change, empty, 0, 0.5)
    assert change.span == 1
    near = [[0.5-lidar_x, -0.4], [0.55-lidar_x, -0.45]]
    step(change, dict(points=near, wide_objects=0), 0, 0.5)
    assert change.span == 1 and 'redecide' not in change.debug, change.debug
    print('  두 칸 → 한 칸 줄이기 통과')


def test_redecide_fork_guard():
    """run_1118_1627 64.7 s: 오른쪽 확정(목적 0)인데 차로 0 앞 장애물을 왼쪽으로 피하는 한 칸 이동을 재판단이 두 칸(→2)으로 넓힘
    지금: 확정 뒤 목적 차로에서 더 멀어지는 넓히기는 막음. 목적 쪽(둘째 차로가 목적)이거나 확정 전이면 예전처럼 넓힘"""
    config = copy.deepcopy(ANSWER_CONFIG)
    config['lane_change']['redecide'] = True
    robot = robot_config()
    lidar_x = robot['lidar']['x']
    near = [[0.9-lidar_x+dx, 0.3+dy] for dx in (0.0, 0.05) for dy in (0.0, 0.05)]   # 도착할 차로(왼쪽 한 칸) 앞 0.9 m 장애물

    def one_lane(fork_lanes, cfg=config):
        change = LaneChange(copy.deepcopy(cfg), robot)
        change.index, change.direction, change.span = 0, 1, 1
        change.active, change.target, change.offset, change.heading = True, cfg['lane_change']['lane_width'], 0.05, 0.0
        change.update(lane_with(stamp=1.0), dict(points=near, wide_objects=0), 0.12, 0.0, 0.05, True,
                      road_heading=0.0, fork_lanes=fork_lanes)
        return change
    change = one_lane(0)          # 목적 = 출발 차로(0): 넓히면 멀어짐 → 막음
    assert change.span == 1 and change.debug.get('redecide_blocked', {}).get('reason') == 'away_from_fork', change.debug
    change = one_lane(1)          # 목적 = 가운데: 막음
    assert change.span == 1, change.debug
    change = one_lane(None)       # 확정 전: 예전처럼 넓힘
    assert change.span == 2 and 'redecide' in change.debug, change.debug
    change = one_lane(2)          # 목적 = 둘째 차로: 넓힘
    assert change.span == 2, change.debug
    off = copy.deepcopy(config)
    off['lane_change']['redecide_fork_guard'] = False
    change = one_lane(0, off)     # 끄면 예전 동작(넓힘)
    assert change.span == 2, change.debug
    print('  확정 뒤 반대쪽 재판단 넓히기 막음 통과')


def test_shrink_drive():
    """줄인 뒤 끝까지 주행: 가운데 차로(1)에서 나란히 끝나고 지나치지 않음 (자전거 모형, 횡이동·방향 적분)
    카메라 차선은 안 보이는 것으로 둠 (카메라 도착 판단 없이 원호만 확인). 조향 지연이 없는 모형이라
    복귀 원호를 여유(short_arc_margin 0.08 m)만큼 일찍 시작해 목표보다 약 0.08 m 덜 감 (실차는 지연으로 메워짐)"""
    config = copy.deepcopy(ANSWER_CONFIG)
    width = config['lane_change']['lane_width']
    change, robot = moving(config, -0.06, 0.5)
    heading, offset, v, dt = 0.5, -0.06, 0.12, 0.05
    empty = dict(points=[], wide_objects=0)
    for _ in range(400):
        if not change.active:
            break
        blind = dict(valid=False, heading=0.0, error=0.0, colors={}, solid={}, stamp=change.clock+1.0)
        result = change.update(blind, empty, v, 0.0, dt, True, road_heading=heading, fork_lanes=0)
        steering = result[1] if result else 0.0
        offset -= v*math.sin(heading)*dt
        heading -= v*math.tan(steering)/robot['wheelbase']*dt     # heading + = 오른쪽, 조향 + = 왼쪽
    assert not change.active and change.index == 1, (change.active, change.index, change.debug)
    assert -width-0.03 < offset < -width+0.10 and abs(heading) < 0.06, (offset, heading)
    print(f'  줄인 이동 끝까지 주행 통과 (끝 횡위치 {offset:+.3f} m, 목표 {-width:+.2f}, 방향 {math.degrees(heading):+.1f}°)')


def test_defaults():
    assert ANSWER_CONFIG['lane_change']['request_shrink'] is True and ANSWER_CONFIG['lane_change']['redecide_fork_guard'] is True
    vote = ANSWER_CONFIG['detector']['arrow_vote']
    assert vote['enabled'] and (vote['window'], vote['count'], vote['opposite_max']) == (6, 4, 1)
    print('  기본값 확인 통과')


if __name__ == '__main__':
    test_defaults()
    test_arrow_vote()
    test_shrink()
    test_redecide_fork_guard()
    test_shrink_drive()
    print('갈림길 다수결·두 칸 줄이기 시험 통과')
