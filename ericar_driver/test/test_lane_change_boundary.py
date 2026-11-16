"""노란 경계 횡단 차단 시험. python ericar_driver/test/test_lane_change_boundary.py"""
import copy
import sys
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
from ericar_driver.lane_change import LaneChange
from ericar_driver.params import ANSWER_CONFIG


def test_boundary():
    robot = yaml.safe_load((HERE.parent/'config/robot.yaml').read_text(encoding='utf-8'))['robot']
    for direction, side in [(1, 'left'), (-1, 'right')]:
        change = LaneChange(copy.deepcopy(ANSWER_CONFIG), robot)
        free = {0: 3., 1: 3., -1: 3.}
        yellow = dict(valid=True, colors={side: 'yellow'}, solid={side: True})
        assert change.choose_direction(free, yellow, requested=direction) == 0
        assert change.choose_direction(free, yellow) != direction
        dashed = dict(valid=True, colors={side: 'white'}, solid={side: False})
        assert change.choose_direction(free, dashed, requested=direction) == direction
        lane = dict(yellow, heading=0., error=0., stamp=1.)
        result = change.update(lane, dict(points=[], wide_objects=0), 0., 0., .05,
                               True, requested=direction, preferred=direction)
        assert not change.active and result is None, '경계 차단 후 이동 재시작'


def test_avoid_only():
    """멈춤 방지가 차선 변경을 끈 동안(avoid_only)에도 앞 장애물은 피함. 갈림길 요청은 무시, 갈 곳 없으면 대기 대신 None"""
    robot = yaml.safe_load((HERE.parent/'config/robot.yaml').read_text(encoding='utf-8'))['robot']
    lane = dict(valid=True, heading=0., error=0., stamp=1., colors={'left': 'white', 'right': 'white'},
                solid={'left': False, 'right': False})
    lidar_x = robot['lidar']['x']
    ahead = dict(points=[[0.9-lidar_x, 0.0]], wide_objects=0)          # 가운데 차로 0.9 m 앞 장애물
    change = LaneChange(copy.deepcopy(ANSWER_CONFIG), robot)
    assert change.update(lane, ahead, 0., 0., .05, True, avoid_only=True) is not None and change.active, '꺼진 동안 회피 안 함'
    change = LaneChange(copy.deepcopy(ANSWER_CONFIG), robot)
    clear = dict(points=[], wide_objects=0)
    assert change.update(lane, clear, 0., 0., .05, True, requested=1, avoid_only=True) is None and not change.active, \
        '꺼진 동안 갈림길 요청으로 이동'
    blocked = dict(points=[[0.9-lidar_x, 0.0], [0.5-lidar_x, 0.52], [0.5-lidar_x, -0.52]], wide_objects=0)
    change = LaneChange(copy.deepcopy(ANSWER_CONFIG), robot)
    assert change.update(lane, blocked, 0., 0., .05, True, avoid_only=True) is None and not change.waiting, \
        '꺼진 동안 갈 곳 없을 때 대기 (멈춤 방지 반복)'


def test_wall_filter():
    """벽(노란 바깥선 위) 점은 버리고 장애물 차 점은 남김. 카메라 방향 5° 틀려도 (소민 11/13)"""
    import math
    import numpy as np
    robot = yaml.safe_load((HERE.parent/'config/robot.yaml').read_text(encoding='utf-8'))['robot']
    change = LaneChange(copy.deepcopy(ANSWER_CONFIG), robot)
    change.index = 2                       # 왼쪽 끝 차로 중앙. 왼쪽 벽은 차 기준 0.26 m
    err = math.radians(5)
    wall = [(x, .26+.01) for x in np.arange(-.3, 3.0, .03)]
    right_wall = [(x, -.26-.52*2-.01) for x in np.arange(-.3, 3.0, .03)]
    obstacle = [(1.0, y) for y in np.arange(-.11, .11, .02)] + [(x, -.11) for x in np.arange(1.0, 1.32, .03)]
    true = np.array(wall+right_wall+obstacle)
    # 차가 도로에 대해 5° 틀어져 있는데 카메라는 0°로 봄: 차 기준 좌표로 돌림
    c, s_ = math.cos(-err), math.sin(-err)
    body = np.column_stack((true[:, 0]*c-true[:, 1]*s_, true[:, 0]*s_+true[:, 1]*c))
    kept = change.drop_walls(body, 0.0, 0.0, 2, {'left': 'yellow', 'right': 'white'})
    assert 'left' in change.walls, '벽 못 찾음'
    n_obstacle = len(obstacle)
    assert len(kept) >= n_obstacle, ('장애물 점 버림', len(kept), n_obstacle)
    left_kept = sum(1 for p in kept if p[1] > .2)
    assert left_kept == 0, ('벽 점 남음', left_kept)
    # 벽 없을 때는 아무것도 안 버림
    change = LaneChange(copy.deepcopy(ANSWER_CONFIG), robot)
    pts = np.array(obstacle + [(1.6, .52), (1.7, .52), (1.6, .45)])
    assert len(change.drop_walls(pts, 0.0, 0.0, 1, {'left': 'white', 'right': 'white'})) == len(pts) and not change.walls


def test_stale_seen_lane():
    """이동 중 비스듬히 본 차로 기억으로 다음 이동이 시작하자마자 끝나지 않음 (11/15 main_1115_1904 180.07 s)
    0→1 이동 중 왼쪽으로 약 30° 틀어진 채 바깥 노란 선 + 흰 점선(= 2번 차로처럼 보임)을 약 1초 봄
    → 1→2 이동 시작 주기에 camera_correct·overshoot로 종료, 가운데 차로인데 index 2 → 교통섬 직진"""
    robot = yaml.safe_load((HERE.parent/'config/robot.yaml').read_text(encoding='utf-8'))['robot']
    clear = dict(points=[], wide_objects=0)
    # 틀어진 자세에서 본 색 조합은 차로 판정에 안 씀
    change = LaneChange(copy.deepcopy(ANSWER_CONFIG), robot)
    change.index = 1
    oblique = dict(valid=True, heading=-.45, error=.2, colors={'left': 'yellow', 'right': 'white'})
    for k in range(20):
        change.observe_lane(dict(oblique, stamp=float(k)))
    assert change.index == 1 and change.seen_count == 0, '틀어진 자세의 선 색으로 차로 번호 변경'
    # 남은 기억이 있어도 새 이동은 시작 주기에 끝나지 않음
    change = LaneChange(copy.deepcopy(ANSWER_CONFIG), robot)
    change.index, change.seen_lane, change.seen_count = 1, 2, 20
    lane = dict(valid=True, heading=-.2, error=-.29, forward=.65, stamp=100., colors={'left': 'white'},
                solid={'left': False})
    result = change.update(lane, clear, .18, 0., .05, True, requested=1, preferred=1)
    assert result is not None and change.active and change.index == 1, '차선 변경이 시작 주기에 종료 (index %d)' % change.index
    # 이동이 끝나면 이동 중 기억은 비움
    change.offset, change.heading, change.back = change.target, 0.0, True   # 복귀 원호까지 끝난 상태
    change.seen_lane, change.seen_count = 2, 20
    change.update(dict(lane, stamp=101.), clear, .18, 0., .05, True, requested=1, preferred=1)
    assert not change.active and change.index == 2 and change.seen_count == 0

def test_island_yellow():
    """교통섬 노란 선(멀리서 시작하거나 비스듬함)은 바깥선으로 안 봄 (소민 11/14 run_1114_2330)"""
    robot = yaml.safe_load((HERE.parent/'config/robot.yaml').read_text(encoding='utf-8'))['robot']
    change = LaneChange(copy.deepcopy(ANSWER_CONFIG), robot)
    free = {0: 3., 1: 3., -1: 3.}
    base = dict(valid=True, heading=0., colors={'left': 'white', 'right': 'yellow'}, solid={'left': False, 'right': True})
    outer = dict(base, near_ends={'left': .6, 'right': .62}, line_headings={'left': 0., 'right': .02})
    far = dict(base, near_ends={'left': .6, 'right': 1.3}, line_headings={'left': 0., 'right': 0.})
    slant = dict(base, near_ends={'left': .6, 'right': .65}, line_headings={'left': 0., 'right': -.5})
    assert change.trusted_colors(outer)['right'] == 'yellow'
    assert change.trusted_colors(far)['right'] == 'island'
    assert change.trusted_colors(slant)['right'] == 'island'
    assert change.choose_direction(free, far, requested=-1) == -1, '교통섬 노란 선 때문에 표지판 이동 거절'
    assert change.choose_direction(free, outer, requested=-1) == 0, '진짜 바깥선은 계속 거절'
    # 가운데 차로에서 교통섬 노란 선만 보이면 차로 번호를 0으로 바꾸지 않음
    for k in range(10):
        change.observe_lane(dict(far, stamp=float(k)))
    assert change.index == 1
    for k in range(10, 20):
        change.observe_lane(dict(outer, stamp=float(k)))
    assert change.index == 0, '진짜 바깥선이면 번호 보정'


def test_fork_keeps_edge_index():
    """갈림길 표지판 확정 뒤에는 흰/흰(가운데) 판정으로 끝 차로 번호를 바꾸지 않음 (run_1114_2330 39.1 s)"""
    robot = yaml.safe_load((HERE.parent/'config/robot.yaml').read_text(encoding='utf-8'))['robot']
    change = LaneChange(copy.deepcopy(ANSWER_CONFIG), robot)
    change.index = 2
    change.fork_running = True
    middle = dict(valid=True, heading=0., colors={'left': 'white', 'right': 'white'})
    for k in range(10):
        change.observe_lane(dict(middle, stamp=float(k)))
    assert change.index == 2, '잠깐 흰/흰으로 끝 차로 번호가 바뀜'
    for k in range(10, 25):
        change.observe_lane(dict(middle, stamp=float(k)))
    assert change.index == 1, '오래 이어진 흰/흰은 고쳐야 함'


def test_chained_heading():
    """연속 이동은 직전 이동에서 적분해 온 방향을 이어 씀 (11/15 fix2_1115_2017 156.9 s: 카메라 한 프레임 방향 −0.48로
    두 번째 이동 시작 → 복귀 조향 과다 → 교통섬 직진). 떨어진 이동은 최근 카메라 방향 중앙값"""
    robot = yaml.safe_load((HERE.parent/'config/robot.yaml').read_text(encoding='utf-8'))['robot']
    clear = dict(points=[], wide_objects=0)
    base = dict(valid=True, error=0., forward=.65, colors={'left': 'white', 'right': 'white'},
                solid={'left': False, 'right': False})
    change = LaneChange(copy.deepcopy(ANSWER_CONFIG), robot)
    change.index = 0
    change.update(dict(base, heading=0., stamp=1.), clear, .18, 0., .05, True, requested=1, preferred=1)
    assert change.active
    change.offset, change.heading, change.back = change.target, -.04, True   # 첫 이동 끝: 도로보다 왼쪽 0.04 rad
    change.update(dict(base, heading=0., stamp=2.), clear, .18, 0., .05, True, requested=1, preferred=1)
    assert not change.active and change.index == 1
    # 바로 이어지는 두 번째 이동: 카메라가 −0.48을 줘도 이어 온 −0.04 사용
    change.update(dict(base, heading=-.48, stamp=3.), clear, .18, 0., .05, True, requested=1, preferred=1)
    assert change.active and abs(change.heading+.04) < .02, change.heading
    # 떨어진 이동: 최근 5프레임 중앙값 (한 프레임 튐 제외)
    change = LaneChange(copy.deepcopy(ANSWER_CONFIG), robot)
    change.index = 0
    for k, heading in enumerate([.02, .03, -.5, .02, .01]):
        change.update(dict(base, heading=heading, stamp=float(k)), clear, .18, 0., .05, False)
    change.update(dict(base, heading=-.6, stamp=9.), clear, .18, 0., .05, True, requested=1, preferred=1)
    assert change.active and abs(change.heading-.02) < .02, change.heading


def test_fork_exit_side_yellow():
    """표지판 확정 뒤: 출구 쪽 노란 선만 바깥선, 반대쪽 노란 선은 거리·각도와 상관없이 교통섬 (분기 근처 이동 중)"""
    robot = yaml.safe_load((HERE.parent/'config/robot.yaml').read_text(encoding='utf-8'))['robot']
    change = LaneChange(copy.deepcopy(ANSWER_CONFIG), robot)
    change.fork_running, change.fork_dir = True, 1          # left 표지판
    both_near = dict(valid=True, heading=.3, colors={'left': 'yellow', 'right': 'yellow'},
                     near_ends={'left': .6, 'right': .6}, line_headings={'left': .3, 'right': .3})
    colors = change.trusted_colors(both_near)
    assert colors == {'left': 'yellow', 'right': 'island'}
    free = {0: 3., 1: 3., -1: 3.}
    change.index = 1
    assert change.choose_direction(free, both_near, requested=1) == 0, '진짜 바깥선(출구 쪽) 넘어감'
    change.fork_dir = -1                                     # right 표지판이면 반대
    assert change.trusted_colors(both_near) == {'left': 'island', 'right': 'yellow'}
    change.fork_dir = 0                                      # 표지판 전: 둘 다 가까이·나란하면 둘 다 노랑
    assert change.trusted_colors(both_near) == {'left': 'yellow', 'right': 'yellow'}


if __name__ == '__main__':
    test_boundary()
    test_avoid_only()
    test_wall_filter()
    test_stale_seen_lane()
    test_island_yellow()
    test_fork_keeps_edge_index()
    test_chained_heading()
    test_fork_exit_side_yellow()
    print('노란 경계 횡단 차단 시험 통과')
