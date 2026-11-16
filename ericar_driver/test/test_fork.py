"""YOLO 분기 상태 시험. ROS 불필요, python ericar_driver/test/test_fork.py"""
import copy
import sys
from pathlib import Path

import yaml
import numpy as np
import cv2

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
from ericar_driver.driver import Driver
from ericar_driver.params import ANSWER_CONFIG
from ericar_driver.lane_vision import LaneVision


def driver():
    c = copy.deepcopy(ANSWER_CONFIG)
    c['missions']['start_stage'] = 'fork'
    robot = yaml.safe_load((HERE.parent / 'config/robot.yaml').read_text(encoding='utf-8'))['robot']
    return Driver({'answer': c, 'robot': robot})


def signs(d, names, t, wide=0):
    detections = [dict(name=n, area=1.0) for n in names]
    d.update_signs({'wide_objects': wide}, detections, t, .2, t, .05)


def test_conflicting_arrows():
    d = driver()
    for t in (0., .1, .2):
        signs(d, ['left', 'right'], t)
    assert 'fork' not in d.done, '동시 화살표로 방향 확정'


def test_both_directions():
    for name, start, destination, direction in [('left', 0, 2, 1), ('right', 2, 0, -1)]:
        d = driver()
        d.lane_change.index = start
        for t in (0., .1, .2):
            signs(d, [name], t, wide=1)
        assert d.fork.phase == 'locked' and d.fork_request == direction
        for i in range(15):
            signs(d, [], .2+i*.05)
        assert d.fork_request == direction
        d.lane_change.active = True
        signs(d, [], 1.)
        assert d.fork_request == 0 and d.fork.direction == direction, '이동 중 목적지 변경'
        d.lane_change.active = False
        d.lane_change.index = start+direction
        signs(d, [], 1.1)
        assert d.fork_request == direction, '두 번째 차로 이동 누락'
        d.lane_change.index = destination
        signs(d, [], 1.2)
        assert d.fork.phase == 'follow' and d.fork_request == 0
        for i in range(400):
            signs(d, [], 2+i*.05)
        assert d.fork.phase == 'done' and d.fork_request == 0
        assert d.fork.sign == name


def test_immediate_and_memory():
    d = driver()
    for t in (0., .1, .2):
        signs(d, ['right'], t)
    assert d.fork_request == -1, '장애물 없을 때 즉시 요청 누락'
    for i in range(4):
        signs(d, ['left'], 1+i*.05, wide=1)
    # 짧은 반대 검출(4프레임)로는 안 바뀜. fork_relock_frames(5) 연속이면 교체 (11/15, test_fork_prepare에서 확인)
    assert d.fork_goal == -1 and d.fork_request == -1, '방향 기억 변경'
    assert d.fork_direction() == 0, '정렬 전 갈림길 선 보정 활성'
    d.lane_change.index = 0
    signs(d, [], 10)
    assert d.fork_request == 0 and d.fork_direction() == -1


def test_fork_boundaries():
    row = 400.
    for direction, inner, outer in [(1, 'right', 'left'), (-1, 'left', 'right')]:
        v = LaneVision(copy.deepcopy(ANSWER_CONFIG['lane']), {}, 0.2)
        v.fork_direction = direction
        slope = direction*.3*v.scale*v.row_m
        fit = np.array([0., slope, v.c['width_px']/2-slope*row])
        # 화면 중앙의 교통섬 경계: 이전 잘못된 이름보다 출구 형상 우선
        got = v.fork_sources({outer: 'left'}, {'left': fit}, {'left': (row, v.c['width_px']/2)},
                             {'left': 'yellow'}, v.c['lane_width']*v.scale)
        assert got == {inner: 'left'}, got
        x = v.c['width_px']/2-direction*.26*v.scale
        got = v.fork_sources({inner: 'left'}, {'left': fit}, {'left': (row, x)},
                             {'left': 'yellow'}, v.c['lane_width']*v.scale)
        assert got == {outer: 'left'}, got
        # BEV 픽셀 → 탐색 → 이름 → 목표점 전체 경로에서 잘못된 바깥선 힌트보다 교통섬 형상 우선
        v = LaneVision(copy.deepcopy(ANSWER_CONFIG['lane']), {}, .2)
        v.fork_direction = direction
        v.outer_yellow = outer
        mask = np.zeros((v.c['height_px'], v.c['width_px']), np.uint8)
        x = int(v.c['width_px']/2)
        shift = int(direction*.3*(v.c['far']-v.c['near'])*v.scale)
        cv2.line(mask, (x, mask.shape[0]-1), (x-shift, 0), 255, 4)
        result = v.analyze_bird(mask)
        assert result['valid'] and set(result['fits']) == {inner}, result['fits']
        assert direction*result['error'] > 0, '교통섬 반대쪽 목표점 생성 실패'
    v.fork_direction = 0
    assert v.fork_sources({'left': 'left'}, {}, {}, {}, 104) == {'left': 'left'}


def test_requested_uses_current_heading():
    d = driver()
    change = d.lane_change
    change.heading = -1.5
    lane = dict(valid=True, heading=0., error=0., colors={}, solid={}, stamp=1.)
    scan = dict(points=[], wide_objects=0)
    result = change.update(lane, scan, 0., 0., .05, True, requested=1)
    assert change.active and change.direction == 1 and change.heading == 0.
    assert result[0] > 0


def test_fork_keeps_tracked_inner_boundary():
    # run_1112_0326: A 오른쪽 경계가 반대 방향으로 꺾인 뒤 왼쪽 이름으로 강제 변경된 조건
    for direction, inner, outer in [(1, 'right', 'left'), (-1, 'left', 'right')]:
        v = LaneVision(copy.deepcopy(ANSWER_CONFIG['lane']), {}, .2)
        v.fork_direction, v.outer_yellow = direction, outer
        rows = np.arange(360., 420.)
        slope = -direction*.5*v.scale*v.row_m
        fit = np.array([0., slope, v.c['width_px']/2-slope*400.])
        v.memory = {inner: (rows, np.polyval(fit, rows), 0)}
        source = v.fork_sources({inner: 'right'}, {'right': fit},
                               {'right': (400., v.c['width_px']/2)}, {'right': 'yellow'},
                               v.c['lane_width']*v.scale)
        assert source == {inner: 'right'}, '교통섬 경계 이름 반전: '+str(source)
        mask = np.zeros((v.c['height_px'], v.c['width_px']), np.uint8)
        near = int(np.polyval(fit, mask.shape[0]-1))
        far = int(np.polyval(fit, 0))
        cv2.line(mask, (near, mask.shape[0]-1), (far, 0), 255, 4)
        # 차선 변경 중(출구 보정 전) → 종료 후 전환에서도 이름 유지
        v.fork_direction = 0
        changing = v.analyze_bird(mask)
        assert changing['valid'] and set(changing['fits']) == {inner}, changing['fits']
        v.fork_direction = direction
        result = v.analyze_bird(mask)
        assert result['valid'] and set(result['fits']) == {inner}, result['fits']
        # A의 오른쪽 경계에서는 경계보다 왼쪽 목표, B는 반대. 꺾임 방향과 독립
        assert direction*result['error'] > 0, result['error']


def test_blocked_target_keeps_avoidance():
    d = driver()
    lane = dict(valid=True, heading=0., error=0., colors={}, solid={}, stamp=1.)
    x = d.robot['lidar']['x']
    # 앞·오른쪽 막힘, 왼쪽 여유. 오른쪽 출구 요청 중에도 왼쪽 회피 가능
    scan = dict(points=[[.8-x, 0.], [.7-x, -.52]], wide_objects=2)
    result = d.lane_change.update(lane, scan, 0., 0., .05, True, requested=-1, preferred=-1)
    assert d.lane_change.active and d.lane_change.direction == 1
    assert result is not None


def arrive_after_diagonal(d, area):
    """right 확정(왼쪽 끝 차로) → 두 칸을 비스듬히 건너는 동안(40° 틀어짐) 표지판을 area로 봄 → 오른쪽 끝 차로 도착"""
    d.lane_change.index = 2
    for t in (0., .1, .2):     # 멀리서 작게 보고 확정 (실차 확정 때 면적 0.002)
        d.update_signs({'wide_objects': 1}, [dict(name='right', area=.004)], t, .2, t, .05)   # 최소 면적 0.003 (11/17)
    assert d.fork.sign == 'right'
    d.lane_change.active = True
    d.sections.turned = 40.0
    for i in range(10):
        d.update_signs({'wide_objects': 0}, [dict(name='right', area=area)], .3+i*.05, .2, .3+i*.05, .05)
    d.lane_change.active, d.lane_change.index, d.sections.turned = False, 0, 0.0


def test_sign_near_recorded_off_axis():
    """11/16 run_1116_2113: 두 칸 이동 중 틀어진 자세에서만 표지판이 커짐 → 도착 뒤 표지판이 안 보이면 입구(exit)로 넘어가야 함"""
    d = driver()
    d.c['missions']['fork_enter_distance'] = None
    arrive_after_diagonal(d, .01)
    for i in range(30):
        signs(d, [], 1.+i*.05)
    assert d.fork.phase == 'exit', '틀어진 자세에서 크게 본 표지판이 통과 판정에 안 쓰임: %s' % d.fork.phase
    # 작게만 봤으면(멀리서) 통과 판정 안 함
    d = driver()
    d.c['missions']['fork_enter_distance'] = None
    arrive_after_diagonal(d, .003)
    for i in range(30):
        signs(d, [], 1.+i*.05)
    assert d.fork.phase == 'follow', d.fork.phase
    # 틀어진 자세의 반대 화살표는 방향 교체에 안 씀
    d = driver()
    arrive_after_diagonal(d, .01)
    assert d.fork.sign == 'right'
    d.lane_change.active, d.sections.turned = True, 40.0
    for i in range(10):
        d.update_signs({'wide_objects': 0}, [dict(name='left', area=.02)], 2.+i*.05, .2, 2.+i*.05, .05)
    assert d.fork.sign == 'right', '틀어진 자세의 화살표로 방향이 바뀜'


def test_follow_enter_distance():
    """입구 판정이 끝내 안 나도 S2 주행 거리 fork_enter_distance에서 입구로 처리 (follow가 구간 끝까지 남아 라바콘을 막던 문제)"""
    d = driver()
    arrive_after_diagonal(d, .003)
    d.sections.distance = 4.0
    for i in range(10):
        signs(d, [], 1.+i*.05)
    assert d.fork.phase == 'follow', d.fork.phase
    d.sections.distance = d.c['missions']['fork_enter_distance']+.01
    signs(d, [], 2.)
    assert d.fork.phase == 'exit' and d.fork.passed, d.fork.phase


if __name__ == '__main__':
    test_conflicting_arrows()
    test_both_directions()
    test_immediate_and_memory()
    test_fork_boundaries()
    test_requested_uses_current_heading()
    test_fork_keeps_tracked_inner_boundary()
    test_blocked_target_keeps_avoidance()
    test_sign_near_recorded_off_axis()
    test_follow_enter_distance()
    print('YOLO 좌·우 분기 상태 시험 통과')
