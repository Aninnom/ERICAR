"""라바콘 직후 한 선 좌우 판단 시험 (ROS·카메라 불필요)

문제 (11/14 실차, 유경): 라바콘을 빠져나온 직후에는 차가 바깥(왼쪽) 선 가까이·비스듬히 있어
왼쪽 선의 가까운 끝이 화면 중앙보다 오른쪽에 찍힘 → lane_vision.single_name이 위치만 보고 '오른쪽 선'으로
오인 → 차로 중앙을 그 선의 왼쪽으로 잡아 더 바깥으로 나감. 그 이름이 기억에 저장되어 고착.

고친 방법: 라바콘 통과 중·직후 cone_lane_left_distance(m) 동안 driver.cone_left_limit()이
lane_vision의 한 선 판단 경계(corner_left_limit 자리)를 cone_lane_left_limit로 옮김 + 힌트가 켜질 때
차선 기억을 한 번 비움(forget_lines). 코너 유지 힌트와 같은 구조라 기억보다 먼저 적용됨.

실행: python3 test/test_cone_lane_exit.py   (numpy·pyyaml 필요)
"""
import copy
import sys
from pathlib import Path

import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))

from ericar_driver.cones import ConeDriving  # noqa: E402
from ericar_driver.lane_vision import LaneVision  # noqa: E402
from ericar_driver.params import ANSWER_CONFIG  # noqa: E402

ROBOT = yaml.safe_load((HERE.parent / 'config' / 'robot.yaml').read_text(encoding='utf-8'))['robot']


def vision():
    c = copy.deepcopy(ANSWER_CONFIG)
    return LaneVision(c['lane'], ROBOT['camera'], ROBOT['front_axle_x']), c


def straight_line(view, lateral):
    """가까운 끝이 차 중앙에서 lateral(m, + 왼쪽)에 있는 곧은 선 → (곡선 계수, 가까운 끝)"""
    c = view.c
    x = c['width_px']/2 - lateral*view.scale
    return np.array([0.0, 0.0, x]), (float(c['height_px']-1), float(x))


def test_single_line_without_hint():
    """힌트 없으면 차가 바깥 선을 넘은 순간 그 선을 '오른쪽'으로 오인 (지금 증상 재현)"""
    view, _ = vision()
    for lateral, expected in ((+0.20, 'left'), (-0.05, 'right'), (-0.20, 'right')):
        fit, anchor = straight_line(view, lateral)
        name = view.single_name(fit, anchor)
        print(f"힌트 없음: 선이 차 기준 {lateral:+.2f} m → {name}")
        assert name == expected, (lateral, name)


def test_single_line_with_cone_hint():
    """라바콘 힌트가 켜지면 차가 바깥 선을 0.2 m 넘어서도 '왼쪽'으로 봄"""
    view, c = vision()
    view.corner_left_limit = c['lidar']['cone_lane_left_limit']
    for lateral in (+0.20, -0.05, -0.20, -0.29):
        fit, anchor = straight_line(view, lateral)
        name = view.single_name(fit, anchor)
        print(f"라바콘 힌트: 선이 차 기준 {lateral:+.2f} m → {name}")
        assert name == ('left' if lateral > -c['lidar']['cone_lane_left_limit'] else 'right'), (lateral, name)


def test_hint_beats_wrong_memory():
    """틀린 이름이 기억에 남아 있어도 힌트가 먼저 적용되어 고착되지 않음"""
    view, c = vision()
    fit, anchor = straight_line(view, -0.20)
    view.memory = {'right': (np.array([anchor[0]]), np.array([anchor[1]]), 0)}
    assert view.single_name(fit, anchor) == 'right', '기억만 있으면 틀린 이름이 이어짐 (문제 상황)'
    view.corner_left_limit = c['lidar']['cone_lane_left_limit']
    assert view.single_name(fit, anchor) == 'left', '힌트가 기억보다 먼저 적용되어야 함'
    view.forget_lines()
    assert view.memory == {}, 'forget_lines가 기억을 비워야 함'
    print('기억에 틀린 이름이 있어도 힌트가 이김, forget_lines로 기억 비움')


def test_hint_window():
    """힌트는 라바콘이 한 번 켜진 뒤부터, 복귀 후 cone_lane_left_distance까지만"""
    c = copy.deepcopy(ANSWER_CONFIG)
    cones = ConeDriving(c, ROBOT)
    limit, reach = c['lidar']['cone_lane_left_limit'], c['lidar']['cone_lane_left_distance']

    class FakeDriver:
        pass
    from ericar_driver.driver import Driver  # noqa: E402
    driver = FakeDriver()
    driver.c, driver.cones = c, cones
    hint = Driver.cone_left_limit.__get__(driver, FakeDriver)

    assert hint() is None, '라바콘이 한 번도 안 켜졌으면 힌트 없음'
    cones.started, cones.active = True, True
    assert hint() == limit, '통과 중에는 힌트 켜짐'
    cones.active, cones.since_exit = False, reach - 0.1
    assert hint() == limit, f'복귀 뒤 {reach} m 안에서는 힌트 유지'
    cones.since_exit = reach + 0.1
    assert hint() is None, f'복귀 뒤 {reach} m를 넘으면 힌트 꺼짐'
    print(f'힌트 창: 통과 중 ~ 복귀 뒤 {reach} m, 경계 {limit} m')


if __name__ == '__main__':
    test_single_line_without_hint()
    test_single_line_with_cone_hint()
    test_hint_beats_wrong_memory()
    test_hint_window()
    print('라바콘 직후 차선 좌우 판단 시험 통과')
