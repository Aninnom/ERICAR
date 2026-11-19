"""SWB 재출발 시험 (맥·차, ROS 불필요)

재출발 절차: 조종기 SWB 수동(아래) → 차를 미션 시작 지점에 놓음 → SWB 자동(위). 노트북 조작 없음
1) section.relocate 단독: 놓인 방향으로 구간 결정 (같은 구간·다음 구간·이전 구간·판단 불가)
2) Driver.restart: 진행 중 동작·끝낸 미션·멈춤 시간 초기화
3) ControlLoop: 첫 자동 전환 = 출발(구간 그대로), 짧은 수동 끊김 무시, 수동 중 들어서 돌리면 구간 따라감

실행: python3 test/test_restart.py   (numpy·opencv·pyyaml 필요)
"""
import copy
import math
import sys
from pathlib import Path

import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))

from ericar_driver.driver import Driver  # noqa: E402
from ericar_driver.loop import ControlLoop, Sensors  # noqa: E402
from ericar_driver.params import ANSWER_CONFIG  # noqa: E402
from ericar_driver.section import SectionManager  # noqa: E402

ROBOT = yaml.safe_load(open(HERE.parent / 'config' / 'robot.yaml'))['robot']
DT = 0.05


def scan_ok():
    return dict(front=4.0, rear=4.0, valid_fraction=1.0, wide_objects=0, narrow_centers=[], points=[])


def lane_ok():
    return dict(valid=True, abnormal=False, error=0.0, heading=0.0, stop_distance=None, forward=0.65,
                pursuit=True, curvature=0.0, solid={'left': True, 'right': True}, colors={},
                lane_mask=np.zeros((8, 8), np.uint8))


def test_relocate():
    c = ANSWER_CONFIG['sections']

    def placed(index, yaw_deg, carried_deg):
        """index 구간에서 누적 회전 yaw_deg(°, + 왼쪽)인 차를 carried_deg만큼 돌려 놓음 → 재출발 때 고르는 구간"""
        m = SectionManager(c, index)
        m.yaw = math.radians(yaw_deg)
        m.distance = 5.0
        m.carry(math.radians(carried_deg), 1.0)
        best, error = m.relocate()
        assert m.index == index and m.distance == 5.0, 'relocate는 값을 바꾸지 않음 (driver.restart가 새로 만듦)'
        return best

    assert placed(2, -180, 8) == 2, '같은 미션 재출발(같은 방향)인데 구간이 바뀜'
    assert placed(2, -190, -360) == 2, '옮기다 한 바퀴 돌려도 같은 방향이면 같은 구간'
    assert placed(2, -180, -82) == 3, '포기 후 다음 구간(오른쪽 90°)으로 옮겼는데 안 넘어감'
    assert placed(4, -350, 85) == 3, '코너에서 이미 넘어간 뒤 이전 구간 미션 시작 지점 재출발'
    assert placed(0, 5, -10) == 0 and placed(4, -355, -10) == 4, 'S0·S4는 방향이 같음: 지금 구간 유지'
    assert placed(3, -270, -95) == 4, 'S3 → S4'
    assert placed(1, -90, 180) == 1, '반대 방향(판단 불가)이면 지금 구간 유지'
    assert placed(1, -90, -45) in (1, 2)
    print('방향으로 구간 재결정 통과')


def make_driver(stage):
    c = copy.deepcopy(ANSWER_CONFIG)
    c['missions']['start_stage'] = stage
    return Driver({'answer': c, 'robot': ROBOT})


def carry(driver, t, degrees, seconds=3.0):
    """수동 상태로 seconds 동안 degrees(°, + 왼쪽)만큼 돌려 놓음"""
    n = int(round(seconds/DT))
    for _ in range(n):
        driver.manual(t, yaw_rate=math.radians(degrees)/seconds)
        t += DT
    return t


def test_driver_restart():
    driver = make_driver('pedestrian')
    t = 0.0
    for _ in range(20):
        driver.update(lane_ok(), scan_ok(), [], t, 0.3, t, yaw_rate=0.0)
        t += DT
    assert driver.sections.distance > 0.2
    # 보행자 정지 도중 막혀서 포기된 상태를 흉내
    driver.state, driver.stall_time, driver.line_distance = 'pedestrian_hold', 6.0, 0.2
    driver.done.add('pedestrian')
    driver.cooldowns['pedestrian'] = t+100
    t = carry(driver, t, 0.0)
    assert driver.state == 'pedestrian_hold', '수동 중에는 상태를 건드리지 않음'
    driver.restart(t)
    assert driver.state == 'lane' and not driver.done and not driver.cooldowns and driver.stall_time == 0.0
    assert driver.line_distance is None and driver.sections.index == 1 and driver.restarts == 1
    assert driver.sections.distance == 0.0, '재출발은 그 구간 start_stage 시작과 같아야 함 (구간 거리 0)'
    assert driver.c['missions']['start_stage'] == 'pedestrian'
    assert driver.allowed('pedestrian'), '재출발 뒤 그 미션이 다시 켜져야 함'
    speed, _, state = driver.update(lane_ok(), scan_ok(), [], t, 0.0, t, yaw_rate=0.0)
    assert speed > 0 and state == 'lane'
    print('미션 상태 초기화 통과')


def test_stage_choice():
    """재출발 구간 → start_stage. S4는 신호등을 끝냈는지로 신호등·주차 지점을 가름"""
    # S2 3차선 이동 도중 실패 → 같은 방향으로 다시 놓음: 차선 변경 상태가 남지 않아야 함
    d = make_driver('lane_change')
    d.lane_change.active, d.entry_from_corner, d.lane_change_off = True, True, 1.0
    d.restart(carry(d, 0.0, 5.0))
    assert d.sections.index == 2 and not d.lane_change.active and not d.entry_from_corner and d.lane_change_off == 0.0
    assert d.c['missions']['start_stage'] == 'lane_change'
    # S3 라바콘 포기 → 오른쪽 90° 돌려 신호등 지점: 신호등부터
    d = make_driver('cones')
    d.restart(carry(d, 0.0, -90.0))
    assert d.sections.index == 4 and d.c['missions']['start_stage'] == 'traffic' and 'traffic' not in d.done
    # S4 신호등 전에 실패 → 신호등 지점
    d = make_driver('slow')
    d.sections.index, d.sections.yaw = 4, math.radians(-360.0)
    d.restart(carry(d, 0.0, 0.0))
    assert d.sections.index == 4 and d.c['missions']['start_stage'] == 'traffic' and 'traffic' not in d.done
    # S4 신호등을 끝낸 뒤(주차 중) 실패 → 주차 지점, 신호등은 지난 것으로 봄
    d = make_driver('slow')
    d.sections.index, d.sections.yaw = 4, math.radians(-360.0)
    d.advance('traffic')
    d.state = 'parking_turn'
    d.restart(carry(d, 0.0, 0.0))
    assert d.sections.index == 4 and d.c['missions']['start_stage'] == 'parking' and d.done == {'traffic'}
    assert d.state == 'lane' and d.sections.distance == 0.0
    # 주차 지점에서 한 번 더 재출발해도 주차 지점 유지
    d.restart(carry(d, 0.0, 0.0))
    assert d.c['missions']['start_stage'] == 'parking' and d.restarts == 2
    # 노드를 켠 뒤 첫 출발(relocate False): start_stage 그대로, 들고 돌린 회전 버림
    d = make_driver('cones')
    d.restart(carry(d, 0.0, 130.0), relocate=False)
    assert d.sections.index == 3 and d.restarts == 0 and d.c['missions']['start_stage'] == 'cones'
    assert abs(math.degrees(d.sections.yaw)+270.0) < 1e-6
    print('재출발 지점 선택 통과')


def test_loop():
    c = copy.deepcopy(ANSWER_CONFIG)
    c['detector']['enabled'] = False
    loop = ControlLoop({'answer': c, 'robot': ROBOT})
    loop.vision.analyze = lambda image, bias: lane_ok()
    image = np.zeros((480, 640, 3), np.uint8)
    clock = {'t': 100.0, 'k': 0}
    states = []

    def run(seconds, auto, turn=0.0, velocity=0.0):
        out = None
        for _ in range(int(round(seconds/DT))):
            clock['t'] += DT
            clock['k'] += 1
            t = clock['t']
            out = loop.step(Sensors(now=t, image=image, image_stamp=t-0.01, detections=[], detection_stamp=-1e9,
                                    detector_ok=True, scan=scan_ok(), scan_stamp=t, velocity=velocity,
                                    angular_velocity=0.0, odom_stamp=t,
                                    yaw_rate=math.radians(turn)/seconds, imu_stamp=t, auto=auto))
            if out.status:
                states.append(out.status)
            if not auto:
                assert out.speed == 0.0 and out.steering == 0.0
        return out

    d = loop.driver
    # 노드를 켠 뒤 출발 전: 수동 상태로 차를 들고 120° 돌려 출발선에 놓음 → 첫 자동 전환은 출발 (S0, 재출발 아님)
    run(2.0, False, turn=120)
    assert 'manual (SWB)' in states
    out = run(1.0, True, velocity=0.3)
    assert d.sections.index == 0 and abs(d.sections.yaw) < 1e-6 and d.restarts == 0 and out.speed > 0
    # 짧은 수동 끊김(0.5초 < min_manual): 하던 동작·끝낸 미션 그대로
    d.done.add('slow')
    run(0.5, False)
    run(0.5, True, velocity=0.3)
    assert d.done == {'slow'} and d.restarts == 0, '짧은 끊김을 재출발로 처리함'
    # 재출발: 수동으로 내리고 다음 구간 미션 시작 지점으로 옮김 (오른쪽 90°) → 자동
    run(4.0, False, turn=-90)
    assert d.sections.index == 0, '수동 중에는 구간을 바꾸지 않음'
    out = run(0.5, True, velocity=0.3)
    assert d.sections.index == 1 and d.restarts == 1 and not d.done and out.speed > 0
    assert abs(d.restart_error) < 1.0
    # 같은 미션 재출발: 뒤로 옮겨 같은 방향으로 놓음
    d.done.add('pedestrian')
    run(3.0, False, turn=6)
    run(0.5, True, velocity=0.3)
    assert d.sections.index == 1 and d.restarts == 2 and d.allowed('pedestrian')
    print('제어 루프 수동·자동 전환 통과')


if __name__ == '__main__':
    test_relocate()
    test_driver_restart()
    test_stage_choice()
    test_loop()
    print('SWB 재출발 시험 통과')
