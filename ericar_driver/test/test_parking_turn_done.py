"""11/14 주차 기본값 시험: 실측 회전 반경으로 회전 시작, 칸 방향이 정면이면 정지 (integ_1114_lap4)"""
import copy
import math
import sys
from pathlib import Path

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ericar_driver.params import ANSWER_CONFIG
from ericar_driver.parking import Parking


def config():
    c = yaml.safe_load((Path(__file__).resolve().parents[1]/'config/robot.yaml').read_text())
    c['answer'] = copy.deepcopy(ANSWER_CONFIG)
    c['answer']['parking']['template_enabled'] = False  # 기존 조향·감속 회귀와 새 템플릿을 분리
    return c


def test_turn_lead_uses_measured_radius():
    c = config()
    assert abs(Parking(c)._turn_lead()-c['answer']['parking']['turn_radius']) < 1e-9
    c['answer']['parking']['turn_radius'] = None
    assert abs(Parking(c)._turn_lead()-c['robot']['wheelbase']/abs(math.tan(c['answer']['parking']['turn_steering']))) < 1e-9


def test_turn_length_fits_limits():
    p = config()['answer']['parking']
    arc = p['turn_radius']*math.pi/2
    assert arc < p['turn_distance_limit'] and arc/p['turn_speed'] < p['turn_timeout']


def test_slot_normal_reaches_front_after_quarter_turn():
    """회전 90°(IMU 회전 속도 적분) 뒤 저장한 칸 방향이 차 정면 → turn_done_heading 조건 성립"""
    c = config(); p = c['answer']['parking']
    target = dict(entrance=[p['turn_radius'], -.22], polygon=[[0., 0.]], tangent=[1., 0.], normal=[0., -1.])
    radius, speed, dt = p['turn_radius'], p['turn_speed'], .05
    steps = 0
    while math.atan2(target['normal'][1], target['normal'][0]) < -p['turn_done_heading']:
        Parking._move_target(target, dt, speed, -speed/radius)
        steps += 1
        assert steps < 400
    turned = steps*dt*speed/radius
    assert abs(turned-math.pi/2) < p['turn_done_heading']+.02
    # 회전이 끝난 자리에서 입구 중심은 차 뒤쪽 정중앙 근처 (좌우 오차 3 cm 안)
    assert abs(target['entrance'][1]) < .03


if __name__ == '__main__':
    for name, fn in list(globals().items()):
        if name.startswith('test_'):
            fn()
    print('test_parking_turn_done 통과')
