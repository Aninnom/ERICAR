"""신호등 정지 중에만 YOLO 입력 크기를 바꾸는 규칙 (traffic_lamp.image_size)"""
from ericar_driver.params import ANSWER_CONFIG
from ericar_driver.traffic_lamp import image_size


def test_hold_uses_small_size():
    c = ANSWER_CONFIG['detector']
    assert image_size(c, 'traffic_hold') == 320
    for state in ('lane', 'traffic_approach', 'pedestrian_hold', 'parking_cruise', 'manual (SWB)'):
        assert image_size(c, state) == c['image_size'] == 640


def test_disabled_keeps_default():
    c = dict(ANSWER_CONFIG['detector'], traffic_hold_image_size=None)
    assert image_size(c, 'traffic_hold') == 640
    assert image_size({'image_size': 640}, 'traffic_hold') == 640
