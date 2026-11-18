"""3차선 진입 흰 선 제한(driver.entry_gate_open) 단위 시험. 실행: python3 test/test_entry_gate.py"""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import fake_ros  # noqa: F401,E402  (rclpy 없이 driver 모듈을 불러오기 위함)
from ericar_driver import driver as D  # noqa: E402
from ericar_driver.params import ANSWER_CONFIG  # noqa: E402


class Sections:
    index, distance = 1, 0.0


def make(from_corner=True, **override):
    car = object.__new__(D.Driver if hasattr(D, 'Driver') else D.AnswerDriver)
    car.c = dict(ANSWER_CONFIG, lane_change=dict(ANSWER_CONFIG['lane_change'], entry_gate=True, **override))
    car.sections, car.line_distance, car.entry_gate, car.entry_from_corner = Sections(), None, None, from_corner
    return car


def test_line_gate():
    car = make()
    car.sections.index = D.START_SECTION['lane_change']
    for travelled, line, expected in ((0.05, 0.95, False), (0.30, 0.70, False), (0.60, 0.40, False), (0.66, 0.34, False),
                                      (0.71, 0.29, True), (0.80, 0.20, True)):
        car.sections.distance, car.line_distance = travelled, line
        assert car.entry_gate_open() is expected, (travelled, line)


def test_no_line_uses_distance():
    car = make()
    car.sections.index = D.START_SECTION['lane_change']
    for travelled, expected in ((0.05, False), (0.20, False), (0.26, True)):
        car.sections.distance = travelled
        assert car.entry_gate_open() is expected, travelled


def test_line_lost_keeps_estimate_and_cap():
    car = make()
    car.sections.index = D.START_SECTION['lane_change']
    car.sections.distance, car.line_distance = 0.05, 0.95        # 열릴 지점 = 0.05 + 0.95 − 0.30 = 0.70
    assert car.entry_gate_open() is False
    car.line_distance = None
    car.sections.distance = 0.60
    assert car.entry_gate_open() is False
    car.sections.distance = 0.71
    assert car.entry_gate_open() is True
    far = make()
    far.sections.index = D.START_SECTION['lane_change']
    far.sections.distance, far.line_distance = 0.05, 1.29        # 열릴 지점 1.04 → 상한 1.0에서 열림
    assert far.entry_gate_open() is False
    far.sections.distance = 1.0
    assert far.entry_gate_open() is True


def test_started_in_section_is_not_gated():
    car = make(from_corner=False)
    car.sections.index = D.START_SECTION['lane_change']
    car.sections.distance, car.line_distance = 0.0, 0.95
    assert car.entry_gate_open() is True


if __name__ == '__main__':
    for name, function in list(globals().items()):
        if name.startswith('test_'):
            function()
    print('3차선 진입 흰 선 제한 시험 통과')
