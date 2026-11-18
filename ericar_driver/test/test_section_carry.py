"""구간 관리자: 들어 돌린 회전은 구간 전환에서 제외, 재출발 이웃 구간 조건 (11/18 track_test_1118_01)

실행: python3 test/test_section_carry.py
"""
import copy
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ericar_driver.params import ANSWER_CONFIG
from ericar_driver.section import SectionManager


def manager(index=2, **override):
    c = copy.deepcopy(ANSWER_CONFIG['sections'])
    c.update(override)
    return SectionManager(c, index)


def test_normal_corner_still_advances():
    m = manager()
    # 0.2 m/s·조향 최대(곡률 2.23): 회전 약 25.6°/s → 90° 코너
    for _ in range(int(4.0/0.05)):
        m.update(-0.2*2.23, 0.2, 0.05)
    assert m.index == 3 and abs(m.carried) < 1e-9, (m.index, m.carried)
    m = manager()
    for _ in range(int(2.2/0.05)):      # 0.4 m/s 최대 조향 약 51°/s
        m.update(-0.4*2.23, 0.4, 0.05)
    assert m.index == 3 and abs(m.carried) < 1e-9, (m.index, m.carried)
    print('  주행 코너는 그대로 구간 전환 통과')


def test_lifted_spin_does_not_advance():
    m = manager()
    # 01번 107~109 s: 바퀴 0.2 m/s로 도는 채 손으로 약 100°/s, 2.4 s
    for _ in range(48):
        m.update(-math.radians(100.0), 0.2, 0.05)
    assert m.index == 2, m.index
    assert abs(math.degrees(m.yaw)+180.0+240.0) < 1.0      # 누적 회전에는 그대로 들어감 (재출발 방향 판단용)
    assert abs(m.turned) < 1.0, m.turned
    print('  들어 돌린 회전은 구간 전환 안 함 통과')


def test_guard_can_be_disabled():
    m = manager(carry_rate_gain=None)
    for _ in range(48):
        m.update(-math.radians(100.0), 0.2, 0.05)
    assert m.index == 4, m.index
    print('  끔(None)이면 예전처럼 넘어감 통과')


def test_relocate_switches_only_when_aligned():
    m = manager()
    m.yaw = -math.radians(474.9)        # 01번 재출발 1: 실제는 S2 방향, 누적 회전이 약 65° 어긋남 (S1 방향과 24.9° 차)
    assert m.relocate()[0] == 2, m.relocate()
    m.yaw = -math.radians(270.0+11.4)   # 다음 구간 방향에 11.4° 차 (11/18 손으로 옮긴 뒤 방향 오차 최대값) → 다음 구간
    assert m.relocate()[0] == 3, m.relocate()
    m.yaw = -math.radians(90.0-8.0)     # 이전 구간
    assert m.relocate()[0] == 1, m.relocate()
    old = manager(restart_switch_tolerance=40.0)
    old.yaw = -math.radians(474.9)
    assert old.relocate()[0] == 1       # 40이면 예전 동작: 24.9° 차인 이전 구간으로 넘어감
    print('  재출발 이웃 구간은 방향이 맞을 때만 통과')


if __name__ == '__main__':
    test_normal_corner_still_advances()
    test_lifted_spin_does_not_advance()
    test_guard_can_be_disabled()
    test_relocate_switches_only_when_aligned()
    print('구간 관리자 들어 돌림·재출발 시험 통과')
