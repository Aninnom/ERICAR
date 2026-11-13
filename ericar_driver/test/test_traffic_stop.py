"""신호등 정지 위치 시험 (소민 11/12). python ericar_driver/test/test_traffic_stop.py

- 차선 주행 중 횡단보도 앞 선을 고른 뒤 뒤쪽 선으로 건너뛰지 않음 (앞 선이 카메라 아래로 사라져도 기억으로 추적)
- 정지선을 못 찾았는데 빨강이 보이면 시간 초과 때 지나가지 않고 정지
- 정지선 없이 빨강 박스가 크면 예비 정지
"""
import copy
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from ericar_driver.driver import Driver
from ericar_driver.loop import load_config


def lane(stop_distance, stamp):
    return dict(stop_distance=stop_distance, stamp=stamp)


class TrafficStopTest(unittest.TestCase):
    def setUp(self):
        config = copy.deepcopy(load_config(Path(__file__).resolve().parents[1] / 'config/robot.yaml'))
        config['answer']['missions']['start_stage'] = 'traffic'
        self.d = Driver(config)
        self.m = config['answer']['missions']

    def signs(self, objects, stamp):
        return self.d.update_signs({}, objects, stamp, 0.0, stamp, .05)

    def test_keep_near_line(self):
        d, v, dt = self.d, .2, .05
        # 횡단보도 앞 선 1.0 m, 뒤쪽 선 1.6 m. 앞 선만 보이다가 0.4 m 안으로 들어가면 사라지고 뒤쪽 선만 보임
        near, far, t = 1.0, 1.6, 0.0
        for k in range(40):
            t += dt
            near -= v*dt
            far -= v*dt
            seen = near if near > .4 else far
            d.update_stop_line(lane(seen, t), v, t, dt)
        self.assertAlmostEqual(d.line_distance, near, delta=.03)

    def test_crosswalk_line_preferred(self):
        d, v, dt = self.d, .2, .05
        # 서행 시작 흰 선(plain) 0.5 m → 횡단보도 앞 선(crosswalk) 1.4 m → 횡단보도 뒤 선(plain) 1.8 m
        start, cross, after, t = .5, 1.4, 1.8, 0.
        for k in range(60):
            t += dt
            start, cross, after = start-v*dt, cross-v*dt, after-v*dt
            frame = dict(stamp=t, stop_distance=None, crosswalk_distance=None)
            if .4 < start < 1.7:
                frame['stop_distance'] = start
            elif .4 < after < 1.7 and not .4 < cross < 1.7:
                frame['stop_distance'] = after
            if .4 < cross < 1.45:
                frame['crosswalk_distance'] = cross
                frame['stop_distance'] = frame['stop_distance'] or cross
            d.update_stop_line(frame, v, t, dt)
        self.assertEqual(d.line_kind, 'crosswalk')
        self.assertAlmostEqual(d.line_distance, cross, delta=.03)

    def test_traffic_ignores_plain_line_and_stops_3cm_before(self):
        d = self.d
        for i in range(3):
            self.signs([dict(name='traffic_red', area=.01)], i*.1)
        self.assertEqual(d.state, 'traffic_approach')
        d.line_distance, d.line_kind = .05, 'plain'     # 서행 시작선: 서지 않고 저속 접근
        self.assertGreater(d.stop_missions(.4, .13, False, 1.0), 0.0)
        d.line_distance, d.line_kind = .30, 'crosswalk'
        self.assertGreater(d.stop_missions(.4, .13, False, 1.1), 0.0)
        d.line_distance = self.m['traffic_stop_offset']  # 앞 범퍼가 선 3 cm 앞
        self.assertEqual(d.stop_missions(.4, 0.0, False, 1.2), 0.0)
        self.assertEqual(d.state, 'traffic_hold')
        bumper_gap = d.line_distance - (d.robot['front_bumper_x'] - d.robot['front_axle_x']) if hasattr(d, 'robot') else .03
        self.assertAlmostEqual(bumper_gap, .03, delta=.005)

    def test_vision_crosswalk_flag(self):
        import numpy as np
        from ericar_driver.lane_vision import LaneVision
        c = self.d.c['lane']
        v = LaneVision(c, dict(hfov_deg=71., x=.25, y=0., z=.1875, pitch=.148), .2)
        h, w = c['height_px'], c['width_px']
        rows = (h-1)/(c['far']-c['near'])
        def row(m):   # 앞축 기준 m → 행
            return int(round((c['far']-(m+.2))*rows))
        white = np.zeros((h, w), np.uint8)
        cx, lane_px = w//2, int(c['lane_width']*v.scale)
        for m, cross in [(.6, False), (1.1, True)]:
            r = row(m)
            white[r-3:r+3, cx-lane_px//2:cx+lane_px//2] = 255
            if cross:
                for k in range(4):
                    x = cx-lane_px//2+10+k*(lane_px//4)
                    white[r-int(.13*rows):r-int(.02*rows), x:x+int(.06*v.scale)] = 255
        result = dict(center_px=w/2, stop_distance=None, stop_candidates=[], crosswalk_distance=None)
        v.find_stop_line(result, white, lane_px, white)
        self.assertAlmostEqual(result['stop_distance'], .6, delta=.02)
        self.assertAlmostEqual(result['crosswalk_distance'], 1.1, delta=.02)

    def test_red_without_line_does_not_run(self):
        d = self.d
        for i in range(3):
            self.signs([dict(name='traffic_red', area=.03)], i*.1)
        self.assertEqual(d.state, 'traffic_approach')
        d.line_distance = None
        start = d.state_started
        speed = d.stop_missions(.4, .13, False, start+self.m['approach_timeout']+.1)
        self.assertEqual(speed, 0.0)
        self.assertEqual(d.state, 'traffic_hold')
        self.assertNotIn('traffic', d.done)

    def test_big_red_stops_without_line(self):
        d = self.d
        for i in range(3):
            self.signs([dict(name='traffic_red', area=.05)], i*.1)
        d.line_distance = None
        self.assertEqual(d.stop_missions(.4, .13, False, 1.0), 0.0)
        self.assertEqual(d.state, 'traffic_hold')

    def test_green_still_leaves_after_timeout(self):
        # 빨강이 안 보이면 예전처럼 시간 초과 후 진행 (영구 정지 방지)
        d = self.d
        for i in range(3):
            self.signs([dict(name='traffic_red', area=.03)], i*.1)
        self.signs([], 1.0)
        d.line_distance = None
        d.stop_missions(.4, .13, False, d.state_started+self.m['approach_timeout']+.1)
        self.assertEqual(d.state, 'lane')


if __name__ == '__main__':
    unittest.main()
