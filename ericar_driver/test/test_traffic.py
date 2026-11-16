"""신호등 합성 검출·정지·재출발 시험. 실차 정지 거리 보정은 별도 필요"""
import copy
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from ericar_driver.driver import Driver
from ericar_driver.loop import load_config


class TrafficTest(unittest.TestCase):
    def setUp(self):
        config = copy.deepcopy(load_config(Path(__file__).resolve().parents[1] / 'config/robot.yaml'))
        config['answer']['missions']['start_stage'] = 'traffic'
        self.driver = Driver(config)
        self.frames = config['answer']['detector']['confirm_frames']

    def signs(self, names, stamp):
        objects = [dict(name=name, area=0.1) for name in names]
        return self.driver.update_signs({}, objects, stamp, 0.0, stamp, .05)

    def green(self):
        for i in range(self.frames):
            result = self.signs(['traffic_green'], i * .1)
        return result

    def test_red_veto_and_green_reconfirmation(self):
        self.assertTrue(self.green())
        self.assertFalse(self.signs(['traffic_green', 'traffic_red'], 1.0))
        for i in range(self.frames - 1):
            self.assertFalse(self.signs(['traffic_green'], 1.1 + i * .1))
        self.assertTrue(self.signs(['traffic_green'], 2.0))

    def test_yellow_veto(self):
        self.green()
        self.assertFalse(self.signs(['traffic_green', 'traffic_yellow'], 1.0))

    def test_repeated_frame_cannot_confirm(self):
        for _ in range(self.frames + 2):
            self.assertFalse(self.signs(['traffic_green'], 0.0))

    def test_conflict_keeps_vehicle_stopped(self):
        self.green()
        d = self.driver
        d.transition('traffic_hold', .5)
        d.stop_missions(.4, 0.0, False, .5)
        green = self.signs(['traffic_green', 'traffic_red'], 1.0)
        self.assertEqual(d.stop_missions(.4, 0.0, green, 1.0), 0.0)
        self.assertEqual(d.state, 'traffic_hold')
        green = self.signs([], 1.1)
        self.assertEqual(d.stop_missions(.4, 0.0, green, 1.1), 0.0)
        self.assertEqual(d.state, 'traffic_hold')

    def test_stop_then_green_crossing(self):
        self.green()
        d = self.driver
        d.line_distance = d.c['missions'].get('traffic_stop_offset', d.c['missions']['stop_offset'])   # 신호등 정지 위치 (11/14 소민)
        d.line_kind = 'crosswalk'   # 신호등은 횡단보도 바로 앞 선에만 정지 (11/12)
        self.assertEqual(d.stop_missions(.4, 0.0, True, 1.0), 0.0)
        self.assertEqual(d.state, 'traffic_hold')
        d.stop_missions(.4, 0.0, False, 1.5)
        self.assertEqual(d.state, 'traffic_hold')
        # 녹색은 traffic_green_hold(1 s) 끊김 없이 보여야 출발 (11/14 소민: 빨강 대기 중 녹색 오검출 1~2프레임에 출발)
        d.stop_missions(.4, 0.0, True, 1.6)
        self.assertEqual(d.state, 'traffic_hold')
        d.line_distance = 0.0   # 앞축이 선 위 (traffic_stop_offset −0.045면 이미 선을 지난 것으로 봐 바로 끝남)
        d.stop_missions(.4, 0.0, True, 2.7)
        self.assertEqual(d.state, 'traffic_approach')
        self.assertGreater(d.stop_missions(.4, 0.0, True, 2.75), 0.0)
        self.assertNotIn('traffic', d.done)
        d.line_distance = -.1
        d.stop_missions(.4, .13, True, 2.8)
        self.assertIn('traffic', d.done)


if __name__ == '__main__':
    unittest.main()
