"""일반 차선 주행: 목표점 pure pursuit (목표점 없으면 lookahead 오차 PID)"""
# 출처: hello-osy/2026-orda-mentoring src/orda_sim/orda_sim/answer.py (613c9b4)
# 11/8 코너: 곡률 앞먹임 제거, 목표점 있으면 pure pursuit만 (PR #8)
import math

from .common import clip, PID


class LaneDriving:
    """차선 담당①: 차선 목표점 → 조향·속도. 반환값은 (속도, 조향)."""
    def __init__(self, config, robot):
        self.c, self.robot = config, robot
        self.pid = PID(config['pid'])
        self.steering = 0.0
        self.last_seen = -1e9

    def update(self, lane, now, dt, slow=False):
        c, limit = self.c, self.robot['max_steering']
        if lane['valid']:
            self.last_seen = lane.get('stamp', now)
            error = lane['error']
            distance = lane.get('forward', c['lane']['lookahead'])
            # pure pursuit: 앞 distance, 옆 error 점을 지나는 원호 조향
            geometric = math.atan2(2*self.robot['wheelbase']*error, distance**2+error**2)
            if lane.get('pursuit'):
                # 목표점이 차로 중앙선 위(lane_vision.pursuit_target) → PID는 같은 정보를 중복 반영하므로 제외
                self.pid.reset()
                self.steering = clip(geometric, -limit, limit)
            else:
                self.steering = clip(geometric+c['pid']['feedback_gain']*self.pid.update(error, dt), -limit, limit)
        speed = max(c['minimum_curve_speed'], c['base_speed']*(1-c['curve_speed_gain']*abs(self.steering)/limit))
        # 코너 미리 감속: 조향 기준 감속은 조향이 커진 뒤라 늦음 (11/9 실차 2번 코너 0.37 m/s 진입 → 바깥 이탈)
        # 보이는 선 곡률이 코너 폭 보간 시작값 이상이면 코너 속도. 회전 시작 약 2.3 s 전 감지 (녹화 bringup_1109_0841)
        if lane['valid'] and lane.get('curvature', 0.0) >= c['lane']['corner_curvature'][0]:
            speed = min(speed, c['corner_speed'])
        if slow or lane['abnormal'] or not lane['valid']:
            speed = min(speed, c['slow_speed'], c['base_speed']/2, .2)
        if now-self.last_seen > c['lane']['lost_grace']:
            speed = 0.0
        return speed, self.steering
