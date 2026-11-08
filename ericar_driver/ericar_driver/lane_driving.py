"""일반 차선 주행: 차선 오차 PID"""
# 출처: hello-osy/2026-orda-mentoring src/orda_sim/orda_sim/answer.py (613c9b4)
import math

from .common import clip, PID


class LaneDriving:
    """차선 담당①: 차선 오차 → 기본 조향 + PID. 반환값은 (속도, 조향)."""
    def __init__(self, config, robot):
        self.c, self.robot = config, robot
        self.pid = PID(config['pid'])
        self.steering = 0.0
        self.curvature = 0.0
        self.last_seen = -1e9

    def update(self, lane, now, dt, slow=False):
        c = self.c
        if lane['valid']:
            self.last_seen = lane.get('stamp', now)
            error = lane['error']
            distance = lane.get('forward', c['lane']['lookahead'])
            geometric = math.atan2(2*self.robot['wheelbase']*error, distance**2+error**2)
            # 곡률 앞먹임: 카메라 시야가 짧아 코너 휨이 오차에 늦게 반영 → 차선 곡률로 미리 조향
            # 반지름 R 원호를 따라가는 조향 = atan(휠베이스/R) = atan(휠베이스·곡률)
            a = c['pid'].get('curvature_smoothing', 0.5)
            self.curvature = a*clip(lane.get('curvature', 0.0), -3.0, 3.0)+(1-a)*self.curvature
            ahead = c['pid'].get('curvature_gain', 0.0)*math.atan(self.robot['wheelbase']*self.curvature)
            self.steering = clip(geometric+ahead+c['pid']['feedback_gain']*self.pid.update(error, dt),
                                 -self.robot['max_steering'], self.robot['max_steering'])
        speed = max(c['minimum_curve_speed'], c['base_speed']*(1-c['curve_speed_gain']*
                    abs(self.steering)/self.robot['max_steering']))
        if slow or lane['abnormal'] or not lane['valid']:
            speed = min(speed, c['slow_speed'], c['base_speed']/2, .2)
        if now-self.last_seen > c['lane']['lost_grace']:
            speed = 0.0
        return speed, self.steering
