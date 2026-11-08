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
        self.last_seen = -1e9

    def update(self, lane, now, dt, slow=False):
        c = self.c
        if lane['valid']:
            self.last_seen = lane.get('stamp', now)
            error = lane['error']
            distance = lane.get('forward', c['lane']['lookahead'])
            geometric = math.atan2(2*self.robot['wheelbase']*error, distance**2+error**2)
            # 차로 방향 앞먹임: 카메라 시야가 짧아 코너 휨이 오차에 늦게 반영 → 방향(heading)으로 미리 조향
            ahead = c['pid'].get('heading_gain', 0.0)*lane['heading']
            self.steering = clip(geometric+ahead+c['pid']['feedback_gain']*self.pid.update(error, dt),
                                 -self.robot['max_steering'], self.robot['max_steering'])
        speed = max(c['minimum_curve_speed'], c['base_speed']*(1-c['curve_speed_gain']*
                    abs(self.steering)/self.robot['max_steering']))
        if slow or lane['abnormal'] or not lane['valid']:
            speed = min(speed, c['slow_speed'], c['base_speed']/2, .2)
        if now-self.last_seen > c['lane']['lost_grace']:
            speed = 0.0
        return speed, self.steering
