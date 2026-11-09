"""일반 차선 주행: 목표점 pure pursuit (목표점 없으면 lookahead 오차 PID)"""
# 출처: hello-osy/2026-orda-mentoring src/orda_sim/orda_sim/answer.py (613c9b4)
# 11/8 코너: 곡률 앞먹임 제거, 목표점 있으면 pure pursuit만 (PR #8)
# 11/9 우회전 코너 유지 추가 (corner_hold)
import math

from .common import clip, PID


class LaneDriving:
    """차선 담당①: 차선 목표점 → 조향·속도. 반환값은 (속도, 조향)."""
    def __init__(self, config, robot):
        self.c, self.robot = config, robot
        self.pid = PID(config['pid'])
        self.steering = 0.0
        self.last_seen = -1e9
        self.armed_until = -1e9   # 코너 감지 후 이 시각까지 코너 유지 시작 가능
        self.rearm_at = -1e9      # 코너 유지 끝난 뒤 다시 준비 가능한 시각
        self.hold = None          # 코너 유지 중: dict(yaw 누적 회전 rad, distance m, two_lines 연속 장수, stamp)

    def update(self, lane, now, dt, slow=False, velocity=0.0, yaw_rate=0.0, allow_hold=True):
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
        corner_seen = lane['valid'] and lane.get('curvature', 0.0) >= c['lane']['corner_curvature'][0]
        if corner_seen:
            speed = min(speed, c['corner_speed'])
        if self.corner_hold(lane, now, dt, velocity, yaw_rate, allow_hold, corner_seen):
            return min(speed, c['corner_speed']), -c['corner_hold']['steering']
        if slow or lane['abnormal'] or not lane['valid']:
            speed = min(speed, c['slow_speed'], c['base_speed']/2, .2)
        if now-self.last_seen > c['lane']['lost_grace']:
            speed = 0.0
        return speed, self.steering

    def corner_hold(self, lane, now, dt, velocity, yaw_rate, allow_hold, corner_seen):
        """우회전 코너 유지 판단. True면 이번 주기 고정 조향 사용 (카메라 결과 무시)"""
        h = self.c.get('corner_hold', {})
        if not h.get('enabled') or not allow_hold:
            self.hold = None
            return False
        if corner_seen and now >= self.rearm_at:
            self.armed_until = now + h['arm_seconds']
        if self.hold is None:
            # 시작: 코너 감지 후 오른쪽 조향이 커짐, 또는 오른쪽으로 꺾던 중 선을 놓침
            turning = self.steering <= -h['start_steering'] or (not lane['valid'] and self.steering < -0.05)
            if now >= self.armed_until or not turning:
                return False
            self.hold = dict(yaw=0.0, distance=0.0, two_lines=0, stamp=lane.get('stamp'))
        self.hold['yaw'] += yaw_rate * dt
        self.hold['distance'] += abs(velocity) * dt
        if lane.get('stamp') != self.hold['stamp']:   # 새 영상마다 두 선 연속 장수 갱신
            self.hold['stamp'] = lane.get('stamp')
            self.hold['two_lines'] = self.hold['two_lines'] + 1 if lane['valid'] and len(lane.get('fits', {})) == 2 else 0
        turned = math.degrees(-self.hold['yaw'])     # 오른쪽 회전 = odom 각속도 음수
        if (turned >= h['angle'] or self.hold['distance'] >= h['max_distance'] or
                (turned >= h['min_angle'] and self.hold['two_lines'] >= h['two_line_frames'])):
            self.hold = None
            self.armed_until = -1e9   # 같은 코너에서 재시작 방지: 코너 출구 곡률로 다시 준비되지 않게 arm_seconds 동안 막음
            self.rearm_at = now + h['arm_seconds']
            self.steering = 0.0       # 놓친 동안 남은 큰 조향으로 재시작하지 않도록
            return False
        return True
