"""일반 차선 주행: 목표점 pure pursuit (목표점 없으면 lookahead 오차 PID)"""
# 출처: hello-osy/2026-orda-mentoring src/orda_sim/orda_sim/answer.py (613c9b4)
# 11/8 코너: 곡률 앞먹임 제거, 목표점 있으면 pure pursuit만 (PR #8)
# 11/9 우회전 코너 유지 추가 (corner_hold)
# 11/9 출력 조향 변화율 제한·한 프레임 급변 무시 추가 (녹화 integ_1109_lap4: 프레임당 오차 5 cm 넘는 점프 108회)
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
        self.hold = None          # 코너 유지 중: dict(yaw 누적 회전 rad, distance m)
        self.output = 0.0         # 실제 내보낸 조향 (변화율 제한 기준)
        self.lane_stamp = None    # 마지막으로 판단한 카메라 결과 시각 (주기 20 Hz·카메라 28 Hz → 같은 결과 반복 가능)
        self.suspect = 0          # 연속으로 무시한 급변 프레임 수

    def update(self, lane, now, dt, slow=False, velocity=0.0, yaw_rate=0.0, allow_hold=True):
        speed, target = self.decide(lane, now, dt, slow, velocity, yaw_rate, allow_hold)
        # 출력 조향 변화율 제한: 인식 튐·코너 유지 시작/해제가 바퀴에 계단으로 들어가지 않게
        step = self.c['steering_rate'] * dt
        self.output += clip(target - self.output, -step, step)
        return speed, self.output

    def decide(self, lane, now, dt, slow, velocity, yaw_rate, allow_hold):
        """이번 주기 목표 (속도, 조향). 조향은 변화율 제한 전 값"""
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
                desired = clip(geometric, -limit, limit)
            else:
                desired = clip(geometric+c['pid']['feedback_gain']*self.pid.update(error, dt), -limit, limit)
            # 한 프레임 급변 무시: 차는 프레임당 1.3 cm 이하로 움직이는데 선 개수 전환·잡음으로 목표가 튐
            # steering_jump 넘게 바뀐 결과는 jump_frames 프레임까지 무시, 계속되면 실제 변화로 보고 받아들임
            stamp = lane.get('stamp', now)
            if stamp != self.lane_stamp:
                self.lane_stamp = stamp
                if abs(desired - self.steering) > c['steering_jump'] and self.suspect < c['jump_frames']:
                    self.suspect += 1
                else:
                    self.suspect = 0
                    self.steering = desired
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
            self.hold = dict(yaw=0.0, distance=0.0)
        self.hold['yaw'] += yaw_rate * dt
        self.hold['distance'] += abs(velocity) * dt
        turned = math.degrees(-self.hold['yaw'])     # 오른쪽 회전 = odom 각속도 음수
        # 두 선 보이면 일찍 끝내는 조건은 제거: 차로 2개 구간 코너 출구에서 옆 차로 선 쌍을 잡아 52°에 종료 → 이탈 (11/9 integ_1109_lap2 172 s)
        if turned >= h['angle'] or self.hold['distance'] >= h['max_distance']:
            self.hold = None
            self.armed_until = -1e9   # 같은 코너에서 재시작 방지: 코너 출구 곡률로 다시 준비되지 않게 arm_seconds 동안 막음
            self.rearm_at = now + h['arm_seconds']
            self.steering = 0.0       # 놓친 동안 남은 큰 조향으로 재시작하지 않도록
            return False
        return True
