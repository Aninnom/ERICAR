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
        self.far_heading = 0.0    # 마지막으로 본 선 먼 끝 방향(rad, + 왼쪽). 코너 유지 시작 판단
        self.last_curvature = 0.0 # 마지막으로 본 선 곡률(1/m). 코너 유지 조향 선택
        self.left_near_at = -1e9  # 가까운 선이 왼쪽으로 휜 마지막 시각 (비스듬한 진입 판단)
        self.hold_ended = -1e9    # 코너 유지 끝난 시각
        self.zigzag_frames = 0    # 지그재그로 판단된 연속 카메라 결과 수
        self.zigzag_left = 0.0    # 지그재그 서행을 유지할 남은 거리(m)
        self.search_distance = 0.0  # 선을 놓친 뒤(lost_grace 이후) 탐색하며 간 거리(m)

    def in_corner(self, now):
        """코너 유지 중 또는 끝난 뒤 corner_hold.after_seconds 이내 (lane_vision 한 선 이름 판단용)"""
        h = self.c.get('corner_hold', {})
        return self.hold is not None or now - self.hold_ended < h.get('after_seconds', 0.0)

    def beside_start_lane(self, sections):
        """마지막(4번) 코너를 돌아 S4에 들어온 뒤 lane.double_line_distance(m)까지, 코너 유지 중 아님 (lane_vision.double_left용)
        그 구간은 왼쪽에 출발 차로 아래 선이 지금 차로 왼쪽 선과 나란함. 유지 중·다른 구간은 예전 동작 그대로
        sections: 구간 관리자 (section.py, index·distance)"""
        return (self.hold is None and sections.index == 4
                and sections.distance < self.c['lane']['double_line_distance'])

    def update(self, lane, now, dt, slow=False, velocity=0.0, yaw_rate=0.0, allow_hold=True, avoiding=False):
        if avoiding:
            # 라바콘·차선 변경이 조향 중: 선 놓침 탐색 시간·거리 누적 안 함 (콘 구간 약 2.1 m에서 선 놓치면
            # lost_search.distance 넘겨 끝나자마자 정지). 끝난 뒤 lost_grace부터 다시 셈
            self.last_seen, self.search_distance = now, 0.0
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
                # 서행 유도선 확인: 연속 confirm_frames면 서행 시작·남은 거리 갱신, 아닌 결과는 하나씩 깎음 (한두 프레임 끊김 허용)
                z = c['zigzag']
                both = lane.get('zigzag', 0) >= 2      # 서로 다른 두 선 모두 톱니일 때만 확정 (한 선은 주름·꺾임 오검출)
                self.zigzag_frames = min(self.zigzag_frames + 1, z['confirm_frames']) if both else max(self.zigzag_frames - 1, 0)
                if both and self.zigzag_frames >= z['confirm_frames']:
                    self.zigzag_left = z['hold_distance']
                if abs(desired - self.steering) > c['steering_jump'] and self.suspect < c['jump_frames']:
                    self.suspect += 1
                else:
                    self.suspect = 0
                    self.steering = desired
        speed = max(c['minimum_curve_speed'], c['base_speed']*(1-c['curve_speed_gain']*abs(self.steering)/limit))
        # 코너 미리 감속: 조향 기준 감속은 조향이 커진 뒤라 늦음 (11/9 실차 2번 코너 0.37 m/s 진입 → 바깥 이탈)
        # 보이는 선 곡률이 코너 폭 보간 시작값 이상이면 코너 속도. 회전 시작 약 2.3 s 전 감지 (녹화 bringup_1109_0841)
        # 코너 유지 준비(corner_seen)는 곡률 시작값 그대로, 감속은 곡률 corner_slow_curvature 구간에서 기본 → 코너 속도로 보간
        # 곡률 0.8에서 바로 감속하면 꺾기 3.8~6.4 s 전부터 0.2 m/s (11/9 confusion_1109_lap1 코너 4곳 중 3곳)
        # 곡률 ≈ 코너까지 거리: 2 → 약 0.5 m 전, 3.5 → 약 0.2 m 전 (같은 녹화, 코너 유지 시작 기준)
        curvature = lane.get('curvature', 0.0) if lane['valid'] else 0.0
        corner_seen = lane['valid'] and curvature >= c['lane']['corner_curvature'][0]
        lo, hi = c['corner_slow_curvature']
        ramp = clip((curvature - lo) / (hi - lo), 0.0, 1.0)
        speed = min(speed, c['base_speed'] - ramp * (c['base_speed'] - c['corner_speed']))
        # 서행 유도선: 규정상 서행 구간(최대 0.2 m/s)인데 인식이 정상이라 0.39 m/s로 지나감 (11/10 skip_1110_lap1 106~110 s)
        # 1번 코너 유지 중·직후에는 바깥 선 하나만 보임 (104.5~105.9 s) → 그동안은 한 선만 톱니여도 서행 (확정·거리 유지는 안 함)
        # 안 하면 유지 끝나고 두 선 확정까지 0.4 s 동안 0.19 → 0.40 m/s로 가속했다가 다시 감속
        self.zigzag_left = max(0.0, self.zigzag_left - abs(velocity) * dt)
        entering = lane['valid'] and lane.get('zigzag', 0) >= 1 and self.in_corner(now)
        if self.zigzag_left > 0.0 or entering:
            speed = min(speed, c['zigzag']['speed'])
        if self.corner_hold(lane, now, dt, velocity, yaw_rate, allow_hold, corner_seen):
            return min(speed, c['corner_speed']), -self.hold['steering']
        if slow or lane['abnormal'] or not lane['valid']:
            speed = min(speed, c['slow_speed'], c['base_speed']/2, .2)
        lost_for = now - self.last_seen
        if lost_for <= c['lane']['lost_grace']:
            self.search_distance = 0.0
            return speed, self.steering
        # 선을 lost_grace 넘게 놓침. 예전: 바로 속도 0 → 선이 다시 보일 일이 없어 그대로 정지 (규정 III-자: 15초 내 미발진이면 미션 0점)
        # (11/10 zigzag_1110_lap1 120.9 s: 3차선 가운데 차로에서 정지, 섬 노란 선은 0.5 m 앞이라 조금만 더 가면 보였음)
        # → lost_search.speed로 계속 가며 선 탐색. 조향은 마지막 값에서 straighten_distance 동안 0으로 (큰 조향 그대로면 제자리 돌아 역주행 방향)
        # 끝: 탐색 거리 distance 또는 놓친 뒤 seconds 초과 → 정지 (트랙 밖으로 계속 가지 않게, 출발 전 선 없는 곳에 세워 둔 차가 움직이지 않게)
        s = c['lost_search']
        self.search_distance += abs(velocity) * dt
        fade = clip(1.0 - self.search_distance / s['straighten_distance'], 0.0, 1.0)
        if self.search_distance >= s['distance'] or lost_for > c['lane']['lost_grace'] + s['seconds']:
            return 0.0, self.steering * fade
        return min(speed, s['speed']), self.steering * fade

    def corner_hold(self, lane, now, dt, velocity, yaw_rate, allow_hold, corner_seen):
        """우회전 코너 유지 판단. True면 이번 주기 고정 조향 사용 (카메라 결과 무시)"""
        h = self.c.get('corner_hold', {})
        if not h.get('enabled') or not allow_hold:
            self.hold = None
            return False
        if corner_seen and now >= self.rearm_at:
            self.armed_until = now + h['arm_seconds']
        if lane['valid'] and 'far_heading' in lane:
            self.far_heading = lane['far_heading']
        if lane['valid']:
            self.last_curvature = lane.get('curvature', 0.0)
            near = lane.get('near_curvature')
            if near is not None and near > h['skip_near_curvature']:
                self.left_near_at = now
        if self.hold is None:
            # 시작: 코너 감지 후 오른쪽 조향이 커짐, 또는 오른쪽으로 꺾던 중 선을 놓침
            turning = self.steering <= -h['start_steering'] or (not lane['valid'] and self.steering < -0.05)
            if now >= self.armed_until or not turning:
                return False
            # 보이는 선 먼 끝이 오른쪽으로 start_far_angle(°) 넘게 꺾여 나갈 때만 시작 (선 놓친 프레임은 마지막 값)
            # 갈림길 끝 45° S자 꺾임: 앞 꺾임에서 준비된 뒤 먼 끝이 왼쪽으로 돌아오는데 고정 65° 회전 → 이탈
            # (11/9 smooth_1109_lap1 212 s, 먼 끝 +1~+25°. 진짜 코너 시작 시 −37~−75°)
            if self.far_heading > -math.radians(h['start_far_angle']):
                return False
            # 비스듬한 진입: 가까운 선이 왼쪽으로 휨 = 차가 바깥 선 쪽으로 비스듬히 다가감 → 고정 원호가 안 맞아 안쪽 선 물음
            # 선이 계속 보이므로 이번 코너는 pursuit만 (arm_seconds 동안 재시작 막음). 시작 순간엔 이미 오른쪽 휨(−0.3)일 수 있어 skip_window 안 기록으로 판단
            # (11/10 오른쪽 갈림길 → 3번 코너: 가까운 휨 +0.3~+0.4. 정상 코너 시작 직전 −0.8~−3.1. main_1110_lap6 106.1 s)
            if now - self.left_near_at < h['skip_window']:
                self.armed_until = -1e9
                self.rearm_at = now + h['arm_seconds']
                return False
            # 조향 선택: 코너가 바로 앞에 보이며 시작(곡률 큼)하면 크게, 이미 pursuit로 돌던 중 시작(곡률 작음)하면 작게
            # (11/9 main_1109_lap3: 1·2번 코너 시작 곡률 5~9 → 0.38로 1번 통과, 3·4번 0.0~0.4 → 0.38이면 3번 안쪽 선 물음)
            sharp = self.last_curvature >= h['sharp_curvature']
            base = h['steering'] if sharp else h['gentle_steering']
            self.hold = dict(yaw=0.0, distance=0.0, base=base, steering=base,
                             error0=lane['error'] if lane['valid'] else None)
        # 유지 중 보정: 보이는 선 오차가 시작 때보다 오른쪽(−)으로 커지면 더, 왼쪽(+)으로 가면 덜 꺾음. 선 놓치면 마지막 값
        # (11/10 main_1109_lap4: 3번 코너 0.31에도 오차 −0.17→+0.15 안쪽 선 물음, 마지막 코너 시작 곡률 튀어 0.31 선택 → 오차 −0.26→−0.37 늦게 돎)
        if lane['valid']:
            if self.hold['error0'] is None:
                self.hold['error0'] = lane['error']
            self.hold['steering'] = clip(self.hold['base'] + h['error_gain'] * (self.hold['error0'] - lane['error']),
                                         h['steering_min'], self.robot['max_steering'])
        self.hold['yaw'] += yaw_rate * dt
        self.hold['distance'] += abs(velocity) * dt
        turned = math.degrees(-self.hold['yaw'])     # 오른쪽 회전 = odom 각속도 음수
        # 두 선 보이면 일찍 끝내는 조건은 제거: 차로 2개 구간 코너 출구에서 옆 차로 선 쌍을 잡아 52°에 종료 → 이탈 (11/9 integ_1109_lap2 172 s)
        if turned >= h['angle'] or self.hold['distance'] >= h['max_distance']:
            self.hold = None
            self.armed_until = -1e9   # 같은 코너에서 재시작 방지: 코너 출구 곡률로 다시 준비되지 않게 arm_seconds 동안 막음
            self.rearm_at = now + h['arm_seconds']
            self.hold_ended = now
            self.steering = 0.0       # 놓친 동안 남은 큰 조향으로 재시작하지 않도록
            return False
        return True
