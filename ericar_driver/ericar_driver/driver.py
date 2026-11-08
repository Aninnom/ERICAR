"""미션 선택·최종 속도/조향 결정"""
# 출처: hello-osy/2026-orda-mentoring src/orda_sim/orda_sim/answer.py (613c9b4)

from .common import clip
from .cones import ConeDriving
from .lane_change import LaneChange
from .lane_driving import LaneDriving
from .parking import Parking

# 정지 미션 시간 초과 후 같은 표지판에 바로 다시 반응하지 않도록 쿨다운을 거는 표지판
MISSION_SIGNS = {
    'pedestrian': ['pedestrian'],
    'traffic': ['traffic_red', 'traffic_yellow', 'traffic_green'],
}


class Driver:
    """공통: 표지판·정지선 상태를 관리하고 담당별 제어기의 출력을 선택한다."""
    def __init__(self, config):
        self.c, self.robot = config['answer'], config['robot']
        self.lane_driver = LaneDriving(self.c, self.robot)
        self.lane_change = LaneChange(self.c, self.robot)
        self.parking = Parking(self.c)
        self.cones = ConeDriving(self.c, self.robot)
        self.pid = self.lane_driver.pid
        self.state, self.state_started = 'lane', 0.0
        self.last_time = self.last_lane_stamp = self.last_detection_stamp = None
        self.stopped_since = self.line_distance = None
        self.line_seen = -1e9
        self.slow_until = self.fork_remaining = self.fork_bias = self.fork_target = 0.0
        self.fork_pending, self.fork_clear_distance = False, 0.0
        self.fork_request = 0
        self.traffic_stop_completed = False
        # 재출발: 심사위원이 차를 미션 시작 지점으로 옮긴 경우 그 미션부터 시작 (node.py --start-stage)
        sequence = self.c['missions']['sequence']
        self.stage_index = sequence.index(self.c['missions'].get('start_stage') or sequence[0])
        self.cooldowns, self.sign_counts, self.debug = {}, {}, {}

    @property
    def avoiding(self):
        return self.lane_change.active or self.cones.active

    def transition(self, state, now):
        self.state, self.state_started, self.stopped_since = state, now, None
        self.pid.reset()
        if state != 'lane':
            self.lane_change.active = self.cones.active = False
        if state == 'parking_align':
            self.parking.start(now)

    def sensor_failure(self, now):
        self.stopped_since = self.parking.stable_since = None
        if self.state == 'parking_sequence':
            # 센서가 돌아올 때까지 시간표를 멈춘다 (영구 정지하면 주차 0점)
            self.parking.pause(now)

    def leave_mission(self, mission, now):
        """정지 미션 시간 초과: 영구 정지(fault) 대신 차선 주행으로 돌아간다.

        미션 단계는 넘기지 않는다. 다음 미션 표지판이 이미 보이는 위치(예: 신호등 옆 주차 표지판)에서
        곧장 다음 미션에 들어가는 오작동을 막기 위함. 재출발은 launch의 start_stage로 한다.
        같은 표지판에 바로 다시 들어가지 않도록 쿨다운만 건다.
        """
        for name in MISSION_SIGNS.get(mission, []):
            self.cooldowns[name] = now + self.c['missions']['sign_cooldown']
        self.line_distance = None
        self.transition('lane', now)

    def stage(self):
        sequence = self.c['missions']['sequence']
        return sequence[self.stage_index] if self.stage_index < len(sequence) else 'complete'

    def allowed(self, mission):
        return not self.c['missions']['enforce_order'] or self.stage() == mission

    def advance(self, mission):
        if self.stage() == mission:
            self.stage_index += 1

    def confirmed(self, detections, stamp):
        truncated_allowed = self.c['detector'].get('allow_truncated_classes', ['traffic_red', 'traffic_yellow', 'traffic_green'])
        names = {d['name'] for d in detections
                 if (not d.get('truncated', False) or d['name'] in truncated_allowed) and d['area'] >= self.c['detector'].get('minimum_area_by_class', {}).get(d['name'], self.c['detector']['minimum_area'])}
        if stamp != self.last_detection_stamp:
            self.sign_counts = {name: self.sign_counts.get(name, 0) + 1 for name in names}
            self.last_detection_stamp = stamp
        return {name for name in names
                if self.sign_counts.get(name, 0) >= self.c['detector']['confirm_frames']}

    def update_stop_line(self, lane, velocity, now, dt):
        m = self.c['missions']
        if self.line_distance is not None:
            self.line_distance -= velocity * dt
        image_stamp = lane.get('stamp', now)
        new_image = image_stamp != self.last_lane_stamp
        self.last_lane_stamp = image_stamp
        if new_image and lane['stop_distance'] is not None:
            # A delayed image measures the stripe at exposure, not at this tick.
            candidate = lane['stop_distance'] - velocity * max(0.0, now-image_stamp)
            tracking_stop = self.state.endswith('_approach') or self.state.endswith('_hold') or self.state == 'parking_align'
            # Once the first stop stripe is selected, do not jump to a farther
            # crosswalk stripe when the first leaves the camera's field of view.
            same_line = self.line_distance is None or abs(candidate - self.line_distance) < m['line_match_tolerance']
            if not tracking_stop or same_line:
                self.line_distance = candidate
                self.line_seen = image_stamp
        if (now - self.line_seen > m['stop_line_memory'] and
                not self.state.endswith(('_hold', '_approach')) and self.state != 'parking_align'):
            self.line_distance = None
        if self.state == 'lane' and self.line_distance is not None and self.line_distance < -0.05:
            self.line_distance = None

    def update_signs(self, scan, detections, detection_stamp, velocity, now, dt):
        c, m = self.c, self.c['missions']
        names = self.confirmed(detections, detection_stamp)
        ready = {name for name in names if now >= self.cooldowns.get(name, 0)}
        green_only = 'traffic_green' in names and not ({'traffic_red', 'traffic_yellow'} & names)
        if 'slow' in ready and self.allowed('slow'):
            self.advance('slow')
            self.slow_until = now + m['slow_seconds']
            self.cooldowns['slow'] = now + m['sign_cooldown']
        if ('left' in ready or 'right' in ready) and self.allowed('fork') and self.fork_remaining <= 0.0:
            self.advance('fork')
            self.fork_target = m['fork_bias'] if 'left' in ready else -m['fork_bias']
            self.fork_pending = m.get('fork_wait_for_obstacles', True)
            self.fork_clear_distance = 0.0
            self.fork_bias = 0.0 if self.fork_pending else self.fork_target
            self.fork_remaining = m.get('fork_distance', 1.6)
            self.cooldowns['left'] = self.cooldowns['right'] = now + m['sign_cooldown']
        # Read the arrow early, but finish the parked-car corridor before
        # changing sides. Require measured travel with no wide object ahead.
        if self.fork_pending:
            if self.lane_change.active or scan.get('wide_objects', 0) > 0:
                self.fork_clear_distance = 0.0
            else:
                self.fork_clear_distance += max(0.0, velocity) * dt
            if self.fork_clear_distance >= m.get('fork_clear_distance', .20):
                # 표지판 방향으로 실제 한 차로 이동한 뒤 새 차선을 따른다.
                direction = 1 if self.fork_target > 0 else -1
                destination = self.c['lane_change']['lane_count']-1 if direction > 0 else 0
                self.fork_request = direction if self.lane_change.index != destination else 0
                self.fork_bias = 0.0
                self.fork_pending = False
        if self.fork_remaining <= 0.0:
            self.fork_bias = 0.0
            self.fork_target = 0.0
            self.fork_pending = False
        if self.state == 'lane':
            if ('traffic_red' in ready or 'traffic_yellow' in ready) and self.allowed('traffic'):
                self.traffic_stop_completed = False
                self.transition('traffic_approach', now)
            elif green_only and 'traffic_green' in ready and self.allowed('traffic'):
                self.traffic_stop_completed = False
                self.transition('traffic_approach', now)
            elif 'pedestrian' in ready and self.allowed('pedestrian'):
                fresh_line = (self.line_distance is not None and m['stop_offset'] < self.line_distance < m.get('trigger_line_max_distance', 1.5) and
                              now - self.line_seen < m.get('trigger_line_freshness', .4))
                if fresh_line or not m.get('pedestrian_require_line', True):
                    self.transition('pedestrian_approach', now)
            elif 'parking' in ready and c['parking']['enabled'] and self.allowed('parking'):
                self.transition('parking_align', now)
        return green_only

    def stop_missions(self, speed, velocity, green_only, now):
        m = self.c['missions']
        if self.state.endswith('_approach'):
            crossing_on_green = (self.state == 'traffic_approach' and green_only and
                                 (not m.get('traffic_require_stop', True) or self.traffic_stop_completed))
            if now - self.state_started > m['approach_timeout']:
                # 정지선에 도달하지 못함: 멈춰 있지 않고 차선 주행으로 복귀 (미션 단계는 유지)
                self.leave_mission(self.state.replace('_approach', ''), now)
                return speed
            if self.line_distance is None:
                # 정지선이 아직 안 보이면 멈추지 않고 저속으로 다가가며 찾는다.
                # (멈춰 있으면 정지선이 계속 안 보여 결국 시간 초과 실패)
                speed = min(speed, m['stop_approach_speed'])
            elif crossing_on_green:
                # Seeing green far away is not mission completion. Keep watching
                # red/yellow until the measured front axle crosses this stripe.
                if self.line_distance < -m['stop_tolerance']:
                    for name in ['traffic_red', 'traffic_yellow', 'traffic_green']:
                        self.cooldowns[name] = now + m['sign_cooldown']
                    self.advance('traffic')
                    self.line_distance = None
                    self.transition('lane', now)
            else:
                remaining = self.line_distance - m['stop_offset']
                speed = min(speed, m['stop_approach_speed'], max(0.0, remaining * m['stop_gain']))
                if remaining <= m['stop_tolerance']:
                    speed = 0.0
                    if abs(velocity) <= m['stopped_speed']:
                        self.transition(self.state.replace('_approach', '_hold'), now)
        if self.state == 'pedestrian_hold':
            speed = 0.0
            if abs(velocity) > m['stopped_speed']:
                self.stopped_since = None
            elif self.stopped_since is None:
                self.stopped_since = now
            elif now - self.stopped_since >= max(3.0, m['pedestrian_wait']):
                self.cooldowns['pedestrian'] = now + m['sign_cooldown']
                self.advance('pedestrian')
                self.line_distance = None
                self.transition('lane', now)
        if self.state == 'traffic_hold':
            speed = 0.0
            if abs(velocity) > m['stopped_speed']:
                self.stopped_since = None
            elif self.stopped_since is None:
                self.stopped_since = now
            minimum_stop = m.get('traffic_min_stop', .3) if m.get('traffic_require_stop', True) else 0.0
            if self.stopped_since is not None and now - self.stopped_since >= minimum_stop:
                self.traffic_stop_completed = True
            if green_only and self.traffic_stop_completed:
                self.transition('traffic_approach', now)
        return speed

    def update(self, lane, scan, detections, detection_stamp, velocity, now, angular_velocity=0.0):
        if self.last_time is not None and now < self.last_time:
            self.__init__({'answer': self.c, 'robot': self.robot})
        dt = .05 if self.last_time is None else clip(now-self.last_time, .001, .2)
        self.last_time = now
        self.fork_remaining = max(0, self.fork_remaining-max(0, velocity)*dt)
        self.update_stop_line(lane, velocity, now, dt)
        green_only = self.update_signs(scan, detections, detection_stamp, velocity, now, dt)
        speed, steering = self.lane_driver.update(lane, now, dt, now < self.slow_until)
        mode = self.state
        if self.state == 'lane':
            change = self.lane_change.update(lane, scan, velocity, angular_velocity, dt,
                                            self.allowed('fork') or self.fork_remaining > 0, self.fork_request)
            if self.fork_request and self.lane_change.active:
                self.fork_request = 0
            cone = None if change is not None else self.cones.update(
                lane, scan, velocity, angular_velocity, dt, self.stage() == 'traffic' and self.fork_remaining <= 0)
            if change is not None:
                speed, steering = change
                mode = 'lane_change' if self.lane_change.active else 'lane_change: waiting for clear lane'
            elif cone is not None:
                speed, steering = cone
                mode = 'cones'
        speed = self.stop_missions(speed, velocity, green_only, now)
        if self.state in ('parking_align', 'parking_sequence'):
            speed, steering, self.state = self.parking.update(lane, self.line_distance, velocity, now, steering)
            if self.state == 'parked':
                self.advance('parking')
        # 모든 담당의 제어 결과에 공통으로 적용하는 최종 정지 조건.
        clearance = scan['front'] if speed >= 0 else scan['rear']
        blocked = scan['valid_fraction'] < .5 or clearance < self.c['lidar']['emergency_distance']
        if blocked and self.state == 'parking_sequence':
            # 주차 중: 영구 정지 대신 시간표를 멈추고 기다린다. 일정 시간 넘게 막혀 있으면
            # (실차 후방 사각으로 rear가 항상 0인 경우 포함) 접촉 감점(-1)을 감수하고 진행한다.
            self.parking.obstacle_wait += dt
            if self.parking.obstacle_wait < self.c['parking'].get('obstacle_wait', 2.0):
                self.parking.pause(now)
            else:
                blocked = False
        if blocked:
            speed = 0.0
        if self.state.startswith('fault') or self.state == 'parked':
            speed = 0.0
        self.debug = dict(lane_valid=lane['valid'], lane_error=lane['error'],
                          pid=dict(self.pid.terms), pid_feedback_gain=self.c['pid']['feedback_gain'],
                          stage=self.stage(), front=scan['front'], rear=scan['rear'],
                          lane_index=self.lane_change.index, lane_change_target=self.lane_change.target,
                          lane_change_offset=self.lane_change.offset, lane_change_heading=self.lane_change.heading,
                          line_distance=self.line_distance, fork_pending=self.fork_pending,
                          fork_remaining=self.fork_remaining, fork_request=self.fork_request,
                          avoiding=self.avoiding, cone_active=self.cones.active,
                          cone_heading=self.cones.heading, cone_center=self.cones.center, speed=speed)
        return float(speed), clip(steering, -self.robot['max_steering'], self.robot['max_steering']), mode if self.state == 'lane' else self.state
