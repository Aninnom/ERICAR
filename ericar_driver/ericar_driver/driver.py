"""미션 선택·최종 속도/조향 결정"""
# 출처: hello-osy/2026-orda-mentoring src/orda_sim/orda_sim/answer.py (613c9b4)

from .common import clip
from .cones import ConeDriving
from .fork import ForkMission
from .lane_change import LaneChange
from .lane_driving import LaneDriving
from .lidar import body_points, clear_steering
from .parking import Parking
from .section import SectionManager, START_SECTION

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
        self.slow_until = 0.0
        m = self.c['missions']
        self.fork = ForkMission(self.c['lane_change']['lane_count'], m['fork_distance'], far=self.c['lane']['far'],
                                dash_gap=m.get('fork_dash_gap', .3), entrance_confirm=m.get('fork_entrance_confirm', .3))
        self.sync_fork()
        self.traffic_stop_completed = False
        self.traffic_seen = -1e9     # 신호등(색 무관, 작은 박스 포함)을 마지막으로 본 시각 (소민)
        self.traffic_red_area = 0.0  # 이번 프레임 빨강·노랑 박스 최대 면적 비율
        self.line_kind = None        # line_distance가 가리키는 선: 'crosswalk'(횡단보도 바로 앞) / 'plain'
        # 구간 관리자: 미션 사슬 대체. 표지판을 놓쳐도 코너를 돌면 다음 구간 미션이 켜짐 (section.py)
        # 재출발: 심사위원이 차를 미션 시작 지점으로 옮긴 경우 그 미션의 구간부터 시작 (node.py --start-stage)
        self.sections = SectionManager(self.c['sections'], START_SECTION[self.c['missions'].get('start_stage') or 'slow'])
        self.done = set()             # 이번 주행에서 끝낸(또는 시간 초과로 포기한) 미션. 같은 구간에서 다시 들어가지 않음
        self.stall_time = 0.0         # 의도하지 않은 정지 누적 시간(s)
        self.stall_moved = 0.0        # 정지가 풀린 뒤 주행 거리(m). stall.reset_distance 넘으면 stall_time 초기화
        self.lane_change_off = 0.0    # 차선 변경 대기·막힘 강제 해제 뒤 차선 변경을 끄는 남은 거리(m)
        self.signs_lost = 0.0         # 표지판 검출이 끊긴 누적 시간(s)
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

        그 미션은 포기(done)로 표시해 같은 구간에서 다시 들어가지 않음. 다음 미션은 구간이 바뀌면 켜짐.
        재출발은 launch의 start_stage로 한다.
        """
        for name in MISSION_SIGNS.get(mission, []):
            self.cooldowns[name] = now + self.c['missions']['sign_cooldown']
        self.done.add(mission)
        self.line_distance = None
        self.transition('lane', now)

    def stage(self):
        """디버그 표시용: 지금 구간과 활성 미션"""
        return 'S%d %s' % (self.sections.index, '·'.join(sorted(self.sections.missions() - self.done)))

    def outer_yellow(self):
        # 3차선 바깥 차로(목적 차로 포함)면 노란 바깥선 쪽. lane_vision 한 선 이름 판단용 (소민)
        change = self.lane_change
        if not (self.sections.allows('lane_change') or self.fork_remaining > 0):
            return None
        lane = change.index + (change.direction if change.active else 0)
        last = change.c['lane_count'] - 1
        return 'left' if lane == last else 'right' if lane == 0 else None

    def allowed(self, mission):
        """지금 구간의 미션이고 아직 끝내지 않았을 때만"""
        return self.sections.allows(mission) and mission not in self.done

    def sync_fork(self):
        """기존 디버그 필드 호환. 판단의 원본은 ForkMission 한 곳"""
        f = self.fork
        self.fork_bias = 0.0
        self.fork_remaining = f.remaining
        self.fork_target = f.direction*self.c['missions']['fork_bias'] if f.running else 0.0
        self.fork_goal = f.direction if f.running else 0
        self.fork_request = f.request(self.lane_change.index, self.lane_change.active)
        self.fork_pending = f.phase == 'locked'
        self.fork_sign, self.fork_sign_stamp = f.sign, f.stamp

    def fork_direction(self):
        """선택 출구 추종 상태에서만 영상 보정 허용"""
        return self.fork.vision_direction if not self.lane_change.active else 0

    def advance(self, mission):
        self.done.add(mission)

    def confirmed(self, detections, stamp):
        truncated_allowed = self.c['detector'].get('allow_truncated_classes', ['traffic_red', 'traffic_yellow', 'traffic_green'])
        names = {d['name'] for d in detections
                 if (not d.get('truncated', False) or d['name'] in truncated_allowed) and d['area'] >= self.c['detector'].get('minimum_area_by_class', {}).get(d['name'], self.c['detector']['minimum_area'])}
        if {'left', 'right'} <= names:
            # 연속 횟수가 다른 양쪽 화살표도 확정 보류. 새 단일 방향부터 재확인
            names -= {'left', 'right'}
            self.sign_counts.pop('left', None)
            self.sign_counts.pop('right', None)
        # 빨강·노랑은 첫 검출부터 녹색 허가 차단. 광고판 등 동시 검출 시 보수적 대기
        # 충돌 해소 후 새 녹색 프레임을 confirm_frames만큼 다시 확인
        if names & {'traffic_red', 'traffic_yellow'}:
            names.discard('traffic_green')
            self.sign_counts.pop('traffic_green', None)
        if stamp != self.last_detection_stamp:
            self.sign_counts = {name: self.sign_counts.get(name, 0) + 1 for name in names}
            self.last_detection_stamp = stamp
        return {name for name in names
                if self.sign_counts.get(name, 0) >= self.c['detector'].get('confirm_frames_by_class', {}).get(
                    name, self.c['detector']['confirm_frames'])}

    def update_stop_line(self, lane, velocity, now, dt):
        m = self.c['missions']
        if self.line_distance is not None:
            self.line_distance -= velocity * dt
        image_stamp = lane.get('stamp', now)
        new_image = image_stamp != self.last_lane_stamp
        self.last_lane_stamp = image_stamp
        cross = lane.get('crosswalk_distance')
        if new_image and (cross is not None or lane['stop_distance'] is not None):
            # 횡단보도 바로 앞 선이 보이면 그 선 우선. 한 번 잡으면 일반 흰 선(서행 시작선·횡단보도 뒤 선)으로 안 바꿈 (소민)
            kind = 'crosswalk' if cross is not None else 'plain'
            # A delayed image measures the stripe at exposure, not at this tick.
            candidate = (lane['stop_distance'] if cross is None else cross) - velocity * max(0.0, now-image_stamp)
            tracking_stop = self.state.endswith('_approach') or self.state.endswith('_hold') or self.state == 'parking_align'
            # Once the first stop stripe is selected, do not jump to a farther
            # crosswalk stripe when the first leaves the camera's field of view.
            same_line = self.line_distance is None or abs(candidate - self.line_distance) < m['line_match_tolerance']
            # 차선 주행 중에도 가까운 선을 고른 뒤에는 더 먼 선(횡단보도 뒤쪽 선 등)으로 건너뛰지 않음.
            # 가까운 선이 카메라 아래(앞 0.4 m 안)로 사라진 뒤 신호등을 확정하면 뒤쪽 선에 서거나
            # 선을 잃고 그냥 지나가던 문제 (11/12 실차, 소민). 지난 선(-0.05 m)은 아래에서 지움
            nearer = self.line_distance is None or candidate < self.line_distance + m['line_match_tolerance']
            upgrade = kind == 'crosswalk' and self.line_kind != 'crosswalk'
            keep_crosswalk = kind == 'plain' and self.line_kind == 'crosswalk' and self.line_distance is not None
            if not keep_crosswalk and ((not tracking_stop and nearer) or same_line or upgrade):
                self.line_distance = candidate
                self.line_seen = image_stamp
                self.line_kind = kind
        if (now - self.line_seen > m['stop_line_memory'] and
                not self.state.endswith(('_hold', '_approach')) and self.state != 'parking_align'):
            self.line_distance = None
        if self.state == 'lane' and self.line_distance is not None and self.line_distance < -0.05:
            self.line_distance = None
        if self.line_distance is None:
            self.line_kind = None

    def update_signs(self, scan, detections, detection_stamp, velocity, now, dt):
        c, m = self.c, self.c['missions']
        names = self.confirmed(detections, detection_stamp)
        ready = {name for name in names if now >= self.cooldowns.get(name, 0)}
        # 신호등 감시: 확정 전·작은 박스라도 신호등이 보이면 시각 기록 (S4 감속용), 빨강·노랑 최대 면적
        traffic = [d for d in detections if d['name'] in MISSION_SIGNS['traffic'] and
                   d['area'] >= m.get('traffic_watch_area', .002)]
        if traffic:
            self.traffic_seen = now
        self.traffic_red_area = max([d['area'] for d in traffic if d['name'] != 'traffic_green'] or [0.0])
        green_only = 'traffic_green' in names and not ({'traffic_red', 'traffic_yellow'} & names)
        if 'slow' in ready and self.allowed('slow'):
            self.advance('slow')
            self.slow_until = now + m['slow_seconds']
            self.cooldowns['slow'] = now + m['sign_cooldown']
        self.fork.step(self.lane_change.index, self.lane_change.active, velocity, dt,
                       left_section=self.c['sections']['enabled'] and self.sections.index > START_SECTION['fork'])
        if self.allowed('fork') and self.fork.lock(ready, detection_stamp):
            self.advance('fork')
            self.cooldowns['left'] = self.cooldowns['right'] = now + m['sign_cooldown']
        self.sync_fork()
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
            # 신호등: 횡단보도 바로 앞 선에만 정지 (서행 시작 흰 선에 서지 않음). 선 못 찾으면 저속 접근·큰 빨강 예비 정지
            traffic = self.state == 'traffic_approach'
            line = self.line_distance
            if traffic and m.get('traffic_crosswalk_only', True) and self.line_kind != 'crosswalk':
                line = None
            offset = m.get('traffic_stop_offset', m['stop_offset']) if traffic else m['stop_offset']
            crossing_on_green = (self.state == 'traffic_approach' and green_only and
                                 (not m.get('traffic_require_stop', True) or self.traffic_stop_completed))
            red_now = self.state == 'traffic_approach' and self.traffic_red_area > 0.0
            if now - self.state_started > m['approach_timeout']:
                if red_now:
                    # 빨강·노랑이 보이는데 정지선을 못 찾음: 지나가지 않고 그 자리 정지 (신호 위반 방지, 소민)
                    self.transition('traffic_hold', now)
                    return 0.0
                # 정지선에 도달하지 못함: 멈춰 있지 않고 차선 주행으로 복귀 (미션 단계는 유지)
                self.leave_mission(self.state.replace('_approach', ''), now)
                return speed
            if line is None and red_now and self.traffic_red_area >= m.get('traffic_stop_area', .04):
                # 정지선 없이 신호등이 충분히 가까움(박스가 큼): 예비 정지
                self.transition('traffic_hold', now)
                return 0.0
            if line is None:
                # 정지선이 아직 안 보이면 멈추지 않고 저속으로 다가가며 찾는다.
                # (멈춰 있으면 정지선이 계속 안 보여 결국 시간 초과 실패)
                speed = min(speed, m['stop_approach_speed'])
            elif crossing_on_green:
                # Seeing green far away is not mission completion. Keep watching
                # red/yellow until the measured front axle crosses this stripe.
                if line < -m['stop_tolerance']:
                    for name in ['traffic_red', 'traffic_yellow', 'traffic_green']:
                        self.cooldowns[name] = now + m['sign_cooldown']
                    self.advance('traffic')
                    self.line_distance = None
                    self.transition('lane', now)
            else:
                remaining = line - offset
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
            elif (self.signs_lost >= self.c['stall']['seconds'] or
                    now - self.state_started > m['traffic_max_wait']):
                # 녹색을 볼 수 없는 상태(YOLO 끊김)거나 너무 오래 기다림: 서 있으면 이후 미션까지 0점 → 포기하고 진행
                self.leave_mission('traffic', now)
                speed = m['stop_approach_speed']
        return speed

    def stall_guard(self, speed, steering, scan, blocked, stuck_change, velocity, dt):
        """멈춤 방지: 의도하지 않은 정지가 stall.seconds 넘으면 다음 동작으로 강제 전환. 반환 (속도, 조향, 막힘)

        대상: 차선 변경 대기(옆 차로 막힘)·차선 변경 중 안전 조향 없음·전방 비상 정지. 차선 주행 상태일 때만
        의도한 정지(보행자·신호등 대기, 주차 완료)·센서 끊김·차선 놓침 탐색 종료(lane_driving lost_search)는 대상 아님
        정지가 풀려도 stall.reset_distance(m) 가기 전에는 시간을 이어서 셈 → 대기 해제 직후 바로 또 막히면 곧장 다음 단계
        """
        s = self.c['stall']
        self.lane_change_off = max(0.0, self.lane_change_off-abs(velocity)*dt)
        emergency = blocked and scan['valid_fraction'] >= .5
        reason = None
        if self.state == 'lane':
            reason = ('lane_change' if self.lane_change.waiting or stuck_change else
                      'blocked' if emergency else None)
        if reason is None:
            self.stall_moved += abs(velocity)*dt
            if self.stall_moved >= s['reset_distance']:
                self.stall_time = 0.0
            return speed, steering, blocked
        self.stall_moved = 0.0
        self.stall_time += dt
        if self.stall_time < s['seconds']:
            return speed, steering, blocked
        if reason == 'lane_change':
            # 차선 변경 포기: lane_change_off_distance 동안 끄고 차선 주행. 앞이 막혀 있으면 아래 비상 정지 → 서행 회피로 이어짐
            self.lane_change.active = self.lane_change.waiting = False
            self.lane_change_off = s['lane_change_off_distance']
        elif speed >= 0:
            # 전방 비상 정지가 안 풀림: 라이다 점을 피하는 조향이 있을 때만 creep_speed로 비켜 감 (없으면 계속 정지)
            safe_steering, safe = clear_steering(body_points(scan, self.robot), steering, self.robot,
                                                 s['creep_horizon'], s['creep_margin'])
            if safe:
                return s['creep_speed'], safe_steering, False
        return speed, steering, blocked

    def update(self, lane, scan, detections, detection_stamp, velocity, now, angular_velocity=0.0,
               yaw_rate=None, signs_ok=True):
        """yaw_rate: 실측 회전 속도(IMU, rad/s). None이면 odom 값 × sections.odom_yaw_scale
        signs_ok: 표지판 검출이 최신인지 (False면 detections는 빈 목록으로 들어옴)"""
        if self.last_time is not None and now < self.last_time:
            self.__init__({'answer': self.c, 'robot': self.robot})
        dt = .05 if self.last_time is None else clip(now-self.last_time, .001, .2)
        self.last_time = now
        # 구간·차선 변경·라바콘 방향 추적은 실측 회전(IMU). odom 회전은 조향 명령으로 계산한 값 (실측 = 0.93 × odom, 11/9)
        # 코너 유지(lane_driving)는 odom 그대로: angle 65°가 odom 기준으로 맞춰져 있음 (11/10 판단, IMU로는 58°)
        turn_rate = angular_velocity*self.c['sections']['odom_yaw_scale'] if yaw_rate is None else yaw_rate
        self.sections.update(turn_rate, velocity, dt)
        self.signs_lost = 0.0 if signs_ok else self.signs_lost+dt
        self.update_stop_line(lane, velocity, now, dt)
        green_only = self.update_signs(scan, detections, detection_stamp, velocity, now, dt)
        # 코너 유지는 일반 차선 주행일 때만 (차로 변경·라바콘·정지·주차 중 제외)
        # 갈림길 표지판 후 fork_distance 동안도 제외: 꺾인 선 구간에서 고정 조향 후 과회전 (11/9 hold_1109_c1 150 s)
        in_fork = self.fork.running
        speed, steering = self.lane_driver.update(lane, now, dt, now < self.slow_until, velocity, angular_velocity,
                                                  allow_hold=self.state == 'lane' and not self.avoiding and not in_fork,
                                                  avoiding=self.avoiding)
        mode = 'corner_hold' if self.lane_driver.hold is not None else self.state
        stuck_change = False
        # 분기 입구(흰 점선 끝) 추적. 입구에서 목표 차로가 아니면 manual(정지), 입구 뒤 차선 변경 금지 (소민)
        if self.sections.allows('fork') or self.fork.running:
            self.fork.observe(lane, self.lane_change.index, self.lane_change.active, velocity, dt)
            self.sync_fork()
        if self.state == 'lane':
            # 최종 목적지 우선 검사. 이동 중인 제어기는 완료 전 새 목표로 전환 금지
            requested = self.fork_request
            if self.sections.allows('lane_change') and not requested:
                requested = self.fork.center_request(self.lane_change.index, self.lane_change.active, lane)
            preferred = self.fork.direction if self.fork.running else 0
            change = self.lane_change.update(lane, scan, velocity, turn_rate, dt,
                                            (self.sections.allows('lane_change') or self.fork.running)
                                            and self.lane_change_off <= 0
                                            and self.fork.allows_change(self.lane_change.active), requested, preferred)
            cone = None if change is not None else self.cones.update(
                lane, scan, velocity, turn_rate, dt, self.sections.allows('cones') and self.fork_remaining <= 0)
            if change is not None:
                stuck_change = self.lane_change.active and change[0] == 0.0
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
        speed, steering, blocked = self.stall_guard(speed, steering, scan, blocked, stuck_change, velocity, dt)
        if blocked:
            speed = 0.0
        if self.fork.phase == 'manual':
            # 분기 입구인데 가운데·반대편 차로: 교통섬을 가로지르지 않고 정지 (멈춤 방지보다 우선)
            speed, steering, mode = 0.0, 0.0, 'fork_manual: wrong lane at fork entrance'
        if self.state == 'parked':
            speed = 0.0
        if (self.state == 'lane' and self.allowed('traffic') and
                now - self.traffic_seen < self.c['missions'].get('traffic_watch_hold', 1.5)):
            # S4에서 신호등이 보이기 시작하면 확정 전에 미리 감속 → 횡단보도 앞 정지선을 카메라에서 놓치기 전에 확정 (소민)
            speed = min(speed, self.c['missions'].get('traffic_watch_speed', .18))
        if self.state == 'lane' and (self.allowed('fork') or self.fork_remaining > 0):
            # 표지판이 가까워질 때까지 기본 0.4 m/s로 접근하지 않도록 준비 속도 적용
            speed = min(speed, self.c['missions'].get('fork_prepare_speed', .18))
        self.debug = dict(lane_valid=lane['valid'], lane_error=lane['error'],
                          pid=dict(self.pid.terms), pid_feedback_gain=self.c['pid']['feedback_gain'],
                          stage=self.stage(), section=self.sections.index,
                          section_distance=self.sections.distance, section_turned=self.sections.turned,
                          yaw_deg=self.sections.yaw*57.29578, yaw_source='odom' if yaw_rate is None else 'imu',
                          stall_time=self.stall_time, lane_change_off=self.lane_change_off, signs_ok=signs_ok,
                          front=scan['front'], rear=scan['rear'],
                          lane_index=self.lane_change.index, lane_change_target=self.lane_change.target,
                          lane_change_debug=dict(self.lane_change.debug),
                          lane_change_offset=self.lane_change.offset, lane_change_heading=self.lane_change.heading,
                          line_distance=self.line_distance, fork_pending=self.fork_pending,
                          fork_phase=self.fork.phase, fork_destination=self.fork.destination,
                          fork_dash_left=self.fork.dash_left, fork_late=self.fork.late,
                          traffic_watch=now - self.traffic_seen < self.c['missions'].get('traffic_watch_hold', 1.5),
                          traffic_red_area=self.traffic_red_area, line_kind=self.line_kind,
                          crosswalk_distance=lane.get('crosswalk_distance'),
                          dash_end=lane.get('dash_end'),
                          fork_follow_travel=self.fork.travel,
                          lane_change_direction=self.lane_change.direction if self.lane_change.active else 0,
                          fork_remaining=self.fork_remaining, fork_request=self.fork_request, fork_goal=self.fork_goal,
                          fork_sign=self.fork_sign, fork_sign_stamp=self.fork_sign_stamp,
                          fork_sign_counts={n: self.sign_counts.get(n, 0) for n in ('left', 'right')},
                          avoiding=self.avoiding, cone_active=self.cones.active,
                          cone_heading=self.cones.heading, cone_center=self.cones.center, speed=speed,
                          corner_hold=self.lane_driver.hold is not None)
        return float(speed), clip(steering, -self.robot['max_steering'], self.robot['max_steering']), mode if self.state == 'lane' else self.state
