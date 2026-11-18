"""미션 선택·최종 속도/조향 결정"""
# 출처: hello-osy/2026-orda-mentoring src/orda_sim/orda_sim/answer.py (613c9b4)

import math

from .common import clip
from .cones import ConeDriving
from .dash_lane import DashLane
from .fork import ForkMission
from .lane_change import BackUp, LaneChange
from .lane_driving import LaneDriving
from .lidar import body_points, clear_steering
from .parking import Parking
from .section import SectionManager, START_SECTION
from .slow_zone import SlowZone

# 정지 미션 시간 초과 후 같은 표지판에 바로 다시 반응하지 않도록 쿨다운을 거는 표지판
# SWB 재출발 때 구간 번호 → start_stage (driver.restart). S4는 신호등을 끝냈으면 parking
RESTART_STAGE = ['slow', 'pedestrian', 'lane_change', 'cones', 'traffic']
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
        self.backup = BackUp(self.c, self.robot)      # 3차선: 앞이 막혀 선 뒤 후진 재시도
        self.dash = DashLane(self.c, self.robot)      # 3차선 점선 추종 (3lane_chu_js 시제품, dash_lane.py)
        self.was_blocked = False
        self.parking = Parking(config)
        self.cones = ConeDriving(self.c, self.robot)
        self.pid = self.lane_driver.pid
        self.state, self.state_started = 'lane', 0.0
        self.last_time = self.last_lane_stamp = self.last_detection_stamp = None
        self.stopped_since = self.line_distance = None
        self.line_seen = -1e9
        self.slow_zone = SlowZone(self.c['slow_zone'])   # 서행·방지턱 (slow_zone.py)
        m = self.c['missions']
        self.fork = ForkMission(self.c['lane_change']['lane_count'], m['fork_distance'], far=self.c['lane']['far'],
                                dash_gap=m.get('fork_dash_gap', .3), entrance_confirm=m.get('fork_entrance_confirm', .3),
                                pass_area=m.get('fork_sign_pass_area', .006),
                                pass_distance=m.get('fork_sign_pass_distance', .15),
                                pass_seconds=m.get('fork_sign_pass_seconds', .5),
                                wrong_lane_distance=m.get('fork_wrong_lane_distance', .4),
                                outer_distance=m.get('fork_outer_distance', .8))
        self.hold_section = 0        # 지금(또는 마지막) 코너 유지가 시작된 구간 번호 (코너 유지 누적 회전 상한용)
        self.signs_fresh = True      # 표지판 검출이 최신인지 (update의 signs_ok)
        self.sync_fork()
        self.traffic_stop_completed = False
        self.cone_exit_left = 0.0    # 라바콘 직후 저속 유지 남은 거리(m)
        self.traffic_box = None      # 지금 따라가는 신호등 박스 (bbox, 면적). 신호등 접근·정지 중에만 유지 (11/14)
        self.traffic_seen = -1e9     # 신호등(색 무관, 작은 박스 포함)을 마지막으로 본 시각 (소민)
        self.traffic_red_area = 0.0  # 이번 프레임 빨강·노랑 박스 최대 면적 비율
        self.line_kind = None        # line_distance가 가리키는 선: 'crosswalk'(횡단보도 바로 앞) / 'plain'
        # 구간 관리자: 미션 사슬 대체. 표지판을 놓쳐도 코너를 돌면 다음 구간 미션이 켜짐 (section.py)
        # 재출발: 심사위원이 차를 미션 시작 지점으로 옮긴 경우 그 미션의 구간부터 시작 (node.py --start-stage)
        self.sections = SectionManager(self.c['sections'], START_SECTION[self.c['missions'].get('start_stage') or 'slow'])
        self.heading_bias = 0.0            # 누적 회전의 치우침 추정(°). observe_heading_bias
        self.heading_bias_samples = 0
        self.done = set()             # 이번 주행에서 끝낸(또는 시간 초과로 포기한) 미션. 같은 구간에서 다시 들어가지 않음
        if self.c['missions'].get('start_stage') == 'parking':
            self.done.add('traffic')  # 주차 지점 재출발: 신호등은 이미 지난 것으로 봄
        self.traffic_done_at = None   # 신호등을 끝낸 시점의 S4 구간 거리(m). parking_auto_start가 사용
        self.stall_time = 0.0         # 의도하지 않은 정지 누적 시간(s)
        self.stall_moved = 0.0        # 정지가 풀린 뒤 주행 거리(m). stall.reset_distance 넘으면 stall_time 초기화
        self.lane_change_off = 0.0    # 차선 변경 대기·막힘 강제 해제 뒤 차선 변경을 끄는 남은 거리(m)
        self.entry_gate = None        # 3차선 진입 흰 선 통과 전 차선 변경 금지 상태 (entry_gate_open)
        self.entry_from_corner = False   # 다른 구간에서 달려와 S2에 들어왔는지 (S2에서 바로 시작한 경우는 제한 없음)
        self.signs_lost = 0.0         # 표지판 검출이 끊긴 누적 시간(s)
        self.cooldowns, self.sign_counts, self.debug = {}, {}, {}
        self.restarts = 0             # SWB 재출발 횟수 (디버그 표시용)
        self.restart_error = None     # 마지막 재출발 때 놓인 방향과 구간 직선 방향의 차(°). 출발은 None
        self.arrow_history = []       # 새 검출 프레임마다 화살표 하나('left'·'right') 또는 None. 다수결 확정용 (arrow_vote)

    @property
    def avoiding(self):
        return self.lane_change.active or self.cones.active or self.dash.active

    @property
    def changing(self):
        """갈림길 판단용 '차선 변경 중': 기존 한 칸 이동 중이거나 점선 추종으로 점선 위(차로 중앙 아님)에 있음
        점선 위에서는 분기 입구·표지판 통과·목적 차로 도착 판단을 미룸 (dash_lane.straddling)"""
        return self.lane_change.active or self.dash.straddling

    @property
    def section_hold(self):
        """구간 전환 보류 조건: 라바콘 통과 중. 콘 배치에 따라 오른쪽으로 크게 돌 수 있어 누적 회전을 코너로 세면 안 됨
        문 사이에서 active가 잠깐 꺼짐 → 첫 진입(started)부터 끝(done)까지로 봄"""
        return self.cones.active or (self.cones.started and not self.cones.done)

    def cone_early(self):
        """라바콘 구간 바로 앞 구간(S2)에서 3번 코너를 early_turn(°) 넘게 돌았는지. loop.py 붉은색 계산도 이 조건 사용"""
        s = self.c['sections']
        early = s.get('cone_early_turn')
        return bool(s['enabled'] and early is not None and 'cones' not in s.get('off', ()) and
                    self.sections.index == START_SECTION['cones']-1 and self.sections.turned >= early)

    def entry_gate_open(self):
        """3차선 진입 흰 선 전에는 새 차선 변경을 시작하지 않음 (11/18 사용자 지시, 실차 redecide_1118_03·04)
        코너를 돌자마자(S2 거리 0.13~0.22 m) 차선 변경이 시작돼, 앞축이 흰 선 0.82~0.88 m 전인데 옆으로 나가 한 차로
        구간에서 이탈함. 차선 변경은 시작 뒤 약 0.4 m를 가야 옆으로 0.15 m(차로 가장자리)를 넘으므로,
        앞축이 선 entry_line_start(m) 전에 왔을 때부터 허용하면 차로를 벗어나는 시점에는 앞바퀴가 선을 지난 뒤임.
        - 흰 선을 따라가는 중(update_stop_line의 line_distance, 앞축 기준): 선까지 entry_line_start 이하면 허용
        - 선을 못 봄(어두운 조명 등): S2 주행 거리 entry_min_distance(m)부터 허용
        - 어느 경우든 entry_max_distance(m)를 넘으면 허용 (선 추적이 틀려도 회피 판단이 영영 막히지 않게)
        진행 중인 이동에는 영향 없음. 끄기: lane_change.entry_gate False"""
        lc = self.c['lane_change']
        if self.sections.index != START_SECTION['lane_change']:
            self.entry_gate = None
            return True
        # S2에서 바로 시작한 주행(start_stage fork·단위 시험)은 코너를 돌아 들어온 것이 아니므로 제한하지 않음
        if not lc.get('entry_gate', False) or not self.entry_from_corner:
            self.entry_gate = None
            return True
        gate = self.entry_gate
        if gate is None:
            gate = self.entry_gate = dict(open=False, open_at=None, line=None)
        if gate['open']:
            return True
        travelled, line = self.sections.distance, self.line_distance
        if line is not None and 0.0 < line <= lc.get('entry_line_range', 1.3):
            # 선까지 남은 거리로 '열릴 지점'을 계속 갱신 (선을 놓쳐도 마지막 추정으로 엶)
            gate['line'], gate['open_at'] = round(float(line), 3), travelled+float(line)-lc.get('entry_line_start', .30)
        open_at = gate['open_at'] if gate['open_at'] is not None else lc.get('entry_min_distance', .25)
        if travelled >= min(open_at, lc.get('entry_max_distance', 1.0)):
            gate['open'] = True
        return gate['open']

    def three_lane_over(self):
        """3차선 구간이 끝났는지 → 차선 변경 판단(회피·정렬·저속 접근) 자체를 끔 (11/15 사용자 지시)
        분기 입구를 지났거나(fork.passed), 표지판·점선을 못 봤어도 S2 주행 거리가 lane_change_max_distance(m) 이상 (None이면 거리 기준 안 씀)
        예전: 표지판을 못 읽거나 점선을 못 본 주행은 3번 코너 15° 전까지 켜져 있어 벽·라바콘을 앞 장애물로 보고
        저속 접근(approaching_blocked_lane)이 조향을 가져감 (main_1115_1904 215.5~217.6 s, fix_1115_1949 204.7~205.5 s)"""
        s = self.c['sections']
        if self.fork.passed:
            return True
        limit = s.get('lane_change_max_distance')
        return bool(s['enabled'] and limit is not None and self.sections.index == START_SECTION['lane_change'] and
                    self.sections.distance >= limit)

    def road_heading(self):
        """3차선 직선 방향 기준 차 방향(rad, + 오른쪽). S2(3차선·갈림길 구간)에서만, 그 밖은 None
        구간 관리자의 누적 회전(IMU, 없으면 odom 보정값). 갈림길 입구를 지난 뒤는 길이 직선이 아니라 안 씀"""
        s = self.c['sections']
        if not s['enabled'] or self.sections.index != START_SECTION['lane_change'] or self.fork.entered:
            return None
        # 누적 회전의 치우침(자이로 누적 오차)을 뺀 값. 치우침은 observe_heading_bias가 직진 구간의 카메라 차선 방향으로 추정
        turned = self.sections.turned-self.heading_bias
        # 직선 방향에서 60° 넘게 벗어난 값은 기준으로 안 씀 (3번 코너 안, 또는 구간 번호와 누적 회전이 안 맞는 상태)
        if abs(turned) > self.c['lane_change'].get('imu_heading_max', 60.0):
            return None
        return math.radians(turned)

    def observe_heading_bias(self, lane, velocity, dt):
        """누적 회전(sections.turned)의 치우침(°) 추정: 직진 중이고 두 선이 다 보일 때 (누적 회전 − 카메라 차선 방향)의 느린 평균
        11/17 early_1117_0015: 노드를 켜고 63 s 정지 뒤 출발 → 자이로 누적으로 3차선에서 +9.6° 치우침 →
        왼쪽 이동은 36° 대신 46°(바깥선 넘음), 두 칸 이동은 45° 대신 37°(0.2 m 모자람). 직진 중 카메라 방향은 믿을 만하고
        짧은 회전은 IMU가 정확 → 느린 성분만 카메라에 맞춤. 구간이 바뀌어도 값은 이어 씀 (녹화: S1 +8.4~10.1°, S2 +9.6°)"""
        c = self.c['lane_change']
        if not (c.get('heading_bias', True) and self.c['sections']['enabled'] and lane.get('valid')):
            return
        colors = lane.get('colors') or {}
        heading = math.degrees(lane.get('heading', 0.0))
        straight = (abs(heading) < c.get('heading_bias_camera', 12.0)
                    and abs(self.debug.get('steering', 0.0) or 0.0) < c.get('heading_bias_steering', 0.08)
                    and abs(velocity) > 0.1 and 'left' in colors and 'right' in colors)
        if not straight or self.lane_driver.hold is not None or self.lane_change.active or self.cones.active:
            return
        if self.sections.distance < c.get('heading_bias_min_distance', 1.0):
            # 구간 전환 직후 = 코너 출구: 곡선이 덜 끝나 차선 방향이 구간 직선 방향과 다름 → 치우침으로 배우면 안 됨
            # (11/17 arrow_1117_0322 38.5~39.4 s: 누적 −23°·카메라 −2~−8°를 1 s 만에 −7.0° 치우침으로 학습)
            return
        difference = self.sections.turned-heading
        if abs(difference) > c.get('heading_bias_max', 25.0):
            return       # 코너 안·구간 번호 어긋남 등: 치우침으로 보지 않음
        self.heading_bias += (difference-self.heading_bias)*min(1.0, dt/c.get('heading_bias_seconds', 2.0))
        self.heading_bias_samples += 1

    def entry_align(self, steering):
        """3차선 진입 정렬: 코너를 덜 돈 채(구간 직선 방향보다 왼쪽) 들어왔으면 오른쪽 조향으로 마저 돎. 반환 (조향, 적용 여부)
        11/17 arrow_1117_0322 38.5~43 s: 코너 유지가 54.7°에서 풀림 → 3차선 왼쪽 차로의 선(왼 노랑·오른 흰)을 잡은 차선 주행이
        왼쪽으로 조향(+0.06~+0.11) → 구간 방향 대비 −22 → −30°로 벌어져 왼쪽 차로 장애물 앞까지 대각선으로 감.
        3차선은 직선이고 방향은 IMU 누적 회전으로 알므로, 진입 뒤 entry_align_distance(m) 안에서 entry_align_angle(°) 넘게
        왼쪽으로 틀어져 있으면 차선 주행 조향 대신 오른쪽 조향(entry_align_gain × 틀어짐, 최대 조향까지). 오른쪽으로 틀어진 경우와
        3번 코너 쪽은 건드리지 않음. 차선 변경·라바콘·코너 유지 중에는 호출 안 함"""
        lc = self.c['lane_change']
        heading = self.road_heading()          # rad, + 오른쪽. S2 밖·갈림길 입구 뒤는 None
        if (not lc.get('entry_align', False) or heading is None
                or self.sections.distance > lc.get('entry_align_distance', 1.5)
                or heading > -math.radians(lc.get('entry_align_angle', 12.0))):
            return steering, False
        want = max(-self.robot['max_steering'], lc.get('entry_align_gain', 1.5)*heading)
        return min(steering, want), True

    def fork_lanes(self):
        """지금 차로에서 표지판 목적 차로까지 칸 수(부호: + 왼쪽). 방향 확정 뒤·입구 전에만, 그 밖은 None
        진행 중인 이동을 두 칸으로 넓히거나(lane_change.try_extend) 목적 차로 쪽으로 되돌릴지(try_reverse) 판단용
        0 = 목적 차로가 이동을 시작한 차로 (모름과 구분하려고 None을 따로 둠)"""
        f = self.fork
        destination = getattr(f, 'destination', None)
        if not f.running or destination is None or f.entered or f.phase not in ('locked', 'align'):
            return None
        return int(destination)-self.lane_change.index

    def request_lanes(self, requested=None):
        """표지판 목적 차로까지 칸 수 (요청 이동이 두 칸을 한 번에 건널지 판단용)"""
        destination = getattr(self.fork, 'destination', None)
        if not (self.fork_request if requested is None else requested) or destination is None:
            return 1
        return abs(int(destination)-self.lane_change.index)

    def request_limit(self):
        """요청 이동을 끝내야 하는 남은 거리(m). 모르면 None. 정확한 것부터 하나만 씀
        1) 섬 표지판 기둥이 라이다에 잡히면 그 거리 + anchor_limit_offset (뒷축이 기둥 옆에 올 때까지 목적 차로 쪽에 들어가 있어야 함)
        2) 점선 끝이 화면 안에 들어왔으면(fork.late) fork.dash_left. 그 전의 dash_left는 '보이는 가장 먼 점선'이라 입구 거리가 아님
        3) S2 주행 거리 기준 lane_change.request_entrance_distance(실측 고정값) − 지금 거리
        (11/16: 예전에는 셋 중 최솟값 → 고정값이 틀리면 한계가 0이 돼 출발 지연이 막힘)"""
        lc = self.c['lane_change']
        anchor = getattr(self.lane_change, 'anchor', None)
        if anchor is not None and lc.get('anchor_enabled', True):
            return max(0.0, float(anchor)+lc.get('anchor_limit_offset', 0.0))
        if self.fork.late:
            return max(0.0, float(self.fork.dash_left))
        entrance = lc.get('request_entrance_distance')
        if entrance is not None and self.sections.index == START_SECTION['lane_change']:
            return max(0.0, float(entrance)-self.sections.distance)
        return None

    def dash_exit_now(self):
        """점선 추종: 분기 입구 앞이라 목적 바깥 차로로 반 칸 옮길 때인지 (dash_lane.exit_distance)
        - 확정 표지판을 크게 봤음(fork.sign_near, 표지판은 섬 앞머리) 또는 점선 끝이 화면 안(fork.late)
        - S2 주행 거리 기준 남은 입구 거리(lane_change.request_entrance_distance − 지금 거리)가 exit_distance 이하
        섬 기둥 라이다 거리(lane_change.anchor)는 점선 추종 중 갱신이 안 돼 안 씀"""
        f = self.fork
        if f.entered:
            return False
        if f.late or (f.running and f.direction and f.sign_near):
            return True
        entrance = self.c['lane_change'].get('request_entrance_distance')
        if not self.c['dash_lane'].get('exit_by_distance', True):
            return False
        if entrance is None or not self.c['sections']['enabled'] or self.sections.index != START_SECTION['lane_change']:
            return False
        return float(entrance)-self.sections.distance <= self.c['dash_lane'].get('exit_distance', 1.2)

    def dash_in_corner(self):
        """점선 추종용 코너 판단: 코너 유지 중이거나 S2에서 오른쪽으로 dash_lane.corner_turn(°) 넘게 돎
        기존 corner3(15°)를 그대로 쓰면 장애물을 비켜 가며 잠깐 15° 넘게 틀어질 때 점선 추종이 꺼짐 (합성 시험 normal)"""
        s = self.c['sections']
        turned = s['enabled'] and self.sections.index == START_SECTION['lane_change'] and \
            self.sections.turned >= self.c['dash_lane'].get('corner_turn', 30.0)
        return self.lane_driver.hold is not None or bool(turned)

    def corner3(self):
        """S2에서 오른쪽으로 lane_change_max_turn(°) 넘게 돈 상태 = 3번 코너 안 (새 차선 변경 금지, 한 선 이름 코너 규칙)"""
        s = self.c['sections']
        return bool(s['enabled'] and self.sections.index == START_SECTION['lane_change'] and
                    self.sections.turned >= s.get('lane_change_max_turn', 15.0))

    def transition(self, state, now):
        self.state, self.state_started, self.stopped_since = state, now, None
        if state!='parking_scan' or not self.c['parking']['cruise_search_enabled']:
            self.pid.reset()
        if state != 'lane':
            self.lane_change.active = self.cones.active = False
            self.dash.reset()
        if state == 'parking_scan':
            self.parking.start(now)
            self.state=self.parking.state

    def manual(self, now, angular_velocity=0.0, yaw_rate=None):
        """조종기 수동(SWB 아래)인 주기: 명령이 차에 안 먹으므로 판단은 멈추고 회전만 적분
        차를 손으로 들어 옮겨도 IMU는 회전을 잼 → 자동으로 돌아올 때 놓인 방향을 알 수 있음 (restart)"""
        dt = .05 if self.last_time is None else clip(now-self.last_time, .001, .2)
        self.last_time = now
        self.sections.carry(angular_velocity*self.c['sections']['odom_yaw_scale'] if yaw_rate is None else yaw_rate, dt)

    def restart(self, now, relocate=True):
        """수동 → 자동 전환: 드라이버를 새로 만들어 미션 상태를 전부 처음으로 되돌림 (노드 재시작 + start_stage와 같은 상태)
        relocate True(재출발): 놓인 방향으로 구간을 다시 정하고 그 구간의 start_stage로 시작 (구간 거리 0)
        - S4는 미션이 둘: 신호등을 이미 끝냈으면(통과·포기) 주차 지점, 아니면 신호등 지점 재출발로 봄
        relocate False(노드를 켠 뒤 첫 출발): start_stage 그대로, 출발 전 차를 들고 돌린 회전은 버림"""
        restarts, error = self.restarts, None
        if relocate:
            index, error = self.sections.relocate()
            stage = RESTART_STAGE[index]
            if stage == 'traffic' and index == self.sections.index and 'traffic' in self.done:
                stage = 'parking'
            self.c['missions']['start_stage'] = stage
            restarts += 1
        self.parking.close()
        self.__init__({'answer': self.c, 'robot': self.robot})
        self.last_time, self.restarts, self.restart_error = now, restarts, error

    def sensor_failure(self, now):
        self.stopped_since = self.parking.stable_since = None
        if self.state in ('parking_cruise', 'parking_scan', 'parking_approach', 'parking_turn'):
            self.parking.pause(now)

    def leave_mission(self, mission, now):
        """정지 미션 시간 초과: 영구 정지(fault) 대신 차선 주행으로 돌아간다.

        그 미션은 포기(done)로 표시해 같은 구간에서 다시 들어가지 않음. 다음 미션은 구간이 바뀌면 켜짐.
        재출발은 launch의 start_stage로 한다.
        """
        for name in MISSION_SIGNS.get(mission, []):
            self.cooldowns[name] = now + self.c['missions']['sign_cooldown']
        if mission == 'traffic' and 'traffic' not in self.done:
            self.traffic_done_at = self.sections.distance
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
        lane = change.index + (change.direction*getattr(change, 'span', 1) if change.active else 0)
        last = change.c['lane_count'] - 1
        return 'left' if lane == last else 'right' if lane == 0 else None

    def cone_left_limit(self):
        """라바콘 통과 중·직후 cone_lane_left_distance(m) 안이면 한 선 이름 판단 경계(m), 아니면 None (유경)

        lane_vision.single_name이 corner_left_limit 자리에서 씀: 선의 가까운 끝이 화면 중앙에서 오른쪽으로
        이 값 안이면 바깥(왼쪽) 선으로 봄. 라바콘 직후에는 차가 바깥 선 가까이·비스듬히 있어 위치만으로는
        왼쪽 선을 오른쪽 선으로 오인함. 기억보다 먼저 적용되므로 틀린 이름이 이어지는 고착도 같이 끊김
        """
        cones = self.cones
        if not cones.started:
            return None
        # S4(4번 코너 뒤 직선)에서는 안 씀: 코너 안은 코너 유지 규칙(corner_left_limit)이 맡음 (11/16)
        if self.c['sections']['enabled'] and self.sections.index >= START_SECTION['traffic']:
            return None
        if not (cones.active or cones.since_exit <= self.c['lidar']['cone_lane_left_distance']):
            return None
        return self.c['lidar']['cone_lane_left_limit']

    def allowed(self, mission):
        """지금 구간의 미션이고 아직 끝내지 않았을 때만"""
        return self.sections.allows(mission) and mission not in self.done

    def sync_fork(self):
        """기존 디버그 필드 호환. 판단의 원본은 ForkMission 한 곳"""
        f = self.fork
        self.fork_bias = 0.0
        # 갈림길 목적 끝 차로에 들어간 뒤(follow·exit)에는 바깥 노란 선 하나만 따라감 (소민 11/15 실차: 차선 따라가다
        # 교통섬 노란 선을 반대쪽 차선으로 잡아 교통섬 쪽으로 꺾어 들어감). fork_bias 부호 = 남길 쪽 (+ 왼쪽).
        # 크기는 0.002 m라 기준 중앙 이동은 무시할 정도 (lane_vision.find_starts: 반대쪽 시작점만 뺌)
        if (self.c['missions'].get('fork_outer_only', True) and f.outer_only and f.direction
                and not self.lane_change.active and self.lane_change.index == f.destination):
            self.fork_bias = 0.002 if f.direction > 0 else -0.002
        self.fork_remaining = f.remaining
        self.fork_target = f.direction*self.c['missions']['fork_bias'] if f.running else 0.0
        self.fork_goal = f.direction if f.running else 0
        self.fork_request = f.request(self.lane_change.index, self.lane_change.active)
        self.fork_pending = f.phase == 'locked'
        self.fork_sign, self.fork_sign_stamp = f.sign, f.stamp

    def fork_direction(self):
        """선택 출구 추종 상태에서만 영상 보정 허용"""
        return self.fork.vision_direction if not self.lane_change.active else 0

    def parking_ready(self):
        """주차 표지판을 받아도 되는 시점인지. 표지판 최소 면적이 작아(0.002) 신호등보다 먼저 확정될 수 있음
        → 신호등을 끝냈거나(포기 포함) 꺼져 있을 때, 또는 신호등을 못 본 경우에 대비해 S4에서 일정 거리 간 뒤에만 허용"""
        if self.c['parking'].get('test_wait_for_sign', False):
            return True
        if 'traffic' in self.done or not self.sections.allows('traffic'):
            return True
        return self.sections.distance >= self.c['parking'].get('after_traffic_distance', 4.0)

    def advance(self, mission):
        if mission == 'traffic' and 'traffic' not in self.done:
            self.traffic_done_at = self.sections.distance
        self.done.add(mission)

    def parking_auto_start(self):
        """표지판 없이 주차 단계를 켤 시점인지 (11/18 경기장: P 표지판을 right로 읽어 주차가 시작되지 않음, params parking.auto_start)
        시작선 기준 틀은 주차 단계가 시작선 전에 켜져 있어야 하므로 횡단보도를 지난 뒤·시작선 전 거리에서 켬"""
        p = self.c['parking']
        if (not p.get('auto_start', False) or p.get('test_wait_for_sign', False) or not self.c['sections']['enabled']
                or self.sections.index != START_SECTION['parking']):
            return False
        stage = self.c['missions'].get('start_stage')
        if stage == 'parking':
            return self.sections.distance >= p.get('auto_start_placed', .2)
        if stage == 'traffic':
            # 신호등 지점에 놓고 시작: 구간 거리가 트랙 기준이 아님 → 신호등을 끝낸 지점 기준
            return (self.traffic_done_at is not None and
                    self.sections.distance-self.traffic_done_at >= p.get('auto_start_after_traffic', .9))
        return self.sections.distance >= p.get('auto_start_distance', 3.0)

    def traffic_filter(self, detections):
        """신호등 박스 중 지금 보는 신호등 것만 남김 (11/14 integ_1114_lap3: 녹색 0.042 + 배경 빨간 상자 0.003 동시 검출 → 54초 대기)

        접근·정지 중: 따라가던 박스와 겹치거나(IoU traffic_box_iou 이상) 면적이 그 traffic_main_ratio배 이상인 박스만 사용,
        남은 것 중 가장 큰 박스를 새 기준으로 삼음. 겹치는 박스가 없는 프레임은 기준 유지
        그 외(차선 주행): 가장 큰 신호등 박스의 traffic_main_ratio배보다 작은 신호등 박스는 버림 (먼 배경의 붉은 물체)
        신호등이 아닌 표지판은 그대로 통과"""
        m = self.c['missions']
        names = MISSION_SIGNS['traffic']
        lights = [d for d in detections if d['name'] in names and d.get('bbox')]
        if not self.state.startswith('traffic'):
            self.traffic_box = None
        if not lights:
            return detections
        ratio = m.get('traffic_main_ratio', .5)
        if self.traffic_box is None:
            largest = max(d['area'] for d in lights)
            keep = [d for d in lights if d['area'] >= ratio*largest]
        else:
            box, area = self.traffic_box
            def overlap(b):
                w = min(b[2], box[2])-max(b[0], box[0])
                h = min(b[3], box[3])-max(b[1], box[1])
                if w <= 0 or h <= 0:
                    return 0.0
                union = (b[2]-b[0])*(b[3]-b[1])+(box[2]-box[0])*(box[3]-box[1])-w*h
                return w*h/union if union > 0 else 0.0
            keep = [d for d in lights if overlap(d['bbox']) >= m.get('traffic_box_iou', .2) or d['area'] >= ratio*area]
        if self.state.startswith('traffic') and keep:
            main = max(keep, key=lambda d: d['area'])
            self.traffic_box = (list(main['bbox']), main['area'])
        kept = {id(d) for d in keep}
        return [d for d in detections if d['name'] not in names or id(d) in kept]

    def arrow_off_axis(self):
        """S2 직선 방향에서 fork_sign_max_turn(°) 넘게 틀어짐 = 교통섬 표지판이 화면 밖일 수 있는 자세"""
        s = self.c['sections']
        return bool(s['enabled'] and self.sections.index == START_SECTION['fork'] and
                    abs(self.sections.turned) > self.c['missions'].get('fork_sign_max_turn', 20.0))

    def arrow_on_island(self, box):
        """틀어진 자세에서 본 화살표 박스가 교통섬 표지판 방향에 있는지 (11/17 bend_1117_0311: 넷째 이동 내내 45° 틀어진 채
        left를 71.9 s부터 신뢰도 0.85~0.98로 봤는데 틀어짐 규칙으로 전부 버림 → 79.7 s(0.67 m 뒤) 확정, 두 칸 넓히기 시점을 놓침)
        박스 중심의 도로 기준 방위(° + 왼쪽) = 화면 방위 − 차 방향. 표지판은 가운데 차로(섬) 위 → 차가 섬보다 오른쪽이면
        왼쪽(−5~+40°), 왼쪽이면 오른쪽(−40~+5°), 가운데 근처면 ±15° 안. 11/15 천막(차 +34°·도로 방위 −25.6°·표지판 예상 왼쪽)은 탈락
        지난 녹화 19개에 적용: 확정 방향이 바뀐 것 없음, 확정이 최대 1.7 m 앞당겨짐"""
        m = self.c['missions']
        if not m.get('fork_sign_bearing', True) or not box or box[2] <= box[0]:
            return False
        half = math.radians(m.get('fork_sign_fov', 71.0)/2)
        camera = -math.degrees(math.atan(((box[0]+box[2])/2-.5)*2*math.tan(half)))
        bearing = camera-(self.sections.turned-self.heading_bias)
        change = self.lane_change
        width = self.c['lane_change']['lane_width']
        # 표지판(가운데 차로 중앙)이 차 기준 왼쪽 몇 m인지: 차로 번호 + 진행 중인 이동의 횡이동
        island = (self.c['lane_change']['lane_count']//2-change.index)*width-(change.offset if change.active else 0.0)
        wide, narrow, back = m.get('fork_sign_bearing_wide', 40.0), m.get('fork_sign_bearing_narrow', 15.0), 5.0
        if island > .15:
            return -back <= bearing <= wide
        if island < -.15:
            return -wide <= bearing <= back
        return abs(bearing) <= narrow

    def arrow_filter(self, detections, any_axis=False):
        """갈림길 화살표 박스 선별 (11/15 fix_1115_1949 168.3 s: 장애물 회피로 오른쪽 34° 틀어진 동안 트랙 밖 파란 천막을
        right로 3프레임 검출 → 방향 확정, 뒤의 진짜 left 51회는 무시됨)
        - 박스 가로:세로(화소)가 fork_sign_aspect 밖이면 버림 (천막 2.45, 진짜 표지판 0.87~1.11)
        - S2 직선 방향에서 fork_sign_max_turn(°) 넘게 틀어져 있으면 버림 (표지판은 교통섬 = 도로 방향 앞에 있음)"""
        m = self.c['missions']
        low, high = m.get('fork_sign_aspect', (.6, 1.6))
        off_axis = self.arrow_off_axis() and not any_axis   # any_axis: 틀어진 자세 박스도 남김 (확정 뒤 '크게 봤음' 기록용)
        kept = []
        for d in detections:
            if d['name'] in ('left', 'right'):
                box = d.get('bbox')
                if off_axis and not self.arrow_on_island(box):
                    continue
                if box and box[3] > box[1]:
                    aspect = (box[2]-box[0])*m.get('fork_sign_image_aspect', 4/3)/(box[3]-box[1])
                    if not low <= aspect <= high:
                        continue
            kept.append(d)
        return kept

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
        if stamp != self.last_detection_stamp:
            arrows = names & {'left', 'right'}
            self.arrow_history = (self.arrow_history+[next(iter(arrows)) if arrows else None])[-20:]
            self.sign_counts = {name: self.sign_counts.get(name, 0) + 1 for name in names}
            if {'traffic_red', 'traffic_yellow'} & names:
                self.sign_counts['traffic_green'] = 0   # 빨강·노랑과 겹친 녹색은 버리고 새 프레임부터 다시 셈 (11/14)
            self.last_detection_stamp = stamp
        confirmed = {name for name in names
                     if self.sign_counts.get(name, 0) >= self.c['detector'].get('confirm_frames_by_class', {}).get(
                         name, self.c['detector']['confirm_frames'])}
        return confirmed | self.arrow_vote(names)

    def arrow_vote(self, names):
        """화살표 다수결 확정 (11/18 소민, 녹화 재현만·실차 전). 연속 확정(confirm_frames_by_class)에 더해 OR로 씀
        최근 window 검출 프레임 중 같은 화살표 count번 이상 + 반대 화살표 opposite_max번 이하 + 이번 프레임도 그 화살표면 확정
        11/18 redecide_1118_19: left·right가 번갈아 나와 연속 3이 계속 끊김 → 첫 화살표 뒤 3.9 s에 확정. 6중 4면 1.8 s 앞당겨짐
        다수결만 쓰면 위험: redecide_1118_20은 처음 프레임이 r l r r(실제 left) → 5중 3은 right로 잘못 확정.
        오늘 녹화 4개(05·17·19·20)에서 '연속 3 또는 6중 4·반대 1 이하'는 틀린 확정 0, 늦어진 확정 0. 표본이 적어 실차 확인 필요"""
        vote = self.c['detector'].get('arrow_vote', {})
        if not vote.get('enabled', False) or not self.arrow_history:
            return set()
        window = self.arrow_history[-int(vote.get('window', 6)):]
        latest = window[-1]
        if latest not in names:
            return set()
        other = 'right' if latest == 'left' else 'left'
        if window.count(latest) >= vote.get('count', 4) and window.count(other) <= vote.get('opposite_max', 1):
            return {latest}
        return set()

    def traffic_line_too_early(self, distance):
        """신호등 정지선으로 쓰기에는 S4에서 너무 앞에 있는 선인지 (첫 선·'천천히' 글씨 오판 방지, params traffic_line_min_distance)"""
        gate = self.c['missions'].get('traffic_line_min_distance')
        if (gate is None or distance is None or not self.c['sections']['enabled']
                or self.sections.index != START_SECTION['traffic']
                or self.c['missions'].get('start_stage') in ('traffic', 'parking')):
            return False
        return self.sections.distance+distance < gate

    def update_stop_line(self, lane, velocity, now, dt):
        m = self.c['missions']
        if self.line_distance is not None:
            self.line_distance -= velocity * dt
        image_stamp = lane.get('stamp', now)
        new_image = image_stamp != self.last_lane_stamp
        self.last_lane_stamp = image_stamp
        if new_image and lane['stop_distance'] is not None:
            # 횡단보도 앞 선을 못 찾을 때 쓰는 '정지선 너머 흰 선' 기록 (원본 거리, 찍힌 시각)
            self.plain_seen = (lane['stop_distance'] - velocity*max(0.0, now-image_stamp), now)
        cross = lane.get('crosswalk_distance')
        if cross is not None and self.allowed('traffic') and self.traffic_line_too_early(cross):
            cross = None   # S4 앞부분의 횡단보도 선 판정은 믿지 않음 → 일반 선으로 취급
        if new_image and (cross is not None or lane['stop_distance'] is not None):
            # 횡단보도 바로 앞 선이 보이면 그 선 우선. 한 번 잡으면 일반 흰 선(서행 시작선·횡단보도 뒤 선)으로 안 바꿈 (소민)
            kind = 'crosswalk' if cross is not None else 'plain'
            # A delayed image measures the stripe at exposure, not at this tick.
            candidate = (lane['stop_distance'] if cross is None else cross) - velocity * max(0.0, now-image_stamp)
            tracking_stop = self.state.endswith('_approach') or self.state.endswith('_hold')
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
                self.line_measured = candidate   # 카메라로 직접 잰 마지막 거리 (오도메트리 추정 아님)
        if (new_image and self.state.startswith('traffic') and self.line_kind == 'crosswalk' and
                self.line_distance is not None):
            # 신호등: 따라가던 횡단보도 앞 선을 가까이서 계속 다시 잼 (소민 11/14). 가까워지면 횡단보도 판정·두께 조건에서
            # 빠져 0.9 m를 오도메트리로만 가던 문제 → 같은 위치(line_match_tolerance 안) 후보면 종류·두께 상관없이 거리 갱신.
            # 종류는 crosswalk 유지. 보행자 상태에는 적용 안 함
            lag = velocity * max(0.0, now-image_stamp)
            near = [cand['distance'] - lag for cand in lane.get('stop_candidates', []) if cand.get('complete', True)]
            near = [d for d in near if abs(d - self.line_distance) < m['line_match_tolerance']]
            if near:
                self.line_distance = min(near, key=lambda d: abs(d - self.line_distance))
                self.line_seen = image_stamp
                self.line_measured = self.line_distance
        if (now - self.line_seen > m['stop_line_memory'] and
                not self.state.endswith(('_hold', '_approach'))):
            self.line_distance = None
        if self.state == 'lane' and self.line_distance is not None and self.line_distance < -0.05:
            self.line_distance = None
        if self.line_distance is None:
            self.line_kind = None
            self.line_measured = None

    def update_signs(self, scan, detections, detection_stamp, velocity, now, dt):
        c, m = self.c, self.c['missions']
        raw = self.traffic_filter(detections)
        detections = self.arrow_filter(raw)
        names = self.confirmed(detections, detection_stamp)
        if c['parking'].get('test_wait_for_sign',False):names&={'parking'}
        ready = {name for name in names if now >= self.cooldowns.get(name, 0)}
        # 신호등 감시: 확정 전·작은 박스라도 신호등이 보이면 시각 기록 (S4 감속용), 빨강·노랑 최대 면적
        traffic = [d for d in detections if d['name'] in MISSION_SIGNS['traffic'] and
                   d['area'] >= m.get('traffic_watch_area', .002)]
        if traffic:
            self.traffic_seen = now
        self.traffic_red_area = max([d['area'] for d in traffic if d['name'] != 'traffic_green'] or [0.0])
        green_only = 'traffic_green' in names and not ({'traffic_red', 'traffic_yellow'} & names)
        # 정지 중에는 표지판 수용 안 함: 출발선에서 표지판이 보이면 출발 전부터 서행 상태가 됨 (11/14 lap3, 감속이 눈에 보여야 함)
        if 'slow' in ready and self.allowed('slow') and abs(velocity) >= m.get('slow_min_speed', .05):
            # 감속·해제는 slow_zone이 판단 (예전: 표지판 뒤 14초 고정). 끝나면 update에서 advance
            self.slow_zone.sign = True
        self.fork.step(self.lane_change.index, self.changing, velocity, dt,
                       left_section=self.c['sections']['enabled'] and self.sections.index > START_SECTION['fork'])
        if self.allowed('fork') and self.fork.lock(ready, detection_stamp):
            self.advance('fork')
            self.cooldowns['left'] = self.cooldowns['right'] = now + m['sign_cooldown']
        elif self.fork.running:
            # 방향 교체: 반대 화살표가 fork_relock_frames 연속이면 목적지 변경 (대기 시간 cooldown과 무관하게 확인)
            again = {n for n in ('left', 'right') if self.sign_counts.get(n, 0) >= m.get('fork_relock_frames', 5)}
            self.fork.relock(again, detection_stamp)
        # 표지판 통과 = 분기 입구 (fork.see_sign). 화살표가 이번 검출에 없으면 None
        arrows = [d['area'] for d in detections if d['name'] in ('left', 'right')]
        self.fork.see_sign(max(arrows) if arrows else None, self.lane_change.index, self.changing,
                           velocity, dt, fresh=self.signs_fresh and not self.arrow_off_axis())
        # 유경 11/16 run_1116_2113: 두 칸을 45°로 건너는 동안 표지판이 커졌는데(면적 0.006~0.0115) 틀어진 자세(20° 초과)라
        # 박스가 버려져 '크게 봤음'이 한 번도 기록 안 됨 → 표지판 통과(입구) 판정 없음 → follow가 끝나지 않아 라바콘 모드가 안 켜짐
        # → 방향 확정 뒤에는 확정된 화살표를 틀어진 자세에서 봐도 '크게 봤음'만 기록 (방향 확정·교체에는 여전히 안 씀)
        if self.fork.running and self.fork.sign:
            near = [d['area'] for d in self.arrow_filter(raw, any_axis=True) if d['name'] == self.fork.sign]
            self.fork.note_near(max(near) if near else None)
        # 안전장치: 목적 차로를 따라가는데(follow) 입구 판정이 끝내 안 나면 S2 주행 거리 fork_enter_distance(m)에서 입구로 처리
        enter_at = m.get('fork_enter_distance')
        if (enter_at is not None and self.fork.phase == 'follow' and not self.changing
                and c['sections']['enabled'] and self.sections.index == START_SECTION['fork']
                and self.sections.distance >= enter_at):
            self.fork.enter(self.lane_change.index)
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
                # 횡단보도 앞 선에서만 접근 시작: 표지판이 횡단보도 뒷선 쪽에 서 있으면 확정이 늦어 뒷선에 서던 문제 (11/17 venue_1117_01)
                # 줄무늬 판정이 안 될 때 대비: 표지판이 크게(pedestrian_near_area 이상) 보이면 예전처럼 가까운 선에도 접근
                near_sign = max([d['area'] for d in detections if d['name'] == 'pedestrian'] or [0.0]) >= m.get('pedestrian_near_area', .012)
                front_line = (not m.get('pedestrian_crosswalk_only', True)) or self.line_kind == 'crosswalk' or near_sign
                if (fresh_line and front_line) or not m.get('pedestrian_require_line', True):
                    self.transition('pedestrian_approach', now)
            elif 'parking' in ready and c['parking']['enabled'] and self.allowed('parking') and self.parking_ready():
                self.transition('parking_scan', now)
            elif c['parking']['enabled'] and self.allowed('parking') and self.parking_auto_start():
                self.transition('parking_scan', now)
        return green_only

    def stop_missions(self, speed, velocity, green_only, now):
        m = self.c['missions']
        if self.state == 'lane':
            self.far_mode = False
        if self.state.endswith('_approach'):
            # 신호등: 횡단보도 바로 앞 선에만 정지 (서행 시작 흰 선에 서지 않음). 선 못 찾으면 저속 접근·큰 빨강 예비 정지
            traffic = self.state == 'traffic_approach'
            line = self.line_distance
            if traffic and m.get('traffic_crosswalk_only', True) and self.line_kind != 'crosswalk':
                line = None
            if traffic and self.traffic_line_too_early(line):
                line = None   # 따라가던 선이 S4 앞부분(첫 선·글씨)에 있음 → 정지 대상 아님 (구간이 바뀌기 전에 잡힌 선 포함)
            offset = m.get('traffic_stop_offset', m['stop_offset']) if traffic else m['stop_offset']
            # 신호등: 선을 가까이(traffic_near_measure m 안)에서 직접 잰 적이 있으면 보행자와 같은 조건 → traffic_stop_offset_near.
            # 먼 곳에서 한 번만 재고 오도메트리로 온 경우는 그 오차를 보정한 traffic_stop_offset (11/14 실측 −0.045)
            measured = getattr(self, 'line_measured', None)
            if traffic and measured is not None and measured <= m.get('traffic_near_measure', .6):
                offset = m.get('traffic_stop_offset_near', m['stop_offset'])
            # 횡단보도 앞 선을 못 찾았을 때: 신호등 가까이(빨강 면적 traffic_far_red_area 이상)에서 보이는 흰 선은
            # 정지선 너머 선. 맞는 정지 위치에서 이 선이 앞축 0.51~0.52 m 앞에 보임 (소민 11/14 run_1957·2001 두 번 일치)
            far = None
            if traffic and line is None:
                if self.traffic_red_area >= m.get('traffic_far_red_area', .025):
                    self.far_mode = True
                seen = getattr(self, 'plain_seen', None)
                if getattr(self, 'far_mode', False) and seen and now - seen[1] < m.get('trigger_line_freshness', .4):
                    d = seen[0] - velocity*(now-seen[1])
                    if .3 < d < 1.0:
                        far = d
            crossing_on_green = (self.state == 'traffic_approach' and green_only and
                                 (not m.get('traffic_require_stop', True) or self.traffic_stop_completed))
            red_now = self.state == 'traffic_approach' and self.traffic_red_area > 0.0
            # 신호등이 정지선을 따라가는 중이면 시간 제한 없음 (소민 11/14 run_1114_1952: 빨강을 약 2.8 m 앞에서 확정 →
            # 0.13 m/s로 선을 따라가던 중 15초 초과 → 선 0.57 m 앞에서 정지). 시간 제한은 선을 끝내 못 찾을 때만
            if now - self.state_started > m['approach_timeout'] and not (traffic and (line is not None or far is not None)):
                if red_now:
                    # 빨강·노랑이 보이는데 정지선을 못 찾음: 지나가지 않고 그 자리 정지 (신호 위반 방지, 소민)
                    self.transition('traffic_hold', now)
                    return 0.0
                # 정지선에 도달하지 못함: 멈춰 있지 않고 차선 주행으로 복귀 (미션 단계는 유지)
                self.leave_mission(self.state.replace('_approach', ''), now)
                return speed
            if line is None and far is None and red_now and self.traffic_red_area >= m.get('traffic_stop_area', .04):
                # 정지선 없이 신호등이 충분히 가까움(박스가 큼): 예비 정지
                self.transition('traffic_hold', now)
                return 0.0
            if line is None and far is None:
                # 정지선이 아직 안 보이면 멈추지 않고 저속으로 다가가며 찾는다.
                # (멈춰 있으면 정지선이 계속 안 보여 결국 시간 초과 실패)
                # 신호등: 빨강 박스가 아직 작으면(멀면) traffic_search_speed (11/16 main_1116_1614: 선 1.65 m 앞부터 0.13 m/s 13 s)
                near = not traffic or self.traffic_red_area >= m.get('traffic_far_red_area', .025)
                speed = min(speed, m['stop_approach_speed'] if near else m.get('traffic_search_speed', m['stop_approach_speed']))
            elif crossing_on_green:
                # Seeing green far away is not mission completion. Keep watching
                # red/yellow until the measured front axle crosses this stripe.
                if (line is not None and line < -m['stop_tolerance']) or (line is None and far is not None):
                    for name in ['traffic_red', 'traffic_yellow', 'traffic_green']:
                        self.cooldowns[name] = now + m['sign_cooldown']
                    self.advance('traffic')
                    self.line_distance = None
                    self.transition('lane', now)
            else:
                remaining = line - offset if line is not None else far - m.get('traffic_far_line_stop', .49)
                # 마지막 구간은 stop_min_speed 밑으로 안 내림: 허용 오차가 작으면 비례 속도가 너무 느려 선에 못 닿음
                # 선까지 stop_slow_distance 넘게 남았으면 stop_far_speed, 그 안은 stop_approach_speed
                cap = m['stop_approach_speed'] if remaining <= m.get('stop_slow_distance', 0.0) else m.get('stop_far_speed', m['stop_approach_speed'])
                speed = min(speed, max(cap, m['stop_approach_speed']),
                            max(m.get('stop_min_speed', 0.0), remaining * m['stop_gain']))
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
            # 녹색이 traffic_green_hold(s) 동안 끊김 없이 보여야 출발. 빨강·노랑이 한 번이라도 보이면 처음부터 다시 셈
            # (소민 11/14 run_1957·2001: 빨간불 대기 중 녹색 오검출 1~2프레임에 출발 → 신호 위반·찔끔 이동)
            if green_only and self.traffic_red_area == 0.0:
                if getattr(self, 'green_since', None) is None:
                    self.green_since = now
            else:
                self.green_since = None
            green_ok = (getattr(self, 'green_since', None) is not None and
                        now - self.green_since >= m.get('traffic_green_hold', 1.0))
            if green_ok and self.traffic_stop_completed:
                self.green_since = None
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

    def main_lane_command(self,lane,now,dt,velocity,angular_velocity,allow_hold,avoiding=False):
        """일반 주행과 주차 검색이 공유하는 기존 main 차선 제어"""
        return self.lane_driver.update(lane,now,dt,False,velocity,angular_velocity,
                                       allow_hold=allow_hold,avoiding=avoiding)

    def update(self, lane, scan, detections, detection_stamp, velocity, now, angular_velocity=0.0,
               yaw_rate=None, signs_ok=True, parking_input=None, tilt_rate=None):
        """yaw_rate: 실측 회전 속도(IMU, rad/s). None이면 odom 값 × sections.odom_yaw_scale
        signs_ok: 표지판 검출이 최신인지 (False면 detections는 빈 목록으로 들어옴)
        tilt_rate: IMU 자이로 (x, y) rad/s, 방지턱 감지용. IMU가 없으면 None"""
        if self.last_time is not None and now < self.last_time:
            if self.state.startswith('parking_') or self.state=='parked':
                speed,steering,self.state=self.parking.fail('CLOCK_RESET_DURING_PARKING')
                self.last_time=now
                self.debug=dict(parking=self.parking.debug,speed=0.,steering=0.)
                return 0.,0.,'parking_failed: CLOCK_RESET_DURING_PARKING'
            self.parking.close()
            self.__init__({'answer': self.c, 'robot': self.robot})
        dt = .05 if self.last_time is None else clip(now-self.last_time, .001, .2)
        self.last_time = now
        # 구간·차선 변경·라바콘 방향 추적은 실측 회전(IMU). odom 회전은 조향 명령으로 계산한 값 (실측 = 0.93 × odom, 11/9)
        # 코너 유지(lane_driving)는 odom 그대로: angle 65°가 odom 기준으로 맞춰져 있음 (11/10 판단, IMU로는 58°)
        turn_rate = angular_velocity*self.c['sections']['odom_yaw_scale'] if yaw_rate is None else yaw_rate
        # 라바콘이 앞 구간(S2)에서 시작됐으면 보류하지 않고 라바콘 구간(S3)으로 넘김 (11/16 anchor_1116_2313: 3번 코너 유지가
        # 누적 회전 59~63°에서 끝나(전환 기준 65°) 라바콘 내내 S2로 남음 → 라바콘 뒤 4번 코너 유지가 S2 상한 70°에 걸려 0.68 s 만에 종료)
        cone_ahead = (self.c['sections'].get('cone_advance', True) and self.cones.active
                      and self.sections.index == START_SECTION['cones']-1)
        self.sections.update(turn_rate, velocity, dt, hold=self.section_hold, advance=cone_ahead)
        if self.sections.index != START_SECTION['lane_change']:
            self.entry_from_corner = True      # 다른 구간을 달려옴 → S2 진입 때 흰 선 제한 적용 (entry_gate_open)
        self.signs_lost = 0.0 if signs_ok else self.signs_lost+dt
        self.signs_fresh = signs_ok
        self.update_stop_line(lane, velocity, now, dt)
        previous_state=self.state
        green_only = self.update_signs(scan, detections, detection_stamp, velocity, now, dt)
        if self.state=='lane' and self.c['parking'].get('test_wait_for_sign',False) and not self.c['parking'].get('test_drive_until_sign',False):
            self.debug=dict(section=self.sections.index,stage=self.stage(),speed=0.,steering=0.)
            return 0.,0.,'parking_test: WAITING_FOR_PARKING_SIGN'
        if self.state.startswith('parking_') or self.state == 'parked':
            search_command=None
            if self.state in ('parking_cruise','parking_scan','parking_approach'):
                # 표지판 전후 동일 LaneDriving 인스턴스·선 놓침 탐색·조향 기억 유지
                search_command=self.main_lane_command(lane,now,dt,velocity,angular_velocity,True)
            if parking_input is None:
                speed,steering,state=self.parking.fail('PARKING_INPUT_UNAVAILABLE')
            else:
                speed,steering,state=self.parking.step(now=now,scan=scan,velocity=velocity,lane=lane,search_command=search_command,angular_velocity=angular_velocity,measured_turn_rate=turn_rate,detections=detections,detection_stamp=detection_stamp,**parking_input)
            self.state=state
            if state=='parked':self.advance('parking')
            self.debug=dict(parking=self.parking.debug,section=self.sections.index,stage=self.stage(),control_stamp=now,
                            speed=speed,steering=steering,front=scan['front'],rear=scan['rear'],
                            velocity=velocity,scan_valid=scan['valid_fraction'],
                            missions=sorted(self.sections.missions()),done=sorted(self.done),
                            off=list(self.c['sections'].get('off',())),sign_counts=dict(self.sign_counts))
            # 주차는 실측 차체의 예상 궤적 검사 적용. 일반 섹터 최소 거리·stall creep·시간 초과 강행 제외
            return speed,steering,state+': '+self.parking.reason
        # 코너 유지는 일반 차선 주행일 때만 (차로 변경·라바콘·정지·주차 중 제외)
        # 갈림길 표지판 후 fork_distance 동안도 제외: 꺾인 선 구간에서 고정 조향 후 과회전 (11/9 hold_1109_c1 150 s)
        # 코너 유지 금지는 갈림길 정렬 중(locked·align)·manual에만. 목적 차로 도착 뒤(follow·exit)는 A·B 모두 허용
        # 예전에는 fork.running 전체를 막아 표지판을 읽으면 S3 전환까지 3번 코너 유지가 꺼짐 → 선을 놓친 뒤 조향이 풀려
        # 벽으로 직진 (11/15 main_1115_1904 216~221 s A 길, fix_1115_1949 204~209 s B 길)
        # 표지판을 못 읽은 주행(11/14 lap5)은 원래 유지가 켜진 채 같은 코너 통과. S자 오시작은 start_far_angle·hold_min_turned가 막음
        # 정렬 중이어도 3차선 구간이 끝났거나(three_lane_over) 3번 코너 안(corner3)이면 허용 (11/16 main_1116_arc1 205~210 s:
        # 차선 변경이 수동 개입으로 늦어져 S2 6 m 뒤 차선 변경이 꺼짐 → 목적 차로에 못 가 align에 고정 → 3번 코너 유지 금지 →
        # 선 놓친 뒤 조향 −0.25 → −0.04로 풀려 라바콘 정면 정지). 갈림길이 어떤 이유로 안 끝나든 코너는 돌게 함
        in_fork = (self.fork.running and self.fork.phase not in ('follow', 'exit')
                   and not self.three_lane_over() and not self.corner3())
        # 갈림길 뒤 S자 구간 (11/14 lap5 161 s): 차가 S2 직선 방향보다 왼쪽으로 35° 틀어진 채 코너 유지 시작 → 일찍 꺾어 안쪽 선 이탈,
        # 65°만 돌아 코너도 20°에서 끝남. 표지판을 읽으면 fork.running이 막지만 못 읽으면 그대로 시작됨
        # → S2에서 왼쪽으로 hold_min_turned(°) 넘게 틀어져 있으면 새 유지 시작 안 함, 시작할 때 틀어진 만큼 회전각 더함
        s = self.c['sections']
        left_of_axis = -self.sections.turned if s['enabled'] and self.sections.index == START_SECTION['fork'] else 0.0
        self.lane_driver.hold_start_ok = left_of_axis <= s.get('hold_min_turned', 15.0)
        # 라바콘 구간(S3)도 틀어진 만큼 더 돎 (시작 금지 조건은 없음): 콘을 나오며 왼쪽으로 15~21° 틀어진 채 4번 코너 유지 시작 →
        # 65°만 돌아 누적 31~37°에서 끝남 → 남은 코너를 한 선 pursuit로 느리게 돌아 옆 차로 이탈, 횡단보도 앞에서 반대쪽 이탈
        # (11/15 fix3_1115_2113 197~217 s. 같은 날 앞선 녹화 3개도 31~37°에서 종료). 과회전은 hold_max_turned가 막음
        tilt = left_of_axis
        if s['enabled'] and self.sections.index == START_SECTION['cones']:
            tilt = -self.sections.turned
        self.lane_driver.hold_extra = max(0.0, tilt)/s['odom_yaw_scale']
        # 코너 유지 상한: 유지를 시작한 구간의 직선 방향에서 오른쪽으로 hold_max_turned(°, IMU) 넘게 돌았으면 끝내고 새로 시작도 안 함
        # 유지는 '시작부터 65°(odom)'라 코너 중간에서 시작하면 과회전 (11/15 fix2_1115_2017 204 s: 44° 돈 지점에서 시작 →
        # 안쪽 첫 콘으로 향함). 정상 코너는 누적 48~67°에서 끝나 영향 없음 (같은 날 녹화 3개)
        if self.lane_driver.hold is None:
            self.hold_section = self.sections.index
        self.lane_driver.hold_stop = bool(
            s['enabled'] and -math.degrees(self.sections.yaw)-90.0*self.hold_section >= s.get('hold_max_turned', 70.0))
        speed, steering = self.main_lane_command(lane,now,dt,velocity,angular_velocity,
                                                  self.state == 'lane' and not self.avoiding and not in_fork,
                                                  self.avoiding)
        mode = 'corner_hold' if self.lane_driver.hold is not None else self.state
        # 서행·방지턱: 차선 주행 결과에 속도 상한·턱 위 조향 고정 (차선 코드는 그대로)
        if self.state == 'lane' and not self.avoiding:
            speed, steering, finished = self.slow_zone.update(
                self.allowed('slow'), self.sections.distance, tilt_rate, velocity, speed, steering, now, dt,
                line=self.line_distance)
            if finished:
                self.advance('slow')
            if self.slow_zone.state == 'slow' and mode == 'lane':
                mode = 'slow_zone'
        stuck_change = False
        # 분기 입구(흰 점선 끝) 추적. 입구에서 목표 차로가 아니면 manual(정지), 입구 뒤 차선 변경 금지 (소민)
        if self.sections.allows('fork') or self.fork.running:
            self.fork.observe(lane, self.lane_change.index, self.changing, velocity, dt)
            self.sync_fork()
        if self.state == 'lane':
            # 최종 목적지 우선 검사. 이동 중인 제어기는 완료 전 새 목표로 전환 금지
            self.observe_heading_bias(lane, velocity, dt)
            requested = self.fork_request
            # 늦은 지점 요청 (11/16): 점선 끝이 보여 '늦음'이어도 경로 계획이 확인해 주면 목적 쪽 이동만 허용
            # 예전: 가운데 차로에 도착한 뒤 새 이동이 금지돼 섬 표지판 기둥 앞까지 직진·정지 (합성 시험)
            late_request = 0
            lc = self.c['lane_change']
            if (not requested and lc.get('request_plan', False) and lc.get('request_late', False)
                    and self.fork.running and not self.fork.entered):
                late_request = self.fork.request(self.lane_change.index, self.lane_change.active, ignore_late=True)
                requested = late_request
            if self.sections.allows('lane_change') and not requested and self.c['missions'].get('fork_center_align', True):
                requested = self.fork.center_request(self.lane_change.index, self.lane_change.active, lane)
            preferred = self.fork.direction if self.fork.running else 0
            # 멈춤 방지가 차선 변경을 끈 동안(lane_change_off)도 앞 장애물 회피는 함 (avoid_only, 소민 11/12)
            # 예전: 1.5 m 동안 회피 판단 자체를 꺼서 그 안의 다음 장애물에 직진 충돌
            # 라바콘 조기 진입 (11/14 lap3): 콘이 3번 코너 안에서 시작하면 S3 전환(코너 65°) 전에 콘에 닿음
            # → S2에서 오른쪽으로 early_turn(°) 넘게 돈 뒤에는 붉은색이 화면에 차면(corridor) 라바콘 먼저, 차선 변경은 끔
            cone_early = self.cone_early()
            cone_first = self.cones.active or (cone_early and self.cones.corridor)
            # 코너에서는 새 차선 변경 안 함 (11/14 lap4 169~175 s: 3번 코너 유지 중 벽·콘을 앞 장애물로 보고 조향을 가로챔 → 왼쪽으로 차선 변경 → 콘 추돌)
            # 코너 유지 중이거나 S2에서 오른쪽으로 lane_change_max_turn(°) 넘게 돈 뒤. 진행 중인 차선 변경은 그대로 끝냄
            in_corner = self.lane_driver.hold is not None or self.corner3()
            # 후진 재시도 (11/17): 후진 중에는 차선 변경 판단을 쉬고 지나온 경로를 되짚음. 끝나면 이번 주기부터 다시 판단
            self.backup.record(velocity, turn_rate, dt)
            was_backing = self.backup.active
            change = self.backup.step(velocity, dt, self.road_heading()) if was_backing else None
            entry_open = self.entry_gate_open()
            change_enabled = ((self.sections.allows('lane_change') or self.fork.running)
                              and (self.fork.allows_change(self.lane_change.active) or bool(late_request))
                              and not cone_first and not in_corner and not self.three_lane_over()
                              and entry_open)
            # 점선 추종 (3lane_chu_js): 켜져 있으면 기존 차선 변경 대신 조향. 위치를 잃으면(None) 같은 주기에 기존 방식으로 넘어감
            # 입구 앞 반 칸 이동은 '늦은 지점'(점선 끝이 보임)이어도 함 → allows_change 대신 입구 통과(entered)만 봄
            dash_enabled = (self.c.get('dash_lane', {}).get('enabled', False) and change is None
                            and (self.sections.allows('lane_change') or self.fork.running) and not self.fork.entered
                            and not cone_first and not self.dash_in_corner() and not self.three_lane_over() and entry_open
                            and self.lane_change_off <= 0 and not self.lane_change.active)
            dash_tracking = (self.c['sections']['enabled'] and self.sections.index == START_SECTION['lane_change']
                             and not self.fork.entered)
            dash = None
            if self.c.get('dash_lane', {}).get('enabled', False):
                self.dash.index_hint = self.lane_change.index
                dash = self.dash.update(lane, scan, velocity, dt, self.road_heading(), dash_enabled,
                                        direction=self.fork.direction if self.fork.running else 0,
                                        exit_now=self.dash_exit_now(), tracking=dash_tracking)
                if self.dash.active:
                    # 기존 코드(갈림길 목적 차로 도착·입구 판단, 끝난 뒤 차선 변경)가 쓰는 차로 번호를 점선 추종 위치로 맞춤
                    self.lane_change.index = self.dash.lane_index()
            if change is not None:
                # 구간 거리는 절댓값으로 쌓임 → 후진한 만큼(이미 더해진 것 포함 2배) 되돌림
                self.sections.distance = max(0.0, self.sections.distance-2*abs(velocity)*dt)
            elif dash is not None:
                change = dash
            else:
                if was_backing:
                    # 후진이 방금 끝남: 차선이 바로 안 잡혀도 판단을 이어 가게 함 (lane_change.lane_memory)
                    self.lane_change.ended_clock = self.lane_change.clock
                change = self.lane_change.update(lane, scan, velocity, turn_rate, dt, change_enabled,
                                                requested, preferred, avoid_only=self.lane_change_off > 0,
                                                request_lanes=self.request_lanes(requested), request_limit=self.request_limit(),
                                                request_only=bool(late_request), road_heading=self.road_heading(),
                                                fork_lanes=self.fork_lanes())
                retreat = self.backup.observe(change, self.lane_change, body_points(scan, self.robot), velocity, dt,
                                              blocked=self.was_blocked, enabled=change_enabled,
                                              heading=self.road_heading())
                if retreat is not None:
                    change = retreat
            cone = None if change is not None else self.cones.update(
                lane, scan, velocity, turn_rate, dt,
                (self.sections.allows('cones') or cone_early) and self.fork_remaining <= 0,
                red_only=cone_early and not self.sections.allows('cones'),
                holding=self.lane_driver.hold is not None)
            if change is not None:
                stuck_change = (self.lane_change.active or self.dash.active) and change[0] == 0.0
                speed, steering = change
                mode = ('lane_change: backing up' if self.backup.active else
                        'dash_lane: ' + self.dash.debug.get('reason', '') if self.dash.active else
                        'lane_change' if self.lane_change.active else 'lane_change: waiting for clear lane')
            elif cone is not None:
                speed, steering = cone
                mode = 'cones'
                self.cone_exit_left = self.c['lidar'].get('cone_exit_slow_distance', 0.0)
            elif self.cone_exit_left > 0:
                # 라바콘 직후: 콘 통과 방향이 틀어진 채 4번 코너에 들어감 → 코너 유지 뒤 0.37 ↔ 0.23 m/s로 출렁임 (lap4 214~216 s)
                self.cone_exit_left -= abs(velocity)*dt
                speed = min(speed, self.c['lidar'].get('cone_exit_slow_speed', 0.22))
            self.entry_aligning = False
            if change is None and cone is None and not self.cones.active and self.lane_driver.hold is None:
                steering, self.entry_aligning = self.entry_align(steering)
        if self.state != 'lane':
            # 라바콘 뒤 거리(한 선 이름 힌트·저속 구간)는 상태와 무관하게 셈. 예전: lane 상태에서만 줄어 정지 접근에 들어가면 멈춤 →
            # 힌트가 접근 내내 켜져 오른쪽 선을 왼쪽으로 이름 → 우회전·신호등 추돌 (11/16 main_1116_fix1 218~225 s, 라바콘 뒤 1.49 m에서 멈춤)
            if not self.cones.active:
                self.cones.since_exit += abs(velocity)*dt
            if self.cone_exit_left > 0:
                self.cone_exit_left -= abs(velocity)*dt
        m = self.c['missions']
        if (m.get('traffic_zone_speed') and self.state in ('lane', 'traffic_approach') and self.allowed('traffic')
                and self.c['sections']['enabled'] and self.sections.index == START_SECTION['traffic']
                and self.sections.distance < m.get('traffic_zone_distance', 3.0)):
            # 신호등 구간 서행 (11/18): 신호등 확정·정지선 측정 시간을 벌고 서행 표지판 구간 속도를 지킴. stop_missions가 이 값 아래로만 더 줄임
            speed = min(speed, m['traffic_zone_speed'])
        lidar_c = self.c['lidar']
        if (lidar_c.get('cone_approach_speed') and self.state == 'lane' and not self.cones.started and not self.cones.active
                and self.c['sections']['enabled'] and self.sections.index == START_SECTION['cones'] and self.sections.allows('cones')
                and self.sections.distance < lidar_c.get('cone_approach_distance', 2.0)):
            # 라바콘 접근 서행 (11/18): 라바콘이 켜지기 전 S3 앞부분. 색 판정이 빗나가도 빠른 속도로 콘에 닿지 않게 함
            speed = min(speed, lidar_c['cone_approach_speed'])
        speed = self.stop_missions(speed, velocity, green_only, now)
        if self.c['parking'].get('test_drive_until_sign',False):
            speed=min(speed,self.c['parking']['scan_speed'])
        # 일반 주행의 섹터 비상 정지는 기존 기준 유지
        clearance = scan['front'] if speed >= 0 else scan['rear']
        # 후진 재시도 중에는 뒤 섹터 검사 제외: 라이다가 뒤를 못 봐 rear가 항상 0(막힘). 지나온 경로를 되짚으므로 생략
        backing = self.backup.active and speed < 0
        blocked = scan['valid_fraction'] < .5 or (not backing and clearance < self.c['lidar']['emergency_distance'])
        self.was_blocked = bool(blocked and speed >= 0)
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
            # 목적 차로에 들어선 뒤(follow·exit)는 fork_exit_speed (11/16 main_1116_1614: 224.5~239.6 s 2.5 m를 0.18로 감)
            m = self.c['missions']
            leaving = self.fork.phase in ('follow', 'exit')
            if self.dash.active:
                # 점선 추종 중: S2 준비 상한(0.18) 대신 dash_lane이 고른 속도 그대로 (그 값이 이미 상한)
                pass
            else:
                speed = min(speed, m.get('fork_exit_speed', .18) if leaving else m.get('fork_prepare_speed', .18))
        self.debug = dict(lane_valid=lane['valid'], lane_error=lane['error'],
                          pid=dict(self.pid.terms), pid_feedback_gain=self.c['pid']['feedback_gain'],
                          stage=self.stage(), section=self.sections.index,
                          section_distance=self.sections.distance, section_turned=self.sections.turned,
                          restarts=self.restarts, restart_error=self.restart_error,
                          yaw_deg=self.sections.yaw*57.29578, yaw_source='odom' if yaw_rate is None else 'imu',
                          slow_zone=self.slow_zone.state, slow_tilt_deg=self.slow_zone.tilt_deg,
                          slow_bump_at=self.slow_zone.bump_at, slow_boost=self.slow_zone.boost,
                          stall_time=self.stall_time, lane_change_off=self.lane_change_off, entry_gate=None if self.entry_gate is None else dict(self.entry_gate), signs_ok=signs_ok,
                          front=scan['front'], rear=scan['rear'],
                          lane_index=self.lane_change.index, lane_change_target=self.lane_change.target,
                          lane_change_debug=dict(self.lane_change.debug),
                          lane_change_offset=self.lane_change.offset, lane_change_heading=self.lane_change.heading,
                          heading_bias=round(self.heading_bias, 2), heading_bias_samples=self.heading_bias_samples,
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
                          fork_arrow_history=''.join((a or '-')[0] for a in self.arrow_history[-8:]),
                          avoiding=self.avoiding, cone_active=self.cones.active, cone_red=self.cones.red,
                          cone_red_ratio=float(lane.get('cone_red', 0.0)), cone_corridor=self.cones.corridor,
                          cone_heading=self.cones.heading, cone_center=self.cones.center, speed=speed,
                          corner_hold=self.lane_driver.hold is not None,
                          # 아래는 디버그 화면(tools/debug_view.py) 표시 전용. 판단에 쓰지 않음
                          velocity=velocity, state_time=now-self.state_started if self.state_started else None,
                          missions=sorted(self.sections.missions()), done=sorted(self.done),
                          off=list(self.c['sections'].get('off', ())),
                          sign_counts=dict(self.sign_counts),
                          cooldowns={name: until-now for name, until in self.cooldowns.items() if until > now},
                          signs_lost=self.signs_lost,
                          stopped_for=None if self.stopped_since is None else now-self.stopped_since,
                          zigzag_left=self.lane_driver.zigzag_left, lane_lost=now-self.lane_driver.last_seen,
                          search_distance=self.lane_driver.search_distance,
                          scan_valid=scan['valid_fraction'], wide_objects=scan.get('wide_objects'),
                          narrow_objects=scan.get('narrow_objects'), blocked=bool(blocked),
                          cones=self.cones.debug,
                          lane_change=dict(self.lane_change.debug, active=self.lane_change.active,
                                           waiting=self.lane_change.waiting, direction=self.lane_change.direction))
        self.debug['dash_lane'] = dict(self.dash.debug, active=self.dash.active)   # 3lane_chu_js 점선 추종 상태
        if getattr(self, 'entry_aligning', False):
            self.debug['entry_align'] = True     # 3차선 진입 정렬 중일 때만 표시
        if self.backup.active or self.backup.stuck > 0:
            # 후진 재시도: 서 있거나 후진 중일 때만 표시 (평소 디버그 내용은 그대로)
            self.debug['backup'] = dict(self.backup.debug, active=self.backup.active, stuck=round(self.backup.stuck, 2))
        return float(speed), clip(steering, -self.robot['max_steering'], self.robot['max_steering']), mode if self.state == 'lane' else self.state
