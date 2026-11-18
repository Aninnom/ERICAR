"""3차선 정적 장애물: 라이다로 전방 장애물 감지 → 왼쪽/오른쪽 판단 → 한 차로 이동

출처: hello-osy/2026-orda-mentoring src/orda_sim/orda_sim/answer.py (613c9b4)
유경 수정: 라이다로 잰 차로별 빈 거리로 좌·우 선택 (기존: 무조건 왼쪽 먼저), 이동 중 경로가 막히면
가까운 안전 조향으로 피함 (기존: 정지), 이동 거리 초과 시 영구 정지 대신 차선 주행으로 복귀.

이 파일은 라이다 판단·조향만 한다. 차선(흰 점선·실선) 인식은 차선 담당 lane_vision.py에서
lane['valid'], lane['heading'], lane['error'], lane['forward'], lane['solid'], lane['colors']로 받는다.
선 색(lane['colors'])으로 차로 번호(index) 보정: observe_lane (소민 추가)
소민 11/12: 갈림길 표지판 방향 우선(preferred), 카메라 종료·지나침 방지(바깥 노란 선 이탈 대책), 목표 횡이동 제한
  근거: test/three_lane.py 합성 시험, 11/11 run_1111_1241 (차선 변경 뒤 노란 바깥선 밖 이탈)
차선 번호: 0=오른쪽, 1=가운데, 2=왼쪽 (진행 방향 기준). direction +1 = 왼쪽, -1 = 오른쪽.
"""
import math
import numpy as np

from .lidar import body_points, clear_steering, footprint_clearance, pursuit_steering, to_body, to_road


def free_distances(forward, left, center, robot, config, side_ahead=None):
    """라이다 점(도로 기준)으로 지금 차로(0)·왼쪽(+1)·오른쪽(-1) 차로가 앞으로 몇 m 비어 있는지.

    지금 차로 = 차체가 지나갈 폭(차 기준). 옆 차로 = 차로 중앙(center ± lane_width) 기준 차폭 통로.
    차 옆이나 바로 뒤에 점이 있으면 0 → 그쪽으로 못 감. 점이 없으면 라이다 범위(3 m).
    바로 뒤 = 차체 뒤끝(뒷축 뒤 length − front_bumper_x = 7.2 cm) + side_behind_margin (소민 11/13)
    예전: 뒷축 뒤 차 길이(33 cm)까지 → 3차로 시작 지점의 가로 벽 등 이미 지나친 것까지 막힘으로 봄 (합성 시험)
    side_ahead(m): 주면 옆 차로는 뒷축 앞 이 거리보다 먼 점만 막힘으로 봄 (표지판 요청 이동을 일찍 시작, 11/16)
    """
    c = config
    half = robot['width']/2+c['margin']
    behind = robot['length']-robot.get('front_bumper_x', robot['length'])+c.get('side_behind_margin', .05)
    if side_ahead is not None:
        behind = -side_ahead
    lanes = {0: 0.0, 1: center+c['lane_width'], -1: center-c['lane_width']}
    free = {}
    for direction, middle in lanes.items():
        inside = (np.abs(left-middle) < half) & (forward > (0 if direction == 0 else -behind))
        free[direction] = float(max(0.0, forward[inside].min())) if np.any(inside) else c['free_range']
    return free


def arc_path(target, yaw0, angle, radius, margin, tail=0.0, step=0.02):
    """원호–직선–원호 이동의 뒷축 경로 (도로 기준 x 앞·y 왼쪽·방향 + 왼쪽). arc_steering과 같은 규칙:
    방향을 angle(rad)까지 틀어 유지 → 남은 횡거리가 복귀 원호 몫(radius·(1−cos 방향)+margin) 이하면 되돌림 → 나란해지면 끝.
    tail(m): 도착 뒤 목적 차로를 따라 더 가는 거리(도착 직후 앞이 막혔는지 확인용). 반환 (N×3 배열, 도착까지 전진 거리)"""
    side = 1.0 if target >= 0 else -1.0
    x = y = 0.0
    yaw = float(yaw0)
    poses, back, reach = [(x, y, yaw)], False, None
    for _ in range(int(8.0/step)):
        if side*yaw >= 0 and side*(target-y) <= radius*(1-math.cos(yaw))+margin:
            back = True
        goal = 0.0 if back else side*angle
        turn = max(-step/radius, min(step/radius, goal-yaw))
        yaw += turn
        x += step*math.cos(yaw)
        y += step*math.sin(yaw)
        poses.append((x, y, yaw))
        if back and abs(yaw) < 1e-3:
            reach = x
            break
    if reach is None:
        reach = x
    for k in range(int(tail/step)):
        poses.append((reach+(k+1)*step, y, 0.0))
    return np.asarray(poses), float(reach)


# 선 색 조합 (왼쪽, 오른쪽) → 차로 번호 (0=오른쪽, 1=가운데, 2=왼쪽). 노랑 = 3차선 바깥 경계, 흰 = 차로 구분 점선
LANE_BY_COLORS = {('yellow', 'white'): 2, ('white', 'white'): 1, ('white', 'yellow'): 0}


class LaneChange:
    """라이다 담당: 전방 차량 감지 → 빈 옆 차로 선택 → 한 차로 이동 → 차선 복귀.

    offset은 카메라로 측정한 도로 방향 기준 횡이동 거리. odom의 속도만 적분한다.
    지도 좌표나 장애물 차량의 배치 좌표는 사용하지 않는다.
    """
    def __init__(self, config, robot):
        self.c, self.robot = config['lane_change'], robot
        self.vision = config['lane']
        self.active = False
        self.index = self.c['initial_lane']
        self.heading = self.offset = self.target = self.travel = 0.0
        self.direction = 0
        self.waiting = False
        self.seen_lane, self.seen_count, self.seen_stamp = None, 0, None
        self.fork_running = False      # 갈림길 표지판 확정 뒤 (driver가 preferred로 알려 줌)
        self.fork_dir = 0              # 확정된 출구 방향 (+1 왼쪽, -1 오른쪽, 0 없음)
        self.walls = {}
        self.debug = {}
        self.clock, self.free_log = 0.0, []   # 좌·우 선택용 최근 빈 거리 기록 (시각, free)
        self.heading_log = []          # 최근 카메라 차선 방향 (새 프레임마다). 이동 시작 방향은 이 값들의 중앙값
        self.back = False              # 원호–직선–원호 조향: 복귀 원호(방향을 도로 방향으로 되돌림) 단계인지
        self.back_clock = self.move_clock = -1e9   # 복귀 시작·이동 시작 시각 (짧은 S자 감속)
        self.short = False             # 지금 이동이 짧은 S자인지 (앞 장애물 short_front 안에서 시작)
        self.span = 1                  # 지금 이동이 건너는 차로 수 (표지판 요청으로 두 칸 떨어진 목적 차로면 2)
        self.move_angle = None         # 지금 이동의 원호 각도(°). None이면 arc_angle·short_arc_angle
        # 라이다 기준점 (11/16, 유경 feat/fork-double-move의 front_object를 옮김): 섬 위 표지판 기둥 = 가운데 차로 앞 좁은 물체
        self.anchor = None             # 가운데 차로 통로의 기둥까지 앞 거리(m, 뒷축 기준). 어느 차로에서든 봄
        self.anchor_front = None       # 지금 차로 통로 바로 앞의 기둥까지 거리(m) = 지금 가운데 차로에 있다는 근거
        self.anchor_gone = 0.0         # 기둥을 못 본 채 간 거리(m)
        # 도로 방향 기준 차 방향(rad, + 오른쪽)의 IMU 값: 구간 관리자가 3차선 직선 방향에서 잰 누적 회전 (driver가 매 주기 전달)
        # 11/16 anchor_1116_2313: 코너 출구에서 카메라 차선 방향이 실제와 45° 달라 차선 변경이 반대로 감 → 방향 기준은 IMU
        self.road_heading = None
        self.requested_move = False    # 지금 이동이 갈림길 요청인지 (pursuit 조향의 보는 거리)
        self.dash_shift = 0.0           # 한 칸 회피 목표의 점선 방향 편차(m)
        self.plan = None
        self.dash_hold = 0.0            # 가운데 차로에서 다음 회피까지 유지할 편차(m)
        self.hold_steering = 0.0       # 짧은 S자 중 마지막 안전 조향 (위험해 멈출 때 바퀴 고정)
        self.engaged, self.clear_since = False, None   # 앞 장애물 회피 판단 중인지, 앞이 빈 채 지난 시각 (front_clear_hold)
        self.ended_clock = -1e9        # 마지막 이동이 끝난 시각 (self.clock 기준)

    def trusted_colors(self, lane):
        """선 색에서 교통섬 노란 선을 걸러 냄 (소민 11/14 run_1114_2330: 가운데 차로에서 교통섬 노란 선을 바깥선으로 봐
        차로 번호 1 → 0 → 왼쪽 끝으로 쓸데없이 이동, 끝 차로 도착 뒤 번호가 다시 바뀌어 재이동)
        바깥 노란 선 = 차 바로 앞(BEV 아래쪽)부터 이어지고 도로 방향과 나란함
        교통섬 노란 선 = 분기 입구 앞쪽 멀리서 시작하거나(가까운 끝이 yellow_near_max m보다 멂) 비스듬함
        → 이런 노란 선은 'island'로 바꿔 차로 번호·노란 경계 판단에 안 씀. 차선 따라가기(lane_vision)는 그대로"""
        colors = dict(lane.get('colors', {}))
        if self.fork_dir:
            # 표지판 확정 뒤(분기까지): 방향 규칙 (소민 11/15). 분기 근처에서 옆으로 옮기는 중에는 교통섬 선과 바깥선이
            # 둘 다 화면 아래까지 들어오고 차가 비스듬해 거리·각도로는 못 가름. 교통섬은 항상 출구 반대쪽(안쪽)에 있으므로
            # 출구 쪽 노란 선 = 바깥선(믿음), 반대쪽 노란 선 = 교통섬. 차로 번호와 상관없어 이동 중에도 그대로 적용
            exit_side = 'left' if self.fork_dir > 0 else 'right'
            return {side: ('island' if color == 'yellow' and side != exit_side else color)
                    for side, color in colors.items()}
        # 표지판 전(보조 기준): 멀리 보이는 교통섬 선을 거리·각도로 거름
        near_ends, headings = lane.get('near_ends', {}), lane.get('line_headings', {})
        for side, color in colors.items():
            if color != 'yellow':
                continue
            far_start = near_ends.get(side, 0.0) > self.c.get('yellow_near_max', .9)
            slanted = (side in headings and
                       abs(headings[side]-lane.get('heading', 0.0)) > self.c.get('yellow_heading_max', .25))
            if far_start or slanted:
                colors[side] = 'island'
        return colors

    def observe_lane(self, lane):
        """선 색으로 본 차로가 새 카메라 프레임 index_confirm_frames번 연속 같으면 index 보정.
        지금까지는 initial_lane에서 차선 변경 횟수만 셈 → 한 번 틀리면 계속 틀림
        노랑/노랑(일반 구간)·선 하나·색 정보 없음 → 무시 (기존 index 유지)"""
        stamp = lane.get('stamp')
        if stamp is not None and stamp == self.seen_stamp:
            return
        self.seen_stamp = stamp
        if lane.get('valid') and 'heading' in lane:
            self.heading_log = (self.heading_log+[float(lane['heading'])])[-self.c.get('heading_frames', 5):]
        else:
            self.heading_log = []
        colors = self.trusted_colors(lane) if lane.get('valid') else {}
        if abs(lane.get('heading', 0.0)) > self.c.get('index_heading_max', .3):
            # 크게 틀어진 자세: 옆 차로 너머 선까지 보여 색 조합이 다른 차로처럼 보임 → 판정 안 함
            colors = {}
        seen = LANE_BY_COLORS.get((colors.get('left'), colors.get('right')))
        if seen is None or seen >= self.c['lane_count']:
            return
        if seen == self.seen_lane:
            self.seen_count += 1
        else:
            self.seen_lane, self.seen_count = seen, 1
        # 갈림길 표지판 확정 뒤 '가운데'(흰/흰) 판정은 index_confirm_frames_fork 프레임 연속일 때만 인정 (소민 11/15)
        # 분기 앞 끝 차로에서 바깥선이 흰색처럼 보이거나 빠지면 잠깐 흰/흰 → 번호 2 → 1 → 다시 이동 (run_1114_2330 39.1 s)
        # 완전히 무시하면 번호가 잘못 끝 차로로 돼 있을 때 못 고침 → 오래 이어질 때만 고침. 끝 차로 판정은 그대로
        need = self.c['index_confirm_frames']
        if self.fork_running and seen == self.c['lane_count']//2:
            need = self.c.get('index_confirm_frames_fork', 20)
        # 차선 변경 중에는 index 그대로 (끝날 때 direction만큼 바꿈). 본 차로는 카메라 종료 판단에만 씀
        if not self.active and self.seen_count >= need:
            self.index = seen

    def narrow_object(self, forward, left, middle, around=0.2, min_points=2):
        """차로 통로(중앙 middle m, 차 기준 왼쪽 +) 앞에서 가장 가까운 물체가 좁으면 (앞 거리, 크기), 아니면 None
        크기 = 가장 가까운 점에서 around(m) 안 점들의 좌우·앞뒤 폭 중 큰 값. anchor_width(m) 이하만 기둥으로 봄
        (섬 표지판 기둥 실측 약 0.05~0.07 m, 11/16 plan_1116_1902 155 s 앞 1.9 m. 장애물 차는 0.22 × 0.32 m)"""
        if len(forward) == 0:
            return None
        front = self.robot.get('front_bumper_x', self.robot['length'])
        inside = (np.abs(left-middle) < self.c['lane_width']/2) & (forward > front) & (forward < self.c['free_range'])
        if not np.any(inside):
            return None
        nearest = int(np.flatnonzero(inside)[np.argmin(forward[inside])])
        group = np.hypot(forward-forward[nearest], left-left[nearest]) < around
        size = float(max(np.ptp(left[group]), np.ptp(forward[group])))
        if np.count_nonzero(group) < min_points or size > self.c.get('anchor_width', .15):
            return None
        return float(forward[nearest]), size

    def observe_anchor(self, forward, left, center):
        """섬 표지판 기둥 추적. 가운데 차로 통로(어느 차로에서든)와 지금 차로 통로를 따로 봄. 못 보면 주행 거리만큼 줄여 유지"""
        middle = center+(self.c['lane_count']//2-self.index)*self.c['lane_width']
        seen = self.narrow_object(forward, left, middle)
        if seen is not None:
            self.anchor, self.anchor_gone = seen[0], 0.0
        own = self.narrow_object(forward, left, center)
        if own is not None and self.index == self.c['lane_count']//2:
            self.anchor_front = own[0]
        self.debug['anchor'] = None if self.anchor is None else round(self.anchor, 2)
        self.debug['anchor_front'] = None if self.anchor_front is None else round(self.anchor_front, 2)

    def choose_direction(self, free, lane, requested=0, preferred=0, exclude=0):
        """왼쪽(+1)·오른쪽(-1) 중 갈 차로. 없으면 0

        - 차로 범위 밖·노란 경계 방향 제외. 일반 회피는 실선 방향도 제외
        - 옆 차로가 min_free_distance 이상 비어 있고, 지금 차로보다 free_gain 이상 더 비어 있어야 함
        - 둘 다 되면 더 멀리 비어 있는 쪽. 같으면 왼쪽 (기존 동작)
        - preferred(이미 읽은 갈림길 표지판 방향) 쪽이 조건을 만족하면 그쪽 먼저 (반대로 피하면 다시 건너와야 함, 소민)
        """
        c = self.c
        rejected = self.debug['rejected'] = {}
        best, best_free = 0, free[0]+c['free_gain']
        colors = lane.get('colors', {})
        between_yellow = (self.index == c['lane_count']//2
                          and colors.get('left') == colors.get('right') == 'yellow')
        anchored = self.anchor_front is not None and c.get('anchor_enabled', True) and not between_yellow
        for direction, side in ((1, 'left'), (-1, 'right')):
            if direction == exclude:
                rejected[side] = 'path_blocked'      # 경로 확인에서 막힌 쪽 (기둥 앞 반대쪽 재선택용)
                continue
            if requested and direction != requested:
                rejected[side] = 'opposite_to_request'
                continue
            if not 0 <= self.index+direction < c['lane_count']:
                rejected[side] = 'outside_lane_count'
                continue
            # YOLO 요청도 노란 바깥선·교통섬 경계 횡단 금지. 점선 차로에서 정렬 필요
            # 기둥이 지금 차로 바로 앞에 보이면 가운데 차로가 확실 → 표지판 요청 방향은 옆 선이 노랑으로 읽혀도 넘음
            # (11/16 sominc_1116_1804·plan_1116_1902: 가운데 차로 도착 뒤 오른쪽 선이 노랑으로 읽혀 요청 거부 → 반대쪽으로 회피)
            # 양옆 노란 선이 보이는 분기 입구에서는 기둥 추정만으로 경계 무시 금지
            if self.trusted_colors(lane).get(side) == 'yellow' and not (requested and anchored):
                rejected[side] = 'yellow_boundary'
                continue
            # 일반 회피의 실선 금지: 차선 변경은 3차선(S2)·갈림길에서만 켜지고 차로 사이는 점선. 점선을 실선으로 오판해
            # 빈 쪽을 막음 (소민 11/15 run_1115_2123 124.4 s) → 기본 끔. 바깥 경계는 노란 선 검사가 따로 막음
            if not requested and c.get('solid_blocks_avoid', False) and lane.get('solid', {}).get(side, False):
                rejected[side] = 'solid_boundary'
                continue
            enough = free[direction] >= c['min_free_distance']
            if not enough:
                rejected[side] = 'insufficient_free_distance'
            elif not requested and free[direction] <= free[0]+c['free_gain']:
                rejected[side] = 'insufficient_free_gain'
            if enough and not requested and direction == preferred and free[direction] > free[0]+c['free_gain']:
                return direction
            if enough and (requested or free[direction] > best_free):
                best, best_free = direction, free[direction]
        return best

    def drop_walls(self, points, heading, center, lane_index, colors=None):
        """대회장 벽(트랙 가장자리 = 노란 바깥선 위) 라이다 점 제거 (소민 11/13)

        벽이 노란선 바로 위면 끝 차로 차체와 15 cm, 옆 차로 통로와 11.5 cm. 카메라 도로 방향(heading)이 3°만 틀려도
        벽 점이 통로에 들어와 빈 차로를 막혔다고 봄 → 장애물 있는 쪽 선택 (합성 시험: 벽 + 방향 오차 ±3·5°)
        방법: 바깥선 근처(±wall_search)에서 앞뒤로 wall_min_length 넘게 이어진 직선을 라이다 점으로 직접 맞춤 →
        카메라 방향 오차와 상관없이 벽 기울기를 따라감. 그 선에서 wall_band 안쪽 ~ 바깥 점만 버림
        장애물 차(길이 0.32 m)는 길이 조건에 안 걸림. 벽과 장애물 차 옆면은 15 cm 떨어져 있어 band(5 cm) 밖
        center: 지금 차로(lane_index) 중앙이 차 기준 도로 방향 왼쪽 몇 m. colors: 그쪽 선이 노랑이면 그 선이 바깥선"""
        c = self.c
        self.walls = {}
        need = c.get('wall_min_points', 12)
        if not c.get('wall_filter', True) or len(points) < need:
            return points
        forward, left = to_road(points, heading)
        width, last = c['lane_width'], c['lane_count']-1
        keep = np.ones(len(points), dtype=bool)
        for side, name in ((1, 'left'), (-1, 'right')):
            beyond = last-lane_index if side > 0 else lane_index
            if colors and colors.get(name) == 'yellow':
                beyond = 0
            edge = center+side*(max(0, beyond)*width+width/2)
            search = c.get('wall_search', .25)
            near = (forward > -.3) & (forward < c['free_range']) & (np.abs(left-edge) < search)
            if np.count_nonzero(near) < need:
                continue
            f, l = forward[near], left[near]
            slope, offset = np.polyfit(f, l, 1)
            inlier = np.ones(len(f), dtype=bool)
            for tolerance in (.08, .04, .03):        # 장애물 차 점을 빼고 다시 맞춤
                inlier = np.abs(l-(slope*f+offset)) < tolerance
                if np.count_nonzero(inlier) < need:
                    break
                slope, offset = np.polyfit(f[inlier], l[inlier], 1)
            if (np.count_nonzero(inlier) < need or np.ptp(f[inlier]) < c.get('wall_min_length', .8) or
                    abs(slope) > c.get('wall_max_slope', .35)):
                continue
            # 벽은 바깥선보다 안쪽에 있을 수 없음: 맞춘 선이 바깥선 안쪽 wall_inside 넘게 들어와 있으면 장애물 차 줄
            if side*(offset-edge) < -c.get('wall_inside', .08):
                continue
            keep &= side*(left-(slope*forward+offset)) < -c.get('wall_band', .05)
            self.walls[name] = (round(float(slope), 3), round(float(offset), 3))
        return points[keep]

    def intermediate_dash_shift(self, lane, direction):
        """바깥에서 가운데로 회피할 때 넘어가는 흰 점선 쪽으로 목표를 당기는 거리(m)."""
        side = 'left' if direction > 0 else 'right'
        color = lane.get('colors', {}).get(side)
        if (self.c['lane_count'] != 3 or self.index+direction != 1 or not direction
                or not (color == 'white' or color is None and lane.get('white_lane'))):
            return 0.0
        return -direction*self.c.get('intermediate_dash_bias', 0.0)

    def steering_shift(self):
        return self.dash_shift if self.span == 1 and not self.requested_move else 0.0

    def dash_follow(self, lane, points, speed):
        """가운데 차로에서 직전에 넘은 점선 옆을 유지하며 새 회피를 기다림."""
        side = 'left' if self.dash_hold > 0 else 'right'
        if (not self.dash_hold or self.index != 1 or self.fork_running or lane.get('abnormal')
                or lane.get('colors', {}).get(side) != 'white'):
            return None
        target = lane.get('dash_targets', {}).get(side)
        if target is None:
            return None
        forward, lateral = target
        heading = lane['heading']
        desired = pursuit_steering(forward-self.dash_hold*math.sin(heading),
                                   lateral+self.dash_hold*math.cos(heading), self.robot)
        steering, safe = clear_steering(points, desired, self.robot, self.c['safety_horizon'], self.c['margin'])
        self.debug.update(reason='dash_follow', dash_hold=self.dash_hold, safe=safe)
        return (speed if safe else 0.0), steering

    def steer(self, points, dt):
        """한 차로 이동 중: 목표 횡이동(target)까지 pure pursuit, 막히면 가까운 안전 조향"""
        c = self.c
        # 이동 중 벽 제거: 목적 차로 중앙(차 기준) = target − offset, 목적 차로 번호 = index + direction
        points = self.drop_walls(points, self.heading, self.target-self.offset, self.index+self.direction*self.span)
        error = self.target-self.offset+self.steering_shift()
        if self.short or c.get('steer_mode', 'arc') == 'arc':
            # 짧은 S자(앞 장애물 short_front 안)도 원호–직선–원호로, 각도만 short_arc_angle로 크게 (소민 11/16)
            desired = self.arc_steering(error)
        else:
            # 보는 거리: 시작은 lookahead(표지판 이동은 request_lookahead)로 세게, 목표에 가까워질수록 lookahead_end까지 늘려
            # 도착 전에 미리 핸들을 풂 (소민 11/15 실차: 고정 0.40이면 비스듬히 들어가 반대로 크게 꺾으며 정렬, 앞으로 많이 감)
            start = c.get('request_lookahead', c['lookahead']) if self.requested_move else c['lookahead']
            progress = min(1.0, abs(self.offset)/max(abs(self.target), 1e-6))
            lookahead = start+(max(start, c.get('lookahead_end', start))-start)*progress
            x, y = to_body(lookahead, error, self.heading)
            desired = pursuit_steering(x, y, self.robot)
        steering, safe = clear_steering(points, desired, self.robot, c['safety_horizon'], c['margin'])
        if (safe and not self.back and c.get('safe_flip_block', False) and desired*steering < 0
                and abs(steering-desired) > c.get('safe_flip_max', .4)
                and (self.travel < c.get('safe_flip_travel', .15) or abs(self.offset) < c.get('safe_flip_offset', .08))):
            # 이동을 막 시작해(safe_flip_travel m 안) 목표 쪽으로 꺾는 중인데 '가장 가까운 안전 조향'이 반대쪽:
            # 따르면 목표에서 멀어지며 앞 장애물에 더 붙음 (11/17 rev_1117_0212 90.8~92.0 s: 시작 0.02 m 만에 왼쪽 0.42 → 오른쪽 −0.26,
            # 0.11 m 전진해 앞끝 0.34 → 0.26 m) → 정지 (후진 재시도가 이어받음). 이동 중간의 잠깐 반대 조향(장애물 모서리 비켜 가기)은
            # 그대로 둠: 다 막으면 임의 배치 합성 평가 46 → 43/50 (시드 4·7·37)
            # safe_flip_offset: 주행 거리와 별개로 횡이동이 이만큼(m) 진행되기 전에도 금지 (11/17 bend_1117_0311 81.3 s: 0.15 m 지나자
            # 금지가 풀려 횡이동 0.01 m인 채 기둥 쪽으로 직진·정지 반복 → 후진 재시도도 못 들어감)
            self.debug['flip_blocked'] = round(float(steering), 3)
            steering, safe = float(desired), False
        if (safe and self.back and c.get('back_end', False)
                and self.direction*error <= c.get('back_end_error', .02)
                and (steering-desired)*self.direction > c.get('back_end_deviation', .15)):
            # 복귀 원호 중 횡이동이 목표에 도달했는데, 도로 방향으로 되돌리는 조향이 막혀 안전 조향이 이동 방향(바깥)쪽으로 나옴
            # (11/17 back_1117_0249 128.9~134.2 s: 횡 −0.54/목표 −0.53·방향 +4.6°에서 종료 조건 2.9° 미달 → 왼쪽 +0.2~0.42 대신
            #  오른쪽 −0.05~−0.21로 0.53 m 더 감, 방향 12.7°·차체가 노란 선 밖, 장애물 옆에 낌) → 따르지 않고 이동을 끝냄 (update)
            self.debug['back_end'] = round(float(steering), 3)
        self.debug.update(desired=desired, safe=safe, error=error, walls=self.walls)
        if self.steering_shift():
            self.debug['dash_shift'] = self.dash_shift
        short = self.short
        self.debug['reason'] = ('changing_short' if short else 'changing') if safe else 'no_safe_steering'
        if short:
            if safe:
                self.hold_steering = steering
            else:
                # 짧은 S자 중 안전한 조향이 없어 멈출 때는 바퀴를 직전 값에 고정 (소민 11/16 run_1116_1347:
                # 멈춘 채 대체 조향이 프레임마다 좌우로 바뀌어 핸들만 왔다갔다)
                steering = self.hold_steering
        # 안전한 조향이 하나도 없을 때만 그 자리에서 기다린다 (정적 장애물은 비켜 갈 길이 생기지 않으면 실패)
        speed = c.get('short_speed', c['speed']) if short else c['speed']
        if short:
            # 조향이 크게 바뀌는 순간(시작 꺾기·복귀 반대 꺾기) short_flip_seconds(s) 동안 short_flip_speed로 감속:
            # 바퀴가 넘어가는 동안 덜 가서 계산한 S자에 가깝게 (소민 11/16)
            flip = c.get('short_flip_seconds', 0.0)
            since = min(self.clock-self.move_clock,
                        self.clock-self.back_clock if self.back else 1e9)
            if since < flip:
                speed = min(speed, c.get('short_flip_speed', speed))
        return (speed if safe else 0.0), steering

    def arc_steering(self, error):
        """원호–직선–원호 (11/15): 차 방향을 도로 방향에서 arc_angle(°)까지 틀어 유지하다가, 남은 횡오차가
        '방향을 되돌리는 원호가 차지할 횡거리' 이하가 되면 도로 방향으로 되돌림. 방향은 IMU 적분값(self.heading) 되먹임

        예전 pure pursuit(목표점 0.40 m 앞): 횡오차 0.57 m면 55°를 조준 → 조향 0.42 포화, 최대 41~47° 틀어짐,
        방향 8.6°(finish_heading) 남은 채 종료 → 차선 주행이 0.2~0.3 m 어긋난 것으로 봄 (fix3_1115_2113 130~149 s)
        heading 부호: + = 차가 도로 방향보다 오른쪽을 봄 (offset은 heading이 −일 때 왼쪽으로 늘어남)"""
        c = self.c
        side = 1.0 if self.target >= 0 else -1.0             # +1 = 왼쪽으로 이동
        yaw = -self.heading                                   # 도로 방향 기준 차 방향 (+ 왼쪽)
        radius = self.robot['wheelbase']/math.tan(self.robot['max_steering'])
        # 지금 방향을 0으로 되돌리는 동안 더 가는 횡거리 + 조향 지연 여유
        short = self.short
        # 짧은 S자는 자기 여유(short_arc_margin)를 씀. 보통 회피(혁준님 원호)는 arc_margin 그대로 (소민 11/16)
        margin = c.get('short_arc_margin', c.get('arc_margin', .03)) if short else c.get('arc_margin', .03)
        need = radius*(1-math.cos(abs(yaw)))+margin
        if side*yaw >= 0 and side*error <= need:
            if not self.back:
                self.back_clock = self.clock     # 복귀(반대 조향) 시작 시각: 짧은 S자 감속용
            self.back = True
        # 짧은 S자: 앞 장애물이 가까워 시작한 이동은 더 크게 틀어 짧게 옮김 (소민 11/16, 장애물 사이 약 1 m에서 복귀)
        angle = c.get('short_arc_angle', 45.0) if self.short else c.get('arc_angle', 30.0)
        if self.move_angle is not None:
            angle = self.move_angle       # 표지판 요청 이동: 시작 때 고른 각도 (plan_request)
        goal = 0.0 if self.back else side*math.radians(angle)
        limit = self.robot['max_steering']
        return float(max(-limit, min(limit, c.get('arc_gain', 2.5)*(goal-yaw))))

    def start_heading(self, camera):
        """새 이동의 시작 방향(도로 방향 기준 차 방향, rad)

        - 직전 이동이 chain_seconds(s) 안에 끝났으면 그 이동에서 IMU로 적분해 온 방향(self.heading)을 이어 씀
          (11/15 fix2_1115_2017 156.9 s: 0→1 이동 직후 1→2 이동이 카메라 한 프레임 방향 −0.48 rad로 시작, 실제 차는 직진 자세
           → 횡이동 과대 추정 + 복귀 조향 0.48 rad 과다 → 도로보다 오른쪽 13~16°로 끝나 교통섬 직진·표지판 추돌)
        - 그 외에는 최근 heading_frames 프레임 카메라 방향의 중앙값 (한 프레임 튐 제외)"""
        if self.road_heading is not None:
            return self.road_heading      # IMU 기준이 있으면 그 값 (카메라 한 프레임·이어받은 값보다 우선)
        if self.clock-self.ended_clock <= self.c.get('chain_seconds', 2.0):
            return self.heading
        log = sorted(self.heading_log)
        return log[len(log)//2] if log else camera

    def camera_trusted(self, lane):
        """카메라의 차로 판정을 믿어도 되는 자세인지 (11/16 anchor_1116_2313 188.6 s: 34° 틀어진 채 흰/흰을 가운데 차로로 보고
        횡이동 −0.11 → −0.54 m·방향 34 → 8°로 덮어써 복귀 원호 없이 종료, 실제는 오른쪽 38°로 대각선 주행)
        IMU로 적분한 방향이 camera_trust_heading(rad) 안이고, 카메라 차선 방향과 camera_agree_heading(rad) 안에서 맞을 때만"""
        c = self.c
        if not c.get('camera_guard', True):
            return True
        return (abs(self.heading) < c.get('camera_trust_heading', .26) and
                abs(lane.get('heading', 0.0)-self.heading) < c.get('camera_agree_heading', .175))

    def camera_finished(self, lane):
        """카메라로 차선 변경 끝 판단 (소민): 반 넘게 옮겼고, 선 색이 목적 차로 + 차로 중앙 근처·나란함
        추측 항법(odom 적분)은 회전 오차(11/9 실측 1.07배)로 덜·더 감 → 카메라가 확인되면 바로 차선 주행에 넘김"""
        c = self.c
        # 중앙 판정 (11/17 m08_1117_0417 31.7 s): 차선 오차(lane['error'])는 앞보기 지점 값이라 차가 틀어져 있으면 실제 위치와 다름.
        # 오른쪽 6.9° 틀어진 채 오차 −0.03 → '중앙'으로 보고 종료했으나 실제는 중앙에서 0.10 m 못 미침.
        # → 방향을 보정한 차로 중앙값(camera_correct와 같은 식)으로 판정. 끄기: camera_finish_center False
        middle = lane.get('error', 0.0)
        if lane.get('valid') and (c.get('camera_finish_center', False) or self.steering_shift()):
            heading = lane.get('heading', 0.0)
            middle = lane['error']*math.cos(heading)-lane.get('forward', .65)*math.sin(heading)
        return (self.seen_lane == self.index+self.direction*self.span and self.seen_count >= 2 and lane.get('valid') and
                self.camera_trusted(lane) and
                abs(self.offset) > c['lane_width']*c.get('camera_finish_progress', .6) and
                abs(middle+self.steering_shift()) < c.get('camera_finish_error', .06) and
                abs(lane['heading']) < c.get('camera_finish_heading', .12))

    def camera_correct(self, lane):
        """목적 차로가 카메라에 확인되면 추측 항법 위치·방향을 카메라 값으로 바로잡음 (소민 11/12)
        추측 항법은 odom 속도·IMU 회전 오차만큼 한 번에 13~19 cm 더 감 (합성 시험: odom 속도 0.8배)
        → 두 번 연속 변경(0→1→2)에서 오차가 쌓여 노란 바깥선 이탈 (11/12 실차)
        조건: 선 색이 목적 차로를 camera_fix_frames번 연속 + 그 차로 중앙이 차 기준 반 차로 안 (걸쳐 있을 때 오인 방지)"""
        c = self.c
        if not (lane.get('valid') and self.seen_lane == self.index+self.direction*self.span and
                self.seen_count >= c.get('camera_fix_frames', 2) and self.camera_trusted(lane)):
            return False
        heading = lane['heading']
        center = lane['error']*math.cos(heading)-lane.get('forward', .65)*math.sin(heading)
        if abs(center) > c['lane_width']*c.get('camera_fix_range', .5):
            return False
        self.offset = self.target-center
        if self.road_heading is None:
            self.heading = heading     # IMU 기준이 없을 때만 방향도 카메라 값으로
        self.debug['camera_fix'] = round(center, 3)
        return True

    def update(self, lane, scan, velocity, yaw_rate, dt, enabled, requested=0, preferred=0, avoid_only=False,
               request_lanes=1, request_limit=None, request_only=False, road_heading=None, fork_lanes=None):
        self.waiting = False
        if not enabled or preferred:
            self.dash_hold = 0.0
        self.clock += dt
        points = body_points(scan, self.robot)
        self.heading -= yaw_rate*dt
        self.road_heading = road_heading if self.c.get('imu_heading', True) else None
        if self.active and self.road_heading is not None:
            self.heading = self.road_heading    # 이동 중 방향도 IMU 기준 그대로 (시작 값 오차가 남지 않게)
        self.fork_running, self.fork_dir = bool(preferred), int(preferred)
        if self.anchor is not None or self.anchor_front is not None:
            # 기둥 거리: 못 본 주기에는 주행 거리만큼 줄임. anchor_memory(m) 넘게 못 보거나 지나치면 버림
            step = abs(velocity)*dt
            self.anchor_gone += step
            self.anchor = None if self.anchor is None else self.anchor-step
            self.anchor_front = None if self.anchor_front is None else self.anchor_front-step
            if self.anchor_gone > self.c.get('anchor_memory', .5) or (self.anchor is not None and self.anchor < 0):
                self.anchor = None
            if self.anchor_gone > self.c.get('anchor_memory', .5) or (self.anchor_front is not None and self.anchor_front < 0):
                self.anchor_front = None
        self.observe_lane(lane)
        if self.active:
            # lateral_scale: 추측 항법이 실제 횡이동보다 덜 셈 (11/15 fix3_1115_2113: 두 번 모두 0.09~0.20 m 더 감, 11/12 소민 13~19 cm)
            self.offset -= velocity*math.sin(self.heading)*dt*self.c.get('lateral_scale', 1.0)
            self.travel += abs(velocity)*dt
            if not self.try_extend(points, fork_lanes, request_limit):
                detouring = self.plan is not None and self.plan.get('fork_detour') == 'return'
                if not (enabled and not detouring and self.try_reverse(points, fork_lanes, request_limit)):
                    self.redecide(points, fork_lanes, request_limit)
            self.camera_correct(lane)
            error = self.target-self.offset+self.steering_shift()
            if self.c.get('steer_mode', 'arc') == 'arc':
                # 복귀 원호가 끝나 도로 방향과 나란해지면 종료 (남은 횡오차는 차선 주행이 처리)
                done = self.back and abs(self.heading) < self.c.get('arc_finish_heading', .05)
            else:
                done = abs(error) < self.c['finish_error'] and abs(self.heading) < self.c['finish_heading']
            shift = self.steering_shift()
            if shift:
                # 차체가 점선을 충분히 넘으면 남은 방향 오차를 다음 조향에 인계
                inset = self.c['lane_width']/2-self.direction*(self.target-self.offset)
                room = self.robot['width']/2+self.c['margin']
                rolling = (self.back and inset >= room and abs(error) < self.c['finish_error']
                           and abs(self.heading) <= self.c.get('start_align', .21))
                done = done or rolling
            done = done or self.camera_finished(lane)
            # 지나침 방지: 목표보다 overshoot(m) 넘게 갔으면 바로 차선 주행으로 (바깥 노란 선 넘음 방지, 소민)
            done = done or abs(self.offset) > abs(self.target)+self.c.get('overshoot', .08)
            if done or self.travel >= self.c['max_distance']:
                # 이동 거리 초과: 영구 정지 대신 차선 주행으로 돌아간다. 반 차로 넘게 갔으면 옮긴 것으로 본다.
                # 두 칸 이동이 거리 초과로 끝나면 실제로 넘어간 칸 수만 반영
                lanes = self.span if done else min(self.span, int(abs(self.offset)/self.c['lane_width']+0.5))
                hold = shift
                if not hold and done and self.span == 1 and not self.requested_move and self.index+self.direction == 1:
                    # 거리 판단으로 먼저 시작한 회피도 도착 차로에서 점선을 확인하면 추종 가능
                    side = 'right' if self.direction > 0 else 'left'
                    if lane.get('colors', {}).get(side) == 'white':
                        hold = -self.direction*self.c.get('intermediate_dash_bias', 0.0)
                self.active = False
                self.index += self.direction*lanes
                self.seen_lane, self.seen_count = None, 0   # 이동 중 본 차로 기억은 다음 판단에 안 씀
                self.ended_clock = self.clock
                if hold and done and enabled and not preferred:
                    self.dash_hold = hold
                    # 같은 주기에 새 장애물 경로를 판단해 중간 중앙 정렬 생략
                    return self.update(lane, scan, 0.0, 0.0, 0.0, enabled, requested, preferred, avoid_only,
                                       request_lanes, request_limit, request_only, road_heading, fork_lanes)
                return None
            result = self.steer(points, dt)
            if self.debug.get('back_end') is not None:
                # 복귀 중 횡이동은 목표에 왔는데 정렬 조향이 막혀 안전 조향이 바깥쪽으로 나옴 → 여기서 이동 종료 (steer 주석)
                # 이번 주기부터 새 판단: 앞 장애물까지 거리가 남아 있을 때 바로 다음 회피를 시작할 수 있게 함
                self.active = False
                self.index += self.direction*self.span
                self.seen_lane, self.seen_count = None, 0
                self.ended_clock = self.clock
                ended = dict(back_end=self.debug['back_end'], ended_heading=round(math.degrees(self.heading), 1))
                result = self.update(lane, scan, 0.0, 0.0, dt, enabled, requested, preferred, avoid_only, request_lanes,
                                     request_limit, request_only, road_heading, None)
                self.debug.update(ended)
            return result
        if (enabled and not lane['valid'] and self.road_heading is not None and self.c.get('lane_memory', False)
                and self.clock-self.ended_clock <= self.c.get('lane_memory_seconds', 3.0)):
            # 이동(또는 후진)이 끝난 직후 차선이 안 잡힘: 가까운 앞 장애물이 화면을 가려 선이 빠짐 (11/17 back_1117_0249
            # 128.9~131.0 s·후진 뒤 143 s: 판단이 꺼지고 차선 놓침 탐색이 0.15~0.18로 전진 → 남은 거리를 다 씀)
            # → lane_memory_seconds(s) 동안은 '차로 중앙, 방향은 IMU 기준'으로 보고 회피 판단을 이어 감. 선 색 정보는 없음
            forward = lane.get('forward', .65)
            lane = dict(lane, valid=True, heading=self.road_heading, error=forward*math.tan(self.road_heading),
                        forward=forward, colors={}, solid={})
            self.remembered = True
        else:
            self.remembered = False
        if self.active or not enabled or not lane['valid']:
            self.free_log = []
        if not enabled or not lane['valid']:
            self.debug = dict(reason='disabled' if not enabled else 'lane_invalid')
            return None
        # avoid_only (멈춤 방지가 차선 변경을 끈 동안, 소민 11/12): 갈림길 정렬·중앙 정렬은 안 하고 앞 장애물 회피만.
        # 예전: 1.5 m 동안 회피 판단 자체를 끔 → 그 안에 다음 장애물이 있으면 차선 주행으로 직진해 충돌
        if avoid_only:
            requested = 0
        # 넓은 군집 분류가 빠져도 실제 주행 통로의 라이다 점으로 회피 판단 필요
        # 새 이동의 방향은 최신 카메라 기준. 이전 코너부터 누적된 heading 사용 금지
        # 도로 방향: 카메라 차선 방향과 IMU 값이 heading_disagree(rad) 넘게 다르면 IMU 값 (코너 출구·비스듬한 자세에서 카메라 오차)
        heading = lane['heading']
        if self.road_heading is not None and abs(heading-self.road_heading) > self.c.get('heading_disagree', .2):
            heading = self.road_heading
        camera_center = lane['error']*math.cos(heading)-lane.get('forward', .65)*math.sin(heading)
        points = self.drop_walls(points, heading, camera_center, self.index, self.trusted_colors(lane))
        forward, left = to_road(points, heading)
        # 지금 차로 중앙이 차 기준 왼쪽으로 몇 m인지 (카메라 차선). 갈림길 요청은 차 기준
        center = 0.0 if requested else camera_center
        anchor_center = max(-self.c['lane_width']/2, min(self.c['lane_width']/2, camera_center))
        # 표지판 요청 이동: 옆 장애물이 뒷축 앞 request_side_ahead(m) 안까지 빠졌으면 시작 (조향해도 옆 차로에 들어가기까지 약 0.5 m 더 감)
        # 11/16 somin_1116_1714: 확정 뒤 옆 장애물 앞면이 뒷축 뒤로 빠질 때까지 0.90 m 대기 → 둘째 이동 거리 부족
        # 일반 회피도 같은 기준(avoid_side_ahead)으로 일찍 시작. 대신 출발 전에 경로를 그려 옆·앞 장애물과 겹치지 않는지 확인 (아래)
        # 11/16 imu_1116_2348 93.8~99.0 s: 옆 장애물이 뒷축 뒤로 다 빠질 때까지 기다려 앞 장애물 0.57 m 앞에서야 시작
        ahead = self.c.get('request_side_ahead') if requested else self.c.get('avoid_side_ahead')
        free = free_distances(forward, left, center, self.robot, self.c, side_ahead=ahead)
        # 옆 차로는 최근 choice_window 동안의 최솟값 사용: 가는 물체(의자 다리 등)가 한 스캔 빠지면 빈 차로로 보임 (11/14 lap3)
        window = self.c.get('choice_window', 0.0)
        # 기록에 옆 막힘 기준(ahead)을 같이 둠: 요청용(뒷축 앞 request_side_ahead)과 일반 회피용 값이 섞이면
        # 같은 주기의 일반 회피 재판단 값(옛 기준)이 최솟값을 차지해 요청 이동이 일찍 시작하지 못함 (11/16 sominb_1116_1753)
        self.free_log = [(t, f, a) for t, f, a in self.free_log if self.clock-t <= window]+[(self.clock, dict(free), ahead)]
        for side in (1, -1):
            free[side] = min(f[side] for _, f, a in self.free_log if a == ahead)
        self.debug = dict(free=free, index=self.index, walls=self.walls)
        if self.remembered:
            self.debug['lane_memory'] = True
        if self.c.get('anchor_enabled', True):
            self.observe_anchor(forward, left, anchor_center)
        # 차체가 진행할 통로 안의 전방 장애물만 확인한다.
        if free[0] >= self.c['trigger_distance'] and not requested:
            # 회피 판단 중 앞이 비어 보여도 front_clear_hold(s) 동안 계속 비어야 판단을 놓음 (소민 11/16: 라이다 점이
            # 한두 스캔 빠지면 front_clear ↔ 대기 사이를 오가며 속도·조향이 출렁임)
            if self.clear_since is None:
                self.clear_since = self.clock
            if not self.engaged or self.clock-self.clear_since >= self.c.get('front_clear_hold', 0.0):
                self.engaged, self.clear_since = False, None
                self.debug['reason'] = 'front_clear'
                return self.dash_follow(lane, points, self.c['speed'])
        else:
            self.clear_since = None
        if not requested:
            self.engaged = True
        direction = self.choose_direction(free, lane, requested, preferred)
        dash_shift = 0.0
        if direction and not requested and self.c.get('avoid_plan', False):
            # 일반 회피 경로 확인: 이 이동이 쓸 각도(앞이 short_front 안이면 short_arc_angle)로 경로를 그려 여유가 모자라면 아직 출발 안 함
            short = free[0] < self.c.get('short_front', 1.0)
            angle = self.c.get('short_arc_angle', 45.0) if short else self.c.get('arc_angle', 30.0)
            plan_center = max(-self.c['lane_width']/2, min(self.c['lane_width']/2, center))
            dash_shift = self.intermediate_dash_shift(lane, direction)
            tail = self.c.get('avoid_plan_tail', .2)
            yaw = -self.start_heading(heading)
            target = direction*self.c['lane_width']+plan_center
            margin = self.c.get('arc_margin', .03)
            if dash_shift and short:
                margin = self.c.get('short_arc_margin', margin)
            gap = self.path_gap(forward, left, target+dash_shift, angle, yaw, tail,
                                margin=margin)
            if dash_shift and gap < self.c.get('avoid_plan_margin', .04):
                dash_shift = 0.0
                gap = self.path_gap(forward, left, target, angle, yaw, tail)
            self.debug['avoid_gap'] = round(min(gap, 9.9), 3)
            if gap < self.c.get('avoid_plan_margin', .04):
                blocked, direction = direction, 0
                self.debug['rejected']['left' if blocked > 0 else 'right'] = 'path_blocked'
                # 기둥이 anchor_escape 안: 고른 쪽 경로가 막혔으면 기다리지 않고 반대쪽도 확인 (기다릴 거리가 없음, 11/17)
                if (self.c.get('request_blocked_escape', False) and self.anchor_front is not None
                        and self.anchor_front <= self.c.get('anchor_escape', 0.0)):
                    other = self.choose_direction(free, lane, 0, preferred, exclude=blocked)
                    if other:
                        gap = self.path_gap(forward, left, other*self.c['lane_width']+plan_center, angle,
                                            -self.start_heading(heading), self.c.get('avoid_plan_tail', .2))
                        self.debug['avoid_gap_other'] = round(min(gap, 9.9), 3)
                        if gap >= self.c.get('avoid_plan_margin', .04):
                            direction = other
                        else:
                            self.debug['rejected']['left' if other > 0 else 'right'] = 'path_blocked'
        cross = None
        if direction == 0 and not requested and self.c.get('redecide', False):
            # 규칙 6: 옆 차로로 한 칸 옮길 자리가 없어도, 두 칸을 직선으로 가로지르는 길이 비어 있으면 그 길로 출발
            cross = self.cross_two(forward, left, center, heading)
            if cross is not None:
                direction = cross['direction']
                self.debug['cross'] = cross
        self.debug['choice'] = direction
        if direction == 0 and not requested and all(
                self.debug['rejected'].get(side) in ('outside_lane_count', 'yellow_boundary') for side in ('left', 'right')):
            # 양쪽 다 차로 자체가 없음 (갈림길 뒤 일반 차로·코너): 차선 변경으로 풀 수 없는 장애물 → 차선 주행·코너 유지에 맡김
            # 예전: 저속 접근 모드가 3번 코너 조향을 대신해 덜 돌고 바깥 라바콘에 닿음 (11/14 lap3 264~273 s)
            self.debug['reason'] = 'no_side_lane'
            if not (self.c.get('no_side_wait', False) and not self.fork_running and not avoid_only):
                return None
            # 갈림길 표지판 확정 전 3차선 안(no_side_wait): 차로 판단이 한 프레임 틀려 양쪽이 '차로 없음'이 돼도
            # 회피 판단을 놓지 않고 아래 '갈 차로 없음'과 같이 천천히 다가가며 다시 판단 (소민 11/16: 그 프레임만 차선 주행
            # 조향이 끼어 핸들이 왔다갔다). 갈림길 뒤·코너는 예전처럼 차선 주행에 맡김
        plan = None
        if direction == 0 and requested and requested == preferred and self.index == self.c['lane_count']//2:
            side = 'left' if requested > 0 else 'right'
            other = 'right' if requested > 0 else 'left'
            near_fork = request_only or lane.get('colors', {}).get(other) == 'yellow'
            if near_fork and self.debug['rejected'].get(side) == 'yellow_boundary':
                if scan.get('valid_fraction', 1.) >= .5:
                    plan = self.plan_fork_boundary_route(lane, body_points(scan, self.robot), heading, requested, request_limit)
                if plan is None:
                    self.debug['reason'] = 'fork_boundary_hold'
                    return 0.0, 0.0
                direction = requested if plan.get('fork_direct') else -requested
                self.debug['choice'] = direction
        if direction == 0 and requested and request_only:
            # 늦은 지점 요청: 목적 쪽 이동만 허용된 상태 → 일반 회피로 넘기지 않음.
            # 단 기둥이 anchor_escape(m) 안까지 왔는데도 목적 쪽이 막혀 있으면 반대쪽이라도 피함
            # (기둥 앞 정지 = 15초 규정으로 이후 미션까지 잃음. 반대 길은 갈림길 점수만 잃음)
            escape = self.anchor_front is not None and self.anchor_front <= self.c.get('anchor_escape', 0.0)
            if not escape:
                self.debug['reason'] = 'late_request_blocked'
                return None
            rejected = dict(self.debug['rejected'])
            result = self.update(lane, scan, 0.0, 0.0, dt, enabled, preferred=preferred)
            self.debug['request_rejected'], self.debug['escape'] = rejected, True
            return result
        if (direction == 0 and requested and self.fork_running and self.c.get('anchor_enabled', True)
                and self.anchor_front is not None and self.anchor_front > self.c.get('anchor_escape', 0.0)):
            # 표지판 확정 + 섬 기둥이 지금 차로 앞: 목적 쪽이 잠깐 막혀도 반대쪽 회피로 넘기지 않고 차로 유지하며 다시 계획
            # (11/16 imu_1116_2348 112.4 s: 가운데 차로에서 오른쪽이 0.3 m쯤 뒤 열릴 상황인데 왼쪽으로 회피 → 반대 길).
            # 기둥이 anchor_escape 안까지 와도 막혀 있으면 아래 일반 회피로 (기둥 앞 정지 방지)
            self.debug['reason'] = 'request_wait_anchor'
            return None
        if direction == 0 and requested:
            # 목표 쪽 막힘: 목표는 유지하고 일반 장애물 회피로 우회 가능
            rejected = dict(self.debug['rejected'])
            result = self.update(lane, scan, 0.0, 0.0, dt, enabled, preferred=preferred)
            self.debug['request_rejected'] = rejected
            return result
        if avoid_only and free[0] <= self.c['stop_distance']:
            # 멈춤 방지가 포기시킨 장애물 바로 앞(stop_distance 안): 다시 시도하면 포기 → 재시도 무한 반복 (11/13 합성 시험)
            # → 차선 주행에 맡김 (비상 정지 → 서행 비켜 가기)
            self.debug['reason'] = 'off_too_close'
            return None
        if direction == 0 and avoid_only:
            # 갈 차로 없음: 또 대기하면 멈춤 방지가 반복 → 차선 주행에 맡김 (비상 정지 → 서행 비켜 가기)
            self.debug['reason'] = 'off_no_lane'
            return None
        if direction == 0:
            # 정적 장애물은 기다려도 비지 않음 → 앞 장애물 stop_distance 전까지는 지금 차로를 천천히 따라가며 다시 판단
            # (옆 차로를 막은 차를 지나치면 그쪽이 열림). 임의 배치 시험에서 옆 차 옆에 선 채 영구 정지하던 문제
            # 옆 차로가 차 바로 옆·코앞 물체(빈 거리 < beside_free) 때문에만 막혔으면 앞 장애물에 stop_distance_beside까지
            # 더 다가감: 방금 지나친 장애물이 차 뒤로 빠지면 그쪽이 열림 (소민 11/15 run_1115_2123: 오른쪽으로 피한 뒤
            # 가운데 장애물이 옆에 남은 채 0.6 m에서 정지 → 75초 끼임)
            stop = self.c['stop_distance']
            if any(self.debug['rejected'].get(side) == 'insufficient_free_distance' and
                   free[direction_] < self.c.get('beside_free', .3) for direction_, side in ((1, 'left'), (-1, 'right'))):
                stop = min(stop, self.c.get('stop_distance_beside', .45))
            if not requested and free[0] > stop:
                following = self.dash_follow(lane, points, self.c['approach_speed'])
                if following is not None:
                    return following
                desired = pursuit_steering(lane.get('forward', .65), lane['error'], self.robot)
                steering, safe = clear_steering(points, desired, self.robot,
                                                self.c['safety_horizon'], self.c['margin'])
                if safe:
                    self.debug['approach'] = True
                    self.debug['reason'] = 'approaching_blocked_lane'
                    return self.c['approach_speed'], steering
            self.waiting = True
            self.debug['reason'] = 'no_available_lane'
            return 0.0, 0.0  # 더 다가가도 차선 변경 불가한 거리 → 정지
        # 정렬 조건 (11/16): 차가 도로 방향에서 start_align(rad) 넘게 틀어져 있으면 새 이동을 시작하지 않고 차선 주행으로 먼저 정렬
        # (코너 출구 직후 등. 틀어진 자세에서는 옆 차로 빈 거리·목표 횡거리 계산도 어긋남). 앞 장애물이 stop_distance 안이면 예외
        if (self.road_heading is not None and abs(self.road_heading) > self.c.get('start_align', .21)
                and free[0] > self.c['stop_distance']):
            self.debug['reason'] = 'not_aligned'
            return None
        # 표지판 요청 이동: 출발 전에 경로를 계획. 어느 각도로도 라이다 점과 겹치면 출발하지 않고 차로 유지 → 다음 주기에 다시 계획
        # (예전: 계획 없이 출발 → 가운데 차로 0.8 m 앞 장애물로 꺾어 들어가 정지·접촉, 합성 시험 11/16)
        if plan is None and requested and self.c.get('request_plan', False):
            plan_center = max(-self.c['lane_width']/2, min(self.c['lane_width']/2, center))
            plan = self.plan_request(forward, left, direction, plan_center, min(2, max(1, int(request_lanes))),
                                     request_limit, -self.start_heading(heading))
            if plan is None:
                # 방향은 열려 있는데 경로만 막힘 + 기둥이 anchor_escape 안: 위 '기둥 앞 정지 방지'와 같이 일반 회피로 넘김
                # (11/17 ext_1117_0135 56.4 s: 가운데 차로 도착 때 기둥 0.45 m 앞, 왼쪽 요청 경로 막힘 → 차로 유지로 기둥 앞까지 직진)
                if (self.c.get('request_blocked_escape', False) and self.anchor_front is not None
                        and self.anchor_front <= self.c.get('anchor_escape', 0.0)):
                    rejected = dict(self.debug['rejected'])
                    result = self.update(lane, scan, 0.0, 0.0, dt, enabled, preferred=preferred)
                    self.debug['request_rejected'], self.debug['escape'] = rejected, 'path'
                    return result
                self.debug['reason'] = 'request_path_blocked'
                return None
            if plan.get('wait'):
                # 두 칸 경로가 지금은 막혔지만 조금 더 가서 꺾으면 열림 → 한 칸으로 내려가지 않고 차로 유지하며 다시 계획
                self.debug['reason'], self.debug['plan'] = 'request_wait_gap', plan
                return None
        self.active, self.direction = True, direction; self.requested_move = bool(requested)
        # 앞 장애물이 short_front(m)보다 가까우면 짧은 S자 (arc_steering 각도 short_arc_angle). 끄려면 short_front: 0
        # short_on_request: 표지판 요청 이동도 짧은 S자로 (한 칸 이동 거리 단축, 11/16)
        self.short = free[0] < self.c.get('short_front', 1.0) or bool(requested and self.c.get('short_on_request', False))
        # 이전에 본 차로 기억 비움 (11/15 main_1115_1904 180.07 s: 0→1 이동 중 비스듬히 본 '2번 차로' 기억이 남아
        # 1→2 이동이 시작한 주기에 camera_correct → overshoot 종료, 가운데 차로인데 index 2 → 교통섬 직진·표지판 추돌)
        self.seen_lane, self.seen_count = None, 0
        # 코너 출구에서도 차량 방향이 아닌 도로 방향으로 한 차로 이동한다.
        self.heading, self.offset, self.travel = self.start_heading(heading), 0.0, 0.0
        self.back = False
        self.move_clock, self.hold_steering = self.clock, 0.0
        self.engaged, self.clear_since = False, None
        # 차가 크게 틀어진 상태에서 목표가 한 차로 넘게 튀지 않도록 center는 반 차로 안으로 제한 (소민)
        center = max(-self.c['lane_width']/2, min(self.c['lane_width']/2, center))
        self.span, self.move_angle = 1, None
        self.dash_hold = 0.0
        self.dash_shift = dash_shift
        if cross is not None:
            self.span, self.move_angle, self.short = 2, cross['angle'], False     # 두 칸 직선 가로지르기 (보통 속도)
        if plan is not None:
            self.span, self.move_angle = plan['span'], plan['angle']
            self.short = False     # 고른 각도·보통 속도로 (짧은 S자 감속 안 씀)
            self.debug['plan'] = plan
        self.plan = plan
        self.target = direction*self.span*self.c['lane_width']+center
        return self.update(lane, scan, 0.0, 0.0, dt, enabled)

    def plan_fork_boundary_route(self, lane, lidar, heading, requested, limit):
        """섬 앞에서 경계 사이 직접 진입을 먼저, 반대 차로 우회는 그다음 검사."""
        c, v, robot = self.c, self.vision, self.robot
        yaw = -self.start_heading(heading)
        if not c.get('request_plan', False) or abs(yaw) > c.get('start_align', .21) or lane.get('abnormal'):
            return None
        edges = []
        row_m = (v['far']-v['near'])/(v['height_px']-1)
        scale = v['width_px']/v['view_width']
        for side in ('left', 'right'):
            fit = lane.get('fits', {}).get(side)
            span = lane.get('fit_spans', {}).get(side)
            if lane.get('colors', {}).get(side) != 'yellow' or fit is None or span is None:
                return None
            first, last = span
            near = v['far']-last*row_m
            # 가까운 구간에 지지가 있는 경계만 사용. 차 앞 사각 구간은 가까운 끝의 접선으로 연결
            if near > v['near']+.12 or (last-first)*row_m < .20:
                return None
            rows = np.arange(first, last, .02/row_m)
            near_rows = np.arange(last, (v['far']-robot['front_axle_x'])/row_m, .02/row_m)
            pixels = np.r_[np.polyval(fit, rows),
                           np.polyval(fit, last)+np.polyval(np.polyder(fit), last)*(near_rows-last)]
            edge = np.column_stack((v['far']-np.r_[rows, near_rows]*row_m,
                                    (v['width_px']/2-pixels)/scale))
            road = np.column_stack(to_road(edge, heading))
            edges.append(road[np.argsort(road[:, 0])])
        obstacles = np.vstack((np.column_stack(to_road(lidar, heading)), *edges))
        radius = robot['wheelbase']/math.tan(robot['max_steering'])
        angle = float(min(c.get('request_angles', [45.])))
        margin = c.get('arc_margin', .03)
        width = c['lane_width']
        clearance = c.get('request_plan_margin', .04)
        direct, direct_reach = arc_path(requested*width, yaw, math.radians(angle), radius, margin,
                                        tail=c.get('request_plan_tail', .3))
        # 도착 차체와 앞쪽 추종 구간을 두 경계가 함께 감싸야 함
        end = int(np.argmin(np.abs(direct[:, 0]-direct_reach)))
        direct[:, 0] += robot['wheelbase']/2*np.cos(direct[:, 2])
        direct[:, 1] += robot['wheelbase']/2*np.sin(direct[:, 2])
        entry = direct[end:]
        bounds = [np.interp(entry[:, 0], edge[:, 0], edge[:, 1], left=np.nan, right=np.nan)
                  for edge in edges]
        inside = bool(np.all((entry[:, 1] < bounds[0]) & (entry[:, 1] > bounds[1])))
        direct_gap = footprint_clearance(obstacles, direct, robot)
        self.debug['fork_direct'] = dict(inside=inside, gap=round(min(direct_gap, 9.9), 3))
        if inside and direct_gap >= clearance:
            return dict(span=1, angle=angle, fork_direct=True, gap=round(min(direct_gap, 9.9), 3))
        tail = c.get('request_reverse_tail', 1.1)
        outbound, reach = arc_path(-requested*width, yaw, math.radians(angle), radius, margin, tail=tail)
        end = int(np.argmin(np.abs(outbound[:, 0]-reach)))
        merged = outbound[end]
        # 첫 이동의 끝 자세에서 반대쪽 끝까지 되돌리는 경로
        back, back_reach = arc_path(requested*width-merged[1], merged[2], math.radians(angle), radius, margin,
                                    tail=c.get('request_plan_tail', .3))
        back[:, :2] += merged[:2]
        for path in (outbound, back):
            path[:, 0] += robot['wheelbase']/2*np.cos(path[:, 2])
            path[:, 1] += robot['wheelbase']/2*np.sin(path[:, 2])
        return_gap = footprint_clearance(obstacles, back, robot)
        return_length = (reach+back_reach)*c.get('request_plan_stretch', 1.1)
        gap = footprint_clearance(obstacles, outbound, robot)
        self.debug['detour'] = dict(gap=round(min(gap, 9.9), 3), return_gap=round(min(return_gap, 9.9), 3),
                                    return_length=round(return_length, 2), limit=limit)
        # 복귀를 고를 때는 출발 원호 구간만 필요. 다른 출구는 도착 뒤 통로까지 확인
        first_gap = footprint_clearance(obstacles, outbound[:end+1], robot)
        if limit is not None and return_length <= limit and min(first_gap, return_gap) >= clearance:
            choice = 'return'
        elif gap >= clearance:
            choice = 'alternate_exit'
        else:
            return None
        return dict(span=1, angle=angle, fork_detour=choice, **self.debug['detour'])

    def cancel(self):
        """진행 중인 이동·대기를 접음 (후진 재시도 직전). 반 차로 넘게 갔으면 옮긴 것으로 봄 (거리 초과 종료와 같은 규칙)"""
        self.dash_hold = 0.0
        if self.active:
            lanes = min(self.span, int(abs(self.offset)/self.c['lane_width']+0.5))
            self.active = False
            self.index += self.direction*lanes
            self.seen_lane, self.seen_count = None, 0
            self.ended_clock = self.clock
        self.waiting, self.engaged, self.clear_since, self.free_log = False, False, None, []

    def try_extend(self, points, fork_lanes, limit):
        """진행 중인 한 칸 이동을 두 칸으로 넓힘 (11/17 fork_1117_0119 62.5~63.2 s)
        왼쪽 끝 차로에서 앞 장애물 때문에 오른쪽으로 일반 회피(한 칸)를 시작한 0.7 s 뒤 오른쪽 표지판이 확정됨.
        이미 시작한 이동은 그대로 한 칸으로 끝나 가운데 차로에 섬 기둥 바로 앞에서 도착 → 둘째 이동 거리 없음.
        조건: 같은 방향으로 목적 차로가 두 칸(fork_lanes = 2 × direction), 복귀 원호 시작 전, 지금 자세에서
        남은 횡거리로 그린 경로의 여유가 request_plan_margin 이상이고 실차 보정 길이가 limit 안. 넓히면 각도 request_angles·보통 속도"""
        c = self.c
        if not (c.get('request_extend', False) and c.get('request_plan', False)):
            return False
        if self.span != 1 or self.back or fork_lanes != 2*self.direction:
            return False
        target = self.target+self.direction*c['lane_width']
        remaining = target-self.offset
        walls = self.walls
        kept = self.drop_walls(points, self.heading, remaining, self.index+2*self.direction)
        self.walls = walls
        forward, left = to_road(kept, self.heading)
        approved, info = self.check_extension(forward, left, remaining, limit, for_fork=True)
        info['extended'] = True
        if not approved:
            self.debug['extend_blocked'] = info
            return False
        self.span, self.target, self.move_angle = 2, target, info['angle']
        self.short, self.requested_move = False, True
        self.plan = self.debug['plan'] = info
        return True

    def check_extension(self, forward, left, remaining, limit, for_fork):
        """이동 중 두 칸 연장의 차체 여유·분기 입구까지 거리 검사."""
        c, robot = self.c, self.robot
        margin = c.get('arc_margin', .03)
        if for_fork:
            angle = min(c.get('request_angles', [45.0]))
            tail, clearance = c.get('request_plan_tail', .3), c.get('request_plan_margin', .04)
        else:
            angle = c.get('redecide_angle', c.get('short_arc_angle', 45.0))
            tail = c.get('redecide_tail', c.get('avoid_plan_tail', .2))
            clearance = c.get('avoid_plan_margin', .04)
        radius = robot['wheelbase']/math.tan(robot['max_steering'])
        path, reach = arc_path(remaining, -self.heading, math.radians(angle), radius, margin, tail=tail)
        length = reach*c.get('request_plan_stretch', 1.1)
        path[:, 0] += robot['wheelbase']/2*np.cos(path[:, 2])
        path[:, 1] += robot['wheelbase']/2*np.sin(path[:, 2])
        gap = footprint_clearance(np.column_stack((forward, left)), path, robot)
        # 기둥 거리로 잰 한계는 그대로, 추정 한계에는 기존 여유 적용
        slack = 0.0 if self.anchor is not None else c.get('request_extend_slack', .5)
        fits = bool(limit is None or length <= limit+slack)
        info = dict(span=2, angle=float(angle), length=round(float(length), 2), gap=round(min(gap, 9.9), 3),
                    limit=None if limit is None else round(float(limit), 2), fits=fits)
        return fits and gap >= clearance, info

    def cross_two(self, forward, left, center, heading):
        """두 칸 직선 가로지르기 후보 (11/18, 사용자 규칙 6). 끝 차로에서 반대쪽 끝 차로까지, 지금 출발하는 경로가 비어 있을 때만
        줄 간격이 짧아 가운데 차로에 나란히 설 자리가 없어도, 비스듬히 가로지르는 직선은 장애물 모서리 사이를 지남
        (현장 수동 주행 check_1118_01의 동선). 반환 dict(direction, angle, gap) 또는 None"""
        c = self.c
        middle = max(-c['lane_width']/2, min(c['lane_width']/2, center))
        best = None
        for direction in (1, -1):
            if not 0 <= self.index+2*direction < c['lane_count']:
                continue
            for angle in c.get('redecide_angles', [45.0]):
                gap = self.path_gap(forward, left, 2*direction*c['lane_width']+middle, angle, -self.start_heading(heading),
                                    c.get('redecide_tail', c.get('avoid_plan_tail', .2)))
                if gap >= c.get('avoid_plan_margin', .04) and (best is None or gap > best['gap']):
                    best = dict(direction=direction, angle=float(angle), gap=round(min(gap, 9.9), 3))
        return best

    def redecide(self, points, fork_lanes=None, limit=None):
        """이동 중 재판단 (11/18, 사용자 규칙 3·6): 일반 회피로 한 칸 옮기는 중 도착할 차로 앞이 막혀 있으면,
        되돌아 정렬하지 않고 같은 방향 다음 차로까지 그대로 가로지름 (두 칸을 직선으로)
        11/17 venue_1117_01 119.7~132.6 s: 오른쪽 → 가운데 이동 중 가운데 차로 0.6 m 앞에 장애물이 보였는데 이동을 끝까지 마치려고
        복귀 원호로 그 장애물 정면을 향함 → 정지. 그때 왼쪽 차로는 1 m 넘게 비어 있었음
        조건: 요청 이동이 아님, 한 칸 이동 중, 다음 차로가 있음, 도착 차로의 앞 빈 거리 < redecide_front(m, 뒷축 기준),
        지금 자세에서 그린 경로에 차체 여유 확보. 목적 차로 확정 뒤에는 try_extend와 같은 거리·여유 기준 적용.
        안 되면 이번 주기는 그대로 두고 다음 주기에 다시 봄
        표지판 방향이 정해졌으면(fork_lanes: 시작 차로에서 목적 차로까지 칸 수, + 왼쪽) 목적 차로에서 더 멀어지는 쪽으로는
        넓히지 않음 (11/18 redecide_1118_06 81.3 s: left 확정 뒤 왼쪽 차로 콘을 오른쪽으로 피하던 중, 가운데 차로 0.98 m 앞
        갈림길 표지판 기둥을 장애물로 보고 오른쪽 끝 차로까지 넘어가 B로 진입). 이동을 마친 뒤 요청 이동이 목적 쪽으로 되돌림"""
        c = self.c
        if not c.get('redecide', False) or self.requested_move or self.span != 1:
            return False
        if fork_lanes is not None and abs(int(fork_lanes)-2*self.direction) > abs(int(fork_lanes)-self.direction):
            self.debug['redecide_blocked'] = dict(reason='away_from_fork', fork_lanes=int(fork_lanes))
            return False
        far = self.index+2*self.direction
        if not 0 <= far < c['lane_count']:
            return False
        target = self.target+self.direction*c['lane_width']
        remaining = target-self.offset
        walls = self.walls
        kept = self.drop_walls(points, self.heading, remaining, far)
        self.walls = walls
        forward, left = to_road(kept, self.heading)
        half = self.robot['width']/2+c['margin']
        inside = (np.abs(left-(self.target-self.offset)) < half) & (forward > 0)
        front = float(forward[inside].min()) if np.any(inside) else c['free_range']
        if front >= c.get('redecide_front', 1.0):
            return False
        for_fork = fork_lanes is not None
        approved, info = self.check_extension(forward, left, remaining, limit if for_fork else None, for_fork)
        info['front'] = round(front, 2)
        if not approved:
            self.debug['redecide_blocked'] = info
            return False
        self.span, self.target, self.move_angle, self.back = 2, target, info['angle'], False
        if for_fork:
            self.short, self.requested_move = False, True
            self.plan = self.debug['plan'] = info
        self.debug['redecide'] = info
        return True

    def try_reverse(self, points, fork_lanes, limit):
        """진행 중인 이동의 도착 차로가 표지판 목적 차로의 반대쪽이면, 경로가 비는 순간 목적 차로 쪽으로 되돌림 (11/17)
        - ext_1117_0144 48.8 s: 가운데 차로 도착 직후 기둥(0.92 m 앞)을 오른쪽으로 피하기 시작, 같은 순간 left 확정 → 반대 길
        - ext_1117_0135 41.9~56.4 s: 왼쪽 끝 차로 앞 장애물을 오른쪽으로 피하는 중 left 확정. 가운데 차로 중앙까지 다 간 뒤에야
          돌아오려니 기둥 0.45 m 앞 → 경로 없음. 장애물을 지나친 순간 바로 돌아오면 거리가 남음
        fork_lanes: 이동 시작 차로에서 목적 차로까지 칸 수(+ 왼쪽), 모르면 None
        방법: 기준을 도착 예정 차로로 옮겨(index·offset) 거기서 목적 차로까지의 새 이동으로 바꿈. 지금 자세에서 남은 횡거리로
        경로를 그려 여유 request_plan_margin 이상일 때만. 도착 뒤 목적 차로가 request_reverse_tail(m)만큼 비어 있어야 함
        (방금 피한 장애물이 있는 차로로 돌아가 다시 피하는 반복 방지). 입구까지 이동을 끝낼 거리도 확보."""
        c = self.c
        if not (c.get('request_reverse', False) and c.get('request_plan', False)) or fork_lanes is None:
            return False
        lanes = int(fork_lanes)-self.direction*self.span      # 도착 예정 차로에서 목적 차로까지 (+ 왼쪽)
        if lanes == 0 or lanes*self.direction > 0 or abs(lanes) > 2:
            return False
        direction = 1 if lanes > 0 else -1
        offset = self.offset-self.target                      # 도착 예정 차로 중앙 기준 지금 위치
        target = lanes*c['lane_width']
        remaining = target-offset
        if direction*remaining <= 0:
            return False                                      # 이미 목적 차로 중앙을 지난 쪽 (시작 때 치우침): 다음 주기에 다시
        base = self.index+self.direction*self.span
        walls = self.walls
        kept = self.drop_walls(points, self.heading, remaining, base+lanes)
        self.walls = walls
        forward, left = to_road(kept, self.heading)
        angle = float(sorted(c.get('request_angles', [45.0]))[0])
        radius = self.robot['wheelbase']/math.tan(self.robot['max_steering'])
        _, reach = arc_path(remaining, -self.heading, math.radians(angle), radius, c.get('arc_margin', .03))
        length = reach*c.get('request_plan_stretch', 1.1)
        tail = c.get('request_reverse_tail', 1.1)
        if limit is not None:
            tail = max(c.get('request_plan_tail', .3), min(tail, limit-length))
        gap = self.path_gap(forward, left, remaining, angle, -self.heading, tail)
        info = dict(span=abs(lanes), angle=angle, length=round(float(length), 2), gap=round(min(gap, 9.9), 3),
                    limit=None if limit is None else round(float(limit), 2), tail=round(float(tail), 2), reversed=True,
                    fits=bool(limit is None or length <= limit))
        if not info['fits'] or gap < c.get('request_plan_margin', .04):
            self.debug['reverse_blocked'] = info
            return False
        self.index, self.offset = base, offset
        self.direction, self.span, self.target, self.move_angle = direction, abs(lanes), target, angle
        self.back, self.short, self.requested_move = False, False, True
        self.travel, self.move_clock = 0.0, self.clock
        self.seen_lane, self.seen_count = None, 0
        self.plan = self.debug['plan'] = info
        return True

    def path_gap(self, forward, left, target, angle, yaw0, tail, margin=None):
        """원호–직선–원호 경로(목표 횡거리 target m, 각도 angle °)를 따라갈 때 차체와 라이다 점 사이 최소 여유(m)"""
        robot = self.robot
        points = np.column_stack((forward, left)) if len(forward) else np.zeros((0, 2))
        radius = robot['wheelbase']/math.tan(robot['max_steering'])
        if margin is None:
            margin = self.c.get('arc_margin', .03)
        path, _ = arc_path(target, yaw0, math.radians(angle), radius, margin, tail=tail)
        body = path.copy()
        body[:, 0] += robot['wheelbase']/2*np.cos(path[:, 2])
        body[:, 1] += robot['wheelbase']/2*np.sin(path[:, 2])
        return footprint_clearance(points, body, robot)

    def plan_request(self, forward, left, direction, center, lanes, limit, yaw0):
        """표지판 요청 이동의 칸 수·원호 각도 선택 (11/16). 장애물 배치가 매번 달라 각도를 고정하지 않음

        후보 각도마다 원호–직선–원호 경로(arc_path)를 그려 라이다 점(도로 기준)과 차체 여유를 봄.
        - 두 칸 떨어진 목적 차로면 두 칸을 한 번에 건너는 경로부터, 안 되면 한 칸
        - 여유가 request_plan_margin(m) 이상인 각도 중, 실차 보정(request_plan_stretch배)한 전진 거리가
          남은 거리 limit(갈림길 입구까지, 모르면 None) 안에 드는 가장 완만한 각도 (완만할수록 횡이동 추정 오차·쏠림이 작음)
        - limit 안에 드는 각도가 없으면 여유 있는 각도 중 가장 가파른 것(가장 짧음)
        - 여유 있는 경로가 하나도 없으면 None → 출발 안 함 (차로 유지하며 다음 주기에 다시 계획)
        - 출발 지연 (유경 11/16, run_1116_2037·2039): 두 칸 경로가 지금은 막혔어도 request_delay_max(m)까지
          request_delay_step씩 더 직진한 뒤 꺾는 경로가 열리면 한 칸으로 내려가지 않고 기다림 (반환 wait=True → 출발 안 함).
          실차: 목적 차로 0.6~1.0 m 앞 상자 때문에 두 칸이 막혀 한 칸만 이동 → 섬 표지판 앞 가운데 차로에서 반대쪽으로 회피.
          0.3 m쯤 더 가서 꺾으면 상자 뒤로 한 번에 건널 수 있었음. 끄기: request_delay_max 0
        yaw0: 도로 방향 기준 차 방향(rad, + 왼쪽)
        경로는 뒷축 기준, 차체 중심은 wheelbase/2 앞. 라이다에 가려 안 보이는 곳은 이동 중 clear_steering이 계속 확인"""
        c, robot = self.c, self.robot
        points = np.column_stack((forward, left)) if len(forward) else np.zeros((0, 2))
        radius = robot['wheelbase']/math.tan(robot['max_steering'])
        stretch = c.get('request_plan_stretch', 1.1)
        tried = []
        for span in range(lanes, 0, -1):
            target = direction*span*c['lane_width']+center
            clear = []
            for angle in sorted(c.get('request_angles', [36.0])):
                path, reach = arc_path(target, yaw0, math.radians(angle), radius, c.get('arc_margin', .03),
                                       tail=c.get('request_plan_tail', .3))
                body = path.copy()
                body[:, 0] += robot['wheelbase']/2*np.cos(path[:, 2])
                body[:, 1] += robot['wheelbase']/2*np.sin(path[:, 2])
                gap = footprint_clearance(points, body, robot)
                tried.append((span, angle, round(reach*stretch, 2), round(min(gap, 9.9), 3)))
                if gap >= c.get('request_plan_margin', .04):
                    clear.append((angle, reach*stretch))
            if not clear and span > 1:
                step = c.get('request_delay_step', .1)
                for k in range(1, int(round(c.get('request_delay_max', 0.)/step))+1):
                    delay = k*step
                    for angle in sorted(c.get('request_angles', [36.0])):
                        # 11/16 혁준: 지연 뒤 이동이 갈림길 입구(limit)를 넘으면 기다려도 못 건넘 → 대기 후보에서 제외
                        # 꺾기 전 직진 구간은 차로 유지(도로 방향)라 방향 0 그대로 계산
                        path, reach = arc_path(target, 0.0, math.radians(angle), radius, c.get('arc_margin', .03),
                                               tail=c.get('request_plan_tail', .3))
                        if limit is not None and delay+reach*stretch > limit:
                            continue
                        count = int(round(delay/.02))      # 꺾기 전 직진 구간 (2 cm 간격)
                        lead = np.column_stack((np.arange(count)*.02, np.zeros(count), np.zeros(count)))
                        path = np.vstack((lead, path+[delay, 0.0, 0.0]))
                        body = path.copy()
                        body[:, 0] += robot['wheelbase']/2*np.cos(path[:, 2])
                        body[:, 1] += robot['wheelbase']/2*np.sin(path[:, 2])
                        gap = footprint_clearance(points, body, robot)
                        if gap >= c.get('request_plan_margin', .04):
                            tried.append((span, angle, round(delay+reach*stretch, 2), round(min(gap, 9.9), 3)))
                            return dict(wait=True, span=span, angle=float(angle), delay=round(delay, 2),
                                        length=round(float(delay+reach*stretch), 2),
                                        limit=None if limit is None else round(float(limit), 2), tried=tried)
            if not clear:
                continue
            fits = [item for item in clear if limit is None or item[1] <= limit]
            angle, length = fits[0] if fits else clear[-1]
            return dict(span=span, angle=float(angle), length=round(float(length), 2),
                        limit=None if limit is None else round(float(limit), 2), fits=bool(fits), tried=tried)
        self.debug['plan_tried'] = tried
        return None


class BackUp:
    """앞 장애물에 너무 가까이 선 채 갈 길이 없을 때, 방금 온 길을 되짚어 조금 후진한 뒤 다시 판단 (11/17)

    rev_1117_0212 90~98 s: 오른쪽 차로에 도착하니 (이동 시작 때 가려져 안 보이던) 장애물이 앞끝 0.34 m. 왼쪽으로 꺾을 공간이 없어
    정지 → 조종기로 0.3 m쯤 후진시키자 바로 통과. 한 차로 이동에는 앞 0.9 m쯤 필요
    - 라이다 시야가 앞 ±110°라 뒤가 안 보임 → 뒤가 비어 있다고 아는 범위에서만 후진. 두 방식:
      · 정렬 후진(align): 차로 안에 선 경우. 지금 차로에서 도로 방향으로 차를 바로 세우며 후진 (known_back 한도 안)
        back_1117_0249: 되짚기만 하면 복귀 원호 중간(20°+ 틀어진 자세)으로 돌아가 다음 이동을 못 시작함
      · 되짚기(retrace): 이동 중간에 선 경우. '방금 지나온 경로'(전진 거리·회전 기록)를 거꾸로 따라감
    - 시작 조건: 차선 변경이 속도 0을 내거나 비상 정지로 backup_wait(s) 넘게 서 있고, 차체 앞 통로의 가장 가까운 점이
      뒷축 앞 backup_front(m) 안. 한 자리에서 backup_max번까지 (backup_reset_distance 전진하면 횟수 초기화)
    - 규정집 v3에 후진 금지 조항은 없음. 심사위원 재량(부칙 나) 여지가 있어 backup: False로 끌 수 있게 둠"""

    def __init__(self, config, robot):
        self.c, self.robot = config['lane_change'], robot
        self.trail = []        # 최근 전진 경로 [거리(m), 회전(rad, + 왼쪽)] 구간 목록. 끝이 가장 최근
        self.active = False
        self.left = 0.0        # 남은 후진 거리(m)
        self.elapsed = 0.0
        self.stuck = 0.0       # 서 있은 시간(s)
        self.count = 0         # 이 자리에서 후진한 횟수
        self.progress = 0.0    # 마지막 후진 뒤 전진 거리(m)
        self.known_back = 0.0  # 지금 차로에서 뒷축 뒤로 비어 있다고 아는 거리(m): 곧게 후진해도 되는 한도
        self.mode = 'retrace'  # 'align' = 지금 차로에서 도로 방향으로 정렬하며 후진, 'retrace' = 지나온 경로 되짚기
        self.was_active = False
        self.debug = {}

    def record(self, velocity, yaw_rate, dt):
        """전진 경로 기록 (매 주기). 구간 길이 backup_bin(m), 최근 backup_keep(m)만 보관"""
        if self.active or velocity <= 0.02:
            return
        step = velocity*dt
        if self.trail and self.trail[-1][0] < self.c.get('backup_bin', .03):
            self.trail[-1][0] += step
            self.trail[-1][1] += yaw_rate*dt
        else:
            self.trail.append([step, yaw_rate*dt])
        while sum(d for d, _ in self.trail) > self.c.get('backup_keep', .6) and len(self.trail) > 1:
            self.trail.pop(0)
        self.progress += step
        self.known_back = min(1.0, self.known_back+step)     # 방금 지나온 자리
        if self.progress > self.c.get('backup_reset_distance', 1.0):
            self.count = 0

    def observe(self, change, lane_change, points, velocity, dt, blocked=False, enabled=True, heading=None):
        """차선 변경 결과(change)를 보고 후진을 시작할지 판단. 시작하면 (속도 < 0, 조향), 아니면 None
        points: 뒷축 기준 라이다 점. blocked: 직전 주기에 전방 비상 정지였는지. enabled: 차선 변경이 켜진 구간인지
        차선 변경이 속도 0을 냈거나, 차선 변경 구간에서 비상 정지로 서 있는 경우 (차선이 안 잡혀 판단이 꺼진 채 선 경우 포함)"""
        c = self.c
        if self.was_active and not lane_change.active:
            # 차선 변경이 방금 끝남: 옆으로 들어온 차로라 바로 뒤는 지나온 자리가 아님. 다만 출발 때 그 차로가 뒷축 앞
            # side_ahead(0.45 m)부터 비어 있음을 확인했으므로, 이동 거리에서 backup_known_offset(m)을 뺀 만큼은 뒤가 빈 것으로 봄
            self.known_back = max(0.0, lane_change.travel-c.get('backup_known_offset', .52))
        self.was_active = lane_change.active
        stopped = enabled and abs(velocity) < .03 and ((change is not None and change[0] == 0.0) or blocked)
        self.stuck = self.stuck+dt if (stopped and c.get('backup', False)) else 0.0
        if self.stuck < c.get('backup_wait', 1.0):
            return None
        points = np.asarray(points, dtype=float).reshape(-1, 2)
        half = self.robot['width']/2+c.get('backup_side', .15)
        ahead = points[(points[:, 0] > 0) & (np.abs(points[:, 1]) < half)]
        front = float(ahead[:, 0].min()) if len(ahead) else 9.0
        have = sum(d for d, _ in self.trail)
        distance = c.get('backup_distance', .3)
        # 방식: 차로 안에 서 있고(이동 중간이 아님) 뒤가 빈 것을 아는 거리가 충분하면 '정렬 후진' (지금 차로에서 도로 방향으로
        # 차를 바로 세우며 후진 → 끝나면 바로 다음 이동을 시작할 수 있는 자세. 11/17 수동 후진과 같은 동작).
        # 아니면 '되짚기' (이동 중간에 선 경우 등: 지나온 경로만 안전)
        mid_move = lane_change.active and lane_change.travel > c.get('backup_mid_travel', .15)
        align = (not mid_move and heading is not None and c.get('backup_align', True) and self.known_back >= distance)
        self.debug = dict(front=round(front, 2), trail=round(have, 2), count=self.count, known_back=round(self.known_back, 2))
        if front >= c.get('backup_front', .9) or self.count >= c.get('backup_max', 2) or (not align and have < .1):
            return None
        self.mode = 'align' if align else 'retrace'
        self.active, self.left, self.elapsed = True, distance if align else min(distance, have), 0.0
        self.count, self.progress, self.stuck = self.count+1, 0.0, 0.0
        lane_change.cancel()
        self.was_active = False
        return self.step(0.0, dt, heading)

    def step(self, velocity, dt, heading=None):
        """후진 한 주기. 끝났으면 None. heading: 도로 방향 기준 차 방향(rad, + 오른쪽), 모르면 None"""
        c = self.c
        self.elapsed += dt
        moved = abs(velocity)*dt if velocity < 0 else 0.0
        self.left -= moved
        self.known_back = max(0.0, self.known_back-moved)
        if self.mode == 'align':
            if self.left <= 0 or self.elapsed > c.get('backup_seconds', 6.0):
                self.active = False
                self.trail = []          # 지나온 경로와 다른 길로 물러남 → 되짚기 기록은 버림
                return None
            # 후진 중 차를 왼쪽으로 돌리려면 바퀴는 오른쪽: 방향(+ 오른쪽)의 반대 부호로 조향
            limit = self.robot['max_steering']
            steering = 0.0 if heading is None else float(max(-limit, min(limit, -c.get('backup_align_gain', 2.5)*heading)))
            self.debug.update(left=round(self.left, 2), steering=round(steering, 3), mode='align')
            return -c.get('backup_speed', .1), steering
        while moved > 0 and self.trail:        # 되짚은 만큼 기록에서 뺌
            distance, yaw = self.trail[-1]
            if distance <= moved:
                moved -= distance
                self.trail.pop()
            else:
                self.trail[-1] = [distance-moved, yaw*(1-moved/distance)]
                moved = 0.0
        if self.left <= 0 or not self.trail or self.elapsed > c.get('backup_seconds', 6.0):
            self.active = False
            return None
        # 바로 뒤 backup_look(m) 구간의 곡률로 조향 (같은 바퀴 각으로 후진하면 같은 원호를 거꾸로 감)
        distance = yaw = 0.0
        for d, y in reversed(self.trail):
            distance, yaw = distance+d, yaw+y
            if distance >= c.get('backup_look', .06):
                break
        curvature = yaw/distance if distance > 1e-4 else 0.0
        limit = self.robot['max_steering']
        steering = float(max(-limit, min(limit, math.atan(self.robot['wheelbase']*curvature))))
        self.debug.update(left=round(self.left, 2), steering=round(steering, 3), mode='retrace')
        return -c.get('backup_speed', .1), steering
