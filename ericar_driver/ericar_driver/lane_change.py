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

from .lidar import body_points, clear_steering, pursuit_steering, to_body, to_road


def free_distances(forward, left, center, robot, config):
    """라이다 점(도로 기준)으로 지금 차로(0)·왼쪽(+1)·오른쪽(-1) 차로가 앞으로 몇 m 비어 있는지.

    지금 차로 = 차체가 지나갈 폭(차 기준). 옆 차로 = 차로 중앙(center ± lane_width) 기준 차폭 통로.
    차 옆이나 바로 뒤에 점이 있으면 0 → 그쪽으로 못 감. 점이 없으면 라이다 범위(3 m).
    바로 뒤 = 차체 뒤끝(뒷축 뒤 length − front_bumper_x = 7.2 cm) + side_behind_margin (소민 11/13)
    예전: 뒷축 뒤 차 길이(33 cm)까지 → 3차로 시작 지점의 가로 벽 등 이미 지나친 것까지 막힘으로 봄 (합성 시험)
    """
    c = config
    half = robot['width']/2+c['margin']
    behind = robot['length']-robot.get('front_bumper_x', robot['length'])+c.get('side_behind_margin', .05)
    lanes = {0: 0.0, 1: center+c['lane_width'], -1: center-c['lane_width']}
    free = {}
    for direction, middle in lanes.items():
        inside = (np.abs(left-middle) < half) & (forward > (0 if direction == 0 else -behind))
        free[direction] = float(max(0.0, forward[inside].min())) if np.any(inside) else c['free_range']
    return free


# 선 색 조합 (왼쪽, 오른쪽) → 차로 번호 (0=오른쪽, 1=가운데, 2=왼쪽). 노랑 = 3차선 바깥 경계, 흰 = 차로 구분 점선
LANE_BY_COLORS = {('yellow', 'white'): 2, ('white', 'white'): 1, ('white', 'yellow'): 0}


class LaneChange:
    """라이다 담당: 전방 차량 감지 → 빈 옆 차로 선택 → 한 차로 이동 → 차선 복귀.

    offset은 카메라로 측정한 도로 방향 기준 횡이동 거리. odom의 속도만 적분한다.
    지도 좌표나 장애물 차량의 배치 좌표는 사용하지 않는다.
    """
    def __init__(self, config, robot):
        self.c, self.robot = config['lane_change'], robot
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

    def choose_direction(self, free, lane, requested=0, preferred=0):
        """왼쪽(+1)·오른쪽(-1) 중 갈 차로. 없으면 0

        - 차로 범위 밖·노란 경계 방향 제외. 일반 회피는 실선 방향도 제외
        - 옆 차로가 min_free_distance 이상 비어 있고, 지금 차로보다 free_gain 이상 더 비어 있어야 함
        - 둘 다 되면 더 멀리 비어 있는 쪽. 같으면 왼쪽 (기존 동작)
        - preferred(이미 읽은 갈림길 표지판 방향) 쪽이 조건을 만족하면 그쪽 먼저 (반대로 피하면 다시 건너와야 함, 소민)
        """
        c = self.c
        rejected = self.debug['rejected'] = {}
        best, best_free = 0, free[0]+c['free_gain']
        for direction, side in ((1, 'left'), (-1, 'right')):
            if requested and direction != requested:
                rejected[side] = 'opposite_to_request'
                continue
            if not 0 <= self.index+direction < c['lane_count']:
                rejected[side] = 'outside_lane_count'
                continue
            # YOLO 요청도 노란 바깥선·교통섬 경계 횡단 금지. 점선 차로에서 정렬 필요
            if self.trusted_colors(lane).get(side) == 'yellow':
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

    def steer(self, points, dt):
        """한 차로 이동 중: 목표 횡이동(target)까지 pure pursuit, 막히면 가까운 안전 조향"""
        c = self.c
        # 이동 중 벽 제거: 목적 차로 중앙(차 기준) = target − offset, 목적 차로 번호 = index + direction
        points = self.drop_walls(points, self.heading, self.target-self.offset, self.index+self.direction)
        error = self.target-self.offset
        if getattr(self, 'short', False) or c.get('steer_mode', 'arc') == 'arc':
            # 짧은 S자(앞 장애물 short_front 안)도 원호–직선–원호로, 각도만 short_arc_angle로 크게 (소민 11/16)
            desired = self.arc_steering(error)
        else:
            # 보는 거리: 시작은 lookahead(표지판 이동은 request_lookahead)로 세게, 목표에 가까워질수록 lookahead_end까지 늘려
            # 도착 전에 미리 핸들을 풂 (소민 11/15 실차: 고정 0.40이면 비스듬히 들어가 반대로 크게 꺾으며 정렬, 앞으로 많이 감)
            start = c.get('request_lookahead', c['lookahead']) if getattr(self, 'requested_move', False) else c['lookahead']
            progress = min(1.0, abs(self.offset)/max(abs(self.target), 1e-6))
            lookahead = start+(max(start, c.get('lookahead_end', start))-start)*progress
            x, y = to_body(lookahead, error, self.heading)
            desired = pursuit_steering(x, y, self.robot)
        steering, safe = clear_steering(points, desired, self.robot, c['safety_horizon'], c['margin'])
        self.debug.update(desired=desired, safe=safe, error=error, walls=self.walls)
        short = getattr(self, 'short', False)
        self.debug['reason'] = ('changing_short' if short else 'changing') if safe else 'no_safe_steering'
        # 안전한 조향이 하나도 없을 때만 그 자리에서 기다린다 (정적 장애물은 비켜 갈 길이 생기지 않으면 실패)
        speed = c.get('short_speed', c['speed']) if short else c['speed']
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
        need = radius*(1-math.cos(abs(yaw)))+c.get('arc_margin', .03)
        if side*error <= need:
            self.back = True
        # 짧은 S자: 앞 장애물이 가까워 시작한 이동은 더 크게 틀어 짧게 옮김 (소민 11/16, 장애물 사이 약 1 m에서 복귀)
        angle = c.get('short_arc_angle', 60.0) if getattr(self, 'short', False) else c.get('arc_angle', 30.0)
        goal = 0.0 if self.back else side*math.radians(angle)
        limit = self.robot['max_steering']
        return float(max(-limit, min(limit, c.get('arc_gain', 2.5)*(goal-yaw))))

    def start_heading(self, camera):
        """새 이동의 시작 방향(도로 방향 기준 차 방향, rad)

        - 직전 이동이 chain_seconds(s) 안에 끝났으면 그 이동에서 IMU로 적분해 온 방향(self.heading)을 이어 씀
          (11/15 fix2_1115_2017 156.9 s: 0→1 이동 직후 1→2 이동이 카메라 한 프레임 방향 −0.48 rad로 시작, 실제 차는 직진 자세
           → 횡이동 과대 추정 + 복귀 조향 0.48 rad 과다 → 도로보다 오른쪽 13~16°로 끝나 교통섬 직진·표지판 추돌)
        - 그 외에는 최근 heading_frames 프레임 카메라 방향의 중앙값 (한 프레임 튐 제외)"""
        if self.clock-self.ended_clock <= self.c.get('chain_seconds', 2.0):
            return self.heading
        log = sorted(self.heading_log)
        return log[len(log)//2] if log else camera

    def camera_finished(self, lane):
        """카메라로 차선 변경 끝 판단 (소민): 반 넘게 옮겼고, 선 색이 목적 차로 + 차로 중앙 근처·나란함
        추측 항법(odom 적분)은 회전 오차(11/9 실측 1.07배)로 덜·더 감 → 카메라가 확인되면 바로 차선 주행에 넘김"""
        c = self.c
        return (self.seen_lane == self.index+self.direction and self.seen_count >= 2 and lane.get('valid') and
                abs(self.offset) > c['lane_width']*c.get('camera_finish_progress', .6) and
                abs(lane['error']) < c.get('camera_finish_error', .06) and
                abs(lane['heading']) < c.get('camera_finish_heading', .12))

    def camera_correct(self, lane):
        """목적 차로가 카메라에 확인되면 추측 항법 위치·방향을 카메라 값으로 바로잡음 (소민 11/12)
        추측 항법은 odom 속도·IMU 회전 오차만큼 한 번에 13~19 cm 더 감 (합성 시험: odom 속도 0.8배)
        → 두 번 연속 변경(0→1→2)에서 오차가 쌓여 노란 바깥선 이탈 (11/12 실차)
        조건: 선 색이 목적 차로를 camera_fix_frames번 연속 + 그 차로 중앙이 차 기준 반 차로 안 (걸쳐 있을 때 오인 방지)"""
        c = self.c
        if not (lane.get('valid') and self.seen_lane == self.index+self.direction and
                self.seen_count >= c.get('camera_fix_frames', 2)):
            return False
        heading = lane['heading']
        center = lane['error']*math.cos(heading)-lane.get('forward', .65)*math.sin(heading)
        if abs(center) > c['lane_width']*c.get('camera_fix_range', .5):
            return False
        self.offset, self.heading = self.target-center, heading
        self.debug['camera_fix'] = round(center, 3)
        return True

    def update(self, lane, scan, velocity, yaw_rate, dt, enabled, requested=0, preferred=0, avoid_only=False):
        self.waiting = False
        self.clock += dt
        points = body_points(scan, self.robot)
        self.heading -= yaw_rate*dt
        self.fork_running, self.fork_dir = bool(preferred), int(preferred)
        self.observe_lane(lane)
        if self.active:
            # lateral_scale: 추측 항법이 실제 횡이동보다 덜 셈 (11/15 fix3_1115_2113: 두 번 모두 0.09~0.20 m 더 감, 11/12 소민 13~19 cm)
            self.offset -= velocity*math.sin(self.heading)*dt*self.c.get('lateral_scale', 1.0)
            self.travel += abs(velocity)*dt
            self.camera_correct(lane)
            error = self.target-self.offset
            if self.c.get('steer_mode', 'arc') == 'arc':
                # 복귀 원호가 끝나 도로 방향과 나란해지면 종료 (남은 횡오차는 차선 주행이 처리)
                done = self.back and abs(self.heading) < self.c.get('arc_finish_heading', .05)
            else:
                done = abs(error) < self.c['finish_error'] and abs(self.heading) < self.c['finish_heading']
            done = done or self.camera_finished(lane)
            # 지나침 방지: 목표보다 overshoot(m) 넘게 갔으면 바로 차선 주행으로 (바깥 노란 선 넘음 방지, 소민)
            done = done or abs(self.offset) > abs(self.target)+self.c.get('overshoot', .08)
            if done or self.travel >= self.c['max_distance']:
                # 이동 거리 초과: 영구 정지 대신 차선 주행으로 돌아간다. 반 차로 넘게 갔으면 옮긴 것으로 본다.
                moved = done or abs(self.offset) > self.c['lane_width']/2
                self.active = False
                self.index += self.direction if moved else 0
                self.seen_lane, self.seen_count = None, 0   # 이동 중 본 차로 기억은 다음 판단에 안 씀
                self.ended_clock = self.clock
                return None
            return self.steer(points, dt)
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
        heading = lane['heading']
        camera_center = lane['error']*math.cos(heading)-lane.get('forward', .65)*math.sin(heading)
        points = self.drop_walls(points, heading, camera_center, self.index, self.trusted_colors(lane))
        forward, left = to_road(points, heading)
        # 지금 차로 중앙이 차 기준 왼쪽으로 몇 m인지 (카메라 차선). 갈림길 요청은 차 기준
        center = 0.0 if requested else camera_center
        free = free_distances(forward, left, center, self.robot, self.c)
        # 옆 차로는 최근 choice_window 동안의 최솟값 사용: 가는 물체(의자 다리 등)가 한 스캔 빠지면 빈 차로로 보임 (11/14 lap3)
        window = self.c.get('choice_window', 0.0)
        self.free_log = [(t, f) for t, f in self.free_log if self.clock-t <= window]+[(self.clock, dict(free))]
        for side in (1, -1):
            free[side] = min(f[side] for _, f in self.free_log)
        self.debug = dict(free=free, index=self.index, walls=self.walls)
        # 차체가 진행할 통로 안의 전방 장애물만 확인한다.
        if free[0] >= self.c['trigger_distance'] and not requested:
            self.debug['reason'] = 'front_clear'
            return None
        direction = self.choose_direction(free, lane, requested, preferred)
        self.debug['choice'] = direction
        if direction == 0 and not requested and all(
                self.debug['rejected'].get(side) in ('outside_lane_count', 'yellow_boundary') for side in ('left', 'right')):
            # 양쪽 다 차로 자체가 없음 (갈림길 뒤 일반 차로·코너): 차선 변경으로 풀 수 없는 장애물 → 차선 주행·코너 유지에 맡김
            # 예전: 저속 접근 모드가 3번 코너 조향을 대신해 덜 돌고 바깥 라바콘에 닿음 (11/14 lap3 264~273 s)
            self.debug['reason'] = 'no_side_lane'
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
        self.active, self.direction = True, direction; self.requested_move = bool(requested)
        # 앞 장애물이 short_front(m)보다 가까우면 짧은 S자 (arc_steering 각도 short_arc_angle). 끄려면 short_front: 0
        self.short = free[0] < self.c.get('short_front', 1.0)
        # 이전에 본 차로 기억 비움 (11/15 main_1115_1904 180.07 s: 0→1 이동 중 비스듬히 본 '2번 차로' 기억이 남아
        # 1→2 이동이 시작한 주기에 camera_correct → overshoot 종료, 가운데 차로인데 index 2 → 교통섬 직진·표지판 추돌)
        self.seen_lane, self.seen_count = None, 0
        # 코너 출구에서도 차량 방향이 아닌 도로 방향으로 한 차로 이동한다.
        self.heading, self.offset, self.travel = self.start_heading(heading), 0.0, 0.0
        self.back = False
        # 차가 크게 틀어진 상태에서 목표가 한 차로 넘게 튀지 않도록 center는 반 차로 안으로 제한 (소민)
        center = max(-self.c['lane_width']/2, min(self.c['lane_width']/2, center))
        self.target = direction*self.c['lane_width']+center
        return self.update(lane, scan, 0.0, 0.0, dt, enabled)
