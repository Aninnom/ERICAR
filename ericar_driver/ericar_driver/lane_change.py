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
    차 옆이나 바로 뒤(차 길이 이내)에 점이 있으면 0 → 그쪽으로 못 감. 점이 없으면 라이다 범위(3 m).
    """
    c = config
    half = robot['width']/2+c['margin']
    lanes = {0: 0.0, 1: center+c['lane_width'], -1: center-c['lane_width']}
    free = {}
    for direction, middle in lanes.items():
        inside = (np.abs(left-middle) < half) & (forward > (0 if direction == 0 else -robot['length']))
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
        self.debug = {}

    def observe_lane(self, lane):
        """선 색으로 본 차로가 새 카메라 프레임 index_confirm_frames번 연속 같으면 index 보정.
        지금까지는 initial_lane에서 차선 변경 횟수만 셈 → 한 번 틀리면 계속 틀림
        노랑/노랑(일반 구간)·선 하나·색 정보 없음 → 무시 (기존 index 유지)"""
        stamp = lane.get('stamp')
        if stamp is not None and stamp == self.seen_stamp:
            return
        self.seen_stamp = stamp
        colors = lane.get('colors', {}) if lane.get('valid') else {}
        seen = LANE_BY_COLORS.get((colors.get('left'), colors.get('right')))
        if seen is None or seen >= self.c['lane_count']:
            return
        if seen == self.seen_lane:
            self.seen_count += 1
        else:
            self.seen_lane, self.seen_count = seen, 1
        # 차선 변경 중에는 index 그대로 (끝날 때 direction만큼 바꿈). 본 차로는 카메라 종료 판단에만 씀
        if not self.active and self.seen_count >= self.c['index_confirm_frames']:
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
            if lane.get('colors', {}).get(side) == 'yellow':
                rejected[side] = 'yellow_boundary'
                continue
            if not requested and lane.get('solid', {}).get(side, False):
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

    def steer(self, points, dt):
        """한 차로 이동 중: 목표 횡이동(target)까지 pure pursuit, 막히면 가까운 안전 조향"""
        c = self.c
        error = self.target-self.offset
        x, y = to_body(c['lookahead'], error, self.heading)
        desired = pursuit_steering(x, y, self.robot)
        steering, safe = clear_steering(points, desired, self.robot, c['safety_horizon'], c['margin'])
        self.debug.update(desired=desired, safe=safe, error=error)
        self.debug['reason'] = 'changing' if safe else 'no_safe_steering'
        # 안전한 조향이 하나도 없을 때만 그 자리에서 기다린다 (정적 장애물은 비켜 갈 길이 생기지 않으면 실패)
        return (c['speed'] if safe else 0.0), steering

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
        points = body_points(scan, self.robot)
        self.heading -= yaw_rate*dt
        self.observe_lane(lane)
        if self.active:
            self.offset -= velocity*math.sin(self.heading)*dt
            self.travel += abs(velocity)*dt
            self.camera_correct(lane)
            error = self.target-self.offset
            done = abs(error) < self.c['finish_error'] and abs(self.heading) < self.c['finish_heading']
            done = done or self.camera_finished(lane)
            # 지나침 방지: 목표보다 overshoot(m) 넘게 갔으면 바로 차선 주행으로 (바깥 노란 선 넘음 방지, 소민)
            done = done or abs(self.offset) > abs(self.target)+self.c.get('overshoot', .08)
            if done or self.travel >= self.c['max_distance']:
                # 이동 거리 초과: 영구 정지 대신 차선 주행으로 돌아간다. 반 차로 넘게 갔으면 옮긴 것으로 본다.
                moved = done or abs(self.offset) > self.c['lane_width']/2
                self.active = False
                self.index += self.direction if moved else 0
                return None
            return self.steer(points, dt)
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
        forward, left = to_road(points, heading)
        # 지금 차로 중앙이 차 기준 왼쪽으로 몇 m인지 (카메라 차선). 갈림길 요청은 차 기준
        center = 0.0 if requested else lane['error']*math.cos(heading)-lane.get('forward', .65)*math.sin(heading)
        free = free_distances(forward, left, center, self.robot, self.c)
        self.debug = dict(free=free, index=self.index)
        # 차체가 진행할 통로 안의 전방 장애물만 확인한다.
        if free[0] >= self.c['trigger_distance'] and not requested:
            self.debug['reason'] = 'front_clear'
            return None
        direction = self.choose_direction(free, lane, requested, preferred)
        self.debug['choice'] = direction
        if direction == 0 and requested:
            # 목표 쪽 막힘: 목표는 유지하고 일반 장애물 회피로 우회 가능
            rejected = dict(self.debug['rejected'])
            result = self.update(lane, scan, 0.0, 0.0, dt, enabled, preferred=preferred)
            self.debug['request_rejected'] = rejected
            return result
        if direction == 0 and avoid_only:
            # 갈 차로 없음: 또 대기하면 멈춤 방지가 반복 → 차선 주행에 맡김 (비상 정지 → 서행 비켜 가기)
            self.debug['reason'] = 'off_no_lane'
            return None
        if direction == 0:
            # 정적 장애물은 기다려도 비지 않음 → 앞 장애물 stop_distance 전까지는 지금 차로를 천천히 따라가며 다시 판단
            # (옆 차로를 막은 차를 지나치면 그쪽이 열림). 임의 배치 시험에서 옆 차 옆에 선 채 영구 정지하던 문제
            if not requested and free[0] > self.c['stop_distance']:
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
        self.active, self.direction = True, direction
        # 코너 출구에서도 차량 방향이 아닌 도로 방향으로 한 차로 이동한다.
        self.heading, self.offset, self.travel = heading, 0.0, 0.0
        # 차가 크게 틀어진 상태에서 목표가 한 차로 넘게 튀지 않도록 center는 반 차로 안으로 제한 (소민)
        center = max(-self.c['lane_width']/2, min(self.c['lane_width']/2, center))
        self.target = direction*self.c['lane_width']+center
        return self.update(lane, scan, 0.0, 0.0, dt, enabled)
