"""라바콘: 진입 판단 + 라이다로 좌우 라바콘 사이(통로 중앙) 통과

참고: young4240/scale_car (2024 국민대 대회) ref_src/final_code_file/scripts/all_lidar2.py 의 Is_Labacon
  - 진입 ("Labacon Start"): 앞쪽 좌우에 콘이 있고, 가장 가까운 좌·우 콘 간격이 정해진 범위
  - 주행: 가장 가까운 좌·우 콘의 가운데(y_ref)를 목표로 조향
  - 종료 ("Labacon End"): 앞에 콘이 더 없음
참고 코드는 노드 2개(라이다 판단 → String 토픽 → 제어)였지만, 우리는 노드 하나라 driver.py가
ConeDriving.update를 부르고 (속도, 조향) 또는 None(차선 주행)을 받는다.

좌표: 차체 기준(원점 = 뒷바퀴 축), 앞 x+, 왼쪽 y+ (REP-103).
도로 기준: (앞 거리 forward, 왼쪽 거리 left). heading = 차체에서 본 콘 줄(도로) 방향, + 왼쪽.
"""
import math
import numpy as np

from .lidar import body_points, clear_steering, pursuit_steering, to_body, to_road


def find_cones(scan, robot):
    """라이다 좁은 군집(폭 small_object_min~max_width) 중심 = 라바콘 후보. 차체 좌표 N×2"""
    return body_points({'points': scan.get('narrow_centers', [])}, robot)


def corridor_line(forward, left, center, width):
    """가까운 콘부터 차례로 왼쪽(+1)/오른쪽(-1) 줄에 배정하고 통로 중앙선을 이어 간다.

    기준 = 직전 콘까지 만든 중앙선을 직선으로 연장한 선. 배정한 콘에서 간격 절반만큼 안쪽을 새 중앙으로 삼는다.
    차 정면 직선 하나로 나누면 통로가 휠 때 먼 콘의 좌우가 바뀌어(11/10 합성 시험) 바깥으로 지나간다.
    반환 (콘별 줄 배열, 중앙선 앞 거리 목록, 중앙선 왼쪽 거리 목록). 중앙선 첫 점 = (0, center)
    """
    sides = np.zeros(len(forward), dtype=int)
    path_f, path_l, slope = [0.0], [center], 0.0
    for i in np.argsort(forward):
        predicted = path_l[-1]+slope*(forward[i]-path_f[-1])
        sides[i] = 1 if left[i] >= predicted else -1
        middle = (predicted+left[i]-sides[i]*width/2)/2
        if forward[i]-path_f[-1] > .15:
            slope = float(np.clip((middle-path_l[-1])/(forward[i]-path_f[-1]), -.5, .5))
            path_f.append(float(forward[i]))
            path_l.append(float(middle))
        else:
            path_l[-1] = (path_l[-1]+middle)/2   # 거의 같은 앞 거리(좌우 쌍)는 한 점으로
    return sides, path_f, path_l


def gate_edges(forward, left, sides, window):
    """좌·우 각각 가장 가까운(앞 거리) 콘부터 window 안의 콘 중 통로 쪽으로 가장 안쪽 콘의 왼쪽 거리.

    sides: 콘별 줄 (+1 왼쪽, -1 오른쪽). 없으면 None. 반환 (왼쪽 경계, 오른쪽 경계)
    """
    edges = []
    for side in (1, -1):
        mask = sides == side
        if not np.any(mask):
            edges.append(None)
            continue
        f, l = forward[mask], left[mask]
        near = f <= f.min()+window
        edges.append(float(l[near].min() if side > 0 else l[near].max()))
    return edges


def row_heading(forward, left, center, minimum_span=0.3):
    """같은 줄 콘 2개 이상(앞뒤 간격 minimum_span 이상)이면 직선 맞춤 → 줄 방향(rad). 양쪽 평균, 없으면 None"""
    angles = []
    for side in (1, -1):
        mask = (left-center)*side > 0
        if np.count_nonzero(mask) >= 2 and np.ptp(forward[mask]) >= minimum_span:
            angles.append(math.atan(np.polyfit(forward[mask], left[mask], 1)[0]))
    return float(np.mean(angles)) if angles else None


class ConeDriving:
    """라바콘 담당: 대기(진입 판단) → active(콘 사이 통과) → done(종료 후 다시 켜지 않음)"""
    def __init__(self, config, robot):
        self.c, self.robot = config['lidar'], robot
        self.active = self.done = False
        self.heading = 0.0   # 콘 줄 방향 (차체 기준, + 왼쪽)
        self.center = 0.0    # 통로 중앙이 차 기준 왼쪽으로 몇 m (도로 기준)
        self.width = self.c['cone_gate_width']   # 좌·우 콘 간격 (진입 시 측정값으로 교체)
        self.remaining = self.travel = self.max_travel = 0.0
        self.confirm = 0
        self.debug = {}

    # ---------- 1. 진입 판단 (참고 코드 Is_Labacon) ----------
    def start_check(self, cones, lane):
        """라바콘 구간 시작이면 (콘 줄 방향, 통로 중앙, 좌우 간격, 첫 콘 앞 거리), 아니면 None

        1) 앞 cone_start_range 안에 콘 cone_start_min_cones개 이상
        2) 차 기준 왼쪽·오른쪽에 콘이 각각 있음 (콘 사이로 지나가는 구간)
        3) 가장 가까운 좌·우 콘 간격이 cone_gate_min ~ cone_gate_max (차가 지나갈 수 있는 통로)
        콘 줄 방향은 같은 줄 콘 직선 맞춤 → 없으면 카메라 차로 방향 → 없으면 차 정면.
        진입 직전 코너 곡선이나 콘에 가린 차선보다 콘 줄이 실제 통로 방향에 가깝다.
        """
        c = self.c
        ahead = (cones[:, 0] > 0) & (cones[:, 0] < c['cone_start_range'])
        cones = cones[ahead]
        if len(cones) < c['cone_start_min_cones']:
            return None
        heading = lane['heading'] if lane.get('valid') and abs(lane['heading']) < .35 else 0.0
        forward, left = to_road(cones, heading)
        rows = row_heading(forward, left, 0.0)
        if rows is not None:
            heading = float(np.clip(heading+rows, -.35, .35))
            forward, left = to_road(cones, heading)
        sides = np.where(left >= 0, 1, -1)   # 진입 전에는 차 정면 기준으로 좌우를 나눔
        left_edge, right_edge = gate_edges(forward, left, sides, c['cone_gate_window'])
        if left_edge is None or right_edge is None:
            return None
        width = left_edge-right_edge
        if not c['cone_gate_min'] <= width <= c['cone_gate_max']:
            return None
        return heading, (left_edge+right_edge)/2, width, float(forward.min())

    # ---------- 2. 콘 사이 통과 ----------
    def drive(self, cones, scan, velocity, dt):
        c = self.c
        forward, left = to_road(cones, self.heading)
        # 통로 안의 콘만 사용 (옆 차로 물체·표지판 기둥 제외)
        inside = ((forward > 0) & (forward < c['cone_view_distance']) &
                  (np.abs(left-self.center) < c['cone_gate_max']/2+c['cone_band_margin']))
        forward, left = forward[inside], left[inside]
        sides, path_f, path_l = corridor_line(forward, left, self.center, self.width)
        left_edge, right_edge = gate_edges(forward, left, sides, c['cone_gate_window'])
        # 참고 코드 y_ref = 좌·우 콘 가운데. 한쪽만 보이면 기억한 간격의 절반만큼 안쪽
        measured = None
        if left_edge is not None and right_edge is not None:
            measured = (left_edge+right_edge)/2
            width = float(np.clip(left_edge-right_edge, c['cone_gate_min'], c['cone_gate_max']))
            self.width += c['cone_center_alpha']*(width-self.width)
        elif left_edge is not None:
            measured = left_edge-self.width/2
        elif right_edge is not None:
            measured = right_edge+self.width/2
        if measured is not None:
            self.center += c['cone_center_alpha']*(measured-self.center)

        # ---------- 3. 종료 (참고 코드 "Labacon End") ----------
        # 앞에 콘이 없어진 뒤(마지막 콘이 라이다 옆을 지남) small_object_memory_distance 더 가면 끝.
        # 안전장치: 진입 시 첫 콘까지 거리 + cone_max_distance 넘게 가면 끝 (늦게 끝나면 다음 코너를 못 돎)
        self.remaining = (c['small_object_memory_distance'] if len(forward)
                          else max(0.0, self.remaining-abs(velocity)*dt))
        if self.remaining <= 0 or self.travel >= self.max_travel:
            self.done, self.active = True, False
            self.debug.update(state='done')
            return None

        # 통로 중앙선의 cone_lookahead 앞 지점으로 조향, 라이다 점과 부딪히는 원호면 가까운 안전 조향
        path_l[0] = self.center
        target = float(np.interp(c['cone_lookahead'], path_f, path_l))
        x, y = to_body(c['cone_lookahead'], target, self.heading)
        desired = pursuit_steering(x, y, self.robot)
        steering, safe = clear_steering(body_points(scan, self.robot), desired, self.robot,
                                        c['cone_check_distance'], c['cone_margin'])
        self.debug = dict(state='active', cones=int(len(forward)), left=left_edge, right=right_edge,
                          width=self.width, center=self.center, heading=self.heading,
                          travel=self.travel, max_travel=self.max_travel, remaining=self.remaining,
                          desired=desired, safe=safe)
        # 안전한 원호가 없어도 멈추지 않고 천천히 (정지 = 미션 실패). 정면 막힘은 driver의 비상 정지가 처리
        return (c['avoidance_speed'] if safe else c['cone_creep_speed']), steering

    def update(self, lane, scan, velocity, yaw_rate, dt, enabled):
        """매 제어 주기 호출. 라바콘 모드면 (속도, 조향), 아니면 None (차선 주행)"""
        if not enabled or self.done:
            self.active, self.confirm = False, 0
            return None
        cones = find_cones(scan, self.robot)
        if not self.active:
            # 넓은 물체(차량 장애물)가 보이면 라바콘 구간으로 보지 않는다.
            start = self.start_check(cones, lane) if scan['wide_objects'] == 0 else None
            self.confirm = self.confirm+1 if start else 0
            self.debug = dict(state='wait', confirm=self.confirm, cones=int(len(cones)))
            if self.confirm < self.c['cone_confirm_count']:
                return None
            self.heading, self.center, self.width, first = start
            self.active, self.travel = True, 0.0
            self.remaining = self.c['small_object_memory_distance']
            self.max_travel = first+self.c['cone_max_distance']
        else:
            # 라이다 사이 주기: 오도메트리로 콘 줄 방향과 통로 중앙 위치를 이어 간다.
            self.heading -= yaw_rate*dt
            self.center += velocity*math.sin(self.heading)*dt
            self.travel += max(0.0, velocity)*dt
        return self.drive(cones, scan, velocity, dt)
