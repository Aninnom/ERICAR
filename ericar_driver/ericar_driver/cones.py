"""라바콘: 라이다로 좌·우 라바콘 쌍(문)을 찾아 그 사이 방향으로 조향

11/12 변경: 콘이 두 줄로 이어진다는 가정 제거. 미션 = 양옆에 놓인 콘 사이 통과, 쌍 개수·간격 모름.
예전 방식(줄 방향 직선 맞춤 + 통로 중앙·폭을 기억해 odom으로 이어 감)은 쌍 배치에서 진입 못 함·바깥 통과 (합성 확인).
참고
  - young4240/scale_car obstacle_side.py: 앞쪽 가장 가까운 좌·우 콘의 가운데 → 조향
  - tjsdn3065/AutoRace-2024 rubber_cone.py (2024 우승): 좌·우 콘 가운데 점들을 pure pursuit, 콘이 줄면 해제 후 다시 대기
방식: 매 주기 지금 라이다만으로 다시 계산. 기억하는 값은 진행 방향(heading)과 문 폭(width)뿐
  1. 기둥: 라이다보다 앞에 있는 군집의 중심. 붙어 놓인 콘 덩어리는 small_object_max_width마다 나눔.
     외곽 가벽 직선 위에 놓인 기둥(끊긴 벽 조각)은 제외 (11/13)
  2. 문: 기둥 쌍 중 간격 cone_gate_min~max이고 진행 방향을 마주 보는 것 (같은 쪽 콘 2개는 진행 방향과 나란해 제외)
  3. 가까운 문부터 문 중점을 이어 cone_lookahead 앞 지점의 각도로 조향 (pure pursuit)
  4. 문이 안 보이고 콘만 보이면 그 콘에서 기억한 문 폭의 절반만큼 안쪽으로 지나감
  5. 지날 문이 없고 화면에 콘 색도 없고 차선이 보이면 바로 차선 주행으로 돌려줌 (11/14).
     그 조건이 아니면 예전대로 small_object_memory_distance 더 간 뒤 복귀. 다음 문이 보이면 다시 켜짐
  6. 카메라 붉은색(red_ratio) 보조 (11/12): 붉은색이 보이면 진입·문 간격 조건 완화, 통과 중에는 모드 유지

좌표: 차체 기준(원점 = 뒷바퀴 축), 앞 x+, 왼쪽 y+ (REP-103). 각도 + = 왼쪽.
"""
import math
import numpy as np

from .lidar import body_points, clear_steering, pursuit_steering


def unit(vector):
    return vector/max(float(np.hypot(vector[0], vector[1])), 1e-9)


WALL_ANGLES = np.deg2rad(np.arange(0, 180, 2))   # 벽 직선 후보의 법선 각도 (허프 변환, 2° 간격)
WALL_NORMALS = np.column_stack((np.cos(WALL_ANGLES), np.sin(WALL_ANGLES)))


def line_distance(points, origin, direction):
    """점(N×2)에서 직선(origin을 지나고 direction 방향)까지 거리"""
    offset = points-origin
    return np.abs(offset[:, 0]*direction[1]-offset[:, 1]*direction[0])


def wall_lines(points, c):
    """라이다 점 전체에서 벽(길게 이어진 곧은 선) 찾기 → [(직선 위 한 점, 방향 단위벡터)]

    허프 변환: 방향 2° 간격 × 거리 칸(2·cone_wall_margin)마다 점 수를 세어 가장 많이 모인 직선을 후보로 뽑고
    그 근처 점으로 직선을 다시 맞춤. 직선 위 점들이 cluster_gap 이내로 이어진 길이의 합이 cone_wall_min_length 이상이면 벽.
    군집 단위(폭 넘는 군집만 벽)로 보지 않는 이유: 콘 그림자·거리 잡음으로 벽이 짧은 조각으로만 보이는 주기가 있음.
    콘은 하나당 이어진 길이 약 0.08 m라 한 줄로 놓여도 합이 기준에 못 미침 (붙은 콘 0.5 m + 콘 3개 = 0.74 m).
    촘촘히 붙여 놓은 콘 줄(합이 기준을 넘음)을 벽으로 지우지 않게 조건 2개 추가 (11/13 유경: 벽은 끊김 없이 사방을 두른 판):
      - 직선이 차 왼쪽에 있음. 시계 방향 주행이라 외곽벽은 항상 왼쪽(코너에서는 앞), 오른쪽은 맵 안쪽이라 벽이 없음.
        cone_wall_side 'both'면 이 조건을 빼고 양쪽 다 봄 (연습장처럼 오른쪽에도 벽이 있는 곳에서 시험할 때)
      - 직선 너머(라이다 반대쪽)에 점이 없음. 벽 너머는 안 보이고, 벽 앞에 선 콘 줄은 그 뒤로 벽이 보임
    """
    lines, margin, pool = [], c['cone_wall_margin'], points
    both = c.get('cone_wall_side', 'left') == 'both'
    for _ in range(5):   # 후보 5개까지: 옆 벽, 앞 벽 + 점이 많이 모인 다른 물체(가까운 붙은 콘 줄 등)
        if len(pool) < 3:
            break
        cell = np.floor(pool@WALL_NORMALS.T/(2*margin)).astype(int)
        cell -= cell.min()
        span = int(cell.max())+1
        votes = np.bincount((cell+np.arange(len(WALL_NORMALS))*span).ravel())
        angle, k = divmod(int(votes.argmax()), span)
        near = cell[:, angle] == k
        for _ in range(2):
            if near.sum() < 3:
                break
            origin = pool[near].mean(axis=0)
            direction = np.linalg.svd(pool[near]-origin, full_matrices=False)[2][0]
            near = line_distance(pool, origin, direction) <= margin
        if near.sum() >= 3:
            along = (pool[near]-origin)@direction
            gaps = np.diff(np.sort(along))
            normal = np.array([-direction[1], direction[0]])
            side = float(origin@normal)   # 라이다(원점)에서 직선까지 부호 있는 거리. 직선에서 가장 가까운 점 = normal·side
            # 너머 점은 직선이 실제로 뻗은 구간 안에서만 셈. 무한 직선으로 세면 벽 끝 너머로 비켜 보이는
            # 물건(연습장 의자·관중석)까지 들어와 벽을 못 찾음 (11/13 사진)
            depth = (points@normal-side)*np.sign(side)
            span = (points-origin)@direction
            beyond = int(np.sum((depth > 2*margin) & (span >= along.min()-margin) & (span <= along.max()+margin)))
            if (float(gaps[gaps <= c['cluster_gap']].sum()) >= c['cone_wall_min_length']
                    and (both or normal[1]*side > 0) and beyond <= c['cone_wall_beyond']):
                lines.append((origin, direction))
        pool = pool[~near]
    return lines


def split_clusters(points, c):
    """각도 순 점열을 이웃 점 간격 cluster_gap으로 나눔 → [(군집 점 K×2, 폭)]. 점 수·폭 하한 미만은 버림"""
    found = []
    if len(points):
        breaks = np.flatnonzero(np.linalg.norm(np.diff(points, axis=0), axis=1) > c['cluster_gap'])+1
        for cluster in np.split(points, breaks):
            if len(cluster) < c['cluster_min_points']:
                continue
            width = float(np.linalg.norm(cluster.max(axis=0)-cluster.min(axis=0)))
            if width >= c['small_object_min_width']:
                found.append((cluster, width))
    return found


def find_posts(scan, robot, c):
    """라이다 군집 → (기둥 N×2, 넓은 군집 중심 M×2). 차체 좌표

    기둥 = 라바콘 후보. 폭 small_object_max_width 이하 군집은 중심 하나, 그보다 넓고 cone_group_max_width 이하면
    붙어 놓인 콘으로 보고 small_object_max_width마다 나눔 (11/12 실차: 붙은 콘이 폭 0.16~0.5 m 한 덩어리).
    라이다보다 뒤(지나친 콘)와 cone_view_distance 밖은 제외.
    벽 점 제외 (11/13): 외곽 가벽은 콘 그림자·점 누락으로 끊겨 폭 cone_group_max_width 이하 조각이 됨 → 붙은 콘으로
    분류되어 벽·콘 사이를 문으로 봄 (합성 확인). 벽 직선(wall_lines)에서 cone_wall_margin 안의 점을 먼저 빼고
    다시 군집을 나눔 → 벽 조각이 사라지고, 벽 가까이 놓여 벽 군집에 묻힌 콘은 따로 떨어짐. 벽이 없으면 예전과 동일.
    넓은 군집 중심은 진입 판단에서 통로를 막은 물체(차량 장애물) 확인용 (벽 점을 빼기 전 군집 기준).
    """
    points = np.asarray(scan['points'], dtype=float).reshape(-1, 2)
    found = split_clusters(points, c)
    wide = [cluster.mean(axis=0) for cluster, width in found if width > c['small_object_max_width']]
    walls = [] if c.get('cone_wall_side', 'left') == 'off' else wall_lines(points, c)
    if walls:
        keep = np.ones(len(points), dtype=bool)
        for origin, direction in walls:
            keep &= line_distance(points, origin, direction) > c['cone_wall_margin']
        found = split_clusters(points[keep], c)
    posts = []
    for cluster, width in found:
        if width > c['cone_group_max_width']:
            continue
        parts = max(1, math.ceil(width/c['small_object_max_width']))
        posts += [part.mean(axis=0) for part in np.array_split(cluster, parts) if len(part)]
    posts = np.asarray(posts, dtype=float).reshape(-1, 2)
    wide = np.asarray(wide, dtype=float).reshape(-1, 2)
    posts = posts[(posts[:, 0] > 0) & (np.hypot(posts[:, 0], posts[:, 1]) < c['cone_view_distance'])]
    offset = [robot['lidar']['x'], robot['lidar'].get('y', 0.0)]
    return posts+offset, wide+offset


def red_pixels(image, c):
    """카메라 화면 아래 cone_red_rows 영역의 붉은 화소 판정 (1/4 크기, bool 배열). 영상이 없으면 None

    1/4 크기로 줄여 계산 (20 Hz 부담 줄임). 노란 차선(색상 약 20~35)은 범위 밖
    """
    if image is None:
        return None
    import cv2
    h = image.shape[0]
    small = cv2.resize(image[int(h*(1-c['cone_red_rows'])):], None, fx=.25, fy=.25, interpolation=cv2.INTER_AREA)
    hsv = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)
    hue, saturation, value = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    return (((hue <= c['cone_red_h_low']) | (hue >= c['cone_red_h_high'])) &
            (saturation >= c['cone_red_s_min']) & (value >= c['cone_red_v_min']))


def red_ratio(image, c):
    """화면 아래 cone_red_rows 영역의 붉은 화소 비율 (0~1). 영상이 없으면 0"""
    red = red_pixels(image, c)
    return float(red.mean()) if red is not None and red.size else 0.0


def red_info(image, c):
    """붉은색 판정 결과 → lane에 넣는 값 {cone_red, cone_red_left, cone_red_right}

    cone_red: 화면 아래 cone_red_rows 영역 전체 비율
    cone_red_left·right: 화면 위쪽 cone_red_side_top을 뺀 아래 영역의 왼쪽·오른쪽 끝 cone_red_side_width 띠 비율
      → 확인용 (red_check·corn_test 출력). 판단에는 안 씀
    """
    red = red_pixels(image, c)
    if red is None or not red.size:
        return dict(cone_red=0.0, cone_red_left=0.0, cone_red_right=0.0)
    full_h = red.shape[0]/c['cone_red_rows']
    top = max(0, int(full_h*c['cone_red_side_top'])-int(full_h-red.shape[0]))
    side = max(1, int(red.shape[1]*c['cone_red_side_width']))
    lower = red[top:]
    return dict(cone_red=float(red.mean()), cone_red_left=float(lower[:, :side].mean()),
                cone_red_right=float(lower[:, -side:].mean()))


def gate_pairs(posts, c, gate_max=None):
    """간격이 cone_gate_min~gate_max(기본 cone_gate_max)인 기둥 쌍 전부 → (중점 K×2, 문 방향 K×2, 간격 K)

    문 방향 = 두 기둥을 잇는 선에 수직인 단위벡터 (앞뒤 부호는 쓰는 쪽에서 맞춤)
    """
    i, j = np.triu_indices(len(posts), 1)
    span = posts[j]-posts[i]
    gap = np.hypot(span[:, 0], span[:, 1])
    ok = (gap >= c['cone_gate_min']) & (gap <= (c['cone_gate_max'] if gate_max is None else gate_max))
    span, gap = span[ok], gap[ok]
    normals = np.column_stack((-span[:, 1], span[:, 0]))/gap[:, None]
    return (posts[i[ok]]+posts[j[ok]])/2, normals, gap


def next_gate(gates, origin, direction, bearing, facing, minimum_forward=0.0, used=()):
    """origin에서 direction(단위벡터)으로 갈 때 다음에 지날 문의 번호. 없으면 None

    조건: 중점이 앞쪽(minimum_forward 초과)이고 방위가 bearing(rad) 이내, 문 방향이 direction에서 facing(rad) 이내.
    같은 쪽 콘 2개는 문 방향이 진행 방향과 직각에 가까워 걸러짐. 여럿이면 앞 거리 + 옆 거리가 가장 작은 문
    """
    mids, normals, gaps = gates
    if not len(gaps):
        return None
    relative = mids-origin
    forward = relative@direction
    lateral = relative[:, 1]*direction[0]-relative[:, 0]*direction[1]
    ok = ((forward > minimum_forward) & (np.abs(np.arctan2(lateral, forward)) <= bearing) &
          (np.abs(normals@direction) >= math.cos(facing)))
    ok[list(used)] = False
    if not ok.any():
        return None
    return int(np.argmin(np.where(ok, forward+np.abs(lateral), np.inf)))


def gate_path(gates, direction, c):
    """차(원점)에서 가까운 문부터 차례로 이음. 반환 (문 번호 목록, 마지막 문을 지난 뒤 진행 방향)

    문을 하나 지날 때마다 진행 방향 = (그 문으로 간 방향 + 문 방향)의 가운데.
    좌우가 앞뒤로 어긋난 쌍은 문 방향이 비스듬해 문 방향만 쓰면 다음 문을 놓침
    """
    mids, normals, _ = gates
    origin, chosen = np.zeros(2), []
    for _ in range(c['cone_path_gates']):
        k = next_gate(gates, origin, direction, c['cone_gate_bearing'], c['cone_gate_angle'],
                      c['cone_gate_step'] if chosen else 0.0, chosen)
        if k is None:
            break
        chosen.append(k)
        normal = normals[k] if normals[k]@direction > 0 else -normals[k]
        direction = unit(unit(mids[k]-origin)+normal)
        origin = mids[k]
    return chosen, direction


def lookahead_point(path, direction, distance):
    """원점(뒷바퀴 축)에서 distance 떨어진 경로 위 점. path = 문 중점들 (가까운 순)

    첫 문이 distance보다 멀면 첫 문 중점 그대로, 마지막 문 뒤는 direction으로 곧게 연장
    """
    if np.hypot(path[0][0], path[0][1]) >= distance:
        return np.asarray(path[0], dtype=float)
    points = [np.asarray(p, dtype=float) for p in path]
    points.append(points[-1]+direction*2*distance)
    for a, b in zip(points, points[1:]):
        if np.hypot(b[0], b[1]) < distance:
            continue
        # a는 원 안, b는 원 밖: 선분 위에서 원점까지 거리가 distance인 점
        d = b-a
        A, B, C = d@d, 2*(a@d), a@a-distance*distance
        return a+d*(-B+math.sqrt(max(B*B-4*A*C, 0.0)))/(2*A)
    return points[-1]


class ConeDriving:
    """라바콘 담당: 대기(문 확인) ↔ active(문 사이 통과). 구간 거리 초과 시 done(다시 켜지 않음)"""
    def __init__(self, config, robot):
        self.c, self.robot = config['lidar'], robot
        self.active = self.done = self.started = False
        self.heading = 0.0   # 진행 방향 (차체 기준 각도, + 왼쪽). 조향 목표 각도를 따라가고 사이 주기는 자이로로 유지
        self.center = 0.0    # 조향 목표 지점이 차 기준 왼쪽으로 몇 m (디버그용)
        self.width = self.c['cone_gate_width']   # 문 폭 (지난 문 측정값). 콘 하나만 보일 때 사용
        self.remaining = self.travel = self.max_travel = 0.0
        self.confirm = 0
        self.red = False     # 카메라 붉은색(콘 색)이 최근에 보였는지. lane['cone_red'] (loop.py가 넣음)
        self.red_now = False    # 지금 화면에 콘 색이 보이는지 (기억 아님). 즉시 복귀 판단용
        self.lane_ok = False    # 차선 인식이 유효한지 (lane['valid']). 즉시 복귀 판단용
        self.red_since = float('inf')   # 붉은색이 마지막으로 보인 뒤 주행 거리(m)
        self.corridor = False           # 화면 전체 붉은 비율이 진입 기준(cone_red_enter_ratio) 이상을 최근에 넘었는지
        self.corridor_since = float('inf')
        self.debug = {}

    def gate_max(self):
        return self.c['cone_red_gate_max'] if self.red else self.c['cone_gate_max']

    # ---------- 1. 진입 판단 ----------
    def start_gate(self, posts, wide):
        """라바콘 모드로 바꿀 때면 (진입 기준점, 문 폭), 아니면 None. 기준점 방향이 첫 진행 방향

        A. 카메라 붉은색 (11/12 유경): 화면 붉은 비율이 cone_red_enter_ratio 이상(corridor)이고 라이다 콘 후보가 라이다 앞
           cone_enter_distance(앞 방향 거리) 안, 좌우 cone_red_half_width 안으로 다가오면 켬. 문(쌍)을 못 찾아도 켜고,
           진행 방향은 정면에서 시작 (문이 보이면 문 중점). 통과 중 조향은 drive가 문·한쪽 콘으로 정함.
           11/13 사진(red_1113_000127): 화면 전체 붉은색만 보면 코너 도는 중 한쪽 콘 줄·관중석 의자로 일찍 켜짐
           11/12 문 중점 0.3 m 기준은 실차에서 안 켜짐: 차가 차로 중앙, 문 중점이 옆으로 비켜 있으면 0.3 m 안에 안 들어옴
        B. 붉은색이 아예 없음 (카메라 끊김 대비): 라이다만으로 문을 찾음. 붉은색은 보이지만 양쪽이 아니면 켜지 않음
           1) 차 정면 좌우 small_object_half_width 안 기둥 2개가 문 (간격 cone_gate_min~max)
           2) 문 중점 방위·문 방향이 차 정면에서 cone_start_angle 이내 (코너 도는 중에는 안 켬)
           3) 문 중점이 라이다 앞 cone_enter_distance(앞 방향 거리) 안
           4) 넓은 군집이 문 안쪽 통로에 없음 (3차선 차량 장애물 오진입 방지)
        신호등(빨간불) 등 다른 붉은 물체: 라바콘은 S3 구간에서만 켜지므로 해당 없음
        """
        c = self.c
        lidar_x = self.robot['lidar']['x']
        if self.red and not self.corridor:
            return None
        if self.corridor:
            near = posts[(posts[:, 0]-lidar_x <= c['cone_enter_distance']) &
                         (np.abs(posts[:, 1]) < c['cone_red_half_width'])]
            if not len(near):
                return None
            gates = gate_pairs(posts[np.abs(posts[:, 1]) < c['cone_red_half_width']], c, self.gate_max())
            k = next_gate(gates, np.zeros(2), np.array([1.0, 0.0]), c['cone_start_angle'], c['cone_start_angle'])
            if k is not None:
                return gates[0][k], float(gates[2][k])
            return np.array([float(near[:, 0].min()), 0.0]), c['cone_gate_width']
        posts = posts[np.abs(posts[:, 1]) < c['small_object_half_width']]
        gates = gate_pairs(posts, c, self.gate_max())
        k = next_gate(gates, np.zeros(2), np.array([1.0, 0.0]), c['cone_start_angle'], c['cone_start_angle'])
        if k is None:
            return None
        middle, normal, gap = gates[0][k], gates[1][k], float(gates[2][k])
        distance = float(np.hypot(middle[0], middle[1]))
        if middle[0]-lidar_x > c['cone_enter_distance']:
            return None
        if len(wide):
            normal = normal if normal[0] > 0 else -normal
            relative = wide-middle
            along = relative@normal
            across = relative[:, 1]*normal[0]-relative[:, 0]*normal[1]
            if np.any((along > -distance) & (along < c['cone_wide_depth']) &
                      (np.abs(across) < gap/2-c['cone_wide_margin'])):
                return None
        return middle, gap

    # ---------- 2. 조향 목표 지점 ----------
    def one_side(self, posts, aim):
        """문 없이 콘만 보일 때: 가장 가까운 콘에서 기억한 문 폭의 절반만큼 안쪽 지점. 없으면 None

        진행 방향선 왼쪽에 있는 콘은 왼쪽 콘 → 그 오른쪽으로 지나감 (오른쪽 콘은 반대).
        진행 방향선에서 cone_gate_max/2보다 먼 콘은 내가 지날 문의 기둥일 수 없어 무시 (트랙 옆 물체)
        """
        if not len(posts):
            return None
        c = self.c
        left = np.array([-aim[1], aim[0]])
        forward, lateral = posts@aim, posts@left
        distance = np.hypot(forward, lateral)
        # 방위 제한 없음: 옆을 지나는 콘은 가까울수록 방위가 커짐 → 제한하면 지나기 전에 모드가 먼저 꺼짐
        ok = (forward > 0) & (np.abs(lateral) <= self.gate_max()/2) & (distance <= c['cone_start_range'])
        if not ok.any():
            return None
        k = int(np.argmin(np.where(ok, distance, np.inf)))
        side = 1.0 if lateral[k] >= 0 else -1.0
        return posts[k]-side*self.width/2*left

    def blocked(self, posts, aim):
        """진행 방향 앞 cone_exit_clear 안·좌우 문 폭 절반 안에 콘이 있으면 참 (즉시 복귀 보류)

        마지막 콘이 차 바로 앞·옆에 남은 채 차선 주행으로 넘기면 차선 조향이 그 콘 쪽으로 돌 수 있음
        """
        if not len(posts):
            return False
        left = np.array([-aim[1], aim[0]])
        forward, lateral = posts@aim, posts@left
        return bool(np.any((forward > 0) & (forward <= self.c['cone_exit_clear']) &
                           (np.abs(lateral) <= self.gate_max()/2)))

    # ---------- 3. 문 사이 통과 ----------
    def drive(self, posts, scan, velocity, dt):
        c = self.c
        aim = np.array([math.cos(self.heading), math.sin(self.heading)])
        gates = gate_pairs(posts, c, self.gate_max())
        chosen, direction = gate_path(gates, aim, c)
        if chosen:
            target = lookahead_point(gates[0][chosen], direction, c['cone_lookahead'])
            self.width += c['cone_aim_alpha']*(float(gates[2][chosen[0]])-self.width)
        else:
            target = self.one_side(posts, aim)
        # 즉시 복귀 (11/14): 지날 문도(chosen) 지나갈 콘도(target) 없고, 지금 화면에 콘 색이 없고, 차선이 보이고,
        # 진행 방향 앞 cone_exit_clear 안에 콘도 없으면 여유 거리 없이 바로 차선 주행으로 넘김.
        # 왜: 아래 remaining(0.3 m) + 붉은색 기억(cone_red_memory 0.5 m) 동안은 기억한 진행 방향으로 직진하므로,
        # 콘 배치가 차를 틀어 놓았으면 그만큼 차로 밖으로 나감 (실차: 복귀 위치가 주행마다 달라짐).
        # 통과 도중에는 화면에 콘 색이 남아 red_now가 참이므로 11/12 cone_lane_3(중간 복귀·충돌) 보호는 유지됨
        if target is None and not chosen and not self.red_now and self.lane_ok and not self.blocked(posts, aim):
            self.active = False
            self.debug.update(state='wait', confirm=0, exit='lane')
            return None
        # 종료: 앞에 지나갈 콘이 없어진 뒤 small_object_memory_distance 더 가면 차선 주행으로 돌려줌 (다음 문이 보이면 다시 켜짐)
        # 그동안은 진행 방향 유지. 도면상 라바콘 끝 선 → 코너 시작 약 0.3 m라 오래 끌지 않음
        # 카메라에 붉은색이 보이는 동안은 라이다에 콘이 안 잡혀도 유지 (11/12 cone_lane_3: 중간에 차선 복귀·충돌)
        seen = target is not None or self.red
        self.remaining = (c['small_object_memory_distance'] if seen
                          else max(0.0, self.remaining-abs(velocity)*dt))
        if self.remaining <= 0:
            self.active = False
            self.debug.update(state='wait', confirm=0)
            return None
        if target is None:
            target = aim*c['cone_lookahead']
        angle = math.atan2(target[1], target[0])
        self.heading += c['cone_aim_alpha']*(angle-self.heading)
        self.center = float(target[1])
        # 목표 지점의 각도 → 조향. 라이다 점과 부딪히는 원호면 가까운 안전 조향
        desired = pursuit_steering(float(target[0]), float(target[1]), self.robot)
        steering, safe = clear_steering(body_points(scan, self.robot), desired, self.robot,
                                        c['cone_check_distance'], c['cone_margin'])
        self.debug = dict(state='active', posts=int(len(posts)), gates=len(chosen), seen=seen, red=self.red,
                          target_x=float(target[0]), target_y=float(target[1]), angle=angle,
                          width=self.width, heading=self.heading, travel=self.travel,
                          max_travel=self.max_travel, remaining=self.remaining, desired=desired, safe=safe)
        # 안전한 원호가 없어도 멈추지 않고 천천히 (정지 = 미션 실패). 정면 막힘은 driver의 비상 정지가 처리
        return (c['avoidance_speed'] if safe else c['cone_creep_speed']), steering

    def update(self, lane, scan, velocity, yaw_rate, dt, enabled):
        """매 제어 주기 호출. 라바콘 모드면 (속도, 조향), 아니면 None (차선 주행)"""
        if not enabled or self.done:
            self.active, self.confirm = False, 0
            return None
        c = self.c
        # 붉은색: 지금 보이거나 마지막으로 보인 뒤 cone_red_memory(m) 안. 콘 바로 앞에서는 콘이 화면 밖으로 빠질 수 있음
        # red: 화면에 콘 색 (통과 중 모드 유지·문 간격 완화). corridor: 붉은 비율 cone_red_enter_ratio 이상 (진입)
        lane = lane or {}
        step = abs(velocity)*dt
        seen = float(lane.get('cone_red', 0.0)) >= c['cone_red_ratio']
        both = float(lane.get('cone_red', 0.0)) >= c['cone_red_enter_ratio']
        # 복귀 판단용 (drive에서 사용). red_now는 기억이 아니라 지금 화면, lane_ok는 차선이 보이는지
        self.red_now, self.lane_ok = seen, bool(lane.get('valid'))
        self.red_since = 0.0 if seen else self.red_since+step
        self.corridor_since = 0.0 if both else self.corridor_since+step
        self.red = self.red_since <= c['cone_red_memory']
        self.corridor = self.corridor_since <= c['cone_red_memory']
        if self.started:
            # 안전장치: 첫 진입 뒤 (첫 문까지 거리 + cone_max_distance) 넘게 가면 끝. 늦게 끝나면 다음 코너를 못 돎
            self.travel += max(0.0, velocity)*dt
            if self.travel >= self.max_travel:
                self.done, self.active = True, False
                self.debug = dict(state='done', travel=self.travel)
                return None
        posts, wide = find_posts(scan, self.robot, c)
        if self.started:
            # 모드가 끝나는 지점(max_travel)보다 먼 콘은 어차피 지나지 않음 → 구간 뒤 트랙 옆 물체로 끌려가지 않게 제외
            posts = posts[np.hypot(posts[:, 0], posts[:, 1]) < self.max_travel-self.travel]
        if not self.active:
            start = self.start_gate(posts, wide)
            self.confirm = self.confirm+1 if start else 0
            self.debug = dict(state='wait', confirm=self.confirm, posts=int(len(posts)), red=self.red)
            if self.confirm < c['cone_confirm_count']:
                return None
            middle, gap = start
            self.active, self.confirm = True, 0
            self.heading = math.atan2(middle[1], middle[0])
            self.width = gap
            self.remaining = c['small_object_memory_distance']
            if not self.started:
                self.started, self.travel = True, 0.0
                self.max_travel = float(np.hypot(middle[0], middle[1]))+c['cone_max_distance']
        else:
            # 차가 도는 만큼 진행 방향(차체 기준 각도)을 되돌려 바깥에서 본 방향을 유지
            self.heading -= yaw_rate*dt
        return self.drive(posts, scan, velocity, dt)
