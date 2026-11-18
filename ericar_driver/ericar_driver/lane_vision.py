"""차선 인식: BEV 변환, sliding window, 정지선 [간단 버전]"""
# 출처: hello-osy/2026-orda-mentoring answer.py (613c9b4), 차선 간단 버전은 소민 realcar 브랜치 66362f3
# 11/8 코너: 목표점 pure pursuit·좌우 판단 추가 (sessions 차선 기록, PR #8)
# 11/9 이상 픽셀 빼고 재맞춤·누운 선 목표점 (녹화 integ_1109_lap4 분석)
import math
import cv2
import numpy as np

from .common import clip


class LaneVision:
    """차선 담당①: 카메라 → 차로 중앙 목표점

    처리 순서 (analyze):
      1. 색 검출   : HSV에서 차선 색(yellow_min~yellow_max)만 남김
      2. 버드아이뷰 : 카메라 장착값으로 바닥을 위에서 본 이미지로 변환 (1픽셀 = 고정 m)
      3. 가로선 분리: 가로로 긴 페인트(정지선)는 차선에서 빼고 정지선 검출에 사용
      4. 차선 찾기 : 아래쪽 히스토그램으로 좌/우 시작점 → 슬라이딩 윈도우로 위로 추적 → 2차식
      5. 좌우 판단 : 같은 선 겹침 제거, 한 선이면 직전 프레임 이름 이어받기 (label_sides)
      6. 목표점    : 선마다 중앙선 점을 만들어 뒷축에서 pursuit_distance인 점 (pursuit_target)
                    error = 목표점 옆 거리(m, + 왼쪽), forward = 앞 거리(m). 목표점 없으면 lookahead 중앙
    center_px·heading은 lookahead 행 기준 (정지선 탐색 범위, 차선 변경·주차가 사용)
    """
    def __init__(self, lane, camera, front_axle_x):
        self.c = lane
        self.camera = camera
        self.front_axle_x = front_axle_x
        self.scale = lane['width_px'] / lane['view_width']                # 1 m = scale 픽셀 (가로)
        self.row_m = (lane['far'] - lane['near']) / (lane['height_px'] - 1)  # 1 행 = row_m m (세로)
        self.maps = None
        self.valid_bird = None
        self.previous_center = None   # 이전 프레임 차로 중앙(px), 시작점 탐색 기준
        self.memory = {}              # 좌/우 선별 마지막으로 본 픽셀 {이름: (행 배열, x 배열, 지난 프레임 수)}, 이름 이어받기용
        self.corner_left_limit = None # 우회전 코너 중이면 m 값 (loop가 설정). 기억 없는 한 선 이름 판단 기준
        self.memory_first = False     # 참이면 한 선 이름을 기억한 선에서 먼저 이어받고, 기억이 없을 때만 corner_left_limit (loop가 정지 접근 중 설정)
        # 3차선 끝 차로(또는 그쪽으로 차선 변경 중)면 노란 바깥선이 있는 쪽 'left'/'right' (loop가 설정), 아니면 None
        # 바깥 차로에서 노란 선만 보일 때 위치로 이름을 붙이면 차가 노란 선에 붙은 순간 '오른쪽 선'으로 오판
        # → 차로 중앙을 더 바깥으로 계산해 트랙 밖으로 나감 (11/11 run_1111_1241 17.8~22.7 s, n=r → 이탈) (소민)
        self.outer_yellow = None
        self.fork_direction = 0  # 끝 차로 정렬 후 선택 출구. +1=A(왼쪽), -1=B(오른쪽)
        self.double_left = False      # 마지막 코너 뒤 왼쪽에 옆 차로 선이 나란한 구간이면 True (loop가 설정). find_starts·label_sides
        self.yellow_level = None      # 도로 위 노란 선 밝기(V) 느린 평균. 정지선 흰색 하한을 조명에 맞추는 기준 (stop_white_ratio)
        self.stop_white_level = None  # 이번 프레임에 쓴 정지선 흰색 V 하한 (디버그)

    # ---------- 2. 버드아이뷰 ----------
    def ground_maps(self, image_shape):
        """버드아이뷰 각 픽셀이 원본 카메라 이미지의 어느 픽셀인지 계산 (한 번만)."""
        h, w = image_shape[:2]
        c, cam = self.c, self.camera
        rows, cols = np.indices((c['height_px'], c['width_px']), dtype=np.float32)
        forward = c['far'] - rows / (c['height_px'] - 1) * (c['far'] - c['near'])  # 차 앞 거리(m)
        left = (0.5 - cols / (c['width_px'] - 1)) * c['view_width'] - cam.get('y', 0.0)  # 왼쪽 거리(m)
        dx, pitch = forward - cam['x'], cam['pitch']
        depth = math.cos(pitch) * dx + math.sin(pitch) * cam['z']
        up = math.sin(pitch) * dx - math.cos(pitch) * cam['z']
        focal = w / (2 * math.tan(math.radians(cam['hfov_deg']) / 2))
        safe = np.maximum(depth, 0.001)
        map_x = np.where(depth > 0, w / 2 - focal * left / safe, -1)
        map_y = np.where(depth > 0, h / 2 - focal * up / safe, -1)
        return map_x.astype(np.float32), map_y.astype(np.float32)

    def row_at(self, forward):
        c = self.c
        return (c['far'] - forward) / (c['far'] - c['near']) * (c['height_px'] - 1)

    def meters_at(self, row):
        c = self.c
        return c['far'] - row / (c['height_px'] - 1) * (c['far'] - c['near'])

    def analyze(self, bgr, fork_bias=0.0):
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)                      # 1. 색 검출
        color = cv2.inRange(hsv, np.array(self.c['yellow_min']), np.array(self.c['yellow_max']))
        white = cv2.inRange(hsv, np.array(self.c['white_min']), np.array(self.c['white_max']))  # 정지선용 흰색
        if self.maps is None or self.maps[0] != bgr.shape[:2]:
            self.maps = (bgr.shape[:2], self.ground_maps(bgr.shape))
        map_x, map_y = self.maps[1]
        self.valid_bird = (map_x >= 1) & (map_x < bgr.shape[1]-1) & (map_y >= 1) & (map_y < bgr.shape[0]-1)
        bird = cv2.remap(color, map_x, map_y, cv2.INTER_NEAREST,           # 2. 버드아이뷰
                         borderMode=cv2.BORDER_CONSTANT)
        white_bird = cv2.remap(white, map_x, map_y, cv2.INTER_NEAREST, borderMode=cv2.BORDER_CONSTANT)
        # 정지선·횡단보도 전용 흰색 하한 (stop_white_min, 없으면 white_min 그대로). 3차선 점선은 white_min 유지
        stop_bird = None
        if self.c.get('stop_white_min'):
            # 조명에 맞춘 하한 (11/18 redecide_1118_26·28, 밝은 연습장): 하한을 85로 고정하니 바닥 반사·'천천히' 글씨가 흰색으로 잡혀
            # 첫 선 구간 94프레임 중 91프레임이 가짜 횡단보도 선 → 마름모 앞 선에 정지. 어두운 경기장은 85가 있어야 블록 판정이 됨.
            # 화면 아래쪽 노란 선 밝기(V 가운데값)의 stop_white_ratio배를 하한으로 쓰되 stop_white_min ~ white_min 사이로 제한
            level = int(self.c['stop_white_min'][2])
            ratio = self.c.get('stop_white_ratio')
            if ratio:
                top = int(bgr.shape[0]*0.45)
                values = hsv[top:, :, 2][color[top:] > 0]
                if values.size >= self.c.get('stop_white_min_pixels', 300):
                    now_level = float(np.median(values))
                    self.yellow_level = now_level if self.yellow_level is None else 0.7*self.yellow_level+0.3*now_level
                if self.yellow_level is not None:
                    level = int(np.clip(ratio*self.yellow_level, self.c['stop_white_min'][2], self.c['white_min'][2]))
            self.stop_white_level = level
            low = [self.c['stop_white_min'][0], self.c['stop_white_min'][1], level]
            stop_white = cv2.inRange(hsv, np.array(low), np.array(self.c['white_max']))
            stop_bird = cv2.remap(stop_white, map_x, map_y, cv2.INTER_NEAREST, borderMode=cv2.BORDER_CONSTANT)
        return self.analyze_bird(self.drop_thick(bird), fork_bias, white_bird, stop_bird)

    def drop_thick(self, bird):
        """차선보다 두꺼운 노란 덩어리(방지턱 줄) 제거. 가로·세로 모두 yellow_max_thickness(m) 넘는 부분과 그 둘레만 지움"""
        limit = self.c.get('yellow_max_thickness')
        if not limit:
            return bird
        size = (max(3, round(limit * self.scale)), max(3, round(limit / self.row_m)))   # (가로 px, 세로 px)
        thick = cv2.morphologyEx(bird, cv2.MORPH_OPEN, np.ones(size[::-1], np.uint8))
        if not thick.any():
            return bird
        margin = self.c.get('yellow_thick_margin', 0.035)
        around = np.ones((2 * round(margin / self.row_m) + 1, 2 * round(margin * self.scale) + 1), np.uint8)
        return cv2.bitwise_and(bird, cv2.bitwise_not(cv2.dilate(thick, around)))

    def drop_beyond_yellow(self, white, yellow):
        """노란 선 너머(차 반대쪽) 흰색 제거 → 트랙 밖 흰 테두리를 점선으로 오인하지 않음
        - 노란 덩어리마다 차에 가까운 끝(아래쪽 0.1 m) 위치로 차 왼쪽/오른쪽 판단 → 멀리서 휘어도 좌우 안 바뀜
        - 길이 ≥ yellow_edge_min_length이고 세로에 가까운(높이 ≥ 폭) 덩어리만 기준 → 작은 얼룩·코너 가로 선 제외
        - 행마다 그 선의 차 쪽 가장자리보다 바깥 흰색 지움. 선이 안 보이는 행은 가까운 행 위치 이어 씀
        - 노란 선이 없는 쪽은 안 지움 (옆 차로에서 반대쪽 노란 선이 화면 밖이어도 점선 유지)"""
        c = self.c
        h, w = white.shape
        car = w / 2
        # 바깥 차로: 노란 선이 차 쪽으로 반 차로까지 붙어도 바깥선으로 판단 (그 너머 흰색 = 트랙 밖)
        if self.outer_yellow == 'left':
            car += c['lane_width'] / 2 * self.scale
        elif self.outer_yellow == 'right':
            car -= c['lane_width'] / 2 * self.scale
        rows = np.arange(h)
        left_edge = np.full(h, -np.inf)    # 이 x 이하 = 차 왼쪽 노란 선 너머
        right_edge = np.full(h, np.inf)    # 이 x 이상 = 차 오른쪽 노란 선 너머
        n, labels, stats, _ = cv2.connectedComponentsWithStats(yellow, connectivity=8)
        for i in range(1, n):
            x, y, bw, bh, _ = stats[i]
            if bh * self.row_m < c['yellow_edge_min_length'] or bw / self.scale > bh * self.row_m:
                continue
            ys, xs = np.nonzero(labels[y:y + bh, x:x + bw] == i)
            ys, xs = ys + y, xs + x
            near = ys >= ys.max() - 0.1 / self.row_m
            if xs[near].mean() < car:
                edge = np.full(h, -np.inf)
                np.maximum.at(edge, ys, xs)            # 행별 차 쪽(오른쪽) 가장자리
            else:
                edge = np.full(h, np.inf)
                np.minimum.at(edge, ys, xs)            # 행별 차 쪽(왼쪽) 가장자리
            seen = np.isfinite(edge)
            edge = np.interp(rows, rows[seen], edge[seen])   # 안 보이는 행: 가까운 행 값 (양 끝은 그대로 유지)
            if xs[near].mean() < car:
                left_edge = np.maximum(left_edge, edge)      # 왼쪽 선 여러 개면 차에 가까운 것
            else:
                right_edge = np.minimum(right_edge, edge)
        cols = np.arange(w)[None, :]
        inside = (cols > left_edge[:, None]) & (cols < right_edge[:, None])
        return np.where(inside, white, 0).astype(white.dtype)

    def white_dashes(self, white_bird, yellow=None):
        """흰 BEV에서 점선 조각만 남김. 흰 덩어리마다 모양을 m 단위로 재서
        white_dash_min_length ≤ 길이 ≤ white_dash_max_length, 폭 ≤ white_dash_max_width,
        진행 방향(세로)에서 기울기 ≤ white_dash_max_tilt(도)인 것만 통과.
        (예전 세로 막대 필터는 점선이 약 10°만 기울어도 지워졌음)
        yellow(노란 차선 BEV)가 있으면 먼저 노란 선 너머 흰색 제거 (drop_beyond_yellow)
        최대 길이: 노란 선을 놓친 쪽은 테두리가 안 지워짐 → 화면 끝까지 이어진 흰 실선(테두리)은 점선 아님"""
        c = self.c
        if yellow is not None:
            white_bird = self.drop_beyond_yellow(white_bird, yellow)
        keep = np.zeros_like(white_bird)
        n, labels, stats, _ = cv2.connectedComponentsWithStats(white_bird, connectivity=8)
        for i in range(1, n):
            x, y, bw, bh, area = stats[i]
            if area < c['minimum_pixels']:
                continue
            box = labels[y:y + bh, x:x + bw] == i
            ys, xs = np.nonzero(box)
            # 픽셀 → m (가로·세로 픽셀 크기가 달라서 m로 바꾼 뒤 모양 판단)
            pts = np.column_stack(((xs + x) / self.scale, (ys + y) * self.row_m))
            pts -= pts.mean(axis=0)
            # 조각이 가장 길게 뻗은 방향(주축)과 그 수직 방향으로 길이·폭 측정
            _, vectors = np.linalg.eigh(np.cov(pts.T))
            major, minor = vectors[:, 1], vectors[:, 0]
            length = np.ptp(pts @ major)
            width = np.ptp(pts @ minor)
            tilt = math.degrees(math.acos(min(1.0, abs(major[1]))))   # 0° = 진행 방향과 나란함
            if (c['white_dash_min_length'] <= length <= c['white_dash_max_length'] and width <= c['white_dash_max_width']
                    and tilt <= c['white_dash_max_tilt']):
                keep[y:y + bh, x:x + bw][box] = 255
        return keep

    def analyze_bird(self, bird, fork_bias=0.0, white_bird=None, stop_bird=None):
        """ROS 없이 테스트할 수 있도록 버드아이뷰 이후 처리를 분리.
        white_bird: 흰색 BEV (정지선·점선용). None이면 예전처럼 노란 가로선에서 정지선 탐색
        stop_bird: 정지선·횡단보도 전용 흰색 BEV (stop_white_min). None이면 white_bird 사용"""
        if stop_bird is None:
            stop_bird = white_bird
        c = self.c
        h, w = bird.shape
        lane_px = c['lane_width'] * self.scale   # 차로 폭(픽셀)

        # ---------- 3. 가로선(정지선) 분리 ----------
        kernel = np.ones((1, max(3, int(c['stop_min_width'] * self.scale))), np.uint8)
        horizontal = cv2.morphologyEx(bird, cv2.MORPH_OPEN, kernel)
        lane_mask = cv2.subtract(bird, horizontal)
        # 정지선은 흰 페인트의 가로선에서 찾음. 차선(lane_mask)은 노랑 그대로
        # 바닥판 이음매 메움 (11/18 경기장 redecide_1118_17·20): 이음매가 차로 가운데를 지나며 흰 선을 둘로 끊음(반쪽 약 0.25 m)
        # → 아래 가로 열기(stop_min_width 0.32 m 연속)에서 통째로 사라져 신호등 앞 정지선 0프레임. 정지선 탐색에 쓰는 마스크만
        # 가로로 stop_seam_close(m) 닫아 틈을 이음. 점선·횡단보도 블록 판정은 원래 마스크 그대로 (블록 틈 약 0.05 m는 안 메워짐)
        seam = max(0, int(round(c.get('stop_seam_close', 0.0) * self.scale)))
        seam_kernel = np.ones((1, seam), np.uint8) if seam >= 2 else None
        stop_source = stop_bird if stop_bird is None or seam_kernel is None else cv2.morphologyEx(stop_bird, cv2.MORPH_CLOSE, seam_kernel)
        stop_lines = horizontal if stop_bird is None else cv2.morphologyEx(stop_source, cv2.MORPH_OPEN, kernel)
        # 3차선: 차로 구분선 = 흰 점선, 바깥 경계 = 노랑. 가운데 차로는 양쪽이 모두 점선
        # 점선이 보이면 항상 차선 후보에 추가 (노란 선 너머 흰색은 제거 → 트랙 밖 흰 테두리 제외)
        # 점선이 없으면(흰 그림 없음 포함) 아래 처리 전부 기존과 동일
        yellow_mask = lane_mask
        dashes = self.white_dashes(white_bird, yellow_mask) if white_bird is not None else None
        white_lane = dashes is not None and bool(dashes.any())
        if white_lane:
            lane_mask = cv2.bitwise_or(yellow_mask, dashes)

        # ---------- 4. 차선 찾기 ----------
        starts = self.find_starts(yellow_mask, lane_px, fork_bias, dashes if white_lane else None)
        ys, xs = lane_mask.nonzero()
        # 점선 사이 빈 칸을 건너뛰도록 점선 구간에서만 빈 창 허용 개수 늘림
        max_missing = c['white_dash_max_missing'] if white_lane else 1
        fits, spans, residuals, windows, solid, picked_sets, anchors, pixels = {}, {}, {}, [], {}, {}, {}, {}
        line_colors = {}
        zigzag = {}
        for side, x in starts.items():
            picked = self.sliding_window(ys, xs, x, h, self.scale, side, windows, max_missing)
            picked_sets[side] = np.asarray(picked)
            if len(picked) < 3 * c['minimum_pixels']:
                continue
            py, px = ys[picked], xs[picked]
            if np.ptp(py) < 0.16 * h:          # 너무 짧은 조각은 곡선으로 안 믿음
                continue
            zigzag[side] = self.zigzag_line(py, px)   # 재맞춤 전 픽셀로 판단 (재맞춤은 톱니 끝을 이상 픽셀로 버림)
            fit = np.polyfit(py, px, 2)          # x = a*y^2 + b*y + c (픽셀)
            # 이상 픽셀 빼고 재맞춤: 선에 붙은 가로 조각·잡음이 곡선을 끌어당김 (lap4 재생: 185~190 s 오차 점프 12 → 3회)
            near = np.abs(px - np.polyval(fit, py)) < c['refit_tolerance'] * self.scale
            if near.sum() >= 3 * c['minimum_pixels'] and not near.all():
                py, px = py[near], px[near]
                fit = np.polyfit(py, px, 2)
            fits[side], spans[side] = fit, (float(py.min()), float(py.max()))
            low = py >= py.max() - 0.1 / self.row_m                     # 가장 가까운 0.1 m 픽셀
            anchors[side] = (float(py[low].mean()), float(px[low].mean()))
            pixels[side] = (py[::4], px[::4])
            residuals[side] = float(np.sqrt(np.mean((px - np.polyval(fit, py)) ** 2))) / self.scale
            # 실선이면 위아래로 끊김이 거의 없음 → 차선 변경 금지 판단에 사용
            solid[side] = len(np.unique(py)) / (np.ptp(py) + 1) > c['solid_row_fraction']
            # 선 색: 픽셀 중 노란 비율 > line_yellow_ratio → 노랑 (3차선 바깥 경계), 아니면 흰 점선
            # 흰색인데 끊김 없는 실선이면 점선 아님(테두리 등) → 'white_solid' (차로 판단에 안 씀)
            if np.mean(yellow_mask[py, px] > 0) > c['line_yellow_ratio']:
                line_colors[side] = 'yellow'
            else:
                line_colors[side] = 'white_solid' if solid[side] else 'white'

        # ---------- 5. 좌우 판단 ---------- source: 확정 이름 → 원래 탐색 쪽
        source = self.label_sides(fits, lane_px, picked_sets, anchors, line_colors)
        if self.outer_yellow is not None and len(source) == 1 and not self.fork_direction:
            kept = next(iter(source.values()))
            if line_colors.get(kept) == 'yellow':
                # 차선 변경 중에도 추적 중인 교통섬 경계를 바깥선 이름으로 덮어쓰기 금지
                source = {self.remembered_side(anchors[kept]) or self.outer_yellow: kept}
        source = self.fork_sources(source, fits, anchors, line_colors, lane_px)
        # 이름별 기억: 이번에 본 선은 갱신, 못 본 선은 memory_frames 프레임까지 유지
        self.memory = {s: (r, x, age + 1) for s, (r, x, age) in self.memory.items() if age < c['memory_frames']}
        self.memory.update({s: (*pixels[k], 0) for s, k in source.items()})
        fits = {s: fits[k] for s, k in source.items()}
        spans = {s: spans[k] for s, k in source.items()}
        solid = {s: solid[k] for s, k in source.items()}
        line_colors = {s: line_colors[k] for s, k in source.items()}
        residuals = [residuals[k] for k in source.values()]

        result = dict(valid=False, error=0.0, heading=0.0, abnormal=False,
                      stop_distance=None, stop_candidates=[], center_px=w / 2, crosswalk_distance=None,
                      mask=bird, lane_mask=lane_mask, white_mask=white_bird, white_lane=white_lane, windows=windows,
                      fits={s: f.tolist() for s, f in fits.items()}, fit_spans=spans, solid=solid,
                      # 가장 먼 흰 점선까지 앞 거리(m). 없으면 None. 분기 입구(점선 끝) 판단용 (fork.py, 소민)
                      dash_end=float(self.meters_at(np.flatnonzero(dashes.any(axis=1)).min())) if white_lane else None,
                      colors=line_colors,
                      # 선별 가까운 끝 앞 거리(m)·가까운 끝 기울기(rad, + 왼쪽). 교통섬 노란 선 구분용 (lane_change.trusted_colors, 소민 11/14)
                      near_ends={s: float(self.meters_at(spans[s][1])) for s in spans},
                      line_headings={s: float(math.atan(float(np.polyval(np.polyder(fits[s]), spans[s][1])) / self.scale / self.row_m))
                                     for s in fits},
                      # 서행 유도선(지그재그): 2 = 서로 다른 좌우 선이 모두 톱니 모양, 1 = 찾은 선 중 하나라도, 0 = 없음
                      # 좌우 창이 같은 선에 붙은 경우는 1 (11/10 skip_1110_lap1 142 s: 현수막 주름으로 한 선이 물결 → 2로 세면 오검출)
                      zigzag=2 if len(source) == 2 and all(zigzag.get(k) for k in source.values()) else int(any(zigzag.values())))

        if not fits:
            # 못 찾으면 탐색 기준 중앙도 버림. 남겨두면 엉뚱한 기준 주변만 계속 찾아 영영 못 찾음 (도면 코너 시뮬: 정지)
            self.previous_center = None
        else:
            # lookahead 행의 차로 중앙·방향 (정지선 탐색, 시작점 기준, 차선 변경·주차용)
            row = self.row_at(c['lookahead'])
            centers, slopes = [], []
            for side, fit in fits.items():
                x = np.polyval(fit, row)
                centers.append(x + (lane_px / 2 if side == 'left' else -lane_px / 2))
                slopes.append(np.polyval(np.polyder(fit), row))          # 행당 x 변화(px/px)
            center = float(np.mean(centers))
            if self.previous_center is not None:
                center = c['smoothing'] * center + (1 - c['smoothing']) * self.previous_center
            # 두 선을 모두 찾았을 때만 기억. 틀린 중심을 기억하면 시작점 탐색이 그 자리에 고정됨
            self.previous_center = center if len(fits) == 2 else None
            # 차로 방향(rad, + 왼쪽): 전방 1m 갈 때 차선이 좌우로 얼마나 움직이는지
            heading = math.atan(float(np.mean(slopes)) / self.scale / self.row_m)
            # 가장 길게 보이는 선의 먼 끝 방향(rad, + 왼쪽). 코너 유지 시작 판단용 (lane_driving.corner_hold)
            # 진짜 우회전 코너는 먼 끝이 계속 오른쪽(−37~−75°), 갈림길 끝 S자 꺾임은 왼쪽으로 돌아옴(+1~+25°)
            longest = max(fits, key=lambda sd: spans[sd][1] - spans[sd][0])
            far_heading = math.atan(float(np.polyval(np.polyder(fits[longest]), spans[longest][0])) / self.scale / self.row_m)
            result.update(valid=True, far_heading=far_heading, near_curvature=self.near_curvature(longest), error=(w / 2 - center) / self.scale, heading=heading,
                          abnormal=max(residuals) > c['abnormal_fit_residual'],
                          center_px=center, forward=c['lookahead'], pursuit=False,
                          curvature=max(float(self.line_curvature(fits[sd], np.arange(spans[sd][0], spans[sd][1] + 1, 3.0)).max())
                                        for sd in fits))
            # ---------- 6. 목표점 ---------- 두 선이면 평균이라 코너에서 차로가 넓어져도(0.52 → 0.78 m) 중앙
            targets = [t for t in (self.pursuit_target(sd, fits[sd], spans[sd]) for sd in fits) if t is not None]
            if targets:
                forward, lateral = np.mean(targets, axis=0)
                result.update(error=float(lateral), forward=float(forward), pursuit=True)

        # ---------- 정지선 ----------
        # 차가 차로 방향에서 틀어져 있으면 BEV에서 정지선이 기울어 가로 열림(1행 × stop_min_width)에 통째로 지워짐
        # (11/16 main_1116_arc1 244.5~250.6 s: 4번 코너 뒤 7° 틀어진 채 접근 → 신호등 정지선 후보 0개 → 빨강 면적 예비 정지로
        #  선 약 0.6 m 앞 '천천히' 위 정지). 차선 방향으로 흰 마스크를 세로 밀기(shear)해 수평으로 편 뒤 탐색. 차로 중심 열은 그대로라 거리 불변
        white_source = bird if stop_bird is None else stop_bird
        self.find_stop_line(result, stop_lines, lane_px, white_source)
        # 원래 탐색이 우선. 횡단보도 앞 선을 못 찾았을 때만 편 마스크로 다시 찾아 보탬 (원래 찾던 선은 그대로 → 보행자 등 기존 동작 유지)
        level = self.stop_level(fits, spans, result['center_px'], w, h) if result['valid'] else None
        if level is not None and result['crosswalk_distance'] is None:
            leveled = cv2.warpAffine(white_source, level, (w, h), flags=cv2.INTER_NEAREST)
            extra = dict(stop_distance=None, stop_candidates=[], crosswalk_distance=None, center_px=result['center_px'])
            valid = self.valid_bird
            if valid is not None and valid.shape == leveled.shape:
                self.valid_bird = cv2.warpAffine(valid.astype(np.uint8), level, (w, h), flags=cv2.INTER_NEAREST) > 0
            leveled_source = leveled if seam_kernel is None else cv2.morphologyEx(leveled, cv2.MORPH_CLOSE, seam_kernel)
            self.find_stop_line(extra, cv2.morphologyEx(leveled_source, cv2.MORPH_OPEN, kernel), lane_px, leveled)
            self.valid_bird = valid
            result['crosswalk_distance'] = extra['crosswalk_distance']
            if result['stop_distance'] is None:
                result['stop_distance'] = extra['stop_distance']
            known = [cand['distance'] for cand in result['stop_candidates']]
            result['stop_candidates'] += [cand for cand in extra['stop_candidates']
                                          if all(abs(cand['distance'] - d) > 0.05 for d in known)]
        return result

    def stop_level(self, fits, spans, center, w, h):
        """정지선을 수평으로 펴는 세로 밀기 행렬 (없으면 None). 차선 기울기 s(px/행) → 차로에 수직인 선의 기울기 k(행/px) = s × (행/m ÷ px/m)²"""
        c = self.c
        limit = c.get('stop_level_max', 0.6)
        if not fits or not limit:
            return None
        slopes = [float(np.mean(np.polyval(np.polyder(fits[sd]), np.arange(spans[sd][0], spans[sd][1] + 1, 5.0)))) for sd in fits]
        k = float(np.clip(np.mean(slopes) * (1.0 / self.row_m / self.scale) ** 2, -limit, limit))
        if abs(k) < c.get('stop_level_min', 0.03):
            return None
        return np.float32([[1, 0, 0], [k, 1, -k * center]])

    @staticmethod
    def band_hist(mask):
        """아래쪽 30% 띠의 열별 픽셀 수 (9픽셀 이동평균)"""
        h = mask.shape[0]
        band = mask[int(h * 0.7):] > 0
        return np.convolve(band.sum(axis=0).astype(float), np.ones(9) / 9, mode='same')

    @staticmethod
    def peaks(hist, lo, hi, minimum=4):
        """[lo, hi) 안에서 minimum 이상인 연속 구간마다 최댓값 위치"""
        above = np.flatnonzero(hist[lo:hi] >= minimum)
        if not len(above):
            return []
        groups = np.split(above, np.flatnonzero(np.diff(above) > 1) + 1)
        return [lo + int(g[np.argmax(hist[lo + g])]) for g in groups]

    def forget_lines(self):
        """좌/우 선 기억 삭제 (11/14 유경)

        라바콘처럼 차선을 제대로 못 보는 구간을 지나는 동안 한 선에 붙은 이름이 틀려 있으면, 그 뒤로도
        기억 이어받기(single_name)가 계속 넘겨 고착됨 → 그 구간을 벗어날 때 한 번 비움
        """
        self.memory = {}

    def remembered_side(self, anchor):
        """기억한 선과 반 차로 이내인 가까운 끝의 이름. 없으면 None"""
        row, x = anchor
        distances = {side: float(np.min(np.hypot((x-mx)/self.scale, (row-mr)*self.row_m)))
                     for side, (mr, mx, _) in self.memory.items() if len(mr)}
        return min(distances, key=distances.get) if distances and min(distances.values()) < self.c['lane_width']/2 else None

    def fork_sources(self, source, fits, anchors, colors, lane_px):
        """출구 정렬 후 교통섬 경계 이름 보정. 선의 위치·선택 방향 휨으로 제한"""
        direction = self.fork_direction
        if not direction or not source:
            return source
        if len(source) == 2:
            row = max(anchors[k][0] for k in source.values())
            gap = abs(np.polyval(fits[source['left']], row)-np.polyval(fits[source['right']], row))
            if gap <= 1.5*lane_px:
                return source
            # 서로 다른 출구 선을 평균내지 않고 선택 출구 쪽 경계만 유지
            side = 'left' if direction > 0 else 'right'
            return {side: source[side]}
        kept = next(iter(source.values()))
        if colors.get(kept) != 'yellow':
            return source
        row, x = anchors[kept]
        # A/B 교통섬 경계의 꺾임 방향 반전만으로 추적 이름 변경 금지
        # 근거: run_1112_0326 45.7초 오른쪽 경계 → 47.9초 왼쪽 이름·오른쪽 목표
        remembered = self.remembered_side((row, x))
        if remembered is not None:
            return {remembered: kept}
        lateral = (self.c['width_px']/2-x)/self.scale
        heading = math.atan(float(np.polyval(np.polyder(fits[kept]), row))/self.scale/self.row_m)
        # 차 중앙 부근에서 선택 출구로 꺾이는 선 = 교통섬 쪽 경계 후보
        inner = direction*heading > .1 and direction*lateral < self.c['single_line_split']
        side = ('right' if direction > 0 else 'left') if inner else ('left' if direction > 0 else 'right')
        return {side: kept}

    def find_starts(self, lane_mask, lane_px, fork_bias, dashes=None):
        """아래쪽 띠의 흰 픽셀 개수(히스토그램)에서 좌/우 차선 시작 x를 고른다.

        dashes(흰 점선 BEV)가 있으면 칸마다 (3차선 구간)
          1) 예상 위치(기준 중앙 ± 차로 폭 절반)에서 start_expect_tolerance 안의 노란 봉우리 → 노란 선
             (일반 차선·옆 차로의 노란 바깥 경계는 기존과 같은 선)
          2) 없으면 흰 점선 봉우리 중 예상 위치에 가장 가까운 것
             (가운데 차로: 치우쳐서 노란 바깥선이 칸에 들어와도 점선 선택. 기존은 픽셀 많은 노란 선 선택)
          3) 둘 다 없으면 기존처럼 노란 최대
        dashes가 없으면 기존과 동일"""
        h, w = lane_mask.shape
        hist = self.band_hist(lane_mask)
        white_hist = self.band_hist(dashes) if dashes is not None else None
        tolerance = self.c['start_expect_tolerance'] * self.scale
        # 기준선: 이전 프레임 차로 중앙(없으면 화면 중앙). 갈림길에선 fork_bias만큼 이동
        center = self.previous_center if self.previous_center is not None else w / 2
        center -= fork_bias * w / self.c['view_width']
        starts = {}
        for side, (lo, hi) in {'left': (center - lane_px, center),
                               'right': (center, center + lane_px)}.items():
            lo, hi = int(clip(lo, 0, w - 1)), int(clip(hi, 1, w))
            if hi <= lo:
                continue
            if side == 'left' and self.double_left:
                # 마지막(4번) 코너 뒤: 왼쪽 칸에 출발 차로 아래 선과 지금 차로 왼쪽 선이 약 0.22 m 간격으로 함께 들어옴
                # 세기가 비슷해 픽셀 최대가 프레임마다 두 선을 오가고, 흰 점선 후보가 있으면 예상 위치 규칙이 옆 차로 선을 고름
                # → 옆 차로 선을 잡은 프레임은 목표가 지금 차로 왼쪽 선 위 (11/11 segA_1111_lap6 218.8~223.2 s, 조향 좌우 뒤집힘 4회)
                # 사이가 빈 강한 봉우리가 둘 이상이고 차 쪽 둘의 간격이 이중선 범위면 차 쪽 선에서 시작. 그 너머는 옆 차로 선
                # 모든 구간에 넣으면 코너에서 누운 선 하나가 여러 봉우리로 쪼개져 선을 놓침 (11/10 55577bf 되돌림) → 이 구간만
                found = self.peaks(hist, lo, hi)
                strong = sorted(x for x in found if hist[x] >= self.c['double_line_ratio'] * max(hist[p] for p in found))
                if (len(strong) >= 2 and self.c['double_line_gap_min'] * self.scale <= strong[-1] - strong[-2]
                        <= self.c['double_line_gap'] * self.scale):
                    starts[side] = strong[-1]              # x 클수록 오른쪽 = 차 쪽
                    continue
            if white_hist is not None:
                expected = center + (-lane_px / 2 if side == 'left' else lane_px / 2)
                yellow = [x for x in self.peaks(hist, lo, hi) if abs(x - expected) <= tolerance]
                white = self.peaks(white_hist, lo, hi)
                if yellow:
                    starts[side] = min(yellow, key=lambda x: abs(x - expected))
                    continue
                if white:
                    starts[side] = min(white, key=lambda x: abs(x - expected))
                    continue
            if hist[lo:hi].max() >= 4:
                starts[side] = lo + int(hist[lo:hi].argmax())
        # 갈림길: 따라갈 쪽 차선 하나만 사용 (+ 왼쪽, - 오른쪽)
        if fork_bias > 0.001:
            starts.pop('right', None)
        elif fork_bias < -0.001:
            starts.pop('left', None)
        return starts

    def label_sides(self, fits, lane_px, picked_sets, anchors, colors=None):
        """좌/우 차선 이름 확정. 반환: {확정 이름: 원래 탐색 쪽}

        11/8 실차: 코너에서 한 선만 남으면 시작점 탐색(화면 중앙 기준)이 이름을 잘못 붙이거나
        좌우 창이 같은 선에 붙음 → 오차 급변·조향 포화
        어느 선이 남는지는 차 자세에 따라 다름 (중앙 주행: 바깥 선 / 안쪽으로 파고듦: 안쪽 선) → 위치보다 연속성
        1) 좌우 창이 같은 픽셀을 30% 넘게 공유하거나 두 곡선 간격 < 차로 폭 절반이면 같은 선 → 하나만 남김
        2) 한 선이면 기억한 좌/우 선(각각 마지막으로 본 지 memory_frames 이내) 픽셀 중 새 선의 가까운 끝과
           가장 가까운 쪽 이름을 이어받음. 끝점끼리 비교하면 보이는 조각이 바뀔 때 놓침 (11/9 도면 코너 시뮬:
           한 프레임 못 찾음 → 기억 삭제 → 안쪽 선 한 번 보임 → 다시 보인 바깥 선을 오른쪽으로 → 좌회전)
        3) 이력 없으면 가까운 끝 좌우 위치로 판단 (경계는 선이 휘는 쪽으로 이동)
        판단 기준은 실제 픽셀의 가까운 끝(anchor). 곡선을 맨 아래 행까지 연장한 값은 선이 위에서 시작하면
        엉뚱한 곳을 가리킴 (11/8 실차 4회 76.1 s)
        """
        c = self.c
        rows = np.array([self.row_at(c['lookahead']), c['height_px'] - 1.0])
        at = {s: np.polyval(f, rows) for s, f in fits.items()}
        pair = None
        # 마지막 코너 뒤(double_left)이고 두 선 모두 노랑이면 이중선 기준을 double_line_gap으로 넓히고 이름은 항상 왼쪽
        # 그 구간 이중선 = 옆(출발) 차로 선 + 지금 차로 왼쪽 선뿐. 기준 0.26 m는 실제 간격 0.21~0.30 m와 겹쳐 정상 차로로 통과 → 목표가 두 선 사이
        # 이름을 한 선 위치로 정하면 차가 왼쪽 선 위에 있을 때(가까운 끝 0 근처) 오른쪽으로 보고 옆 차로 선을 남김
        beside = (self.double_left and len(fits) == 2 and colors is not None
                  and all(colors.get(sd) == 'yellow' for sd in fits))
        if len(fits) == 2:
            # 두 창이 같은 픽셀을 집었으면 같은 선. 곡선 맞춤 비교는 끝부분 맞춤 오차로 놓침 (11/8 실차 4회 76.5 s)
            a, b = picked_sets['left'], picked_sets['right']
            if len(np.intersect1d(a, b)) > 0.3 * min(len(a), len(b)):
                kept = max(fits, key=lambda sd: len(picked_sets[sd]))      # 픽셀 많은 쪽 곡선 사용
                fits = {kept: fits[kept]}
            elif np.mean(np.abs(at['left'] - at['right'])) >= (c['double_line_gap'] * self.scale if beside else lane_px / 2):
                return {'left': 'left', 'right': 'right'}
            else:
                # 픽셀 안 겹치는데 간격 < 차로 폭 절반 = 서로 다른 선 2개 (이중선). 도면 상단: 시작 차로 선과
                # 주차 진입 차로 선 간격 약 0.2 m. 픽셀 많은 쪽만 남기면 바깥 선을 차로 경계로 잡아 목표가
                # 안쪽 선 위로 감 (11/9 smooth_1109_lap1 271~275 s 마지막 코너 뒤 선 밟음)
                # → 이름은 픽셀 많은 선으로 정하고, 남기는 선은 차로 쪽 (왼쪽 선이면 오른쪽 것, 오른쪽 선이면 왼쪽 것)
                pair = {sd: float(np.mean(x)) for sd, x in at.items()}
                kept = max(fits, key=lambda sd: len(picked_sets[sd]))
                fits = {kept: fits[kept]}
        if not fits:
            return {}
        kept = 'left' if 'left' in fits else 'right'
        name = 'left' if beside and pair is not None else self.single_name(fits[kept], anchors[kept])
        if pair is not None:
            pick = max if name == 'left' else min                         # x 클수록 오른쪽
            kept = pick(pair, key=pair.get)
        return {name: kept}

    def single_name(self, fit, anchor):
        """한 선의 이름(left/right). 기억한 좌/우 선과 가까우면 이어받고, 없으면 가까운 끝 위치로"""
        c = self.c
        row, x = anchor
        lateral = (c['width_px'] / 2 - x) / self.scale                   # 가까운 끝 좌우 위치(m, + 왼쪽)
        if self.memory_first and self.memory:
            distances = {sd: float(np.min(np.hypot((x - mx) / self.scale, (row - mr) * self.row_m)))
                         for sd, (mr, mx, _) in self.memory.items()}
            if min(distances.values()) < c['lane_width'] / 2:
                return min(distances, key=distances.get)
        if self.corner_left_limit is not None:
            # 우회전 코너 유지 중·직후(loop가 설정): 안쪽 선은 오른쪽 사각으로 빠지고 앞에 남는 건 바깥(왼쪽) 선
            # 위치 경계(±0.14)로 나누면 차 앞 선을 오른쪽으로 오판 → 왼쪽 목표 → 바깥 선 넘음
            # (11/9 slow_1109_lap1 686.2 s 3번 코너, +0.07 m 선을 오른쪽으로 판단 → 688~690 s 좌회전·이탈)
            # 기억보다 먼저 적용: 유지 중 두 선 본 기억이 남은 선에 오른쪽 이름을 넘김
            # (11/9 main_1109_lap2b 1번 코너 103.4 s, 유지 끝 직후 r 판단 → 좌회전 +0.27·바깥 이탈)
            return 'left' if lateral > -self.corner_left_limit else 'right'
        if self.memory:
            distances = {sd: float(np.min(np.hypot((x - mx) / self.scale, (row - mr) * self.row_m)))
                         for sd, (mr, mx, _) in self.memory.items()}
            if min(distances.values()) < c['lane_width'] / 2:
                return min(distances, key=distances.get)
        curvature = -2 * fit[0] / self.row_m ** 2 / self.scale           # 선 곡률(1/m, + 왼쪽으로 휨)
        split = c['single_line_split'] * clip(curvature / c['split_curvature'], -1.0, 1.0)
        return 'left' if lateral > split else 'right'

    def pursuit_target(self, side, fit, span):
        """선 하나로 만든 중앙선 위 pure pursuit 목표점 (앞 m, 옆 m, + 왼쪽). 기준 = 뒷바퀴 축

        보이는 구간의 선 점마다 진행 방향 법선으로 차로 폭 절반만큼 안쪽 → 중앙선 점들.
        그중 뒷축에서 거리 pursuit_distance에 가장 가까운 점. 외삽 없음.
        11/8 실차 3회: 가장 가까운 선 점의 수직 발을 목표로 하면 차 방향이 차로와 어긋날 때
        목표가 차 옆에 생겨 되돌리는 힘이 없음 → 진행 방향으로 일정 거리 앞의 점 사용
        """
        c, scale, dy = self.c, self.scale, self.row_m
        rows = np.arange(span[0], span[1] + 1, 3.0)
        if len(rows) < 3:
            return None
        x = np.polyval(fit, rows)
        slope = np.polyval(np.polyder(fit), rows)                      # 행당 x 변화(px/px)
        forward = np.array([self.meters_at(r) for r in rows])
        lateral = (c['width_px'] / 2 - x) / scale
        # 앞으로 갈 때(행 −1) 진행 방향: 앞 +dy, 옆 +slope/scale
        tf, tl = np.full_like(rows, dy), slope / scale
        norm = np.hypot(tf, tl)
        tf, tl = tf / norm, tl / norm
        # 차로 폭: 도면상 코너 원호 구간은 넓어짐 (직선 0.52 → 코너 0.76~0.78 m, 안쪽 R590·바깥 R660 비동심)
        # 점마다 선 곡률(1/m)로 직선 폭 ↔ 코너 폭 보간. 한 선만 보일 때 중앙이 선 쪽으로 쏠리는 것 방지
        k = self.line_curvature(fit, rows)
        k0, k1 = c['corner_curvature']
        blend = np.clip((k - k0) / (k1 - k0), 0.0, 1.0)
        half = (c['lane_width'] + blend * (c['corner_lane_width'] - c['lane_width'])) / 2
        # 차로 안쪽 법선: 왼쪽 선이면 진행 방향의 오른쪽 (tl, −tf), 오른쪽 선이면 왼쪽 (−tl, tf)
        nf, nl = (tl, -tf) if side == 'left' else (-tl, tf)
        cf, cl = forward + half * nf, lateral + half * nl
        ahead = cf > 0.1
        if not ahead.any():
            return None
        cf, cl, tf, tl = cf[ahead], cl[ahead], tf[ahead], tl[ahead]
        distance = np.hypot(cf, cl)
        i = int(np.argmin(np.abs(distance - c['pursuit_distance'])))
        # 중앙선 전체가 원 밖이고 그 점에서 선이 크게 누움(덜 돈 코너 출구 등): 가장 가까운 점은 차 옆이라
        # 되돌리는 방향이 틀림 (11/9 lap4 169.7 s 우회전 직후 왼쪽 목표 → S자)
        # → 선을 따라 pursuit_ahead m 앞 점. 점 순서 = 먼 쪽 → 가까운 쪽이라 앞으로 = 번호 감소
        # 누운 선에만 적용: 직선에서도 쓰면 목표가 1 m 가까이 멀어져 선 개수 전환 때 오차 차이 커짐 (lap4 143 s)
        if distance.min() > c['pursuit_distance'] and math.degrees(math.atan2(abs(tl[i]), tf[i])) > c['pursuit_ahead_tilt']:
            arc = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(cf[:i + 1][::-1]), np.diff(cl[:i + 1][::-1])))])
            i -= int(min(np.searchsorted(arc, c['pursuit_ahead']), i))
        return float(cf[i]), float(cl[i])

    def zigzag_line(self, py, px):
        """선 하나가 지그재그(서행 유도선, 규정 미션 6)인지. 서행 판단용 (lane_driving)

        행을 zigzag_bin_rows씩 묶어 평균 x → 2차식 잔차(m)가 좌우로 번갈아 바뀌는 횟수와 크기로 판단
        11/10 녹화 skip_1110_lap1: 지그재그 104.5~109.5 s 바뀜 6~10회·잔차 0.016~0.039 m,
        코너 3~6회·잔차 0.06 m 이상(창이 다른 선으로 넘어감), 직선 0회·0.002 m → 잔차 상한으로 코너 제외
        """
        c = self.c
        bins = (py // c['zigzag_bin_rows']).astype(int)
        count = np.bincount(bins)
        used = count > 0
        if used.sum() < 8:
            return False
        forward = np.bincount(bins, py)[used] / count[used] * self.row_m     # 행 → m (기울기·잔차 단위만 필요)
        lateral = np.bincount(bins, px)[used] / count[used] / self.scale
        if np.ptp(forward) < c['zigzag_min_span']:
            return False
        residual = lateral - np.polyval(np.polyfit(forward, lateral, 2), forward)
        rms = float(np.sqrt(np.mean(residual ** 2)))
        signs = np.sign(residual[np.abs(residual) > c['zigzag_deadband']])
        crossings = int(np.count_nonzero(signs[1:] != signs[:-1]))
        low, high = c['zigzag_residual']
        return crossings >= c['zigzag_min_crossings'] and low <= rms <= high

    def near_curvature(self, side):
        """차 앞 near_curvature_distance(m) 안 픽셀만으로 맞춘 선 곡률(1/m, + 왼쪽 휨). 픽셀 부족하면 None
        전체 2차식 곡률은 먼 꺾임이 선 전체에 퍼져 가까운 휨 방향을 못 봄 (lane_driving.corner_hold 비스듬한 진입 판단)"""
        c = self.c
        if side not in self.memory or self.memory[side][2] != 0:
            return None
        py, px = self.memory[side][0], self.memory[side][1]
        forward = c['far'] - py / (c['height_px'] - 1) * (c['far'] - c['near'])
        m = forward <= c['near_curvature_distance']
        if m.sum() < 8 or np.ptp(forward[m]) < 0.25:
            return None
        fit = np.polyfit(py[m], px[m], 2)
        dxdf = np.polyval(np.polyder(fit), np.median(py[m])) / (self.scale * self.row_m)
        return float(-2 * fit[0] / self.scale / self.row_m ** 2 / (1 + dxdf ** 2) ** 1.5)

    def line_curvature(self, fit, rows):
        """선 곡률(1/m, 크기만) — 행마다. 2차식 x(행)을 바닥 좌표(앞 m, 옆 m)로 바꿔 계산"""
        dxdf = np.polyval(np.polyder(fit), rows) / (self.scale * self.row_m)   # 앞 1 m당 옆 변화
        return np.abs(2 * fit[0]) / self.scale / self.row_m ** 2 / (1 + dxdf ** 2) ** 1.5

    def sliding_window(self, ys, xs, x, h, scale, side, windows, max_missing=1):
        """아래에서 위로 창을 옮기며 차선 픽셀을 모은다. max_missing: 연속 빈 창 허용 개수 (점선 구간만 늘림)"""
        c = self.c
        margin, picked, last, missing = c['margin'] * scale, [], None, 0
        for i in range(c['windows']):
            low, high = h - (i + 1) * h // c['windows'], h - i * h // c['windows']
            windows.append(dict(side=side, x0=float(x - margin), x1=float(x + margin), y0=low, y1=high))
            inside = np.flatnonzero((ys >= low) & (ys < high) & (np.abs(xs - x) < margin))
            if len(inside) >= c['minimum_pixels']:
                mean = float(xs[inside].mean())
                # 직전 이동량만큼 다음 창을 미리 옮겨서 급커브를 따라감
                x = mean + (0.0 if last is None else clip(mean - last, -margin, margin))
                last = mean
                picked.extend(inside.tolist())
                missing = 0
            elif picked:
                missing += 1
                if missing > max_missing:          # 기본: 두 칸 연속 비면 중단
                    break
        return picked

    def crosswalk_beyond(self, white, top, x0, x1):
        """가로선 바로 너머(먼 쪽)에 세로 흰 블록(횡단보도 줄무늬)이 3개 이상 나란히 있으면 참 (소민 11/12)
        트랙: 흰 선(서행 시작) → ◇·천천히 → 흰 선(정지선) → 횡단보도 블록 2줄 → 흰 선.
        신호등·보행자는 '횡단보도 바로 앞 선'에 서야 하므로 블록이 붙은 선만 횡단보도 선으로 구분"""
        c = self.c
        rows_per_m = (c['height_px'] - 1) / (c['far'] - c['near'])
        start = int(top - c.get('crosswalk_gap', .01) * rows_per_m)
        end = int(top - c.get('crosswalk_depth', .12) * rows_per_m)
        if end < 0 or start <= end:
            return False   # 블록 볼 공간이 화면 밖: 아직 판단 못 함
        profile = np.mean(white[end:start, x0:x1] > 0, axis=0) > c.get('crosswalk_fill', .5)
        edges = np.diff(np.concatenate(([0], profile.astype(np.int8), [0])))
        widths = np.flatnonzero(edges == -1) - np.flatnonzero(edges == 1)
        blocks = int(np.sum(widths >= c.get('crosswalk_block_width', .03) * self.scale))
        return blocks >= c.get('crosswalk_blocks', 3)

    def find_stop_line(self, result, horizontal, lane_px, white=None):
        """차로 폭의 대부분을 가로지르는 가로 페인트 = 정지선. 차 앞축 기준 거리(m).
        너머에 횡단보도 블록이 있는 가장 가까운 선 = crosswalk_distance"""
        c = self.c
        h, w = horizontal.shape
        center = result['center_px']
        x0 = int(clip(center - 0.38 * lane_px, 0, w - 1))
        x1 = int(clip(center + 0.38 * lane_px, x0 + 1, w))
        rows = np.flatnonzero(np.mean(horizontal[:, x0:x1] > 0, axis=1) > 0.7)
        if not len(rows):
            return
        for group in reversed(np.split(rows, np.flatnonzero(np.diff(rows) > 1) + 1)):  # 가까운 것부터
            thickness = len(group) / (h - 1) * (c['far'] - c['near'])
            distance = self.meters_at(float(group.max())) - self.front_axle_x
            # 화면 가장자리에 잘린 선은 거리가 틀리므로 제외
            before, after = max(0, int(group.min()) - 2), min(h - 1, int(group.max()) + 2)
            complete = (self.valid_bird is None or self.valid_bird.shape != horizontal.shape or
                        bool(np.all(self.valid_bird[[before, after], x0:x1])))
            if distance > -0.04:
                crosswalk = white is not None and self.crosswalk_beyond(white, int(group.min()), x0, x1)
                result['stop_candidates'].append(dict(distance=float(distance), crosswalk=crosswalk,
                                                      thickness=float(thickness), complete=complete))
                if complete and thickness <= c['stop_max_thickness'] and result['stop_distance'] is None:
                    result['stop_distance'] = float(distance)
                if (crosswalk and complete and thickness <= c['stop_max_thickness'] and
                        result['crosswalk_distance'] is None):
                    result['crosswalk_distance'] = float(distance)

