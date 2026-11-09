"""차선 인식: BEV 변환, sliding window, 정지선 [간단 버전]"""
# 출처: hello-osy/2026-orda-mentoring answer.py (613c9b4), 차선 간단 버전은 소민 realcar 브랜치 66362f3
# 11/8 코너: 목표점 pure pursuit·좌우 판단 추가 (sessions 차선 기록, PR #8)
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
        return self.analyze_bird(bird, fork_bias, white_bird)

    def white_dashes(self, white_bird):
        """흰 BEV에서 점선 조각만 남김. 흰 덩어리마다 모양을 m 단위로 재서
        길이 ≥ white_dash_min_length, 폭 ≤ white_dash_max_width,
        진행 방향(세로)에서 기울기 ≤ white_dash_max_tilt(도)인 것만 통과.
        (예전 세로 막대 필터는 점선이 약 10°만 기울어도 지워졌음)"""
        c = self.c
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
            if (length >= c['white_dash_min_length'] and width <= c['white_dash_max_width']
                    and tilt <= c['white_dash_max_tilt']):
                keep[y:y + bh, x:x + bw][box] = 255
        return keep

    def analyze_bird(self, bird, fork_bias=0.0, white_bird=None):
        """ROS 없이 테스트할 수 있도록 버드아이뷰 이후 처리를 분리.
        white_bird: 흰색 BEV (정지선용). None이면 예전처럼 노란 가로선에서 정지선 탐색"""
        c = self.c
        h, w = bird.shape
        lane_px = c['lane_width'] * self.scale   # 차로 폭(픽셀)

        # ---------- 3. 가로선(정지선) 분리 ----------
        kernel = np.ones((1, max(3, int(c['stop_min_width'] * self.scale))), np.uint8)
        horizontal = cv2.morphologyEx(bird, cv2.MORPH_OPEN, kernel)
        lane_mask = cv2.subtract(bird, horizontal)
        # 정지선은 흰 페인트의 가로선에서 찾음. 차선(lane_mask)은 노랑 그대로
        stop_lines = horizontal if white_bird is None else cv2.morphologyEx(white_bird, cv2.MORPH_OPEN, kernel)
        # 3차선 가운데 차로: 노란 바깥선이 약 0.78 m 밖이라 탐색 범위(차로 폭 1개) 밖 → 노란 후보가 하나도 없음
        # 그때만 흰 세로 획(차로 구분 점선)을 차선 후보에 추가. 노란 선이 보이는 곳(코너 등)은 기존과 동일
        white_lane = False
        if white_bird is not None and not self.find_starts(lane_mask, lane_px, fork_bias):
            # 길쭉하고 가는 흰 조각만 남김 (기울어도 됨) → 마름모 고리·글씨 조각·정지선(가로) 제외
            dashes = self.white_dashes(white_bird)
            lane_mask = cv2.bitwise_or(lane_mask, dashes)
            white_lane = True

        # ---------- 4. 차선 찾기 ----------
        starts = self.find_starts(lane_mask, lane_px, fork_bias)
        ys, xs = lane_mask.nonzero()
        fits, spans, residuals, windows, solid, picked_sets, anchors, pixels = {}, {}, {}, [], {}, {}, {}, {}
        for side, x in starts.items():
            picked = self.sliding_window(ys, xs, x, h, self.scale, side, windows)
            picked_sets[side] = np.asarray(picked)
            if len(picked) < 3 * c['minimum_pixels']:
                continue
            py, px = ys[picked], xs[picked]
            if np.ptp(py) < 0.16 * h:          # 너무 짧은 조각은 곡선으로 안 믿음
                continue
            fit = np.polyfit(py, px, 2)          # x = a*y^2 + b*y + c (픽셀)
            fits[side], spans[side] = fit, (float(py.min()), float(py.max()))
            low = py >= py.max() - 0.1 / self.row_m                     # 가장 가까운 0.1 m 픽셀
            anchors[side] = (float(py[low].mean()), float(px[low].mean()))
            pixels[side] = (py[::4], px[::4])
            residuals[side] = float(np.sqrt(np.mean((px - np.polyval(fit, py)) ** 2))) / self.scale
            # 실선이면 위아래로 끊김이 거의 없음 → 차선 변경 금지 판단에 사용
            solid[side] = len(np.unique(py)) / (np.ptp(py) + 1) > c['solid_row_fraction']

        # ---------- 5. 좌우 판단 ---------- source: 확정 이름 → 원래 탐색 쪽
        source = self.label_sides(fits, lane_px, picked_sets, anchors)
        # 이름별 기억: 이번에 본 선은 갱신, 못 본 선은 memory_frames 프레임까지 유지
        self.memory = {s: (r, x, age + 1) for s, (r, x, age) in self.memory.items() if age < c['memory_frames']}
        self.memory.update({s: (*pixels[k], 0) for s, k in source.items()})
        fits = {s: fits[k] for s, k in source.items()}
        spans = {s: spans[k] for s, k in source.items()}
        solid = {s: solid[k] for s, k in source.items()}
        residuals = [residuals[k] for k in source.values()]

        result = dict(valid=False, error=0.0, heading=0.0, abnormal=False,
                      stop_distance=None, stop_candidates=[], center_px=w / 2,
                      mask=bird, lane_mask=lane_mask, white_mask=white_bird, white_lane=white_lane, windows=windows,
                      fits={s: f.tolist() for s, f in fits.items()}, fit_spans=spans, solid=solid)

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
            result.update(valid=True, error=(w / 2 - center) / self.scale, heading=heading,
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
        self.find_stop_line(result, stop_lines, lane_px)
        return result

    def find_starts(self, lane_mask, lane_px, fork_bias):
        """아래쪽 띠의 흰 픽셀 개수(히스토그램)에서 좌/우 차선 시작 x를 고른다."""
        h, w = lane_mask.shape
        band = lane_mask[int(h * 0.7):] > 0
        hist = np.convolve(band.sum(axis=0).astype(float), np.ones(9) / 9, mode='same')
        # 기준선: 이전 프레임 차로 중앙(없으면 화면 중앙). 갈림길에선 fork_bias만큼 이동
        center = self.previous_center if self.previous_center is not None else w / 2
        center -= fork_bias * w / self.c['view_width']
        starts = {}
        for side, (lo, hi) in {'left': (center - lane_px, center),
                               'right': (center, center + lane_px)}.items():
            lo, hi = int(clip(lo, 0, w - 1)), int(clip(hi, 1, w))
            if hi > lo and hist[lo:hi].max() >= 4:
                starts[side] = lo + int(hist[lo:hi].argmax())
        # 갈림길: 따라갈 쪽 차선 하나만 사용 (+ 왼쪽, - 오른쪽)
        if fork_bias > 0.001:
            starts.pop('right', None)
        elif fork_bias < -0.001:
            starts.pop('left', None)
        return starts

    def label_sides(self, fits, lane_px, picked_sets, anchors):
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
        if len(fits) == 2:
            # 두 창이 같은 픽셀을 집었으면 같은 선. 곡선 맞춤 비교는 끝부분 맞춤 오차로 놓침 (11/8 실차 4회 76.5 s)
            a, b = picked_sets['left'], picked_sets['right']
            if len(np.intersect1d(a, b)) > 0.3 * min(len(a), len(b)):
                kept = max(fits, key=lambda sd: len(picked_sets[sd]))      # 픽셀 많은 쪽 곡선 사용
                fits = {kept: fits[kept]}
            elif np.mean(np.abs(at['left'] - at['right'])) >= lane_px / 2:
                return {'left': 'left', 'right': 'right'}
        if not fits:
            return {}
        kept = 'left' if 'left' in fits else 'right'
        row, x = anchors[kept]
        if self.memory:
            distances = {sd: float(np.min(np.hypot((x - mx) / self.scale, (row - mr) * self.row_m)))
                         for sd, (mr, mx, _) in self.memory.items()}
            if min(distances.values()) < c['lane_width'] / 2:
                return {min(distances, key=distances.get): kept}
        lateral = (c['width_px'] / 2 - x) / self.scale                   # 가까운 끝 좌우 위치(m, + 왼쪽)
        curvature = -2 * fits[kept][0] / self.row_m ** 2 / self.scale    # 선 곡률(1/m, + 왼쪽으로 휨)
        split = c['single_line_split'] * clip(curvature / c['split_curvature'], -1.0, 1.0)
        return {('left' if lateral > split else 'right'): kept}

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
        cf, cl = cf[ahead], cl[ahead]
        i = int(np.argmin(np.abs(np.hypot(cf, cl) - c['pursuit_distance'])))
        return float(cf[i]), float(cl[i])

    def line_curvature(self, fit, rows):
        """선 곡률(1/m, 크기만) — 행마다. 2차식 x(행)을 바닥 좌표(앞 m, 옆 m)로 바꿔 계산"""
        dxdf = np.polyval(np.polyder(fit), rows) / (self.scale * self.row_m)   # 앞 1 m당 옆 변화
        return np.abs(2 * fit[0]) / self.scale / self.row_m ** 2 / (1 + dxdf ** 2) ** 1.5

    def sliding_window(self, ys, xs, x, h, scale, side, windows):
        """아래에서 위로 창을 옮기며 차선 픽셀을 모은다."""
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
                if missing > 1:                    # 두 칸 연속 비면 중단
                    break
        return picked

    def find_stop_line(self, result, horizontal, lane_px):
        """차로 폭의 대부분을 가로지르는 가로 페인트 = 정지선. 차 앞축 기준 거리(m)."""
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
                result['stop_candidates'].append(dict(distance=float(distance),
                                                      thickness=float(thickness), complete=complete))
                if complete and thickness <= c['stop_max_thickness'] and result['stop_distance'] is None:
                    result['stop_distance'] = float(distance)

