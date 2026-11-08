"""차선 인식: BEV 변환, sliding window, 정지선 [간단 버전]"""
# 출처: hello-osy/2026-orda-mentoring answer.py (613c9b4), 차선 간단 버전은 소민 realcar 브랜치 66362f3
import math
import cv2
import numpy as np

from .common import clip


class LaneVision:
    """차선 담당①: 카메라 → 차선 오차(m). [간단 버전]

    처리 순서 (analyze):
      1. 색 검출   : HSV에서 차선 색(white_min~white_max)만 남김
      2. 버드아이뷰 : 카메라 장착값으로 바닥을 위에서 본 이미지로 변환 (1픽셀 = 고정 m)
      3. 가로선 분리: 가로로 긴 페인트(정지선)는 차선에서 빼고 정지선 검출에 사용
      4. 차선 찾기 : 아래쪽 히스토그램으로 좌/우 시작점 → 슬라이딩 윈도우로 위로 추적
      5. 오차 계산 : 차선 곡선을 2차식으로 맞추고, 앞 lookahead(m) 지점에서
                    차로 중앙과 차 중심의 좌우 거리 = error (+ 왼쪽)
    반환 dict의 키 이름은 원본과 같아서 Driver·LaneChange·Parking·viewer는 그대로 동작.
    """
    def __init__(self, lane, camera, front_axle_x):
        self.c = lane
        self.camera = camera
        self.front_axle_x = front_axle_x
        self.maps = None
        self.valid_bird = None
        self.previous_center = None   # 이전 프레임 차로 중앙(px), 흔들림 완화용
        self.previous_fits = {}       # 이전 프레임 좌/우 차선식, 시작점 추적용
        self.previous_boundaries = {}  # (AnswerNode가 초기화할 때 쓰는 이름 유지)

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
        color = cv2.inRange(hsv, np.array(self.c['white_min']), np.array(self.c['white_max']))
        if self.maps is None or self.maps[0] != bgr.shape[:2]:
            self.maps = (bgr.shape[:2], self.ground_maps(bgr.shape))
        map_x, map_y = self.maps[1]
        self.valid_bird = (map_x >= 1) & (map_x < bgr.shape[1]-1) & (map_y >= 1) & (map_y < bgr.shape[0]-1)
        bird = cv2.remap(color, map_x, map_y, cv2.INTER_NEAREST,           # 2. 버드아이뷰
                         borderMode=cv2.BORDER_CONSTANT)
        return self.analyze_bird(bird, fork_bias)

    def analyze_bird(self, bird, fork_bias=0.0):
        """ROS 없이 테스트할 수 있도록 버드아이뷰 이후 처리를 분리."""
        c = self.c
        h, w = bird.shape
        scale = w / c['view_width']              # 1 m = scale 픽셀
        lane_px = c['lane_width'] * scale        # 차로 폭(픽셀)

        # ---------- 3. 가로선(정지선) 분리 ----------
        kernel = np.ones((1, max(3, int(c['stop_min_width'] * scale))), np.uint8)
        horizontal = cv2.morphologyEx(bird, cv2.MORPH_OPEN, kernel)
        lane_mask = cv2.subtract(bird, horizontal)

        # ---------- 4. 차선 찾기 ----------
        starts = self.find_starts(lane_mask, lane_px, fork_bias)
        ys, xs = lane_mask.nonzero()
        fits, spans, residuals, windows, solid = {}, {}, [], [], {}
        for side, x in starts.items():
            picked = self.sliding_window(ys, xs, x, h, scale, side, windows)
            if len(picked) < 3 * c['minimum_pixels']:
                continue
            py, px = ys[picked], xs[picked]
            if np.ptp(py) < 0.16 * h:          # 너무 짧은 조각은 곡선으로 안 믿음
                continue
            fit = np.polyfit(py, px, 2)          # x = a*y^2 + b*y + c (픽셀)
            fits[side], spans[side] = fit, (float(py.min()), float(py.max()))
            residuals.append(float(np.sqrt(np.mean((px - np.polyval(fit, py)) ** 2))) / scale)
            # 실선이면 위아래로 끊김이 거의 없음 → 차선 변경 금지 판단에 사용
            solid[side] = len(np.unique(py)) / (np.ptp(py) + 1) > c['solid_row_fraction']

        result = dict(valid=False, error=0.0, heading=0.0, curvature=0.0, abnormal=False,
                      stop_distance=None, stop_candidates=[], center_px=w / 2,
                      mask=bird, lane_mask=lane_mask, windows=windows,
                      fits={s: f.tolist() for s, f in fits.items()}, fit_spans=spans, solid=solid)

        # ---------- 5. 오차 계산 ----------
        if fits:
            self.previous_fits = {s: f.copy() for s, f in fits.items()}
            row = self.row_at(c['lookahead'])
            dy = (c['far'] - c['near']) / (h - 1)  # 1픽셀 행 = dy m
            centers, slopes = [], []
            for side, fit in fits.items():
                x = np.polyval(fit, row)
                slope = np.polyval(np.polyder(fit), row)   # 행당 x 변화(px/px)
                # 차선에서 차로 폭 절반만큼 안쪽이 차로 중앙
                centers.append(x + (lane_px / 2 if side == 'left' else -lane_px / 2))
                slopes.append(slope)
            center = float(np.mean(centers))
            if self.previous_center is not None:
                center = c['smoothing'] * center + (1 - c['smoothing']) * self.previous_center
            # 두 선을 모두 찾았을 때만 기억. 한 선만이면 좌우 판단이 틀릴 수 있고, 틀린 중심을 기억하면
            # 다음 시작점 탐색이 그 자리에 고정되어 계속 한 선만 잡힘 (11/8 실차: 오차 +0.55 m 고정)
            self.previous_center = center if len(fits) == 2 else None
            # 차로 방향(rad, + 왼쪽): 전방 1m 갈 때 차선이 좌우로 얼마나 움직이는지
            heading = math.atan(float(np.mean(slopes)) / scale / dy)
            # 곡률(1/m, + 왼쪽): 2차식 x = a·행² + ... 의 a는 곡선 전체에서 부호 일정 →
            # heading(가까운 끝 기울기)과 달리 "가까이 직선, 멀리 휨"에서도 휘는 방향이 뒤집히지 않음
            curvature = float(np.mean([-2 * f[0] / dy ** 2 / scale for f in fits.values()]))
            result.update(valid=True, error=(w / 2 - center) / scale, heading=heading, curvature=curvature,
                          abnormal=max(residuals) > c['abnormal_fit_residual'],
                          center_px=center, forward=c['lookahead'])

        # ---------- 정지선 ----------
        self.find_stop_line(result, horizontal, lane_px)
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

