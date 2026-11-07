"""차선 인식: BEV 변환, sliding window, 점선 헤딩"""
# 출처: hello-osy/2026-orda-mentoring src/orda_sim/orda_sim/answer.py (613c9b4)
import math
import cv2
import numpy as np

from .common import clip


def estimate_dash_heading(bird, config):
    """Local direction agreed by separate thin paint components; no world pose."""
    h, w = bird.shape
    lateral_scale = w / config['view_width']
    forward_scale = (h - 1) / (config['far'] - config['near'])
    count, labels, stats, _ = cv2.connectedComponentsWithStats(bird)
    angles, weights = [], []
    for label in range(1, count):
        if stats[label, cv2.CC_STAT_AREA] < config['minimum_pixels']:
            continue
        x, y, width, height = stats[label, :4]
        rows, cols = np.nonzero(labels[y:y+height, x:x+width] == label)
        points = np.column_stack((config['far'] - (rows+y)/forward_scale,
                                  (w/2-cols-x)/lateral_scale))
        short, long = sorted(cv2.minAreaRect(points.astype(np.float32))[1])
        if long < config['dash_min_height'] or short > long * config['dash_max_aspect']:
            continue
        _, axes = np.linalg.eigh(np.cov(points.T))
        direction = axes[:, -1]
        if direction[0] < 0:
            direction = -direction
        angle = math.atan2(direction[1], direction[0])
        if abs(angle) > config['dash_heading_max_angle']:
            continue
        angles.append(angle)
        weights.append(min(long, config['dash_heading_length_cap']))
    if not angles:
        return None
    angles, weights = np.asarray(angles), np.asarray(weights)
    tolerance = config['dash_heading_angle_tolerance']
    support = np.array([weights[np.abs(angles-angle) < tolerance].sum() for angle in angles])
    agreed = np.abs(angles - angles[support.argmax()]) < tolerance
    if (np.count_nonzero(agreed) < config['dash_heading_min_components'] or
            support.max() < config['dash_heading_support_ratio'] * weights.sum()):
        return None
    return float(np.average(angles[agreed], weights=weights[agreed]))



class LaneVision:
    """Flat-ground camera projection preserves metres for stop-line distance."""
    def __init__(self, lane, camera, front_axle_x):
        self.c = lane
        self.camera = camera
        self.front_axle_x = front_axle_x
        self.maps = None
        self.previous_center = None
        self.previous_boundaries = {}
        self.previous_fits = {}
        self.valid_bird = None

    def ground_maps(self, image_shape):
        h, w = image_shape[:2]
        c = self.c
        rows, cols = np.indices((c['height_px'], c['width_px']), dtype=np.float32)
        forward = c['far'] - rows / (c['height_px'] - 1) * (c['far'] - c['near'])
        left = (0.5 - cols / (c['width_px'] - 1)) * c['view_width']
        camera = self.camera
        dx = forward - camera['x']
        left = left - camera.get('y', 0.0)
        pitch = camera['pitch']
        depth = math.cos(pitch) * dx + math.sin(pitch) * camera['z']
        up = math.sin(pitch) * dx - math.cos(pitch) * camera['z']
        focal = w / (2 * math.tan(math.radians(camera['hfov_deg']) / 2))
        safe_depth = np.maximum(depth, 0.001)
        map_x = w / 2 - focal * left / safe_depth
        map_y = h / 2 - focal * up / safe_depth
        map_x[depth <= 0] = -1
        map_y[depth <= 0] = -1
        return map_x.astype(np.float32), map_y.astype(np.float32)

    def row_at(self, forward):
        c = self.c
        return (c['far'] - forward) / (c['far'] - c['near']) * (c['height_px'] - 1)

    def meters_at(self, row):
        c = self.c
        return c['far'] - row / (c['height_px'] - 1) * (c['far'] - c['near'])

    def analyze(self, bgr, fork_bias=0.0):
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        white = cv2.inRange(hsv, np.array(self.c['white_min']), np.array(self.c['white_max']))
        if self.maps is None or self.maps[0] != bgr.shape[:2]:
            self.maps = (bgr.shape[:2], self.ground_maps(bgr.shape))
        map_x, map_y = self.maps[1]
        self.valid_bird = (map_x >= 1) & (map_x < bgr.shape[1]-1) & (map_y >= 1) & (map_y < bgr.shape[0]-1)
        bird = cv2.remap(white, *self.maps[1], cv2.INTER_NEAREST,
                         borderMode=cv2.BORDER_CONSTANT)
        return self.analyze_bird(bird, fork_bias)

    def analyze_bird(self, bird, fork_bias=0.0):
        """Separated from ROS so lane recovery and stop distance can be tested."""
        c = self.c
        h, w = bird.shape
        scale = w / c['view_width']
        lane_width = c['lane_width'] * scale
        # Horizontal paint is a stop line/crosswalk, not a longitudinal lane.
        horizontal = cv2.morphologyEx(bird, cv2.MORPH_OPEN,
                                      np.ones((1, max(3, int(c['stop_min_width'] * scale))), np.uint8))
        lane_mask = cv2.subtract(bird, horizontal)
        # A diamond is a closed white ring, not a lane boundary. Remove its
        # whole component before the near-field histogram chooses lane starts.
        count, labels, stats, _ = cv2.connectedComponentsWithStats(lane_mask)
        contours, hierarchy = cv2.findContours(lane_mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
        ring_labels = set()
        hole_area = c.get('closed_hole_min_area', .003) * scale * h / (c['far']-c['near'])
        if hierarchy is not None:
            for index, relation in enumerate(hierarchy[0]):
                parent = relation[3]
                if parent >= 0 and cv2.contourArea(contours[index]) >= hole_area:
                    x, y = contours[parent][0, 0]
                    ring_labels.add(int(labels[y, x]))
        clean = np.zeros_like(lane_mask)
        minimum_height = c.get('component_min_height', 0.24) / (c['far'] - c['near']) * h
        for label in range(1, count):
            if label in ring_labels:
                continue
            component_height = stats[label, cv2.CC_STAT_HEIGHT]
            component_width = stats[label, cv2.CC_STAT_WIDTH]
            x, y, width, height = stats[label, :4]
            rows, cols = np.nonzero(labels[y:y+height, x:x+width] == label)
            coordinates = np.column_stack((cols / scale, rows * (c['far']-c['near']) / (h-1))).astype(np.float32)
            short, long = sorted(cv2.minAreaRect(coordinates)[1])
            narrow_dash = long >= c['dash_min_height'] and short <= long * c['dash_max_aspect']
            if component_height >= minimum_height or narrow_dash:
                clean[labels == label] = 255
        lane_mask = clean
        occupied = np.flatnonzero(np.any(lane_mask > 0, axis=1))
        bottom = int(occupied.max()) + 1 if len(occupied) else h
        top = max(0, bottom - int(c.get('histogram_band_height', 0.20) / (c['far'] - c['near']) * h))
        histogram = np.sum(lane_mask[top:bottom] > 0, axis=0).astype(float)
        smooth = int(c.get('histogram_smooth_pixels', 9))
        histogram = np.convolve(histogram, np.ones(smooth) / smooth, mode='same')
        peaks = []
        peak_components = set()
        work = histogram.copy()
        while work.max() >= c.get('minimum_peak', 4) and len(peaks) < c.get('max_peaks', 8):
            peak = int(work.argmax())
            local = labels[top:bottom, max(0, peak-4):min(w, peak+5)]
            ids, counts = np.unique(local[local > 0], return_counts=True)
            component = int(ids[np.argmax(counts)]) if len(ids) else -1
            if component not in peak_components:
                peaks.append(peak)
                peak_components.add(component)
            radius = max(10, int(lane_width * c.get('peak_suppression_ratio', .75)))
            work[max(0, peak - radius):min(w, peak + radius + 1)] = 0
        peaks.sort()
        association_row = (top + bottom - 1) / 2
        previous_x = {side: float(np.polyval(fit, association_row)) for side, fit in self.previous_fits.items()}
        identity_gate = lane_width * c.get('identity_gate', .45)
        target = w / 2 - fork_bias * scale
        if self.previous_center is not None:
            weight = c.get('target_history_weight', .35)
            target = (1-weight)*target + weight*self.previous_center
        pairs = []
        for left in peaks:
            for right in peaks:
                gap = right - left
                if c.get('pair_width_min_ratio', .70)*lane_width < gap < c.get('pair_width_max_ratio', 1.75)*lane_width:
                    conflict = False
                    if abs(fork_bias) < .001:
                        for side, peak in [('left', left), ('right', right)]:
                            if previous_x:
                                previous_side = min(previous_x, key=lambda key: abs(previous_x[key]-peak))
                                if abs(previous_x[previous_side]-peak) < identity_gate and previous_side != side:
                                    conflict = True
                    if conflict:
                        continue
                    center = (left + right) / 2
                    cost = abs(center - target) + c.get('pair_width_weight', .3) * abs(gap - lane_width)
                    pairs.append((cost, left, right))
        starts = []
        if pairs:
            _, left, right = min(pairs)
            starts = [(left, 'left'), (right, 'right')]
        elif peaks:
            if previous_x:
                distance, nearest, side = min((abs(value-peak), peak, side)
                    for peak in peaks for side, value in previous_x.items())
                if distance >= identity_gate:
                    side = 'left' if nearest < w/2 else 'right'
            else:
                nearest = min(peaks, key=lambda x: abs(abs(x-target)-lane_width/2))
                side = 'left' if nearest < w/2 else 'right'
            starts = [(nearest, side)]
        nonzero_y, nonzero_x = lane_mask.nonzero()
        fits = {}
        fit_spans = {}
        solid_boundaries = {}
        fit_residuals = []
        windows = []
        for start, side in starts:
            current = float(start)
            selected = []
            last_mean = None
            missing_windows = 0
            for window in range(c['windows']):
                low = h - (window + 1) * h // c['windows']
                high = h - window * h // c['windows']
                inside = ((nonzero_y >= low) & (nonzero_y < high) &
                          (np.abs(nonzero_x - current) < c['margin'] * scale))
                windows.append(dict(side=side, x0=float(current-c['margin']*scale),
                                    x1=float(current+c['margin']*scale), y0=low, y1=high))
                indices = np.flatnonzero(inside)
                if len(indices) >= c['minimum_pixels']:
                    mean = float(np.mean(nonzero_x[indices]))
                    # Predict the next window along the observed boundary. This
                    # follows a tight bend without requiring a prewritten route.
                    change = 0.0 if last_mean is None else mean - last_mean
                    current = mean + clip(change, -c['margin'] * scale, c['margin'] * scale)
                    last_mean = mean
                    selected.extend(indices.tolist())
                    missing_windows = 0
                elif selected:
                    missing_windows += 1
                    if missing_windows > c.get('max_missing_windows', 1):
                        break
            if len(selected) >= c['minimum_pixels'] * c.get('fit_minimum_windows', 3):
                ys = nonzero_y[selected]
                xs = nonzero_x[selected]
                if np.ptp(ys) > h * c.get('fit_min_span', 0.16):
                    fits[side] = np.polyfit(ys, xs, 2)
                    fit_spans[side] = (float(ys.min()), float(ys.max()))
                    solid_boundaries[side] = len(np.unique(ys)) / (np.ptp(ys) + 1) > c.get('solid_row_fraction', .85)
                    fit_residuals.append(float(np.sqrt(np.mean((xs - np.polyval(fits[side], ys)) ** 2))) / scale)
        result = dict(valid=False, error=0.0, heading=0.0, abnormal=False,
                      stop_distance=None, stop_candidates=[], center_px=w / 2, mask=bird, lane_mask=lane_mask,
                      windows=windows, fits={side: fit.tolist() for side, fit in fits.items()},
                      fit_spans=fit_spans,
                      dash_heading=estimate_dash_heading(bird, c))
        if fits:
            self.previous_fits = {side: fit.copy() for side, fit in fits.items()}
            self.previous_boundaries = {side: float(np.polyval(fit, self.row_at(c['lookahead'])))
                                        for side, fit in fits.items()}
            # Shift each measured boundary by half a lane along its normal.
            # A horizontal pixel shift is wrong in tight bends and cuts corners.
            forwards, laterals = [], []
            boundaries = {}
            row_scale = (h - 1) / (c['far'] - c['near'])
            for side, fit in fits.items():
                rows = np.linspace(*fit_spans[side], int(c.get('fit_samples', 50)))
                forward = c['far'] - rows / row_scale
                lateral = (w / 2 - np.polyval(fit, rows)) / scale
                slope = np.polyval(np.polyder(fit), rows) * row_scale / scale
                normal_size = np.sqrt(1 + slope * slope)
                boundaries[side] = dict(fit=np.polyfit(forward, lateral, 2).tolist(),
                                        near=float(forward.min()), far=float(forward.max()))
                sign = 1 if side == 'left' else -1
                forwards.extend(forward + sign * c['lane_width'] / 2 * slope / normal_size)
                laterals.extend(lateral - sign * c['lane_width'] / 2 / normal_size)
            forwards, laterals = np.asarray(forwards), np.asarray(laterals)
            useful = (forwards > c['near']) & (forwards < c['far'])
            if np.count_nonzero(useful) < 5:
                return result
            forwards, laterals = forwards[useful], laterals[useful]
            center_fit = np.polyfit(forwards, laterals, 2)
            preview = clip(c['lookahead'], float(forwards.min()), float(forwards.max()))
            error = float(np.polyval(center_fit, preview))
            center = w / 2 - error * scale
            if self.previous_center is not None:
                center = c['smoothing'] * center + (1 - c['smoothing']) * self.previous_center
            self.previous_center = center
            error = (w / 2 - center) / scale
            heading = math.atan(float(np.polyval(np.polyder(center_fit), preview)))
            # Smooth corners also have large curvature. Zigzag paint instead
            # leaves a large residual around its smooth fitted boundary.
            residual = max(fit_residuals) if fit_residuals else 0.0
            result.update(valid=True, error=error, heading=heading,
                          abnormal=residual > c.get('abnormal_fit_residual', 0.035), center_px=center, forward=preview, solid=solid_boundaries, boundaries=boundaries,
                          center_fit=center_fit.tolist(), fit_near=float(forwards.min()), fit_far=float(forwards.max()))
        # A stripe must span the selected lane, not merely a parallel lane.
        center = result['center_px']
        x0 = int(clip(center - lane_width * c.get('stop_lane_width_ratio', .38), 0, w - 1))
        x1 = int(clip(center + lane_width * c.get('stop_lane_width_ratio', .38), x0 + 1, w))
        rows = np.flatnonzero(np.mean(horizontal[:, x0:x1] > 0, axis=1) > c.get('stop_row_coverage', .7))
        if len(rows):
            groups = np.split(rows, np.flatnonzero(np.diff(rows) > 1) + 1)
            for group in reversed(groups):
                thickness = len(group) / (h - 1) * (c['far'] - c['near'])
                distance = self.meters_at(float(group.max())) - self.front_axle_x
                complete = True
                if self.valid_bird is not None and self.valid_bird.shape == bird.shape:
                    before = max(0, int(group.min())-int(c.get('stop_edge_margin_pixels', 2)))
                    after = min(h-1, int(group.max())+int(c.get('stop_edge_margin_pixels', 2)))
                    complete = bool(np.all(self.valid_bird[[before, after], x0:x1]))
                if distance > -0.04:
                    result['stop_candidates'].append(dict(distance=float(distance), thickness=float(thickness), complete=complete))
                    if complete and thickness <= c['stop_max_thickness'] and result['stop_distance'] is None:
                        result['stop_distance'] = float(distance)
        return result
