"""라이다 공통: 차체 기준 점, 경로 여유, 섹터 거리"""
# 출처: hello-osy/2026-orda-mentoring src/orda_sim/orda_sim/answer.py (613c9b4)
import math
import numpy as np


def body_points(scan, robot):
    points = np.asarray(scan['points'], dtype=float).reshape(-1, 2).copy()
    points += [robot['lidar']['x'], robot['lidar'].get('y', 0.0)]
    return points



def footprint_clearance(points, path, robot):
    """차량을 점이 아닌 직사각형으로 검사한다. path의 좌표는 차체 중심."""
    if len(points) == 0:
        return float('inf')
    dx, dy = points[None, :, 0]-path[:, None, 0], points[None, :, 1]-path[:, None, 1]
    cosine, sine = np.cos(path[:, 2, None]), np.sin(path[:, 2, None])
    longitudinal = np.maximum(np.abs(dx*cosine+dy*sine)-robot['length']/2, 0)
    lateral = np.maximum(np.abs(-dx*sine+dy*cosine)-robot['width']/2, 0)
    return float(np.hypot(longitudinal, lateral).min())



def path_clearance(points, steering, robot, distance):
    curvature = math.tan(steering)/robot['wheelbase']
    travel = np.linspace(0, distance, 16)
    angle = travel*curvature
    x = travel if abs(curvature) < 1e-6 else np.sin(angle)/curvature
    y = np.zeros_like(travel) if abs(curvature) < 1e-6 else (1-np.cos(angle))/curvature
    path = np.column_stack((x+robot['wheelbase']/2*np.cos(angle),
                            y+robot['wheelbase']/2*np.sin(angle), angle))
    return footprint_clearance(points, path, robot)



def scan_sectors(ranges, angle_min, angle_increment, range_min, range_max, config):
    """Validate scan data and expose measured local obstacle clusters."""
    values = np.asarray(ranges, dtype=float)
    angles = angle_min + np.arange(len(values)) * angle_increment
    # Positive infinity is a valid no-return. NaN/negative/zero is unknown and unsafe.
    clean = np.where(np.isposinf(values), range_max, values)
    valid = np.isfinite(clean) & (clean >= range_min)
    clean = np.where(valid, np.minimum(clean, range_max), 0.0)
    def minimum(center, half):
        delta = np.arctan2(np.sin(angles - center), np.cos(angles - center))
        sector = clean[np.abs(delta) < half]
        return float(np.min(sector)) if len(sector) else 0.0
    front = minimum(0.0, config['front_half_angle'])
    rear = minimum(math.pi, config['front_half_angle'])
    hits = valid & (clean < min(range_max, config.get('planning_point_range', 3.0)))
    points = np.column_stack((clean[hits] * np.cos(angles[hits]), clean[hits] * np.sin(angles[hits])))
    narrow_objects = 0
    wide_objects = 0
    narrow_centers = []
    if len(points):
        breaks = np.flatnonzero(np.linalg.norm(np.diff(points, axis=0), axis=1) > config.get('cluster_gap', .12)) + 1
        for cluster in np.split(points, breaks):
            if len(cluster) < config.get('cluster_min_points', 3):
                continue
            width = float(np.linalg.norm(np.max(cluster, axis=0) - np.min(cluster, axis=0)))
            center = np.mean(cluster, axis=0)
            if width > config.get('small_object_max_width', .12) and 0 < center[0] < config.get('small_object_range', 1.8) and abs(center[1]) < config.get('wide_object_half_width', .9):
                wide_objects += 1
            if (config.get('small_object_min_width', .025) <= width <= config.get('small_object_max_width', .12) and
                    0 < center[0] < config.get('small_object_range', 1.8) and abs(center[1]) < config.get('small_object_half_width', .75)):
                narrow_objects += 1
                narrow_centers.append(center.tolist())
    return dict(front=front, rear=rear, points=points,
                narrow_objects=narrow_objects, narrow_centers=narrow_centers,
                wide_objects=wide_objects, valid_fraction=float(np.mean(valid)))
