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



# ---------- 라바콘·3차선 공통 (유경) ----------

def to_road(points, heading):
    """차체 좌표 점(N×2) → 도로 기준 (앞 거리, 왼쪽 거리). heading = 차체에서 본 도로 방향(rad, + 왼쪽)"""
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    cosine, sine = math.cos(heading), math.sin(heading)
    return points[:, 0]*cosine+points[:, 1]*sine, -points[:, 0]*sine+points[:, 1]*cosine


def to_body(forward, left, heading):
    """도로 기준 (앞, 왼쪽) → 차체 좌표 (x, y)"""
    cosine, sine = math.cos(heading), math.sin(heading)
    return forward*cosine-left*sine, forward*sine+left*cosine


def pursuit_steering(x, y, robot):
    """뒷바퀴 축 기준 목표점 (x, y)를 지나는 원호 조향 (pure pursuit)"""
    steering = math.atan2(2*robot['wheelbase']*y, x*x+y*y)
    return float(max(-robot['max_steering'], min(robot['max_steering'], steering)))


def clear_steering(points, desired, robot, distance, margin, samples=33):
    """desired 조향으로 distance(m) 가는 동안 차체가 라이다 점에 margin보다 가까워지면
    desired에 가장 가까운 안전한 조향으로 바꾼다. 반환 (조향, 안전 여부)

    이미 margin 안에 물체가 있으면(옆 라바콘 등) 지금보다 더 가까워지지 않는 조향을 안전으로 본다.
    안전한 조향이 없으면 여유가 가장 큰 조향과 False.
    """
    if len(points) == 0:
        return float(desired), True
    here = np.array([[robot['wheelbase']/2, 0.0, 0.0]])
    need = min(margin, footprint_clearance(points, here, robot)-0.005)
    if path_clearance(points, desired, robot, distance) >= need:
        return float(desired), True
    choices = np.linspace(-robot['max_steering'], robot['max_steering'], samples)
    best, best_clearance = float(desired), -1.0
    for steering in choices[np.argsort(np.abs(choices-desired))]:
        clearance = path_clearance(points, steering, robot, distance)
        if clearance >= need:
            return float(steering), True
        if clearance > best_clearance:
            best, best_clearance = float(steering), clearance
    return best, False



def scan_sectors(ranges, angle_min, angle_increment, range_min, range_max, config):
    """Validate scan data and expose measured local obstacle clusters."""
    values = np.asarray(ranges, dtype=float)
    angles = angle_min + np.arange(len(values)) * angle_increment
    measured=np.isfinite(values)&(values>=range_min)&(values>0)&(values<range_max)
    if config.get('zero_is_no_return', False):
        # 실차 YDLidar는 반사가 없는 방향을 inf가 아니라 0으로 보낸다(한 프레임의 50~60%).
        # 그대로 두면 valid_fraction<0.5, front=0이 되어 속도가 항상 0으로 묶이므로 inf와 같게 취급한다.
        # 주의: 최소 측정거리보다 가까운 물체도 0으로 올 수 있다.
        values = np.where(values == 0.0, np.inf, values)
    # Positive infinity is a valid no-return. NaN/negative/zero is unknown and unsafe.
    clean = np.where(np.isposinf(values), range_max, values)
    valid = np.isfinite(clean) & (clean >= range_min)
    clean = np.where(valid, np.minimum(clean, range_max), 0.0)
    def minimum(center, half):
        delta = np.arctan2(np.sin(angles - center), np.cos(angles - center))
        in_sector = np.abs(delta) < half
        sector = clean[in_sector & valid]
        # 무효 점이 절반 이상인 구간은 안전하게 0(막힘)으로 본다.
        if len(sector) and np.mean(valid[in_sector]) >= .5:
            return float(np.min(sector))
        return 0.0
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
                parking_points=np.column_stack((clean[measured]*np.cos(angles[measured]),clean[measured]*np.sin(angles[measured]))),
                parking_rays=dict(angles=angles, distances=clean, valid=valid, measured=measured, increment=angle_increment),
                narrow_objects=narrow_objects, narrow_centers=narrow_centers,
                wide_objects=wide_objects, valid_fraction=float(np.mean(valid)))
