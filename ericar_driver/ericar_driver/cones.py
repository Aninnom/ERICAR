"""라바콘: 안쪽 경로 계획"""
# 출처: hello-osy/2026-orda-mentoring src/orda_sim/orda_sim/answer.py (613c9b4)
import math
import numpy as np

from .lidar import body_points, footprint_clearance


def passes_inside_cones(path_forward, path_lateral, cones, road_center, config):
    """Pass each measured cone on the side toward the remembered road centre."""
    if not len(cones):
        return True
    if np.any(np.diff(path_forward) <= 0):
        return False
    side = road_center - cones[:, 1]
    future = ((cones[:, 0] > path_forward[0]) & (cones[:, 0] <= path_forward[-1]) &
              (np.abs(side) > config.get('cone_side_deadband', .02)))
    crossing = np.interp(cones[future, 0], path_forward, path_lateral)
    inside = (crossing - cones[future, 1]) * np.sign(side[future])
    return bool(np.all(inside >= config.get('cone_inside_margin', .01)))



def cone_steering(scan, road_heading, road_center, robot, config, previous):
    """라바콘 담당: 두 번 조향하는 짧은 경로 후보에서 충돌·바깥 통과를 제외."""
    points = body_points(scan, robot)
    centers = body_points({'points': scan['narrow_centers']}, robot)
    cosine, sine = math.cos(road_heading), math.sin(road_heading)
    cones = np.column_stack((centers[:, 0]*cosine+centers[:, 1]*sine,
                             -centers[:, 0]*sine+centers[:, 1]*cosine))
    distance = config['cone_planning_distance']
    step = distance/config['planning_steps']
    choices = np.linspace(-robot['max_steering'], robot['max_steering'], config['planning_steering_samples'])
    best = None
    for switch in config['cone_turn_distances']:
        for first in choices:
            for second in choices:
                x = y = theta = 0.0
                path = []
                for index in range(config['planning_steps']):
                    steering = first if index*step < switch else second
                    theta += math.tan(steering)/robot['wheelbase']*step
                    x += math.cos(theta)*step
                    y += math.sin(theta)*step
                    path.append((x+robot['wheelbase']/2*math.cos(theta),
                                 y+robot['wheelbase']/2*math.sin(theta), theta))
                path = np.asarray(path)
                forward = np.r_[robot['wheelbase']/2*cosine, path[:, 0]*cosine+path[:, 1]*sine]
                lateral = np.r_[-robot['wheelbase']/2*sine, -path[:, 0]*sine+path[:, 1]*cosine]
                if np.max(np.abs(path[:, 2]-road_heading)) > max(config['road_heading_limit'], abs(road_heading)+.03):
                    continue
                if not passes_inside_cones(forward, lateral, cones, road_center, config):
                    continue
                clearance = footprint_clearance(points, path, robot)
                if clearance < config['planning_margin']+config['small_object_clearance']:
                    continue
                cost = (2*(lateral[-1]-road_center)**2 + (theta-road_heading)**2 +
                        .8*(first-previous)**2 + .6*second**2 - .1*min(clearance, .5))
                if best is None or cost < best[0]:
                    best = cost, first
    return None if best is None else float(best[1])



class ConeDriving:
    """라바콘 담당: 좁은 라이다 군집을 인식하고 도로 중심 쪽으로 통과한다."""
    def __init__(self, config, robot):
        self.c, self.robot = config['lidar'], robot
        self.active = False
        self.remaining = self.heading = self.center = self.previous = 0.0
        self.travel = 0.0  # 라바콘 모드 시작 후 전진 거리
        self.done = False

    def update(self, lane, scan, velocity, yaw_rate, dt, enabled):
        if not enabled or self.done:
            self.active = False
            return None
        if self.active:
            self.heading -= yaw_rate*dt
            self.center += velocity*math.sin(self.heading)*dt
            self.travel += max(0.0, velocity)*dt
        detected = scan['narrow_objects'] > 0 and scan['wide_objects'] == 0
        if detected and not self.active:
            # 같은 쪽 라바콘 사이의 방향으로 도로 축을 잡는다.
            # 진입 코너의 곡선이나 라바콘에 가린 흰 선을 기준으로 삼지 않는다.
            cones = np.asarray(scan['narrow_centers'], dtype=float).reshape(-1, 2)
            directions = []
            for i, first in enumerate(cones):
                for second in cones[i+1:]:
                    delta = second-first if second[0] > first[0] else first-second
                    if abs(np.linalg.norm(delta)-self.c['cone_row_spacing']) < self.c['cone_spacing_tolerance']:
                        directions.append(math.atan2(delta[1], delta[0]))
            if len(cones) >= 3 and directions:
                heading = float(np.median(directions))
                if abs(heading) < .35:
                    cones += [self.robot['lidar']['x'], self.robot['lidar'].get('y', 0)]
                    left = -cones[:, 0]*math.sin(heading)+cones[:, 1]*math.cos(heading)
                    self.heading, self.center = heading, float((left.min()+left.max())/2)
                    self.active = True
                    self.travel = 0.0
        self.remaining = self.c['small_object_memory_distance'] if detected else max(0, self.remaining-abs(velocity)*dt)
        # 종료: 마지막 라바콘을 지나 small_object_memory_distance만큼 가거나(라바콘 끝 선 → 코너 약 0.3 m),
        # 안전장치로 라바콘 모드에서 cone_max_distance(라바콘 구간 약 2.1 m + 여유)를 넘게 주행하면 끝낸다.
        # 늦게 끝나면 차선 인식 없이 직진하다 다음 코너를 못 돈다.
        if self.active and (self.remaining <= 0 or self.travel >= self.c.get('cone_max_distance', 2.5)):
            self.done = True  # 구간 통과 후 표지판 기둥을 라바콘으로 다시 취급하지 않는다.
            self.active = False
        if not self.active:
            return None
        steering = cone_steering(scan, self.heading, self.center, self.robot, self.c, self.previous)
        if steering is None:
            return 0.0, 0.0
        self.previous = steering
        return self.c['avoidance_speed'], steering
