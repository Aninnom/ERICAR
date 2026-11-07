"""3차선 정적 장애물: 전방 차량 감지 후 차선 변경"""
# 출처: hello-osy/2026-orda-mentoring src/orda_sim/orda_sim/answer.py (613c9b4)
import math
import numpy as np

from .common import clip
from .lidar import body_points, path_clearance


class LaneChange:
    """차선 담당②: 전방 차량 감지 → 빈 옆 차로 선택 → 한 차로 이동 → 차선 복귀.

    offset은 카메라로 측정한 도로 방향 기준 횡이동 거리. odom의 속도만 적분한다.
    지도 좌표나 장애물 차량의 배치 좌표는 사용하지 않는다.
    """
    def __init__(self, config, robot):
        self.c, self.robot = config['lane_change'], robot
        self.active = False
        self.index = self.c['initial_lane']
        self.heading = self.offset = self.target = self.travel = 0.0
        self.waiting = False

    def update(self, lane, scan, velocity, yaw_rate, dt, enabled, requested=0):
        self.waiting = False
        points = body_points(scan, self.robot)
        self.heading -= yaw_rate*dt
        if self.active:
            self.offset -= velocity*math.sin(self.heading)*dt
            self.travel += abs(velocity)*dt
            error = self.target-self.offset
            if abs(error) < self.c['finish_error'] and abs(self.heading) < self.c['finish_heading']:
                self.active = False
                self.index += 1 if self.target > 0 else -1
                return None
            distance = self.c['lookahead']
            x = distance*math.cos(self.heading)-error*math.sin(self.heading)
            y = distance*math.sin(self.heading)+error*math.cos(self.heading)
            steering = clip(math.atan2(2*self.robot['wheelbase']*y, x*x+y*y),
                            -self.robot['max_steering'], self.robot['max_steering'])
            safe = path_clearance(points, steering, self.robot, self.c['safety_horizon']) >= self.c['margin']
            speed = self.c['speed'] if safe and self.travel < self.c['max_distance'] else 0.0
            return speed, steering
        if not enabled or not lane['valid'] or (not requested and not scan['wide_objects']):
            return None
        # 갈림길의 꺾인 경계선 대신 직전 차선 변경의 도로 방향을 이어 쓴다.
        heading = self.heading if requested else lane['heading']
        forward = points[:, 0]*math.cos(heading)+points[:, 1]*math.sin(heading)
        left = -points[:, 0]*math.sin(heading)+points[:, 1]*math.cos(heading)
        # 차체가 진행할 통로 안의 전방 장애물만 확인한다.
        blocked = np.any((forward > 0) & (forward < self.c['trigger_distance']) &
                         (np.abs(left) < self.robot['width']/2+self.c['margin']))
        if not blocked and not requested:
            return None
        directions = [(requested, 'left' if requested > 0 else 'right')] if requested else [(1, 'left'), (-1, 'right')]
        for direction, side in directions:
            if not 0 <= self.index+direction < self.c['lane_count'] or (not requested and lane.get('solid', {}).get(side, False)):
                continue
            target = direction*self.c['lane_width']
            occupied = (forward > -self.robot['length']) & (forward < self.c['check_distance'])
            occupied &= np.abs(left-target) < self.robot['width']/2+self.c['margin']
            if np.any(occupied):
                continue
            self.active = True
            # 코너 출구에서도 차량 방향이 아닌 도로 방향으로 한 차로 이동한다.
            center = 0.0 if requested else lane['error']*math.cos(heading)-lane.get('forward', .65)*math.sin(heading)
            self.heading, self.offset, self.target, self.travel = heading, 0.0, target+center, 0.0
            return self.update(lane, scan, 0.0, 0.0, dt, enabled)
        self.waiting = True
        return 0.0, 0.0  # 옆 차로가 막혀 있으면 정지하고 다음 센서 입력을 기다린다.
