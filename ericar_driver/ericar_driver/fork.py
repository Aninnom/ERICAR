"""3차선·갈림길 상태 관리. 방향 기억·목표 차로·상태 전환만 담당"""


class ForkMission:
    """lane → locked → align → follow → done. +1=A(왼쪽), -1=B(오른쪽)"""

    def __init__(self, lane_count, follow_distance):
        self.lane_count = lane_count
        self.follow_distance = follow_distance
        self.phase = 'lane'
        self.sign = self.stamp = self.destination = None
        self.direction = 0
        self.travel = 0.
        self.centered = False

    @property
    def running(self):
        return self.phase in ('locked', 'align', 'follow')

    @property
    def remaining(self):
        return max(0., self.follow_distance-self.travel) if self.running else 0.

    @property
    def vision_direction(self):
        return self.direction if self.phase == 'follow' else 0

    def lock(self, signs, stamp):
        arrows = set(signs) & {'left', 'right'}
        if self.phase != 'lane' or len(arrows) != 1:
            return False
        self.sign = next(iter(arrows))
        self.direction = 1 if self.sign == 'left' else -1
        self.destination = self.lane_count-1 if self.direction > 0 else 0
        self.stamp, self.phase = stamp, 'locked'
        return True

    def step(self, index, changing, velocity, dt, left_section=False):
        if left_section:
            self.phase = 'done'
            return
        if self.phase == 'locked':
            self.phase = 'align'
        if self.phase == 'align':
            if not changing and index == self.destination:
                self.phase, self.travel = 'follow', 0.
        elif self.phase == 'follow':
            if changing or index != self.destination:
                # 회피로 목표 차로를 벗어나면 같은 목적지로 재정렬
                self.phase = 'align'
            else:
                self.travel += max(0., velocity)*dt
                if self.travel >= self.follow_distance:
                    self.phase = 'done'

    def request(self, index, changing):
        if changing or self.phase not in ('locked', 'align') or index == self.destination:
            return 0
        return 1 if self.destination > index else -1

    def center_request(self, index, changing, lane):
        """일반 주행: 점선 구간에서 중앙 차로 정렬. 장애물 회피 중 새 요청 금지"""
        if self.phase != 'lane' or changing or not lane.get('valid') or self.centered:
            return 0
        center = self.lane_count//2
        if index == center:
            colors = lane.get('colors', {})
            self.centered = colors.get('left') == colors.get('right') == 'white'
            return 0
        direction = 1 if index < center else -1 if index > center else 0
        side = 'left' if direction > 0 else 'right'
        return direction if direction and lane.get('colors', {}).get(side) == 'white' else 0
