"""구간 관리자: 트랙의 우회전 코너를 세어 지금 구간(S0~S4)과 구간 내 주행 거리를 알려 줌

미션 사슬(표지판을 봐야 다음 미션이 켜짐) 대체. 표지판 하나를 놓쳐도 다음 구간 미션은 그대로 동작.
지도·절대 위치 없음. 입력은 회전 속도(IMU 자이로, 없으면 odom × 보정)와 주행 속도(odom)뿐.

코너 판정: 최근 window(m) 주행 안에서 오른쪽으로 turn(°) 넘게 돌았으면 코너 1개 통과 → 구간 +1
- 누적 각도를 직접 쓰지 않는 이유: 자이로 치우침이 한 바퀴(약 3분) 동안 쌓여도 코너 하나(약 1 m)에는 영향 없음
- 갈림길 45° 꺾임·차선 변경·라바콘은 turn 미만이라 세지 않음. 왼쪽으로 꺾인 직후 코너는 창 안 최대 방향 기준이라 그대로 셈
"""
import math
from collections import deque

# 구간별 활성 미션. 트랙 순서 고정 (시계 방향, 코너 4개 모두 우회전 90°)
SECTION_MISSIONS = [
    {'slow'},                    # S0 출발 직선: 서행·방지턱
    {'pedestrian'},              # S1 우측 직선: 서행 유도선(카메라, 상시)·보행자
    {'lane_change', 'fork'},     # S2 하단 직선: 3차선 장애물·갈림길
    {'cones'},                   # S3 좌측 직선: 라바콘
    {'traffic', 'parking'},      # S4 상단 안쪽 직선: 신호등·주차
]
# 재출발 인자(start_stage) → 시작 구간
START_SECTION = {'slow': 0, 'pedestrian': 1, 'fork': 2, 'lane_change': 2, 'cones': 3, 'traffic': 4, 'parking': 4}


class SectionManager:
    def __init__(self, config, start=0):
        self.c = config
        self.index = int(start)   # 지금 구간 번호 (0~4)
        self.distance = 0.0       # 이 구간에 들어온 뒤 주행 거리(m)
        self.yaw = 0.0            # 출발 뒤 누적 회전(rad, + 왼쪽). 기록·확인용
        self.travel = 0.0
        self.history = deque([(0.0, 0.0)])   # (주행 거리, 누적 회전)
        self.turned = 0.0         # 창 안 최대 방향에서 지금까지 오른쪽 회전(°)

    def missions(self):
        return SECTION_MISSIONS[self.index]

    def allows(self, mission):
        """지금 구간의 미션인지. sections.off에 있는 미션은 구간과 무관하게 꺼짐 (담당자가 실차 확인 뒤 목록에서 뺌)"""
        if mission in self.c.get('off', ()):
            return False
        return not self.c['enabled'] or mission in self.missions()

    def update(self, yaw_rate, velocity, dt):
        step = abs(velocity)*dt
        self.yaw += yaw_rate*dt
        self.travel += step
        self.distance += step
        self.history.append((self.travel, self.yaw))
        while self.travel-self.history[0][0] > self.c['window']:
            self.history.popleft()
        self.turned = math.degrees(max(yaw for _, yaw in self.history)-self.yaw)
        if self.turned >= self.c['turn'] and self.index < len(SECTION_MISSIONS)-1:
            self.index += 1
            self.distance = 0.0
            self.history = deque([(self.travel, self.yaw)])   # 같은 코너를 두 번 세지 않음
            self.turned = 0.0
            return True
        return False
