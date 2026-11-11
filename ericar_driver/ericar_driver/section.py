"""구간 관리자: 출발 뒤 누적 회전각으로 지금 구간(S0~S4)과 구간 내 주행 거리를 알려 줌

미션 사슬(표지판을 봐야 다음 미션이 켜짐) 대체. 표지판 하나를 놓쳐도 다음 구간 미션은 그대로 동작.
지도·절대 위치 없음. 입력은 회전 속도(IMU 자이로, 없으면 odom × 보정)와 주행 속도(odom)뿐.

코너 판정: 코너 4개가 모두 우회전 90° → 구간 k의 직선 방향 = −90k°. 누적 회전이 다음 직선 방향까지
margin(°) 안으로 들어오면 구간 +1 (margin 25 → −65°, −155°, −245°, −335°). 되돌아가지 않음
- 갈림길 45° 꺾임·차선 변경·수동 조작 뒤 복귀처럼 구간 안에서 좌우로 크게 흔들려도 누적 각도는 기준에 못 미침
  (11/11 segA_1111_lap2b: "최근 2 m 안 65° 회전" 방식은 수동 개입 뒤 복귀 회전을 코너로 잘못 셈 → 누적 각도로 변경)
- IMU 실측: 정지 치우침 0.3°/분 이하, 한 바퀴 누적 −354~−366° → 한 바퀴 동안 오차는 여유 안
"""
import math

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
        self.yaw = -math.radians(90.0*self.index)   # 출발 뒤 누적 회전(rad, + 왼쪽). 재출발은 그 구간 직선 방향에서 시작
        self.turned = 0.0         # 이 구간 직선 방향에서 오른쪽으로 돈 각도(°)

    def missions(self):
        return SECTION_MISSIONS[self.index]

    def allows(self, mission):
        """지금 구간의 미션인지. sections.off에 있는 미션은 구간과 무관하게 꺼짐 (담당자가 실차 확인 뒤 목록에서 뺌)"""
        if mission in self.c.get('off', ()):
            return False
        return not self.c['enabled'] or mission in self.missions()

    def update(self, yaw_rate, velocity, dt):
        self.yaw += yaw_rate*dt
        self.distance += abs(velocity)*dt
        self.turned = -math.degrees(self.yaw)-90.0*self.index
        if self.turned >= 90.0-self.c['margin'] and self.index < len(SECTION_MISSIONS)-1:
            self.index += 1
            self.distance = 0.0
            self.turned -= 90.0
            return True
        return False
