"""구간 관리자: 출발 뒤 누적 회전각으로 지금 구간(S0~S4)과 구간 내 주행 거리를 알려 줌

미션 사슬(표지판을 봐야 다음 미션이 켜짐) 대체. 표지판 하나를 놓쳐도 다음 구간 미션은 그대로 동작.
지도·절대 위치 없음. 입력은 회전 속도(IMU 자이로, 없으면 odom × 보정)와 주행 속도(odom)뿐.

코너 판정: 코너 4개가 모두 우회전 90° → 구간 k의 직선 방향 = −90k°. 누적 회전이 다음 직선 방향까지
margin(°) 안으로 들어오면 구간 +1 (margin 25 → −65°, −155°, −245°, −335°). 되돌아가지 않음
- 갈림길 45° 꺾임·차선 변경·수동 조작 뒤 복귀처럼 구간 안에서 좌우로 크게 흔들려도 누적 각도는 기준에 못 미침
  (11/11 segA_1111_lap2b: "최근 2 m 안 65° 회전" 방식은 수동 개입 뒤 복귀 회전을 코너로 잘못 셈 → 누적 각도로 변경)
- 전환 보류(hold): 라바콘 통과 중에는 콘 배치가 차를 오른쪽으로 크게 돌릴 수 있음 → 그동안 누적 회전은 계속 쌓되
  구간은 올리지 않음. 보류가 풀린 시점에도 기준을 넘어 있으면 그때 전환 (라바콘이 4번 코너 안에서 끝난 경우)
  (11/13 유경 라바콘 영상: 우회전을 강제하는 배치에서 통과 도중 S3 → S4, 라바콘 꺼지고 차선 추종)
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
        self.held = False         # 전환 기준을 넘었지만 보류 중인지 (디버그 표시용)
        self.held_distance = 0.0  # 기준을 넘은 채 보류로 주행한 거리(m)

    def missions(self):
        return SECTION_MISSIONS[self.index]

    def allows(self, mission):
        """지금 구간의 미션인지. sections.off에 있는 미션은 구간과 무관하게 꺼짐 (담당자가 실차 확인 뒤 목록에서 뺌)"""
        if mission in self.c.get('off', ()):
            return False
        return not self.c['enabled'] or mission in self.missions()

    def carry(self, yaw_rate, dt):
        """조종기 수동(SWB 아래): 차를 손으로 옮기는 동안 회전만 적분. 구간·거리는 그대로"""
        self.yaw += yaw_rate*dt

    def relocate(self):
        """재출발: 차가 놓인 방향으로 구간을 다시 고름. 반환: (구간 번호, 그 구간 직선 방향과의 차 °). 값은 바꾸지 않음

        직선 4개 방향이 모두 달라 방향만 알면 구간이 정해짐. 후보는 지금·다음·이전 구간뿐 (S0과 S4는 방향이 같음)
        - 같은 미션 재출발 = 지금 구간, 포기 후 다음 미션 = 지금 또는 다음 구간, 코너에서 이미 넘어간 뒤 재출발 = 이전 구간
        - 가장 가까운 후보와도 restart_tolerance(°) 넘게 다르면 판단 불가 → 지금 구간 유지"""
        heading = -math.degrees(self.yaw)
        best, error = self.index, None
        for k in (self.index, self.index+1, self.index-1):
            if 0 <= k < len(SECTION_MISSIONS):
                e = (heading-90.0*k+180.0) % 360.0-180.0
                if error is None or abs(e) < abs(error):
                    best, error = k, e
        if abs(error) > self.c.get('restart_tolerance', 40.0):
            best = self.index
        return best, error

    def update(self, yaw_rate, velocity, dt, hold=False, advance=False):
        """hold: 이번 주기 구간 전환 보류 (driver.section_hold). 회전·거리 적분은 그대로
        advance: 회전 기준과 무관하게 지금 다음 구간으로 넘김 (driver: 라바콘이 앞 구간에서 시작됨, 11/16)"""
        self.yaw += yaw_rate*dt
        self.distance += abs(velocity)*dt
        self.turned = -math.degrees(self.yaw)-90.0*self.index
        if advance and self.index < len(SECTION_MISSIONS)-1:
            self.held, self.held_distance = False, 0.0
            self.index += 1
            self.distance = 0.0
            self.turned -= 90.0
            return True
        if self.turned >= 90.0-self.c['margin'] and self.index < len(SECTION_MISSIONS)-1:
            # 안전장치: 보류가 안 풀려도 기준을 넘은 채 hold_distance(m) 가면 전환 (다음 구간 미션이 영영 안 켜지는 일 방지)
            if hold and self.held_distance < self.c.get('hold_distance', 3.0):
                self.held = True
                self.held_distance += abs(velocity)*dt
                return False
            self.held, self.held_distance = False, 0.0
            self.index += 1
            self.distance = 0.0
            self.turned -= 90.0
            return True
        self.held, self.held_distance = False, 0.0
        return False
