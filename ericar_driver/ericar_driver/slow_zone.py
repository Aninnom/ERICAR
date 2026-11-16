"""서행·방지턱 (규정 미션 5): 서행 표지판 뒤 감속 → 방지턱 통과 확인 → 기본 속도 복귀

규정: 기본 속도의 1/2 수준(최대 0.2 m/s), 심사위원 육안 판정. 실패 = 감속이 안 보임, 방지턱에서 멈춤.
방지턱 통과 뒤 일반 속도 복귀 가능 (v3). 판정 기준은 뒷바퀴.

상태: idle(기본 속도) → slow(속도 상한) → done
- 진입: slow 표지판 확정 뒤 서행 시작 흰 가로선 도달(앞축 기준 entry_line_distance 안) 또는 S0 주행 거리 entry_distance 중 먼저 오는 것
  출발 직후부터 서행하지 않음: 육안 판정이라 줄어드는 모습이 보여야 함
  11/16 main_1116_1605·1614: 출발 위치에서 표지판이 이미 보여(면적 0.0040~0.0043) 0.02 m부터 서행 → 4.0 m 상한이 일찍 끝나 구간 뒤쪽 1 m를 기본 속도로 지남
- 방지턱 감지: 자이로 x·y를 짧게 적분한 기울기 크기. 턱 높이와 축거로 정해져 속도와 무관 (24 mm면 최대 약 7°),
  두 축을 합쳐 IMU 장착 방향과 무관. 감속 직후 차체 숙임을 피하려고 진입 뒤 arm_distance부터 감지
- 해제 (먼저 오는 것): 첫 감지 뒤 release_distance (뒷바퀴까지 통과: 축거 0.2 + 턱 길이 + 여유)
  / S0 거리 exit_distance (현장 실측, 없으면 미사용) / 1번 코너(S1 진입) / 진입 뒤 max_distance
- 턱 위: 카메라가 흔들려 차선 오차가 튐 → 첫 감지 뒤 hold_distance(앞·뒷바퀴가 모두 넘는 거리) 동안 직전 조향 유지
- 턱에서 멈춤: 실제 속도 stuck_speed 미만이 stuck_seconds 이어질 때마다 boost_speeds를 한 단계씩 올림,
  다시 구른 뒤 boost_distance 가면 원래 서행 속도
"""
import math


class SlowZone:
    def __init__(self, config):
        self.c = config
        self.state = 'idle'
        self.sign = False          # slow 표지판 확정 (driver.update_signs)
        self.line_passed = False   # 서행 시작 흰 가로선 도달 (확정이 선보다 늦어도 바로 진입)
        self.entered = 0.0         # 서행 시작 뒤 주행 거리(m)
        self.tilt = [0.0, 0.0]     # 자이로 x·y 누설 적분(rad)
        self.tilt_deg = 0.0        # 기울기 크기(°)
        self.bump_at = None        # 첫 방지턱 감지 시 서행 시작 뒤 거리(m)
        self.held = 0.0            # 고정할 조향 (감지 전 조향의 느린 평균)
        self.stuck = 0.0           # 구르지 않은 시간(s)
        self.boost = 0             # 멈춤 대응 단계 (0 = 없음)
        self.boost_moved = 0.0     # 대응 뒤 다시 구른 거리(m)

    def cap(self):
        """지금 속도 상한. 서행 중이 아니면 None"""
        if self.state != 'slow':
            return None
        steps = self.c['boost_speeds']
        return steps[min(self.boost, len(steps))-1] if self.boost else self.c['speed']

    def update(self, active, section_distance, tilt_rate, velocity, speed, steering, now, dt, line=None):
        """active: S0이고 서행 미션이 아직 안 끝남. tilt_rate: 자이로 (x, y) rad/s, IMU 없으면 None
        speed·steering: 차선 주행 결과. line: 추적 중인 흰 가로선까지 거리(m, 앞축 기준), 없으면 None
        반환 (속도, 조향, 이번 주기에 끝났는지)"""
        c = self.c
        if self.state == 'done':
            return speed, steering, False
        if not active:
            # 1번 코너 진입(S1) 또는 미션 꺼짐: 서행 중이었으면 해제
            finished = self.state == 'slow'
            self.state = 'done' if finished else self.state
            return speed, steering, finished
        if self.state == 'idle':
            far = c['entry_distance'] is not None and section_distance >= c['entry_distance']
            if line is not None and line <= c.get('entry_line_distance', 0.1):
                self.line_passed = True
            ready = self.sign and (self.line_passed or not c.get('entry_at_line', True))
            if not (ready or far):
                self.held += (steering-self.held)*min(1.0, dt/0.5)
                return speed, steering, False
            self.state = 'slow'
        moved = abs(velocity)*dt
        self.entered += moved
        # 방지턱 감지
        if tilt_rate is not None:
            leak = math.exp(-dt/c['tilt_decay'])
            self.tilt = [(a+rate*dt)*leak for a, rate in zip(self.tilt, tilt_rate)]
            self.tilt_deg = math.degrees(math.hypot(*self.tilt))
            if self.entered >= c['arm_distance'] and self.tilt_deg >= c['tilt_threshold']:
                if self.bump_at is None:
                    self.bump_at = self.entered
        # 해제
        if ((self.bump_at is not None and self.entered-self.bump_at >= c['release_distance']) or
                (c['exit_distance'] is not None and section_distance >= c['exit_distance']) or
                self.entered >= c['max_distance']):
            self.state = 'done'
            return speed, steering, True
        # 턱에서 멈춤 대응: 가라는 명령인데 구르지 않을 때만 셈
        if speed > 0.01 and abs(velocity) < c['stuck_speed']:
            self.stuck += dt
            self.boost_moved = 0.0
            if self.stuck >= c['stuck_seconds']:
                self.boost = min(self.boost+1, len(c['boost_speeds']))
                self.stuck = 0.0
        else:
            self.stuck = 0.0
            if self.boost:
                self.boost_moved += moved
                if self.boost_moved >= c['boost_distance']:
                    self.boost = 0
        # 턱 위 조향 고정
        if self.bump_at is not None and self.entered-self.bump_at < c['hold_distance']:
            steering = self.held
        elif self.tilt_deg < 0.5*c['tilt_threshold']:   # 기울기 시작 뒤 값은 평균에 넣지 않음
            self.held += (steering-self.held)*min(1.0, dt/0.5)
        return min(speed, self.cap()), steering, False
