"""3차선·갈림길 상태 관리. 방향 기억·목표 차로·상태 전환만 담당"""


class ForkMission:
    """lane → locked → align → follow → (분기 입구) exit → done. 입구에서 목표 차로가 아니면 manual(정지).
    +1=A(왼쪽), -1=B(오른쪽)

    분기 입구 = 흰 점선이 끝나는 곳 (소민 11/12). 카메라로 본 가장 먼 점선까지 거리(lane['dash_end'])를
    기억하고, 안 보이면 주행 거리만큼 줄여 남은 점선 거리(dash_left)를 추정
    - 보여도 '이전 추정 − 주행 거리'보다 짧으면 이전 추정 유지 (11/12 run_1112_0408: 점선 사이 빈칸이 화면 끝에 걸리거나
      화면 끝에서 잘린 조각이 길이 필터에 걸려 dash_end가 순간 짧아짐 → 늦음 착각 → 회피 전부 막힘).
      실제 끝은 주행 거리만큼 꾸준히 줄어듦, 빈칸은 다음 점선이 들어오면 다시 far로 복귀
    - 점선 끝이 화면 안에 보이면(dash_left < far − dash_gap) = 늦음: 새 차선 변경 시작 금지 (진행 중인 변경은 끝까지)
    - 점선이 끝나고 entrance_confirm(m) 더 감 = 입구 도착:
      목표 차로면 exit(선택한 길 경계 추적, 차선 변경 금지), 아니면 manual(정지, 수동 개입)
    - 점선을 한 번도 못 봤으면 예전 동작 그대로 (follow에서 follow_distance 뒤 done)
    """

    def __init__(self, lane_count, follow_distance, far=1.7, dash_gap=0.3, entrance_confirm=0.3):
        self.lane_count = lane_count
        self.follow_distance = follow_distance
        self.far, self.dash_gap, self.entrance_confirm = far, dash_gap, entrance_confirm
        self.phase = 'lane'
        self.sign = self.stamp = self.destination = None
        self.direction = 0
        self.travel = 0.
        self.centered = False
        self.dash_seen = False
        self.dash_left = None    # 남은 흰 점선 거리 추정(m). None = 아직 못 봄
        self.dash_gone = 0.      # 마지막으로 점선을 본 뒤 주행 거리(m)

    @property
    def running(self):
        return self.phase in ('locked', 'align', 'follow', 'exit', 'manual')

    @property
    def remaining(self):
        return max(0., self.follow_distance-self.travel) if self.running else 0.

    @property
    def vision_direction(self):
        return self.direction if self.phase in ('follow', 'exit') else 0

    @property
    def late(self):
        """점선 끝이 이미 화면 안에 보임 → 한 차로 이동할 거리 부족 (늦어서 이동을 포기해야 하는 지점)"""
        return self.dash_seen and self.dash_left is not None and self.dash_left < self.far-self.dash_gap

    @property
    def entered(self):
        return self.phase in ('exit', 'manual') or (self.phase == 'done' and self.dash_seen)

    def allows_change(self, changing):
        """차선 변경 허용: 입구를 지났거나(exit·manual), 늦은 지점에서 새로 시작하는 변경은 금지"""
        if self.entered:
            return False
        return changing or not self.late

    def observe(self, lane, index, changing, velocity, dt):
        """매 주기 점선 끝 추적 + 입구 도착 판단"""
        dash_end = lane.get('dash_end') if lane.get('valid') else None
        step = max(0., velocity)*dt
        held = None if self.dash_left is None else max(0., self.dash_left-step)
        if dash_end is not None:
            # 순간 짧아진 값(빈칸·잘린 조각)은 무시: 이전 추정에서 주행 거리만 뺀 값보다 짧아지지 않음
            self.dash_seen, self.dash_gone = True, 0.
            self.dash_left = float(dash_end) if held is None else max(float(dash_end), held)
        elif self.dash_seen:
            self.dash_left = held or 0.
            self.dash_gone += step
        if self.entered or self.phase == 'done' or not self.dash_seen or changing:
            return
        if self.dash_left > 0. or self.dash_gone < self.entrance_confirm:
            return
        # 분기 입구 도착
        center = self.lane_count//2
        if self.destination is None and index != center:
            # 표지판 없이 끝 차로: 그 차로의 출구로 진행
            self.destination = index
            self.direction = 1 if index > center else -1
        if self.destination is not None and index == self.destination:
            self.phase, self.travel = 'exit', 0.
        else:
            self.phase = 'manual'   # 가운데·반대편: 교통섬을 가로지르지 않고 정지

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
            elif not self.dash_seen:
                # 점선 정보 없음: 예전처럼 거리로 종료
                self.travel += max(0., velocity)*dt
                if self.travel >= self.follow_distance:
                    self.phase = 'done'
        elif self.phase == 'exit':
            # 선택한 길: 차로 번호 무시, 경계 추적만. follow_distance 뒤 종료
            self.travel += max(0., velocity)*dt
            if self.travel >= self.follow_distance:
                self.phase = 'done'

    def request(self, index, changing):
        if changing or self.phase not in ('locked', 'align') or index == self.destination or self.late:
            return 0
        return 1 if self.destination > index else -1

    def center_request(self, index, changing, lane):
        """일반 주행: 점선 구간에서 중앙 차로 정렬. 장애물 회피 중·늦은 지점 새 요청 금지"""
        if self.phase != 'lane' or changing or not lane.get('valid') or self.centered or self.late:
            return 0
        center = self.lane_count//2
        if index == center:
            colors = lane.get('colors', {})
            self.centered = colors.get('left') == colors.get('right') == 'white'
            return 0
        direction = 1 if index < center else -1 if index > center else 0
        side = 'left' if direction > 0 else 'right'
        return direction if direction and lane.get('colors', {}).get(side) == 'white' else 0
