"""3차선·갈림길 상태 관리. 방향 기억·목표 차로·상태 전환만 담당"""


class ForkMission:
    """lane → locked → align → follow → (분기 입구) exit → done. 입구에서 가운데 차로면 manual(정지),
    반대편 끝 차로면 그 길로 진행 (미션 점수는 잃지만 멈추지 않음). +1=A(왼쪽), -1=B(오른쪽)

    11/15 (fix_1115_1949): 점선 끝 판정이 실제 트랙에서 안 걸림 (길 안 흰 표시를 점선으로 봄) → follow가 구간 전환까지 유지.
    - 분기 입구 1순위 = 표지판 통과 (see_sign): 표지판이 크게 보이다가 사라짐 = 교통섬 바로 앞. 점선 끝은 보조
    - 방향 교체 (relock): 입구 전까지 반대 화살표가 다시 확정되면 목적지 변경 (트랙 밖 물체를 표지판으로 오검출)

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

    def __init__(self, lane_count, follow_distance, far=1.7, dash_gap=0.3, entrance_confirm=0.3,
                 pass_area=0.006, pass_distance=0.15, pass_seconds=0.5, wrong_lane_distance=0.4,
                 outer_distance=0.8):
        self.lane_count = lane_count
        self.follow_distance = follow_distance
        self.far, self.dash_gap, self.entrance_confirm = far, dash_gap, entrance_confirm
        self.lock_distance = 0.6   # 남은 점선이 이 거리(m) 아래면 끝 확정 (far − dash_gap 1.25보다 충분히 짧음)
        self.phase = 'lane'
        self.sign = self.stamp = self.destination = None
        self.direction = 0
        self.travel = 0.
        self.centered = False
        self.dash_seen = False
        self.dash_left = None    # 남은 흰 점선 거리 추정(m). None = 아직 못 봄
        self.dash_gone = 0.      # 마지막으로 점선을 본 뒤 주행 거리(m)
        self.dash_locked = False  # 점선 끝 확정: 이후 더 먼 '점선'(길 안 흰 글자·흰 선)으로 남은 거리 안 늘림
        # 표지판 통과 판단: 면적 pass_area 이상으로 본 뒤(sign_near) pass_distance(m)·pass_seconds(s) 넘게 안 보이면 통과
        # 반대편 끝 차로(align)는 wrong_lane_distance(m)까지 안 보여야 그 길로 진행 (검출 끊김에 목적지를 쉽게 포기하지 않게)
        self.pass_area, self.pass_distance, self.pass_seconds = pass_area, pass_distance, pass_seconds
        self.wrong_lane_distance = wrong_lane_distance
        self.outer_distance = outer_distance   # 분기 입구 뒤 바깥선 하나만 따라가는 거리(m)
        self.sign_near = False
        self.sign_gone = self.sign_gone_time = 0.
        self.passed = False       # 분기 입구를 지났는지 (enter). 한 번 지나면 3차선 구간 끝 = 차선 변경 판단 자체를 끔

    @property
    def running(self):
        return self.phase in ('locked', 'align', 'follow', 'exit', 'manual')

    @property
    def remaining(self):
        return max(0., self.follow_distance-self.travel) if self.running else 0.

    @property
    def outer_only(self):
        """목적 쪽 바깥 노란 선 하나만 따라갈 구간: 목적 차로 도착 ~ 분기 입구 뒤 outer_distance(m) = 교통섬 앞머리
        그 뒤(exit 나머지)는 두 선 모두 사용 (11/15 fix2_1115_2017 186.6~190.8 s: A 길에서 왼쪽 바깥선이 화면 밖으로 나가고
        오른쪽 안쪽 선만 보이는데 그 선을 버려 4.5 s 차선 무효 → 직진 → 3번 코너 벽. B 길도 같은 문제, fix_1115_1949)"""
        return self.phase == 'follow' or (self.phase == 'exit' and self.travel < self.outer_distance)

    @property
    def vision_direction(self):
        return self.direction if self.outer_only else 0

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
        if self.dash_locked:
            # 끝 확정 뒤: 주행 거리만큼만 줄임 (11/15 main_1115_1904 198~227 s: A 길 안 흰 글자·라바콘 앞 흰 선을 점선으로 봐
            # dash_left가 0.5 → 1.5 m로 되돌아가 입구 판정이 안 남 → follow가 S3 전환까지 유지)
            self.dash_left = held or 0.
            self.dash_gone += step
        elif dash_end is not None:
            # 순간 짧아진 값(빈칸·잘린 조각)은 무시: 이전 추정에서 주행 거리만 뺀 값보다 짧아지지 않음
            self.dash_seen, self.dash_gone = True, 0.
            self.dash_left = float(dash_end) if held is None else max(float(dash_end), held)
        elif self.dash_seen:
            self.dash_left = held or 0.
            self.dash_gone += step
        # 끝 확정: 목적 차로를 따라가는 중(follow), 차선이 보이는 프레임에서 남은 점선이 lock_distance 아래
        # = 화면에 그보다 먼 점선이 없음. 차선을 못 본 프레임·차선 변경 중에는 확정 안 함 (못 본 동안 줄어든 값일 수 있음)
        if (self.phase == 'follow' and not changing and lane.get('valid') and self.dash_seen
                and self.dash_left is not None and self.dash_left < self.lock_distance):
            self.dash_locked = True
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
        self.enter(index)

    def enter(self, index):
        """분기 입구 도착 처리. 목표 차로 → exit, 반대편 끝 차로 → 그 길로 exit, 가운데 → manual(정지)"""
        center = self.lane_count//2
        self.passed = True
        if self.destination is not None and index != self.destination and index != center:
            # 반대편 끝 차로: 교통섬을 가로지를 수 없음. 멈추지 않고 지금 차로의 길로 진행 (11/15, 예전: manual 정지)
            self.destination = index
            self.direction = 1 if index > center else -1
        if self.destination is not None and index == self.destination:
            self.phase, self.travel = 'exit', 0.
        else:
            self.phase = 'manual'   # 가운데: 교통섬을 가로지르지 않고 정지

    def see_sign(self, area, index, changing, velocity, dt, fresh=True):
        """매 주기 호출. area = 이번 주기 화살표 표지판 최대 면적 비율 (안 보이면 None)
        fresh = 이번 주기에 표지판을 볼 수 있었는지 (검출 최신 + 차가 도로 방향을 봄)
        표지판은 교통섬 앞머리에 있음 → 크게 보이다가 화면에서 사라지면 섬 바로 앞 = 분기 입구
        목적 차로를 따라가는 중(follow)이거나 반대편 끝 차로(align)일 때만 판정. 가운데 차로는 점선 판정에 맡김
        차선 변경 중·볼 수 없는 주기는 '안 보인 거리'를 세지 않음 (틀어진 자세라 화면 밖일 뿐)"""
        if not self.running or self.entered:
            return
        if area is not None:
            self.sign_gone = self.sign_gone_time = 0.
            if area >= self.pass_area:
                self.sign_near = True
            return
        if changing or not fresh:
            self.sign_gone = self.sign_gone_time = 0.
            return
        if not self.sign_near:
            return
        self.sign_gone += max(0., velocity)*dt
        self.sign_gone_time += dt
        if self.sign_gone < self.pass_distance or self.sign_gone_time < self.pass_seconds:
            return
        if self.phase == 'follow' or (self.phase == 'align' and index != self.lane_count//2
                                      and self.sign_gone >= self.wrong_lane_distance):
            self.enter(index)

    def note_near(self, area):
        """확정된 화살표를 틀어진 자세(차선 변경 중 등)에서 본 면적. '크게 봤음'(sign_near)만 기록 (유경 11/16)
        see_sign은 틀어진 자세의 박스를 못 받음(오검출 방지로 버림) → 두 칸을 비스듬히 건너면 가까이서 본 기록이 빠짐"""
        if self.running and not self.entered and area is not None and area >= self.pass_area:
            self.sign_near = True

    def relock(self, signs, stamp):
        """확정 뒤 반대 화살표가 다시 확정되면 목적지 교체. 입구를 지난 뒤(exit·manual)에는 안 바꿈"""
        arrows = set(signs) & {'left', 'right'}
        if self.phase not in ('locked', 'align', 'follow') or len(arrows) != 1 or arrows == {self.sign}:
            return False
        self.sign = next(iter(arrows))
        self.direction = 1 if self.sign == 'left' else -1
        self.destination = self.lane_count-1 if self.direction > 0 else 0
        self.stamp, self.phase = stamp, 'align'
        self.sign_near, self.sign_gone, self.sign_gone_time = False, 0., 0.
        return True

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

    def request(self, index, changing, ignore_late=False):
        """목적 차로 쪽 이동 요청(+1 왼쪽, −1 오른쪽, 0 없음). ignore_late: 늦은 지점(점선 끝이 보임)이어도 요청
        (11/16: 경로 계획이 이동 가능 여부를 직접 확인하는 경우. 입구를 지난 뒤에는 phase가 바뀌어 요청 없음)"""
        if changing or self.phase not in ('locked', 'align') or index == self.destination or (self.late and not ignore_late):
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
