"""T 주차: 정렬 후 시간표 주행"""
# 출처: hello-osy/2026-orda-mentoring src/orda_sim/orda_sim/answer.py (613c9b4)
from .common import clip


class Parking:
    """주차 담당: 진입 거리·차선 중심·방향 정렬 → 전진/후진 시간표 → 정차."""
    def __init__(self, config):
        self.c = config['parking']
        self.stop_speed = config['missions']['stopped_speed']
        self.state = 'parking_align'
        self.started = self.step_started = 0.0
        self.stable_since = None
        self.index = 0
        self.paused_since = None   # 시간표 일시정지 시작 시각 (장애물·센서 끊김)
        self.obstacle_wait = 0.0   # 시간표 중 장애물로 기다린 누적 시간 (driver.py에서 증가)

    def start(self, now):
        self.state, self.started, self.stable_since = 'parking_align', now, None
        self.index = 0
        self.paused_since, self.obstacle_wait = None, 0.0

    def pause(self, now):
        """이번 주기 정지. 다음 update에서 멈춘 시간만큼 시간표를 뒤로 민다."""
        if self.paused_since is None:
            self.paused_since = now

    def resume(self, now):
        if self.paused_since is not None:
            self.step_started += now-self.paused_since
            self.paused_since = None

    def update(self, lane, line_distance, velocity, now, steering):
        c = self.c
        speed = 0.0
        if self.state == 'parking_align':
            if now-self.started > c['align_timeout']:
                # 정렬 실패로 영구 정지하면 0점. 현 위치에서 시간표를 시작해 부분 점수라도 노린다.
                self.state, self.step_started = 'parking_sequence', now
            elif lane['valid'] and line_distance is not None:
                error = line_distance-c['line_distance']
                speed = clip(.8*error, -c['align_speed'], c['align_speed'])
                aligned = (abs(error) <= c['distance_tolerance'] and
                           abs(lane['error']) <= c['lateral_tolerance'] and
                           abs(lane['heading']) <= c['heading_tolerance'])
                if aligned:
                    speed = 0.0
                if aligned and abs(velocity) <= self.stop_speed:
                    if self.stable_since is None:
                        self.stable_since = now
                    if now-self.stable_since >= c['stable_seconds']:
                        self.state, self.step_started = 'parking_sequence', now
                else:
                    self.stable_since = None
        if self.state == 'parking_sequence':
            self.resume(now)
            sequence = c['sequence']
            while self.index < len(sequence) and now-self.step_started >= sequence[self.index]['seconds']:
                self.step_started += sequence[self.index]['seconds']
                self.index += 1
            if self.index == len(sequence):
                self.state, speed = 'parked', 0.0
            else:
                step = sequence[self.index]
                speed, steering = step['speed'], step['steering']
        return speed, steering, self.state
