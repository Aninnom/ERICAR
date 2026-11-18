"""3차선 점선 추종 (3lane_chu_js 시제품, 11/19 추추). 차로 사이 흰 점선을 차 중앙에 두고 달리다가 장애물만 살짝 비켜 감

기존 방식(lane_change.py): 차로 중앙 주행 → 막히면 0.52 m 한 칸 이동(원호–직선–원호, 0.18 m/s) → 앞 거리 1.6~1.8 m 필요
이 방식: 차 위치를 '도로 기준 옆 위치 y'(가운데 차로 중앙 = 0, + 왼쪽) 하나로 관리하고 매 주기 갈 위치를 고름
- 점선(±0.26)에 차 중심을 두면 차폭 0.225 m라 양옆 차로 중앙의 장애물(폭 0.22)과 약 4 cm씩 떨어짐 → 대부분 조향 없이 통과
- 장애물이 점선 쪽으로 치우쳐 있으면 라이다로 비는 y를 찾아 몇 cm만 옮김 (최대 y_max, 바깥 차로 중앙까지)
- 갈림길: 표지판 쪽 점선으로 옮겨 달리다가 입구 앞에서 반 칸(0.26 m)만 바깥 차로로 → 차로 번호 세기·두 칸 이동이 필요 없음

위치 추정: IMU 도로 방향(driver.road_heading)으로 적분 + 카메라 흰 점선 위치로 보정. 점선은 ±0.26 두 개뿐이라
추정 위치에서 가까운 쪽으로 이름을 붙임 (추정이 0.15 m 넘게 틀리면 보정 안 함 → dash_memory 넘게 못 맞추면 기존 방식으로 넘김)
경로 모델: 지금 y에서 목표 y까지 앞으로 갈수록 기울기 slope(m/m)로 옮기는 직선. 차체(뒷축 기준 뒤 0.072 ~ 앞 0.258)가
그 경로를 지날 때 라이다 점과 margin 넘게 떨어지는 목표 y만 후보. 후보 중 선호 위치에 가깝고 지금 목표와 크게 다르지 않은 것
"""
import math

import cv2
import numpy as np

from .lane_change import LANE_BY_COLORS
from .lidar import body_points, clear_steering, pursuit_steering, to_body, to_road


def measure_dashes(dashes, scale, row_m, far, near):
    """흰 점선 BEV 마스크 → 점선 직선 목록. 각 항목: lateral(near 거리에서 차 기준 왼쪽 m), slope(앞 1 m당 왼쪽 m),
    forward(= near), span(앞뒤로 보인 길이 m), count(픽셀 수). 차 쪽에 가까운 순서

    점선 조각마다 기울기를 재서 가중 중앙값으로 공통 기울기를 정함 (차로 선은 서로 나란함) → 그 기울기로 near 거리에 옮긴
    옆 위치가 0.12 m 넘게 떨어지면 다른 선. 앞뒤로 0.25 m 넘게 보인 선만 자기 기울기로 다시 맞춤"""
    if dashes is None or not dashes.any():
        return []
    h, w = dashes.shape
    n, labels, stats, _ = cv2.connectedComponentsWithStats(dashes, connectivity=8)
    slopes, weights = [], []
    for i in range(1, n):
        if stats[i, cv2.CC_STAT_AREA] < 15:
            continue
        ys, xs = np.nonzero(labels == i)
        f = far-ys*row_m
        if np.ptp(f) < .05:
            continue
        lateral = (w/2-xs)/scale
        slopes.append(float(np.polyfit(f, lateral, 1)[0]))
        weights.append(len(xs))
    if slopes:
        order = np.argsort(slopes)
        cumulative = np.cumsum(np.asarray(weights, float)[order])
        k = float(np.asarray(slopes)[order][np.searchsorted(cumulative, cumulative[-1]/2)])
        k = max(-.6, min(.6, k))
    else:
        k = 0.0
    ys, xs = np.nonzero(dashes)
    f = far-ys*row_m
    lateral = (w/2-xs)/scale
    q = lateral-k*(f-near)
    order = np.argsort(q)
    q, f, lateral = q[order], f[order], lateral[order]
    breaks = np.flatnonzero(np.diff(q) > .12)+1
    lines = []
    for idx in np.split(np.arange(len(q)), breaks):
        if len(idx) < 30:
            continue
        span = float(np.ptp(f[idx]))
        if span >= .25:
            slope, at_near = np.polyfit(f[idx]-near, lateral[idx], 1)
            slope, at_near = float(slope), float(at_near)
        else:
            slope, at_near = k, float(np.median(q[idx]))
        lines.append(dict(lateral=at_near, slope=slope, forward=float(near), span=span, count=int(len(idx)),
                          nearest=float(f[idx].min())))
    return sorted(lines, key=lambda line: abs(line['lateral']))


class DashLane:
    def __init__(self, config, robot):
        self.c = config['dash_lane']
        self.width = config['lane_change']['lane_width']
        self.count = config['lane_change']['lane_count']
        self.robot = robot
        self.reset()

    def reset(self):
        self.active = False
        self.known = False      # 위치 추정이 살아 있음 (조향을 안 맡은 동안에도 S2 안이면 적분·보정 계속)
        self.zone_ahead = None  # 3차로 구간 시작(첫 흰 점선)까지 앞 거리 (m, 뒷축 기준). 지나면 음수
        self.entry_centre = 0.0 # 들어온 1차로 중앙의 옆 위치 (노란 선 = 이 값 ± 차로 폭/2)
        self.y = 0.0            # 차(뒷축) 도로 기준 옆 위치 (m, 가운데 차로 중앙 0, + 왼쪽)
        self.y_cmd = 0.0        # 지금 고른 목표 옆 위치
        self.seen_stamp = None
        self.unfixed = 0.0      # 마지막 점선 보정 뒤 주행 거리 (m)
        self.fixes = 0
        self.debug = dict(reason='off')

    # ---- 갈림길·기존 코드와 맞추는 값 ----
    def lane_index(self):
        """지금 위치에서 가장 가까운 차로 번호 (0 오른쪽, 1 가운데, 2 왼쪽)"""
        return int(max(0, min(self.count-1, round(self.y/self.width)+self.count//2)))

    @property
    def straddling(self):
        """차로 중앙에서 lane_tolerance 넘게 떨어짐 = 점선 위 (갈림길 입구 판단은 '차선 변경 중'처럼 미룸)"""
        if not self.active:
            return False
        centre = (self.lane_index()-self.count//2)*self.width
        return abs(self.y-centre) > self.c.get('lane_tolerance', .12)

    def preferred(self, direction, exit_now, center_now=False):
        """선호 위치: 표지판 전 default_side 쪽 점선, 표지판 확정 뒤 그쪽 점선, 입구 앞(exit_now)이면 그쪽 바깥 차로 중앙
        표지판 없이 입구 앞이면 지금 점선 쪽 바깥 차로 (가운데 차로로 입구에 가면 manual 정지)
        center_now: 입구가 가까운데 표지판이 아직 없음 → 가운데 차로 중앙(0)에서 대기. 어느 쪽이 나와도 0.52 m만 옮기면 됨
        (점선에서 반대쪽 바깥 차로는 0.78 m → 늦은 표지판이면 못 건넘, 합성 시험 reverse_right)"""
        if center_now and not direction and not exit_now:
            return 0.0
        side = direction or self.c.get('default_side', 1)
        if exit_now:
            if not direction:
                side = 1 if self.y >= 0 else -1
            return side*self.width
        return side*self.width/2

    # ---- 위치 추정 ----
    def start(self, lane, index, heading):
        """위치 추정 시작. 차로 번호는 선 색 조합(노랑/흰)이 있으면 그 값, 없으면 index(기존 lane_change.index)
        (합성 시험 start_left: 왼쪽 차로에서 출발했는데 기존 번호 1로 시작 → 점선이 0.52 m 어긋나 바깥선 밖으로 나감)"""
        colors = lane.get('colors') or {}
        seen = LANE_BY_COLORS.get((colors.get('left'), colors.get('right')))
        if seen is not None and seen < self.count:
            index = seen
        self.debug['start_index'] = index
        forward = lane.get('forward', .65)
        centre = lane['error']*math.cos(heading)-forward*math.sin(heading)     # 차 기준 지금 차로 중앙 (+ 왼쪽)
        self.y = (index-self.count//2)*self.width-centre
        self.y_cmd = self.y
        self.unfixed, self.fixes, self.seen_stamp = 0.0, 0, None
        self.entry_centre = (index-self.count//2)*self.width
        nearest = [line['nearest'] for line in lane.get('dash_lines') or [] if 'nearest' in line]
        self.zone_ahead = min(nearest) if nearest else None

    def fix(self, lane, heading):
        """새 영상의 흰 점선으로 y 보정. 반환: 이번 프레임에 보정했는지"""
        stamp = lane.get('stamp')
        if stamp is not None and stamp == self.seen_stamp:
            return None
        self.seen_stamp = stamp
        residuals = []
        for line in lane.get('dash_lines') or []:
            # 차 기준 (앞 forward, 왼쪽 lateral) 점 → 도로 기준 옆 거리 (heading = 차에서 본 도로 방향, + 왼쪽)
            offset = -line['forward']*math.sin(heading)+line['lateral']*math.cos(heading)
            measured = self.y+offset
            truth = min((-self.width/2, self.width/2), key=lambda t: abs(t-measured))
            if abs(truth-measured) <= self.c.get('fix_window', .15):
                residuals.append((truth-measured, line['count']))
        if not residuals:
            return False
        total = sum(weight for _, weight in residuals)
        error = sum(r*weight for r, weight in residuals)/total
        self.y += self.c.get('fix_gain', .5)*error
        self.unfixed, self.fixes = 0.0, self.fixes+1
        self.debug['fix_error'] = round(error, 3)
        return True

    # ---- 경로 고르기 ----
    def feasible(self, forward, lateral, candidates, margin, heading=0.0, slope=None):
        """후보 목표 y마다 경로가 비는지 (bool 배열). forward·lateral: 도로 기준 라이다 점 (차 기준 앞, 도로 기준 옆 위치 y)
        경로 모델 (뒷축): delay(m) 동안은 지금 방향 그대로(조향 지연), 그 뒤 slope 기울기로 목표 y까지 옮기고 유지
        지금 방향을 넣는 이유: 반대쪽으로 꺾던 중이면 그만큼 더 갔다가 돌아옴 (합성 시험 sign_first: 왼쪽으로 가던 중 오른쪽 표지판)"""
        if len(forward) == 0:
            return np.ones(len(candidates), bool)
        c = self.c
        front = self.robot.get('front_bumper_x', self.robot['length'])
        rear = self.robot['length']-front
        half = self.robot['width']/2+margin
        slope = c.get('slope', .35) if slope is None else slope
        delay = c.get('delay', .15)
        drift = -math.tan(max(-.7, min(.7, heading)))          # 지금 방향의 옆 기울기 (heading > 0 = 차가 오른쪽을 봄)
        start = self.y+drift*delay
        cand = np.asarray(candidates)[:, None]
        gap = cand-start

        def path(at):
            at = np.maximum(at, 0.0)
            early = self.y+drift*np.minimum(at, delay)
            late = start+np.sign(gap)*np.minimum(np.abs(gap), slope*np.maximum(at-delay, 0.0))
            return np.where(at <= delay, early, late)

        # 점 하나를 차체가 지나는 동안 뒷축은 (점 − 앞 범퍼) ~ (점 + 뒤 끝) 사이. 양 끝과 가운데 y로 그 사이 범위 근사
        f = forward[None, :]
        ys = [path(f-front), path(f-front/2+rear/2), path(f+rear)]
        lo = np.minimum.reduce(ys)-half
        hi = np.maximum.reduce(ys)+half
        hit = (lateral[None, :] > lo) & (lateral[None, :] < hi)
        return ~hit.any(axis=1)

    def entry_walls(self, forward, lateral):
        """3차로 구간 시작 전 1차로 노란 선(들어온 차로 중앙 ± 차로 폭/2)을 가상 점으로 더함 (11/19 사용자: 진입 때 너무 일찍
        조향해 노란 선을 밟거나 넘음, 합성 시험에서 6~8 cm 넘음). 차체 뒤끝까지 구간 시작을 지나면 없음
        경로 검사가 이 점을 피하므로 옆 이동은 차체가 1차로를 다 빠져나가는 만큼 늦게 시작함"""
        if self.zone_ahead is None or not self.c.get('entry_wall', True):
            return forward, lateral
        rear = self.robot['length']-self.robot.get('front_bumper_x', self.robot['length'])
        if self.zone_ahead < -rear-.02:
            return forward, lateral
        f = np.arange(-rear-.05, self.zone_ahead, .02)
        if not len(f):
            return forward, lateral
        half = self.width/2
        wall_f = np.concatenate((f, f))
        wall_y = np.concatenate((np.full(len(f), self.entry_centre+half), np.full(len(f), self.entry_centre-half)))
        return np.concatenate((forward, wall_f)), np.concatenate((lateral, wall_y))

    def track(self, lane, velocity, dt, heading):
        """위치 추정만 (조향 안 맡음). 반환: 추정이 살아 있는지"""
        if not self.known:
            # 3차로 구간이 차 바로 앞까지 와야 시작 (1차로 구간 끝에서 멀리 보이는 점선으로 미리 옮기지 않게)
            near = [line for line in lane.get('dash_lines') or []
                    if line.get('nearest', 0.0) <= self.c.get('start_near', .8)]
            if not (lane.get('valid') and near):
                self.debug = dict(reason='waiting_for_dashes')
                return False
            self.start(lane, self.index_hint, heading)
            self.known = True
        # 추측 항법: 차가 도로보다 오른쪽을 보면(heading > 0) 오른쪽(− y)으로 감
        self.y -= velocity*math.sin(heading)*dt
        if self.zone_ahead is not None:
            self.zone_ahead -= velocity*math.cos(heading)*dt
            # 구간 시작이 아직 화면(앞 near 0.6 m 넘게)에 보이면 가장 가까운 점선 끝으로 다시 맞춤. 그 안은 주행 거리로만
            nearest = [line['nearest'] for line in lane.get('dash_lines') or [] if 'nearest' in line]
            if nearest and self.zone_ahead > self.c.get('zone_refresh', .7):
                self.zone_ahead = min(nearest)
        self.unfixed += abs(velocity)*dt
        self.fix(lane, heading)
        if self.unfixed > self.c.get('dash_memory', 2.0):
            # 점선을 오래 못 맞춤 → 위치를 믿을 수 없음. 기존 차선 주행·차선 변경으로 넘김
            self.known = self.active = False
            self.debug = dict(self.debug, reason='lost_dashes')
            return False
        return True

    def update(self, lane, scan, velocity, dt, heading, enabled, direction=0, exit_now=False, tracking=True,
               center_now=False):
        """반환 (속도, 조향) 또는 None(기존 제어에 맡김). heading: 차에서 본 도로 방향(rad, + 왼쪽), None이면 카메라 차선 방향
        tracking: S2 안이라 위치 추정을 이어 갈지 (enabled가 잠깐 꺼져도 추정은 유지 → 다시 켜질 때 그 위치로 이어 감)"""
        c = self.c
        if heading is None:
            heading = lane['heading'] if lane.get('valid') else 0.0
        if not tracking:
            if self.known:
                self.debug = dict(self.debug, reason='left_zone')
            self.known = self.active = False
            return None
        if not self.track(lane, velocity, dt, heading):
            self.active = False
            return None
        if not enabled:
            if self.active:
                self.debug = dict(self.debug, reason='disabled')
            self.active = False
            self.y_cmd = self.y
            return None
        if not self.active:
            self.active, self.y_cmd = True, self.y
        points = body_points(scan, self.robot)
        points = points[np.isfinite(points).all(axis=1)] if len(points) else points
        forward, left = to_road(points, heading)
        lateral = self.y+left
        rear = self.robot['length']-self.robot.get('front_bumper_x', self.robot['length'])
        keep = (forward > -rear-.05) & (forward < c.get('window', 1.4)) & (np.abs(lateral) < c.get('wall_y', .72))
        forward, lateral = forward[keep], lateral[keep]
        forward, lateral = self.entry_walls(forward, lateral)
        limit = c.get('y_max', .52)
        candidates = np.arange(-limit, limit+1e-9, c.get('step', .005))
        want = self.preferred(direction, exit_now, center_now)
        slope = c.get('exit_slope', .5) if exit_now else c.get('slope', .35)
        # 여유 단계: 넉넉(soft_margin) → 보통(margin) → 좁음(tight_margin, 천천히). 앞 단계에 비는 위치가 있으면 그 단계에서 고름
        tier, ok = None, np.zeros(len(candidates), bool)
        for name, margin in (('soft', c.get('soft_margin', .07)), ('normal', c.get('margin', .025)),
                             ('tight', c.get('tight_margin', .012))):
            ok = self.feasible(forward, lateral, candidates, margin, heading, slope)
            if ok.any():
                tier = name
                break
        half = self.robot['width']/2+c.get('margin', .025)
        ahead = forward[(np.abs(lateral-self.y) < half) & (forward > 0)]
        front = float(ahead.min()) if len(ahead) else 9.9
        self.debug = dict(reason='tracking', y=round(self.y, 3), want=round(want, 3), front=round(front, 2),
                          fixes=self.fixes, unfixed=round(self.unfixed, 2), lane_index=self.lane_index(),
                          straddling=self.straddling, feasible=int(ok.sum()), tier=tier, points=int(len(forward)),
                          fix_error=self.debug.get('fix_error'), exit=bool(exit_now), center=bool(center_now),
                          zone_ahead=None if self.zone_ahead is None else round(self.zone_ahead, 2))
        if tier is None and c.get('handoff', True) and (front <= c.get('stop_distance', .6) or c.get('handoff_early', True)):
            # 비는 위치 없음: 기존 방식(짧은 S자·서행 접근·후진 재시도 BackUp)에 이번 주기를 맡김. 기존 방식이 이동을 시작하면
            # 끝날 때까지 기존 방식 (driver: lane_change.active면 점선 추종 끔), 끝나면 점선 추종이 그 위치에서 이어 감
            # (합성 시험 backup_tight: 더 다가간 뒤 넘기면 기존 짧은 S자가 들어갈 자리가 없어 58 s 정지)
            self.active, self.y_cmd = False, self.y
            self.debug['reason'] = 'handoff'
            return None
        if tier is None:
            # 비는 위치 없음: 지금 줄을 천천히 따라가며 다시 봄 (옆 장애물을 지나치면 열림)
            self.y_cmd = self.y
            speed = c.get('approach_speed', .15)
            target = 0.0
            self.debug['reason'] = 'no_gap'
        else:
            cost = np.abs(candidates-want)+c.get('hold_weight', .3)*np.abs(candidates-self.y_cmd)
            cost[~ok] = np.inf
            self.y_cmd = float(candidates[int(np.argmin(cost))])
            moving = abs(self.y_cmd-self.y) > c.get('settle', .06)
            speed = c.get('shift_speed', .22) if moving else c.get('speed', .30)
            if tier == 'tight':
                speed = min(speed, c.get('approach_speed', .15))
            look = c.get('lookahead', .45)
            reach = max(abs(self.y_cmd-self.y)/slope, .05)
            target = (self.y_cmd-self.y)*min(1.0, look/reach)              # 경로 위 look 앞 지점의 옆 거리 (지금 위치 기준)
            self.debug['reason'] = 'shifting' if moving else 'holding'
        self.debug['y_cmd'] = round(self.y_cmd, 3)
        x, y = to_body(c.get('lookahead', .45), target, heading)
        steering = pursuit_steering(x, y, self.robot)
        steering, safe = clear_steering(body_points(scan, self.robot), steering, self.robot,
                                        c.get('safety_horizon', .3), c.get('safety_margin', .02))
        if not safe:
            if c.get('handoff', True):
                # 경로는 빈다고 봤지만 지금 조향으로 0.3 m 안에 닿음: 기존 방식(후진 재시도 등)에 이번 주기를 맡김
                self.active, self.y_cmd = False, self.y
                self.debug['reason'] = 'handoff_unsafe'
                return None
            speed = 0.0
            self.debug['reason'] = 'no_safe_steering'
        self.debug.update(speed=round(speed, 3), steering=round(float(steering), 3))
        return speed, float(steering)

    index_hint = 1     # 시작 때 차로 번호 (driver가 lane_change.index로 채움)
