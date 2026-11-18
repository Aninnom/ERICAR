#!/usr/bin/env python3
"""주행 노드 디버그 화면을 브라우저로 중계 (MJPEG). 보기 전용, 차량 제어 없음

위 왼쪽   카메라: YOLO 상자(초록 = 확정, 노랑 = 확인 중, 회색 = 버림: 작음·잘림·오래됨), 차선 곡선, 목표점(빨강), 정지선(자홍)
위 가운데 BEV: 차선 마스크, 흰색 노면 표시(자홍), 윈도우·곡선·목표점, 정지선 후보, 선 색·실선/점선, 지그재그
위 오른쪽 라이다: 군집(빨강 = 넓은 물체, 주황 = 라바콘 후보), 비상 정지 범위, 명령 조향 경로(하늘색),
                 3차선 차로별 빈 거리·선택, 라바콘 통로, 주차 빈칸 판정(FREE 초록·확인 부족 주황·점유 빨강·선택 칸 노랑)
주차: 화면 구성은 다른 구간과 같음. 회전 중 제어기가 쓴 흰 직선을 카메라에 노랑으로 덧그림 (텔레메트리 값, 화면 쪽 재계산 없음)
      주행 노드 없이 센서만 켰을 때만 화면에서 빈칸 판정을 직접 계산 (PARK live)
아래 왼쪽   상태: 모드, 명령·실제 속도, 센서 나이, 차선, 정지선, 멈춤 방지, 속도 0인 이유
아래 가운데 미션: 구간(S0~S4)·누적 회전, 미션 8개 상태(ACTIVE/ARMED/DONE/MISSED/OFF)와 세부 값
아래 오른쪽 사건 기록: 상태 전환, 구간 전환, 표지판 확정, 센서 끊김, 막힘 (최신이 위)

입력: /camera/color/image_raw/compressed, /answer/debug_image(mono8), /answer/detections(JSON), /answer/status, /scan
화면 글자는 영어: OpenCV 글꼴에 한글 없음, 브라우저 표시 방식은 한글 글꼴 없는 환경(차 브라우저)에서 깨짐 → 한 장 영상으로 통일 (11/12)
화면 용어 풀이는 README "디버그 화면 용어"

실행 (차, 터미널 환경 1, 주행 노드 실행 중):
  python3 debug_view.py [포트]
보기: 맥 http://<차 IP>:8080 / 차 http://localhost:8080
"""
import json
import math
import socket
import sys
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import cv2
import numpy as np

MAX_FPS = 8
PANEL_H = 360          # 위 패널 높이
INFO_H = 236           # 아래 글자 패널 높이
LIDAR_W, PX_M = 400, 115.0   # 라이다 패널 폭(px), 1 m = PX_M px
CAMERA_TOPIC = '/camera/color/image_raw/compressed'
MASK_TOPIC = '/answer/debug_image'
TELEMETRY_TOPIC = '/answer/detections'
STATUS_TOPIC = '/answer/status'

FONT = cv2.FONT_HERSHEY_SIMPLEX
WHITE, GREY, DIM = (235, 235, 235), (150, 150, 150), (80, 80, 80)
GREEN, RED, YELLOW, ORANGE = (0, 220, 0), (60, 60, 255), (0, 220, 255), (0, 150, 255)
CYAN, MAGENTA, BLUE = (255, 220, 0), (255, 0, 255), (255, 120, 0)

# 트랙 순서 미션: (표시 이름, 구간 관리자 미션 이름, 구간 번호, 관련 표지판)
MISSIONS = [
    ('1 slow/bump', 'slow', 0, ['slow']),
    ('2 zigzag', None, 1, []),
    ('3 pedestrian', 'pedestrian', 1, ['pedestrian']),
    ('4 3-lane', 'lane_change', 2, []),
    ('5 fork', 'fork', 2, ['left', 'right']),
    ('6 cones', 'cones', 3, []),
    ('7 traffic', 'traffic', 4, ['traffic_red', 'traffic_yellow', 'traffic_green']),
    ('8 parking', 'parking', 4, ['parking']),
]


def load_settings():
    """차량 치수(config/robot.yaml)와 주행 파라미터(params.py). 못 읽으면 기본값으로 표시만 계속"""
    here = Path(__file__).resolve().parent
    robot = dict(wheelbase=0.2, width=0.225, length=0.33, front_axle_x=0.2, front_bumper_x=0.258,
                 camera=dict(hfov_deg=71.0, x=0.25, y=0.0, z=0.1875, pitch=0.148), lidar=dict(x=0.214, y=0.0))
    answer = {}
    try:
        import yaml
        robot.update(yaml.safe_load((here.parent / 'config' / 'robot.yaml').read_text())['robot'])
    except Exception as error:
        print(f'robot.yaml 읽기 실패, 기본 치수 사용: {error}', flush=True)
    try:
        try:
            from ericar_driver.params import ANSWER_CONFIG
        except ImportError:
            sys.path.insert(0, str(here.parent))   # 설치 없이 레포에서 바로 실행
            from ericar_driver.params import ANSWER_CONFIG
        answer = ANSWER_CONFIG
    except Exception as error:
        print(f'params.py 읽기 실패, 기본 기준 사용: {error}', flush=True)
    return robot, answer


ROBOT, ANSWER = load_settings()


def put(img, text, x, y, color=WHITE, scale=0.45, thick=1):
    cv2.putText(img, str(text), (int(x), int(y)), FONT, scale, color, thick, cv2.LINE_AA)


def num(value, fmt='%.2f', none='-'):
    return none if value is None else fmt % value


def blank(width, text):
    img = np.zeros((PANEL_H, width, 3), np.uint8)
    put(img, text, 10, 24, RED)
    return img


# ---------- 카메라 ----------

def lane_config(telemetry):
    return ((telemetry or {}).get('display') or {}).get('lane') or ANSWER.get('lane')


def bev_to_ground(col, row, lc):
    """BEV 픽셀 → 바닥 (앞 m, 왼쪽 m), 뒷축 기준. lane_vision.ground_maps와 같은 식"""
    forward = lc['far'] - row / (lc['height_px'] - 1) * (lc['far'] - lc['near'])
    return forward, (0.5 - col / (lc['width_px'] - 1)) * lc['view_width']


def ground_to_bev(forward, left, lc):
    row = (lc['far'] - forward) / (lc['far'] - lc['near']) * (lc['height_px'] - 1)
    return (0.5 - left / lc['view_width']) * (lc['width_px'] - 1), row


def to_image(forward, left, w, h):
    """바닥 점(뒷축 기준) → 카메라 화면 픽셀. 카메라 뒤·바로 밑이면 None"""
    cam = ROBOT['camera']
    dx, pitch = forward - cam['x'], cam['pitch']
    depth = math.cos(pitch) * dx + math.sin(pitch) * cam['z']
    if depth <= 0.03:
        return None
    up = math.sin(pitch) * dx - math.cos(pitch) * cam['z']
    focal = w / (2 * math.tan(math.radians(cam['hfov_deg']) / 2))
    return int(w / 2 - focal * (left - cam.get('y', 0.0)) / depth), int(h / 2 - focal * up / depth)


def sign_status(obj, control, stale):
    """YOLO 상자 하나가 판단에 쓰이는지 (driver.confirmed와 같은 기준). 반환 (색, 꼬리표)"""
    d = ANSWER.get('detector', {})
    name = obj.get('name', '?')
    if stale:
        return DIM, 'stale'
    allowed = d.get('allow_truncated_classes', ['traffic_red', 'traffic_yellow', 'traffic_green'])
    if obj.get('truncated') and name not in allowed:
        return GREY, 'cut'
    if obj.get('area', 0) < d.get('minimum_area_by_class', {}).get(name, d.get('minimum_area', 0.0)):
        return GREY, 'small'
    if name in (control.get('cooldowns') or {}):
        return GREY, 'cooldown'
    count = (control.get('sign_counts') or {}).get(name, 0)
    need = d.get('confirm_frames', 2)
    return (GREEN, 'ok') if count >= need else (YELLOW, f'{count}/{need}')


def draw_camera(camera, telemetry):
    if camera is None:
        return blank(480, 'no camera image')
    h = PANEL_H
    w = int(camera.shape[1] * h / camera.shape[0])
    img = cv2.resize(camera, (w, h), interpolation=cv2.INTER_AREA)
    # 주차 회전 중: 제어기가 정지 판단에 쓴 흰 직선(텔레메트리 값, 화면에서 다시 계산하지 않음)
    white = (((telemetry or {}).get('control', {}).get('parking') or {}).get('white_line') or {})
    scale = h / camera.shape[0]
    for x0, y0, x1, y1 in white.get('segments') or []:
        cv2.line(img, (int(x0 * scale), int(y0 * scale)), (int(x1 * scale), int(y1 * scale)), YELLOW, 2)
    feedback=(((telemetry or {}).get('control', {}).get('parking') or {}).get('feedback') or {})
    if feedback:
        color=CYAN if feedback.get('active') else GREY
        for x0,y0,x1,y1 in feedback.get('pixel_segments') or []:
            cv2.line(img,(int(x0*scale),int(y0*scale)),(int(x1*scale),int(y1*scale)),color,3)
        target=feedback.get('reference_pixel')
        if target:cv2.drawMarker(img,tuple(int(q*scale) for q in target),color,cv2.MARKER_CROSS,14,2)
        park=(telemetry or {}).get('control', {}).get('parking') or {}
        mode=park.get('turn_phase','align') if park.get('state')=='parking_turn' else park.get('state','parking').removeprefix('parking_')
        put(img,f"PARK {mode.upper()} {park.get('turn_angle_deg',0):.0f}deg {feedback.get('reason','')} e {feedback.get('lateral_m',0):+.2f}m h {feedback.get('heading_rad',0):+.2f}",6,36,color,.42)
    t = telemetry or {}
    control, lane, lc = t.get('control', {}), t.get('lane', {}), lane_config(t)
    if lc:
        # 차선 곡선·목표점을 카메라 화면에 다시 투영: BEV 결과가 실제 선 위에 놓이는지 확인 (보정 틀어짐 포함)
        for side, fit in lane.get('fits', {}).items():
            lo, hi = lane.get('fit_spans', {}).get(side, (0, lc['height_px'] - 1))
            rows = np.arange(lo, hi + 1, 6.0)
            pts = [to_image(*bev_to_ground(x, r, lc), w, h) for x, r in zip(np.polyval(fit, rows), rows)]
            pts = [p for p in pts if p is not None]
            if len(pts) > 1:
                cv2.polylines(img, [np.array(pts, np.int32)], False, GREEN if side == 'left' else BLUE, 2)
        if lane.get('valid'):
            target = to_image(lane.get('forward', 0.65), lane.get('error', 0.0), w, h)
            if target is not None:
                cv2.line(img, (w // 2, h - 1), target, RED, 1)
                cv2.circle(img, target, 5, RED, -1)
    line = control.get('line_distance')
    if line is not None:
        forward = line + ROBOT['front_axle_x']   # line_distance = 앞바퀴 축 기준
        ends = [to_image(forward, left, w, h) for left in (0.3, -0.3)]
        if None not in ends:
            cv2.line(img, ends[0], ends[1], MAGENTA, 2)
            put(img, f'stop {line:.2f} m', ends[0][0], min(h - 6, ends[0][1] - 6), MAGENTA, 0.5)
    # YOLO 상자. bbox = 화면 비율 [x0, y0, x1, y1]
    timeout = ((t.get('display') or {}).get('detector') or {}).get('timeout', 1.5)
    age = t.get('age')
    stale = age is None or not 0 <= age < timeout
    for obj in t.get('objects') or []:
        x0, y0, x1, y1 = obj.get('bbox', (0, 0, 0, 0))
        color, tag = sign_status(obj, control, stale)
        cv2.rectangle(img, (int(x0 * w), int(y0 * h)), (int(x1 * w), int(y1 * h)), color, 2)
        label = f"{obj.get('name')} {obj.get('confidence', 0):.2f} a{obj.get('area', 0) * 100:.2f}% {tag}"
        put(img, label, x0 * w, max(12, y0 * h - 5), color, 0.42)
    yolo = 'yolo off/none' if age is None or age > 1e6 else f'yolo {age:.2f}s' + (' STALE' if stale else '')
    cv2.rectangle(img, (0, 0), (w, 18), (0, 0, 0), -1)
    put(img, f"CAMERA  image {t.get('image_age', 0):.2f}s  {yolo}  n={len(t.get('objects') or [])}", 6, 13,
        WHITE if t else RED, 0.42)
    return img


# ---------- BEV ----------

def draw_bev(mask, telemetry):
    """BEV 마스크 위에 윈도우(회색)·차선 곡선(좌 초록/우 파랑)·목표 중심(빨강)·목표점·정지선 후보"""
    t = telemetry or {}
    lane, control, lc = t.get('lane', {}), t.get('control', {}), lane_config(t)
    if mask is None:
        if not lc:
            return blank(411, 'no BEV mask')
        mask = np.zeros((lc['height_px'], lc['width_px']), np.uint8)   # 녹화 재생 등 마스크 토픽이 없을 때: 곡선만
    bev = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR) // 2
    bev[mask == 100] = MAGENTA  # 흰색 마스크 (정지선·노면 표시)
    h, w = mask.shape[:2]
    for win in lane.get('windows', []):
        cv2.rectangle(bev, (int(win['x0']), int(win['y0'])), (int(win['x1']), int(win['y1'])), (90, 90, 90), 1)
    rows = np.arange(h, dtype=float)
    for side, fit in lane.get('fits', {}).items():
        lo, hi = lane.get('fit_spans', {}).get(side, (0, h - 1))
        keep = (rows >= lo) & (rows <= hi)
        points = np.stack([np.polyval(fit, rows[keep]), rows[keep]], axis=1).astype(np.int32)
        cv2.polylines(bev, [points], False, GREEN if side == 'left' else BLUE, 2)
    center = lane.get('center_px')
    if center is not None:
        cv2.line(bev, (int(center), 0), (int(center), h - 1), RED, 1)
    cv2.line(bev, (w // 2, h - 20), (w // 2, h - 1), WHITE, 2)  # 차량 중심
    if lc:
        if lane.get('valid'):
            col, row = ground_to_bev(lane.get('forward', 0.65), lane.get('error', 0.0), lc)
            cv2.circle(bev, (int(col), int(np.clip(row, 0, h - 1))), 6, RED, -1)
        # 정지선 후보(가는 선)와 채택한 정지선(굵은 선). 거리 = 앞바퀴 축 기준
        chosen = lane.get('stop_distance')
        for cand in lane.get('stop_candidates', []):
            row = int(ground_to_bev(cand['distance'] + ROBOT['front_axle_x'], 0.0, lc)[1])
            picked = chosen is not None and abs(cand['distance'] - chosen) < 1e-6
            cv2.line(bev, (0, row), (w - 1, row), YELLOW if picked else GREY, 2 if picked else 1)
            note = '' if picked else (' cut' if not cand.get('complete') else ' thick')
            put(bev, f"{cand['distance']:.2f}m t{cand['thickness']:.2f}{note}", 6, max(14, row - 4),
                YELLOW if picked else GREY, 0.45)
    cv2.rectangle(bev, (0, 0), (w, 20), (0, 0, 0), -1)
    colors, solid = lane.get('colors', {}), lane.get('solid', {})
    sides = '  '.join(f"{s[0].upper()}:{colors.get(s, '?')}{'(solid)' if solid.get(s) else ''}"
                      for s in ('left', 'right') if s in lane.get('fits', {}))
    put(bev, f'BEV  {sides}' if lane.get('valid') else 'BEV  NO LANE', 6, 15, WHITE if lane.get('valid') else RED, 0.5)
    notes = []
    if lane.get('zigzag'):
        notes.append(f"zigzag {lane['zigzag']}")
    if lane.get('abnormal'):
        notes.append('abnormal fit')
    if control.get('corner_hold'):
        notes.append('corner hold')
    if notes:
        put(bev, '  '.join(notes), 6, h - 8, ORANGE, 0.55)
    return cv2.resize(bev, (int(w * PANEL_H / h), PANEL_H), interpolation=cv2.INTER_AREA)


# ---------- 라이다 ----------

def lp(x, y):
    """차체 좌표(뒷축 원점, 앞 x, 왼쪽 y, m) → 라이다 패널 픽셀. 위 = 앞"""
    return int(LIDAR_W / 2 - y * PX_M), int(PANEL_H - 58 - x * PX_M)


def road_to_body(forward, left, heading):
    """도로 기준 (앞, 왼쪽) → 차체 좌표. lidar.to_body와 같음"""
    return forward * math.cos(heading) - left * math.sin(heading), forward * math.sin(heading) + left * math.cos(heading)


def scan_clusters(scan):
    """/scan → 차체 좌표 점, 점별 종류(0 기타, 1 넓은 물체, 2 좁은 물체 = 라바콘 후보), 좁은 물체 중심

    lidar.scan_sectors의 군집 규칙을 표시용으로 그대로 옮김 (주행 노드가 본 것과 같은 분류)
    """
    c = ANSWER.get('lidar', {})
    values = np.asarray(scan.ranges, dtype=float)
    angles = scan.angle_min + np.arange(len(values)) * scan.angle_increment
    if ROBOT['lidar'].get('zero_is_no_return', False):
        values = np.where(values == 0.0, np.inf, values)
    hits = np.isfinite(values) & (values >= scan.range_min) & (values < min(scan.range_max, c.get('planning_point_range', 3.0)))
    points = np.column_stack((values[hits] * np.cos(angles[hits]), values[hits] * np.sin(angles[hits])))
    kinds, centers = np.zeros(len(points), int), []
    if len(points):
        breaks = np.flatnonzero(np.linalg.norm(np.diff(points, axis=0), axis=1) > c.get('cluster_gap', .12)) + 1
        for index in np.split(np.arange(len(points)), breaks):
            if len(index) < c.get('cluster_min_points', 3):
                continue
            cluster = points[index]
            width = float(np.linalg.norm(cluster.max(axis=0) - cluster.min(axis=0)))
            center = cluster.mean(axis=0)
            if not 0 < center[0] < c.get('small_object_range', 1.8):
                continue
            if width > c.get('small_object_max_width', .12):
                if abs(center[1]) < c.get('wide_object_half_width', .9):
                    kinds[index] = 1
            elif width >= c.get('small_object_min_width', .025) and abs(center[1]) < c.get('small_object_half_width', .75):
                kinds[index] = 2
                centers.append(center)
    offset = np.array([ROBOT['lidar']['x'], ROBOT['lidar'].get('y', 0.0)])
    return points + offset, kinds, [center + offset for center in centers]


def road_box(img, forward0, forward1, middle, half, heading, color, thick=1):
    corners = [lp(*road_to_body(f, l, heading)) for f, l in
               ((forward0, middle - half), (forward1, middle - half), (forward1, middle + half), (forward0, middle + half))]
    cv2.polylines(img, [np.array(corners, np.int32)], True, color, thick)
    return corners


def draw_lane_change(img, control, lane):
    """3차선: 판단 중이면 차로별 빈 거리(가운데·왼쪽·오른쪽)와 선택, 이동 중이면 목표점"""
    c, d = ANSWER.get('lane_change', {}), control.get('lane_change') or {}
    if d.get('active'):
        error = control.get('lane_change_target', 0.0) - control.get('lane_change_offset', 0.0)
        target = lp(*road_to_body(c.get('lookahead', 0.55), error, control.get('lane_change_heading', 0.0)))
        cv2.line(img, lp(0, 0), target, YELLOW, 1)
        cv2.circle(img, target, 6, YELLOW, -1)
        safe = '' if d.get('safe', True) else '  NO SAFE STEER'
        return f"LANE CHANGE {'left' if d.get('direction', 0) > 0 else 'right'}  remaining {error:+.2f} m{safe}"
    free = d.get('free')
    if not free or not (control.get('wide_objects') or d.get('waiting')) or not lane.get('valid'):
        return None
    heading = lane.get('heading', 0.0)
    center = lane.get('error', 0.0) * math.cos(heading) - lane.get('forward', .65) * math.sin(heading)
    half = ROBOT['width'] / 2 + c.get('margin', 0.035)
    choice = d.get('choice', 0)
    for direction in (1, 0, -1):
        distance = free.get(str(direction), free.get(direction))
        if distance is None:
            continue
        short = distance < (c.get('trigger_distance', 1.15) if direction == 0 else c.get('min_free_distance', 1.0))
        color = GREEN if direction == choice and direction != 0 else RED if short else GREY
        corners = road_box(img, 0.0, min(distance, 2.6), center + direction * c.get('lane_width', 0.52), half, heading,
                           color, 2 if direction == choice and direction != 0 else 1)
        put(img, f'{distance:.2f}', corners[1][0] - 4, max(34, corners[1][1] - 4), color, 0.42)
    state = 'WAITING: no free lane' if d.get('waiting') else 'approach' if d.get('approach') else \
        'go ' + {1: 'left', -1: 'right'}.get(choice, 'none')
    return f"3-LANE lane {d.get('index', '?')}  {state}"


def draw_cones(img, control):
    """라바콘: 통로 중앙선(노랑)·좌우 경계(주황)·목표점"""
    c, d = ANSWER.get('lidar', {}), control.get('cones') or {}
    if d.get('state') != 'active':
        if d.get('state') == 'wait' and 'cones' in (control.get('missions') or []) and d.get('cones'):
            return f"CONES wait  cones {d.get('cones')}  confirm {d.get('confirm')}/{c.get('cone_confirm_count', '?')}"
        return None
    heading, center, width = d.get('heading', 0.0), d.get('center', 0.0), d.get('width', 0.6)
    far = c.get('cone_view_distance', 1.5)
    for offset, color in ((0.0, YELLOW), (width / 2, ORANGE), (-width / 2, ORANGE)):
        cv2.line(img, lp(*road_to_body(0.0, center + offset, heading)),
                 lp(*road_to_body(far, center + offset, heading)), color, 1)
    cv2.circle(img, lp(*road_to_body(c.get('cone_lookahead', 0.5), center, heading)), 6, YELLOW, -1)
    safe = '' if d.get('safe', True) else '  NO SAFE STEER (creep)'
    return (f"CONES n {d.get('cones')}  width {width:.2f}  travel {d.get('travel', 0):.2f}/{d.get('max_travel', 0):.2f}"
            f"{safe}")


def draw_parking(img, control, live=None):
    """주차 빈칸 판정: 물체 군집(자홍)·빈틈 후보(FREE 초록, 확인 부족 주황, 점유 빨강)·선택 칸(노랑)
    주행 노드가 켜져 있으면 텔레메트리(control.parking) 값만 그림. live는 주행 노드 없이 센서만 볼 때의 계산 결과"""
    park = control.get('parking') or {}
    gap = park.get('gap') or live or {}
    if not park and not gap:
        return None
    colors = {'FREE': GREEN, 'UNKNOWN': ORANGE, 'OCCUPIED': RED}
    states = []
    try:
        def polygon(points, color, thick):
            cv2.polylines(img, [np.array([lp(x, y) for x, y in points], np.int32)], True, color, thick)
        for box in gap.get('clusters', []):
            polygon(box['points'], MAGENTA, 1)
        for candidate, result in zip(gap.get('candidates', []), gap.get('evaluations', [])):
            state, evidence = result.get('state'), result.get('evidence') or {}
            states.append(state)
            polygon(candidate['polygon'], colors.get(state, GREY), 2)
            put(img, f"{state} {evidence.get('covered_samples', 0)}/12", *lp(*candidate['entrance']),
                colors.get(state, GREY), 0.4)
        template=park.get('parking_template') or {}
        fusion=park.get('parking_fusion') or {}
        for box in fusion.get('clusters',[]):
            polygon(box.get('points',[]),CYAN,1)
            centre=box.get('centroid',[0.,0.]);slot=box.get('slot_index')
            put(img,f"L{box.get('id','?')} -> S{'-' if slot is None else slot+1}",
                *lp(*centre),CYAN,.35)
        for result in template.get('evaluations', []):
            state=result.get('state','UNKNOWN');slot=result.get('slot_index','?')
            color=colors.get(state,GREY);states.append(state)
            polygon(result.get('polygon',[]),color,2)
            evidence=result.get('evidence') or {};hits=evidence.get('interior_hits',0)
            label=f"S{slot+1 if isinstance(slot,int) else slot} {state} p={hits}"
            put(img,label,*lp(*result.get('entrance',[0.,0.])),color,.38)
        for point in (park.get('turn_path') or {}).get('points',[]):
            cv2.circle(img,lp(point[0],point[1]),2,CYAN,-1)
        if park.get('selected_gap'):
            polygon(park['selected_gap']['polygon'], YELLOW, 2)
            approach=park.get('approach') or {}
            if park.get('state')=='parking_approach' and approach:
                target=lp(approach['remaining_m'],0.)
                cv2.drawMarker(img,target,CYAN,cv2.MARKER_CROSS,12,2)
                put(img,f"TURN IN {approach['remaining_m']:+.2f}m",target[0]+5,target[1]-8,CYAN,.45)
                path=approach.get('turn_path') or []
                if len(path)>1:
                    cv2.polylines(img,[np.array([lp(item[0],item[1]) for item in path],np.int32)],False,CYAN,1)
    except Exception:
        pass
    if not park:
        return f"PARK live (driver off) {','.join(states) or 'no gap'}"[:64]
    if park.get('state')=='parking_cruise':
        sign=park.get('sign') or {}
        offset=(park.get('parking_fusion') or {}).get('offset',[0.,0.])
        return f"PARK CRUISE fusion {offset[0]:+.2f},{offset[1]:+.2f} near {sign.get('near_seen')}"
    if park.get('state')=='parking_approach':
        approach=park.get('approach') or {}
        return f"PARK APPROACH turn in {approach.get('remaining_m',0):+.2f}m travel {approach.get('travel_m',0):.2f}m"
    feedback=park.get('feedback') or {}
    if feedback.get('active'):
        return f"PARK PD {park.get('turn_phase','align').upper()} {park.get('turn_angle_deg',0):.0f}deg {feedback.get('source')} e {feedback.get('lateral_m',0):+.2f}m h {feedback.get('heading_rad',0):+.2f} steer {feedback.get('steering_rad',0):+.2f}"
    return (f"PARK {park.get('state', '')} {','.join(states) or 'no gap'}  "
            f"search {park.get('search_distance_m', 0):.2f} turn {park.get('turn_distance_m', 0):.2f}m")[:64]


def live_gap(scan):
    """주행 노드 없이 센서만 켠 화면용: 제어기와 같은 함수로 빈칸 판정 (주행 노드가 켜져 있으면 호출 안 함)"""
    try:
        from ericar_driver.lidar import scan_sectors
        from ericar_driver.parking_gap import gap_candidates, classify_gap
        config = ANSWER['parking']
        data = scan_sectors(scan.ranges, scan.angle_min, scan.angle_increment,
                            scan.range_min, scan.range_max, ANSWER['lidar'])
        candidates, debug = gap_candidates(data, ROBOT, config)
        debug['evaluations'] = [dict(zip(('state', 'evidence'), classify_gap(bay, data, ROBOT, config)))
                                for bay in candidates]
        return debug
    except Exception:
        return None


def draw_lidar(scan, telemetry):
    t = telemetry or {}
    control, lane, c = t.get('control', {}), t.get('lane', {}), ANSWER.get('lidar', {})
    img = np.zeros((PANEL_H, LIDAR_W, 3), np.uint8)
    for meters in (0.5, 1.0, 1.5, 2.0, 2.5):   # 앞 거리 눈금
        y = lp(meters, 0)[1]
        cv2.line(img, (0, y), (LIDAR_W, y), (38, 38, 38), 1)
        put(img, f'{meters:.1f}', LIDAR_W - 28, y - 3, DIM, 0.38)
    for meters in (-1.0, -0.5, 0.5, 1.0):      # 좌우 눈금
        x = lp(0, meters)[0]
        cv2.line(img, (x, 20), (x, PANEL_H), (38, 38, 38), 1)
    # 명령 조향으로 갈 경로 (뒷축 중심선 + 차폭). 속도 0이면 어둡게
    steering, speed = control.get('steering', 0.0), control.get('speed', 0.0)
    curvature = math.tan(steering) / ROBOT['wheelbase']
    travel = np.linspace(0.0, 1.0, 24)
    angle = travel * curvature
    xs = travel if abs(curvature) < 1e-6 else np.sin(angle) / curvature
    ys = np.zeros_like(travel) if abs(curvature) < 1e-6 else (1 - np.cos(angle)) / curvature
    for offset in (0.0, ROBOT['width'] / 2, -ROBOT['width'] / 2):
        path = [lp(x - offset * math.sin(a), y + offset * math.cos(a)) for x, y, a in zip(xs, ys, angle)]
        cv2.polylines(img, [np.array(path, np.int32)], False, DIM if abs(speed) < 1e-3 else CYAN, 2 if offset == 0 else 1)
    # 차체, 전방 비상 정지 범위 (라이다 기준 부채꼴)
    rear = ROBOT.get('front_bumper_x', 0.258) - ROBOT['length']
    body = [lp(x, y) for x, y in ((rear, -ROBOT['width'] / 2), (ROBOT.get('front_bumper_x', 0.258), -ROBOT['width'] / 2),
                                  (ROBOT.get('front_bumper_x', 0.258), ROBOT['width'] / 2), (rear, ROBOT['width'] / 2))]
    cv2.polylines(img, [np.array(body, np.int32)], True, WHITE, 1)
    emergency, half = c.get('emergency_distance', 0.18), math.degrees(c.get('front_half_angle', 0.3))
    blocked = control.get('blocked', control.get('front', 9.9) < emergency)
    cv2.ellipse(img, lp(ROBOT['lidar']['x'], 0), (int(emergency * PX_M),) * 2, 0, -90 - half, -90 + half,
                RED if blocked else (0, 0, 130), 2 if blocked else 1)
    notes = []
    if scan is not None:
        points, kinds, centers = scan_clusters(scan)
        palette = ((0, 170, 0), RED, ORANGE)
        for (x, y), kind in zip(points, kinds):
            px = lp(x, y)
            if 0 <= px[0] < LIDAR_W and 20 <= px[1] < PANEL_H:
                cv2.circle(img, px, 2, palette[kind], -1)
        for x, y in centers:
            cv2.circle(img, lp(x, y), 7, ORANGE, 1)
    else:
        notes.append('no /scan')
    for note in (draw_lane_change(img, control, lane), draw_cones(img, control), draw_parking(img, control, live_gap(scan) if scan is not None and not telemetry else None)):
        if note:
            notes.append(note)
    cv2.rectangle(img, (0, 0), (LIDAR_W, 18), (0, 0, 0), -1)
    valid = control.get('scan_valid')
    put(img, f"LIDAR  front {num(control.get('front'))} rear {num(control.get('rear'))}  "
             f"valid {num(None if valid is None else valid * 100, '%.0f%%')}  wide {num(control.get('wide_objects'), '%d')} "
             f"narrow {num(control.get('narrow_objects'), '%d')}", 6, 13, RED if blocked else WHITE, 0.42)
    for i, note in enumerate(notes):
        put(img, note, 6, PANEL_H - 8 - 17 * (len(notes) - 1 - i), YELLOW, 0.42)
    return img


# ---------- 아래 글자 패널 ----------

def stop_reasons(t):
    """명령 속도가 0인 이유 (주행 중 멈춤 원인 추적). 속도가 있으면 빈 목록"""
    control, lane = t.get('control', {}), t.get('lane', {})
    if abs(control.get('speed', 0.0)) > 1e-3:
        return []
    timeout = (t.get('display') or {}).get('sensor_timeout', ANSWER.get('sensor_timeout', 0.7))
    state, reasons = str(control.get('controller_state', '')), []
    if not control.get('sensor_ready', True):
        stale = [name for name, age in (('image', t.get('image_age')), ('scan', t.get('scan_age')))
                 if age is None or not 0 <= age < timeout]
        reasons.append('sensors: ' + (' '.join(stale) + ' stale' if stale else 'odom stale or first YOLO wait'))
    if state.endswith('_hold'):
        reasons.append(f"{state}: waiting (stopped {num(control.get('stopped_for'), '%.1f')} s)")
    elif state.endswith('_approach'):
        reasons.append(f'{state}: at stop line')
    elif state == 'parked' or state.startswith('fault'):
        reasons.append(state)
    if control.get('blocked'):
        valid = control.get('scan_valid')
        reasons.append('lidar valid < 50%' if valid is not None and valid < .5 else
                       f"emergency stop: front {num(control.get('front'))} m")
    change = control.get('lane_change') or {}
    if change.get('waiting'):
        reasons.append('lane change: no free lane')
    elif change.get('active') and change.get('safe') is False:
        reasons.append('lane change: no safe steering')
    grace = (ANSWER.get('lane') or {}).get('lost_grace', 0.5)
    if not lane.get('valid', True) and control.get('lane_lost', 0) > grace:
        reasons.append(f"lane lost {control['lane_lost']:.1f} s, search ended ({control.get('search_distance', 0):.2f} m)")
    return reasons or ['unknown (check image age limit / mission code)']


def state_color(control):
    state = str(control.get('state', ''))
    if not control.get('sensor_ready', True) or state.startswith('fault') or control.get('blocked'):
        return RED
    if state == 'lane':
        return GREEN
    return YELLOW if state == 'corner_hold' else ORANGE


def status_panel(width, t):
    panel = np.zeros((INFO_H, width, 3), np.uint8)
    if not t:
        put(panel, 'no telemetry (/answer/detections): driver node not running?', 8, 28, RED, 0.5)
        return panel
    control, lane = t.get('control', {}), t.get('lane', {})
    mode = str(control.get('state', '?'))
    put(panel, mode[:48], 8, 28, state_color(control), 0.8 if len(mode) <= 24 else 0.5, 2)
    y = 50
    def line(text, color=WHITE):
        nonlocal y
        put(panel, text, 8, y, color)
        y += 19
    line(f"controller {control.get('controller_state', '?')}  for {num(control.get('state_time'), '%.1f')} s", GREY)
    line(f"cmd  speed {control.get('speed', 0):+.3f} m/s  steer {control.get('steering', 0):+.3f} rad   "
         f"actual {num(control.get('velocity'), '%+.3f')} m/s")
    signs = 'ok' if control.get('signs_ok', True) else f"LOST {num(control.get('signs_lost'), '%.1f')} s"
    line(f"age  image {t.get('image_age', 0):.2f}  scan {t.get('scan_age', 0):.2f}  yolo {min(t.get('age', 0), 99):.2f} s   "
         f"signs {signs}   ready {control.get('sensor_ready')}",
         WHITE if control.get('sensor_ready', True) and control.get('signs_ok', True) else RED)
    method = 'pursuit' if lane.get('pursuit') else 'pid'
    line(f"lane {'valid' if lane.get('valid') else 'NOT VALID'}  error {lane.get('error', 0):+.3f} m  "
         f"heading {lane.get('heading', 0):+.3f}  curv {lane.get('curvature', 0):.1f}  {method}",
         WHITE if lane.get('valid') else RED)
    line(f"stop line {num(control.get('line_distance'))} m (front axle)   stopped {num(control.get('stopped_for'), '%.1f')} s")
    flags = []
    if control.get('corner_hold'):
        flags.append('corner_hold')
    if control.get('slow_zone') == 'slow':
        flags.append(f"slow zone tilt {control.get('slow_tilt_deg', 0):.1f}deg" + (f" BOOST {control['slow_boost']}" if control.get('slow_boost') else ''))
    if control.get('zigzag_left'):
        flags.append(f"zigzag {control['zigzag_left']:.2f}m")
    if control.get('fork_pending') or control.get('fork_remaining'):
        flags.append(f"fork {control.get('fork_remaining', 0):.2f}m{' pending' if control.get('fork_pending') else ''}")
    if control.get('lane_change_off'):
        flags.append(f"lane-change off {control['lane_change_off']:.2f}m")
    if 0.3 < control.get('lane_lost', 0) < 1e6:
        flags.append(f"lane lost {control['lane_lost']:.1f}s search {control.get('search_distance', 0):.2f}m")
    line('flags  ' + ('  '.join(flags) or '-'), ORANGE if flags else GREY)
    park = control.get('parking', {})
    if park:
        white = park.get('white_line', {})
        line(f"WHITE present {white.get('present')} seen {white.get('seen_during_turn')} loss {white.get('loss_frames', 0)}", YELLOW)
    if park.get('state')=='parking_approach':
        approach=park.get('approach') or {}
        line(f"TURN IN {approach.get('remaining_m',0):+.2f} m  lead {approach.get('turn_lead_m',0):.2f} m", CYAN)
    stall, limit = control.get('stall_time', 0.0), (ANSWER.get('stall') or {}).get('seconds', 8.0)
    feedback=park.get('feedback') or {}
    if park.get('state')=='parking_turn' and feedback:
        line(f"{park.get('turn_phase','?').upper()} {park.get('turn_angle_deg',0):.0f}deg released {park.get('arc_released')} steer {feedback.get('steering_rad',0):+.2f}",CYAN if feedback.get('active') else ORANGE)
    elif park.get('state')=='parking_cruise':
        sign=park.get('sign') or {}
        line(f"CRUISE LiDAR ON near {sign.get('near_seen')} missing {sign.get('missing_results',0)}",CYAN)
    else:
        line(f'stall guard {stall:.1f} / {limit:.0f} s', RED if stall > 0 else GREY)
    reasons = stop_reasons(t)
    if reasons:
        put(panel, 'SPEED 0: ' + ' / '.join(reasons), 8, min(y + 4, INFO_H - 8), RED, 0.5)
    return panel


def mission_state(key, section, signs, control, lane):
    """미션 한 줄의 (상태, 색, 세부 값)"""
    state = str(control.get('controller_state', ''))
    index = control.get('section', 0)
    change, cones = control.get('lane_change') or {}, control.get('cones') or {}
    counts, cooldowns = control.get('sign_counts') or {}, control.get('cooldowns') or {}
    seen = '  '.join(f"{name.replace('traffic_', '')}:{counts.get(name, 0)}" + (f"(cd {cooldowns[name]:.0f}s)" if name in cooldowns else '')
                     for name in signs if name in counts or name in cooldowns)
    line = f"line {num(control.get('line_distance'))} m"
    active, detail = False, seen
    if key is None:
        active = bool(control.get('zigzag_left'))
        detail = f"camera, any section  seen {lane.get('zigzag', 0)}  left {control.get('zigzag_left', 0):.2f} m"
        return ('ACTIVE' if active else 'ON'), (ORANGE if active else GREY), detail
    if key == 'slow':
        # slow_zone.py: idle → slow → done. 방지턱 = 자이로 기울기(°)가 문턱을 넘은 지점(서행 시작 뒤 m), boost = 턱에서 멈춤 대응 단계
        zone = ANSWER.get('slow_zone', {})
        active = control.get('slow_zone') == 'slow'
        if active:
            detail = (f"tilt {control.get('slow_tilt_deg', 0):.1f}/{zone.get('tilt_threshold', 0):.0f} deg  "
                      f"bump {num(control.get('slow_bump_at'))} m  boost {control.get('slow_boost', 0)}")
        elif control.get('slow_zone') == 'idle' and zone.get('entry_distance') is not None:
            detail = f"sign or {control.get('section_distance', 0):.2f}/{zone['entry_distance']:.1f} m  {seen}"
    elif key in ('pedestrian', 'traffic'):
        active = state.startswith(key)
        if active:
            detail = f"{state.split('_')[-1]}  {line}  stopped {num(control.get('stopped_for'), '%.1f')} s  {seen}"
    elif key == 'lane_change':
        active = bool(change.get('active') or change.get('waiting') or change.get('approach'))
        free = change.get('free') or {}
        detail = f"lane {control.get('lane_index', '?')}"
        if free and (active or control.get('wide_objects')):
            detail += '  free L %s C %s R %s' % tuple(num(free.get(k)) for k in ('1', '0', '-1'))
        if control.get('lane_change_off'):
            detail += f"  off {control['lane_change_off']:.1f} m"
    elif key == 'fork':
        active = bool(control.get('fork_pending') or control.get('fork_remaining'))
        if active:
            detail = (f"remain {control.get('fork_remaining', 0):.2f} m  request {control.get('fork_request', 0)}"
                      f"{'  pending' if control.get('fork_pending') else ''}")
    elif key == 'cones':
        active = bool(control.get('cone_active'))
        detail = '  '.join(f'{k} {num(cones.get(k)) if isinstance(cones.get(k), float) else cones.get(k)}'
                           for k in ('state', 'cones', 'confirm', 'width') if k in cones)
    elif key == 'parking':
        active = state.startswith('park')
        if active:
            detail = state + '  ' + str((control.get('parking') or {}).get('reason', ''))
    if active:
        return 'ACTIVE', ORANGE, detail
    if key in (control.get('done') or []):
        return 'DONE', GREEN, detail
    if key in (control.get('off') or []):
        return 'OFF', RED if section == index else DIM, 'sections.off in params.py'
    if section == index:
        return 'ARMED', YELLOW, detail
    if section < index:
        # 표지판 미션은 시작하면 done에 들어감 → 구간을 지났는데 done이 아니면 못 본 것. 라이다 미션은 done 표시 없음
        return ('MISSED', RED, detail) if signs else ('passed', GREY, detail)
    return '-', DIM, ''


def mission_panel(width, t):
    panel = np.zeros((INFO_H, width, 3), np.uint8)
    control, lane = (t or {}).get('control', {}), (t or {}).get('lane', {})
    if 'section' not in control:
        put(panel, 'MISSIONS  (no section data)', 8, 20, GREY)
        return panel
    put(panel, f"SECTION S{control['section']}   {control.get('section_distance', 0):.2f} m   "
               f"turned {control.get('section_turned', 0):.0f} deg   yaw {control.get('yaw_deg', 0):.0f} ({control.get('yaw_source', '?')})",
        8, 22, CYAN, 0.5)
    for i, (label, key, section, signs) in enumerate(MISSIONS):
        status, color, detail = mission_state(key, section, signs, control, lane)
        y = 48 + i * 23
        if section == control['section']:
            cv2.rectangle(panel, (2, y - 15), (width - 3, y + 6), (45, 45, 45), -1)
        put(panel, f'S{section}', 8, y, DIM, 0.42)
        put(panel, label, 34, y, WHITE if section == control['section'] else GREY)
        put(panel, status, 140, y, color, 0.45, 2 if status == 'ACTIVE' else 1)
        put(panel, detail[:48], 206, y, GREY, 0.4)
    return panel


def log_panel(width, events):
    panel = np.zeros((INFO_H, width, 3), np.uint8)
    put(panel, 'EVENTS (newest first)', 8, 20, CYAN, 0.5)
    for i, (stamp, text, color) in enumerate(list(events)[:10]):
        put(panel, f'{stamp} {text}'[:60], 8, 42 + i * 19, color if i == 0 else tuple(int(v * 0.7) for v in color), 0.4)
    return panel


def compose(camera, mask, telemetry, scan=None, events=()):
    """위: 카메라·BEV·라이다 / 아래: 상태·미션·사건 기록. 입력이 없으면 그 칸만 안내 글자"""
    top = np.hstack([draw_camera(camera, telemetry), draw_bev(mask, telemetry), draw_lidar(scan, telemetry)])
    width = top.shape[1]
    left, middle = int(width * 0.36), int(width * 0.38)
    bottom = np.hstack([status_panel(left, telemetry), mission_panel(middle, telemetry),
                        log_panel(width - left - middle, events)])
    bottom[:, [left, left + middle]] = (60, 60, 60)
    return np.vstack([top, bottom])


# ---------- 수신 상태·사건 기록 ----------

class State:
    def __init__(self):
        self.lock = threading.Condition()
        self.camera = self.mask = self.telemetry = self.scan = None
        self.events = deque(maxlen=40)   # (시각, 내용, 색), 최신이 앞
        self.previous = None             # 직전 텔레메트리 control (변화 감지)
        self.seq = 0

    def update(self, **kw):
        with self.lock:
            for key, value in kw.items():
                setattr(self, key, value)
            self.seq += 1
            self.lock.notify_all()

    def event(self, text, color=WHITE):
        self.events.appendleft((time.strftime('%H:%M:%S'), text, color))

    def track(self, telemetry):
        """텔레메트리 변화 → 사건 기록 (구간 전환·미션 종료·표지판 확정·센서·막힘). 상태 전환은 /answer/status로 받음"""
        control = telemetry.get('control', {})
        before, self.previous = self.previous, control
        if before is None:
            return
        if 'section' in control and control.get('section') != before.get('section'):
            self.event(f"section S{before.get('section')} -> S{control['section']}  yaw {control.get('yaw_deg', 0):.0f} deg", CYAN)
        for mission in sorted(set(control.get('done') or []) - set(before.get('done') or [])):
            self.event(f'mission done/left: {mission}', GREEN)
        need = (ANSWER.get('detector') or {}).get('confirm_frames', 2)
        counts, old = control.get('sign_counts') or {}, before.get('sign_counts') or {}
        for name, count in counts.items():
            if count >= need > old.get(name, 0):
                self.event(f'sign confirmed: {name}', YELLOW)
        if control.get('sensor_ready') != before.get('sensor_ready'):
            ready = control.get('sensor_ready')
            self.event('sensors ready' if ready else f"SENSORS LOST image {telemetry.get('image_age', 0):.2f} "
                       f"scan {telemetry.get('scan_age', 0):.2f} s", GREEN if ready else RED)
        if control.get('signs_ok', True) != before.get('signs_ok', True):
            self.event('yolo ok' if control.get('signs_ok') else 'YOLO LOST (sign missions off)',
                       GREEN if control.get('signs_ok') else RED)
        if bool(control.get('blocked')) != bool(before.get('blocked')):
            self.event(f"BLOCKED front {num(control.get('front'))} valid {num(control.get('scan_valid'))}"
                       if control.get('blocked') else 'unblocked', RED if control.get('blocked') else GREEN)
        new, was = control.get('lane_change') or {}, before.get('lane_change') or {}
        if bool(new.get('active')) != bool(was.get('active')):
            self.event(f"lane change {'start dir %+d' % new.get('direction', 0) if new.get('active') else 'end'}"
                       f"  lane {control.get('lane_index')}", ORANGE)
        if control.get('slow_zone') != before.get('slow_zone') and control.get('slow_zone') in ('slow', 'done'):
            self.event(f"slow zone {'start' if control['slow_zone'] == 'slow' else 'end'} at S0 {control.get('section_distance', 0):.2f} m", ORANGE)
        if control.get('slow_bump_at') is not None and before.get('slow_bump_at') is None:
            self.event(f"bump detected  tilt {control.get('slow_tilt_deg', 0):.1f} deg", ORANGE)
        if control.get('slow_boost', 0) > before.get('slow_boost', 0):
            self.event(f"stuck on bump: boost {control['slow_boost']}", RED)
        cones, cones_was = (control.get('cones') or {}).get('state'), (before.get('cones') or {}).get('state')
        if cones != cones_was and cones in ('active', 'done'):
            self.event(f'cones {cones}', ORANGE)
        if stop_reasons(telemetry) and not stop_reasons(dict(telemetry, control=before)):
            self.event('speed 0: ' + ' / '.join(stop_reasons(telemetry)), RED)


state = State()


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def do_GET(self):
        if self.path == '/':
            page = ('<!doctype html><meta charset="utf-8"><title>ERICAR debug</title>'
                    '<body style="margin:0;background:#111"><img src="/stream.mjpg" style="max-width:100%">')
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.end_headers()
            self.wfile.write(page.encode())
        elif self.path == '/stream.mjpg':
            self.stream()
        else:
            self.send_error(404)

    def stream(self):
        self.connection.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 65536)
        self.send_response(200)
        self.send_header('Cache-Control', 'no-store')
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
        self.end_headers()
        seen, last = 0, 0.0
        try:
            while True:
                with state.lock:
                    state.lock.wait_for(lambda: state.seq != seen, timeout=2)
                    seen = state.seq
                    camera, mask, telemetry, scan = state.camera, state.mask, state.telemetry, state.scan
                    events = list(state.events)
                if time.time() - last < 1.0 / MAX_FPS:
                    continue
                last = time.time()
                try:
                    frame = compose(camera, mask, telemetry, scan, events)
                except Exception as error:   # 그리기 오류로 중계가 끊기지 않게: 오류 내용을 화면에 표시
                    frame = blank(900, f'debug_view draw error: {error!r}'[:110])
                ok, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
                if ok:
                    data = jpeg.tobytes()
                    self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '
                                     + str(len(data)).encode() + b'\r\n\r\n' + data + b'\r\n')
        except (BrokenPipeError, ConnectionResetError):
            pass


def main():
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CompressedImage, Image, LaserScan
    from std_msgs.msg import String

    port = int(sys.argv[1]) if len(sys.argv) > 1 else 8080

    def on_camera(m):
        state.update(camera=cv2.imdecode(np.frombuffer(m.data, np.uint8), cv2.IMREAD_COLOR))

    def on_mask(m):
        state.update(mask=np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)[:, :m.width].copy())

    def on_telemetry(m):
        telemetry = json.loads(m.data)
        with state.lock:
            state.track(telemetry)
        state.update(telemetry=telemetry)

    def on_status(m):
        with state.lock:
            state.event(f'state -> {m.data}', WHITE)

    rclpy.init()
    node = rclpy.create_node('debug_view')
    node.create_subscription(CompressedImage, CAMERA_TOPIC, on_camera, qos_profile_sensor_data)
    node.create_subscription(Image, MASK_TOPIC, on_mask, qos_profile_sensor_data)
    node.create_subscription(String, TELEMETRY_TOPIC, on_telemetry, 10)
    node.create_subscription(String, STATUS_TOPIC, on_status, 10)
    node.create_subscription(LaserScan, '/scan', lambda m: setattr(state, 'scan', m), qos_profile_sensor_data)
    server = ThreadingHTTPServer(('0.0.0.0', port), Handler)
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print(f'http://<차 IP>:{port}', flush=True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
