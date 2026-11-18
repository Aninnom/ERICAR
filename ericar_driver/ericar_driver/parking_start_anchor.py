"""주차 줄 시작점을 시작선 기준 고정 거리로 잡음 (11/18 혁준). parking_template의 색 전환 기준점 대체

색 전환(차로 선 노란색 → 흰색) 기준점은 조명·노출에 따라 어긋남: redecide_1118_06 바깥 선, _12 1.4 m 앞, _15 0.3 m 앞
(옅은 노란 선이 흰색으로 읽힘, 노출 600에서는 기준 선분 자체가 S 47~88)
시작선(차로를 가로지르는 흰 선)은 노출 300·600 모두 S 5~26으로 안정적이고, 실제 줄 시작은 다섯 주행(06·07·08·12·15)에서
시작선 뒤 0.40~0.46 m (라이다상 표지판 받침·노출 300 기준점) → 줄 시작 = 시작선 + template_start_to_row
옆 위치는 차로 오른쪽 선(노란색·흰색 구분 없이)의 가운데값, 선이 안 보이면 template_row_lateral. 방향은 차 진행 방향
정윤 코드와 겹치지 않게 별도 파일. parking._observe_template에서 template_anchor_from_start가 켜져 있을 때만 사용
"""
import math
import numpy as np


def anchor_from_start_line(start_x, right_ground, config):
    """start_x: 시작선까지 앞 거리(m, 차체 좌표). right_ground: 차로 오른쪽 세로 선분들의 바닥 좌표 [[x0,y0],[x1,y1]], …
    반환 형식은 parking_template_landmarks의 anchors 항목과 같음"""
    x = float(start_x)+config.get('template_start_to_row', .45)
    lateral, slope = config.get('template_row_lateral', -.26), 0.
    lines = np.asarray(right_ground, float).reshape(-1, 2, 2) if len(right_ground) else np.zeros((0, 2, 2))
    if len(lines):
        # 차로 오른쪽 선만: 차가 차로 가운데 근처라는 전제로 예상 옆 위치 ± window, 가까운 선분만 (먼 선분은 위치 오차 큼)
        middle = lines.mean(axis=1)
        near = ((np.abs(middle[:, 1]-lateral) <= config.get('template_row_lateral_window', .12)) &
                (middle[:, 0] <= config.get('template_row_line_far', 1.6)))
        lines, middle = lines[near], middle[near]
    if len(lines):
        median = float(np.median(middle[:, 1]))
        points = lines[np.abs(middle[:, 1]-median) <= config.get('template_row_line_band', .08)].reshape(-1, 2)
        lateral = median
        # 방향 맞춤은 기본 끔: 차선 주행 중이라 차 진행 방향이 곧 줄 방향 (redecide_1118_04 재현에서 다른 선분이 섞여 7° 틀어짐)
        if (config.get('template_row_fit_slope', False) and len(points) >= 4 and
                np.ptp(points[:, 0]) >= config.get('template_row_fit_length', .30)):
            slope, offset = np.polyfit(points[:, 0], points[:, 1], 1)
            limit = config.get('template_anchor_max_slope') or .12
            slope = float(np.clip(slope, -limit, limit))
            lateral = float(offset+slope*x)
    norm = math.hypot(1., slope)
    return dict(anchor=[x, lateral], tangent=[1/norm, slope/norm], normal=[slope/norm, -1/norm],
                confidence='START_LINE_OFFSET', length=0., pixel=None, support_pixel=None)
