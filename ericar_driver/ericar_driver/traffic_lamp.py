"""신호등 박스 안 램프 위치로 색 재확인 (11/14)

YOLO가 같은 신호등을 빨강 ↔ 녹색으로 번갈아 읽음 (integ_1114_lap4 226~230 s: 빨강인데 녹색 15프레임 → 정지 뒤 0.18 m 더 나감)
세로 신호등: 위 = 빨강, 가운데 = 노랑, 아래 = 녹색. 박스를 세로 3등분해 밝고 채도 있는 화소 비율이 가장 큰 칸이 켜진 램프
녹화 3개 확인: 빨강 구간에서 녹색 판정 0회, 녹색 구간(lap3) 210/210. 판정이 약하면 None → YOLO 이름 그대로 사용
"""
import cv2
import numpy as np

NAMES = ('traffic_red', 'traffic_yellow', 'traffic_green')


def lamp_name(frame, box, config):
    """frame: BGR 영상, box: (x0, y0, x1, y1) 화소. 켜진 램프 이름 또는 None"""
    h, w = frame.shape[:2]
    x0, y0, x1, y1 = (int(round(v)) for v in box)
    crop = frame[max(0, y0):min(h, y1), max(0, x0):min(w, x1)]
    if crop.shape[0] < 9 or crop.shape[1] < 3:
        return None
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    lit = (hsv[..., 2] > config.get('lamp_value_min', 180)) & (hsv[..., 1] > config.get('lamp_saturation_min', 60))
    third = crop.shape[0]//3
    fractions = [float(lit[:third].mean()), float(lit[third:2*third].mean()), float(lit[2*third:].mean())]
    best = int(np.argmax(fractions))
    second = sorted(fractions)[1]
    if fractions[best] < config.get('lamp_min_fraction', .002) or fractions[best] < config.get('lamp_ratio', 3.)*second:
        return None
    return NAMES[best]


def box_name(frame, box, name, confidence, config):
    """YOLO 박스 하나의 최종 이름 (11/18 혁준, track_test_1118_02~05). 반환 (이름 또는 None, 덧붙일 값). None이면 버림

    정지선 앞에서 YOLO 초록 신뢰도가 기준(confidence 0.45) 근처를 오르내려 절반쯤 탈락 (04번 64프레임 중 34개, 최저 0.09)
    → 초록 연속 조건(traffic_green_hold)을 못 채워 첫 초록 뒤 20.6 s에 출발. 램프 위치 판정은 같은 프레임 전부 초록,
    빨강·노랑 구간에서 초록 오판 0 (네 주행 218프레임)
    - 신뢰도 기준 이상: 예전과 같음 (신호등은 램프 위치로 색 재확인)
    - 기준 미만: 신호등 박스이고 신뢰도 lamp_low_confidence 이상, 면적 lamp_low_area 이상, 램프 위치 판정이 확실할 때만
      그 색으로 인정 (가까운 신호등만). 그 밖의 낮은 신뢰도 박스는 버림. 끄기: lamp_low_confidence None
    """
    h, w = frame.shape[:2]
    x0, y0, x1, y1 = box
    area = (x1-x0)*(y1-y0)/(w*h)
    traffic = name in NAMES
    lamp = None
    if traffic and config.get('lamp_check', True) and area >= config.get('lamp_min_area', .005):
        lamp = lamp_name(frame, box, config)
    if confidence < config['confidence']:
        low = config.get('lamp_low_confidence')
        if not (traffic and lamp is not None and low is not None and confidence >= low
                and area >= config.get('lamp_low_area', .02)):
            return None, {}
        return lamp, dict(yolo_name=name, low_confidence=True)
    if lamp is not None and lamp != name:
        return lamp, dict(yolo_name=name)
    return name, {}


def predict_confidence(config):
    """YOLO 추론에 넘길 신뢰도 하한. 낮은 신뢰도 신호등 박스를 box_name이 고를 수 있게 lamp_low_confidence까지 내림"""
    low = config.get('lamp_low_confidence')
    if low is None or not config.get('lamp_check', True):
        return config['confidence']
    return min(config['confidence'], low)


def image_size(config, state):
    """YOLO 입력 크기. 신호등 앞 정지 중(traffic_hold)에는 traffic_hold_image_size, 그 외에는 image_size (11/19 혁준)

    정지선에서는 신호등이 화면에 크게 찍혀 640 입력이면 초록 신뢰도가 떨어짐 (track_testv2_1118_10 정지 뒤 초록 26프레임
    중앙 0.04, 같은 화면 320은 0.70). 먼 빨강·보행자는 640이 일찍 봄 → 정지한 동안에만 320. 끄기: traffic_hold_image_size None"""
    hold = config.get('traffic_hold_image_size')
    return hold if hold and state == 'traffic_hold' else config['image_size']
