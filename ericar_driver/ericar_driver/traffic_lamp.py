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
