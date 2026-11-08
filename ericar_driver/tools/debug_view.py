#!/usr/bin/env python3
"""주행 노드 디버그 화면을 브라우저로 중계 (MJPEG). 보기 전용, 차량 제어 없음

왼쪽: 카메라 / 오른쪽: BEV 차선 마스크 + 슬라이딩 윈도우·차선 곡선·목표 중심 / 아래: 상태·제어값
입력: /camera/color/image_raw/compressed, /answer/debug_image(mono8), /answer/detections(JSON)

실행 (차, 터미널 환경 1, 주행 노드 실행 중):
  python3 debug_view.py [포트]
보기: 맥 http://<차 IP>:8080 / 차 http://localhost:8080
"""
import json
import socket
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 8080
MAX_FPS = 8
PANEL_H = 360
CAMERA_TOPIC = '/camera/color/image_raw/compressed'
MASK_TOPIC = '/answer/debug_image'
TELEMETRY_TOPIC = '/answer/detections'


def draw_bev(mask, lane):
    """BEV 마스크 위에 윈도우(회색)·차선 곡선(좌 초록/우 파랑)·목표 중심(빨강)"""
    bev = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR) // 2
    h, w = mask.shape[:2]
    for win in lane.get('windows', []):
        cv2.rectangle(bev, (int(win['x0']), int(win['y0'])), (int(win['x1']), int(win['y1'])), (90, 90, 90), 1)
    colors = {'left': (0, 200, 0), 'right': (255, 120, 0)}
    rows = np.arange(h, dtype=float)
    for side, fit in lane.get('fits', {}).items():
        lo, hi = lane.get('fit_spans', {}).get(side, (0, h - 1))
        keep = (rows >= lo) & (rows <= hi)
        xs = np.polyval(fit, rows[keep])
        points = np.stack([xs, rows[keep]], axis=1).astype(np.int32)
        cv2.polylines(bev, [points], False, colors.get(side, (0, 255, 255)), 2)
    center = lane.get('center_px')
    if center is not None:
        cv2.line(bev, (int(center), 0), (int(center), h - 1), (0, 0, 255), 1)
    cv2.line(bev, (w // 2, h - 20), (w // 2, h - 1), (255, 255, 255), 2)  # 차량 중심
    return bev


def text_panel(width, telemetry):
    panel = np.zeros((96, width, 3), np.uint8)
    if not telemetry:
        cv2.putText(panel, 'no telemetry', (8, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1)
        return panel
    control, lane = telemetry.get('control', {}), telemetry.get('lane', {})
    lines = [
        f"state {control.get('state', '?')}   ready {control.get('sensor_ready')}",
        f"speed {control.get('speed', 0):+.3f} m/s   steer {control.get('steering', 0):+.3f} rad",
        f"lane valid {lane.get('valid')}   error {lane.get('error', 0):+.3f} m   heading {lane.get('heading', 0):+.3f} rad",
        f"front {control.get('front', 0):.2f} m   stop line {control.get('line_distance')}   "
        f"image age {telemetry.get('image_age', 0):.2f}s",
    ]
    for i, line in enumerate(lines):
        color = (0, 255, 0) if i != 0 or control.get('sensor_ready') else (0, 0, 255)
        cv2.putText(panel, line, (8, 20 + i * 22), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
    return panel


def compose(camera, mask, telemetry):
    """카메라·BEV·상태 글자를 한 장으로 합성. 입력이 없으면 빈 칸"""
    def fit(image):
        if image is None:
            return np.zeros((PANEL_H, PANEL_H, 3), np.uint8)
        scale = PANEL_H / image.shape[0]
        return cv2.resize(image, (int(image.shape[1] * scale), PANEL_H), interpolation=cv2.INTER_AREA)
    left = fit(camera)
    right = fit(draw_bev(mask, (telemetry or {}).get('lane', {})) if mask is not None else None)
    top = np.hstack([left, right])
    return np.vstack([top, text_panel(top.shape[1], telemetry)])


class State:
    def __init__(self):
        self.lock = threading.Condition()
        self.camera = self.mask = self.telemetry = None
        self.seq = 0

    def update(self, **kw):
        with self.lock:
            for key, value in kw.items():
                setattr(self, key, value)
            self.seq += 1
            self.lock.notify_all()


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
                    camera, mask, telemetry = state.camera, state.mask, state.telemetry
                if time.time() - last < 1.0 / MAX_FPS:
                    continue
                last = time.time()
                ok, jpeg = cv2.imencode('.jpg', compose(camera, mask, telemetry), [cv2.IMWRITE_JPEG_QUALITY, 70])
                if ok:
                    data = jpeg.tobytes()
                    self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '
                                     + str(len(data)).encode() + b'\r\n\r\n' + data + b'\r\n')
        except (BrokenPipeError, ConnectionResetError):
            pass


def main():
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CompressedImage, Image
    from std_msgs.msg import String

    def on_camera(m):
        state.update(camera=cv2.imdecode(np.frombuffer(m.data, np.uint8), cv2.IMREAD_COLOR))

    def on_mask(m):
        state.update(mask=np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)[:, :m.width].copy())

    def on_telemetry(m):
        state.update(telemetry=json.loads(m.data))

    rclpy.init()
    node = rclpy.create_node('debug_view')
    node.create_subscription(CompressedImage, CAMERA_TOPIC, on_camera, qos_profile_sensor_data)
    node.create_subscription(Image, MASK_TOPIC, on_mask, qos_profile_sensor_data)
    node.create_subscription(String, TELEMETRY_TOPIC, on_telemetry, 10)
    server = ThreadingHTTPServer(('0.0.0.0', PORT), Handler)
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print(f'http://<차 IP>:{PORT}', flush=True)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
