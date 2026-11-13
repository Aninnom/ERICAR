#!/usr/bin/env python3
"""주행 노드 디버그 화면을 브라우저로 중계 (MJPEG). 보기 전용, 차량 제어 없음

왼쪽: 흰 선 원본 오버레이 / 가운데: 흰색 마스크 / 오른쪽: LiDAR 장애물·입구 후보 / 아래: 상태·제어값
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
from ericar_driver.parking_white import straight_overlay
from ericar_driver.parking_gap import gap_candidates,classify_gap

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 8080
MAX_FPS = 8
PANEL_H = 360
CAMERA_TOPIC = '/camera/color/image_raw/compressed'
MASK_TOPIC = '/answer/debug_image'
TELEMETRY_TOPIC = '/answer/detections'
VIEW_CONFIG=None


def draw_bev(mask, lane):
    """BEV 마스크 위에 윈도우(회색)·차선 곡선(좌 초록/우 파랑)·목표 중심(빨강)"""
    bev = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR) // 2
    bev[mask == 100] = (255, 0, 255)  # 흰색 마스크 (정지선·노면 표시), 자홍
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
    panel = np.zeros((118, width, 3), np.uint8)
    if not telemetry:
        cv2.putText(panel, 'no telemetry', (8, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1)
        return panel
    control, lane = telemetry.get('control', {}), telemetry.get('lane', {})
    parking=control.get('parking',{});white=parking.get('white_line',{})
    lines=[f"state {control.get('state','?')} | ready {control.get('sensor_ready')}",
           f"speed {control.get('published_speed',0):+.3f} m/s | steer {control.get('steering',0):+.3f} rad",
           f"WHITE present {white.get('present')} | seen {parking.get('line_seen')} | loss frames {white.get('loss_frames',0)}",
           f"search {parking.get('search_distance_m',0):.2f}m | turn {parking.get('turn_distance_m',0):.2f}m",
           'parking_2: FREE once -> fixed right turn -> observed WHITE line disappears -> STOP']
    for i, line in enumerate(lines):
        color = (0, 255, 0) if i != 0 or control.get('sensor_ready') else (0, 0, 255)
        cv2.putText(panel, line, (8, 20 + i * 22), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
    return panel


def draw_scan(scan,telemetry=None):
    """LiDAR 좌표 점군. 상단 전방·우측 차량 오른쪽, 미측정은 점으로 표시 안 함"""
    image=np.zeros((PANEL_H,PANEL_H,3),np.uint8)
    origin=np.array([PANEL_H//2,PANEL_H-70]);scale=90.
    for radius in (1.,2.,3.):
        cv2.circle(image,tuple(origin),int(radius*scale),(50,50,50),1)
    cv2.arrowedLine(image,tuple(origin),(origin[0],origin[1]-30),(255,255,255),2)
    cv2.putText(image,'LIDAR: forward up | 1m rings',(8,18),cv2.FONT_HERSHEY_SIMPLEX,.40,(255,255,255),1)
    if scan is None:
        cv2.putText(image,'WAITING FOR /scan',(10,45),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,0,255),1)
        return image
    ranges,angles,lo,hi,received=scan
    valid=np.isfinite(ranges)&(ranges>=lo)&(ranges<=hi)&(ranges>0)
    points=np.column_stack((origin[0]-ranges[valid]*np.sin(angles[valid])*scale,
                            origin[1]-ranges[valid]*np.cos(angles[valid])*scale))
    for x,y in np.rint(points).astype(int):
        if 0<=x<PANEL_H and 0<=y<PANEL_H:cv2.circle(image,(x,y),1,(0,255,100),-1)
    if VIEW_CONFIG is not None:
        robot,config=VIEW_CONFIG
        clean=np.where(valid,ranges,0.)
        data=dict(points=np.column_stack((clean[valid]*np.cos(angles[valid]),clean[valid]*np.sin(angles[valid]))),
                  parking_rays=dict(angles=angles,distances=clean,measured=valid&(ranges<hi),valid=valid,
                                    increment=float(angles[1]-angles[0]) if len(angles)>1 else 0.))
        bays,diagnostics=gap_candidates(data,robot,config)
        def pixels(points):
            q=np.asarray(points,float).copy();q[:,0]-=robot['lidar']['x'];q[:,1]-=robot['lidar'].get('y',0.)
            return np.rint(np.column_stack((origin[0]-q[:,1]*scale,origin[1]-q[:,0]*scale))).astype(np.int32)
        colors=[(255,140,0),(255,0,255),(0,180,255)]
        for rejected in diagnostics.get('cluster_rejections',[]):
            if rejected['hits']<3:continue
            cv2.polylines(image,[pixels(rejected['points'])],True,(100,100,100),1)
            center=pixels(np.mean(rejected['points'],axis=0,keepdims=True))[0]
            cv2.putText(image,f"reject {rejected['extent_m']:.2f}m",tuple(center),cv2.FONT_HERSHEY_SIMPLEX,.3,(150,150,150),1)
        for box in diagnostics['clusters']:
            color=colors[box['id']%len(colors)]
            cv2.polylines(image,[pixels(box['points'])],True,color,2)
            center=pixels(np.mean(box['points'],axis=0,keepdims=True))[0]
            cv2.putText(image,f"obj {box['id']}",tuple(center),cv2.FONT_HERSHEY_SIMPLEX,.4,color,1)
        assessed=[]
        for bay in bays:
            result,evidence=classify_gap(bay,data,robot,config)
            assessed.append(dict(evidence,state=result))
            color={'FREE':(0,255,0),'UNKNOWN':(0,180,255),'OCCUPIED':(0,0,255)}[result]
            cv2.polylines(image,[pixels(bay['polygon'])],True,color,2)
            center=pixels([bay['entrance']])[0]
            cv2.putText(image,f"{result} {bay['width']:.2f}m",tuple(center),cv2.FONT_HERSHEY_SIMPLEX,.4,color,1)
        label=diagnostics['gaps'][0]['reason'] if diagnostics['gaps'] else diagnostics['reason']
        cv2.putText(image,'LIVE: '+label,(8,36),cv2.FONT_HERSHEY_SIMPLEX,.35,(0,255,255),1)
        if assessed:
            e=assessed[0]
            detail=f"{e['state']}: coverage {e.get('covered_samples',0)}/{e.get('total_samples',12)} need {e.get('required_samples',8)}"
            cv2.putText(image,detail,(8,52),cv2.FONT_HERSHEY_SIMPLEX,.35,(0,180,255),1)
            detail=f"occluded {e.get('occluded_or_short_samples',0)} no-return {e.get('no_return_or_invalid_samples',0)}"
            cv2.putText(image,detail,(8,68),cv2.FONT_HERSHEY_SIMPLEX,.35,(0,180,255),1)
    parking=(telemetry or {}).get('control',{}).get('parking',{})
    if parking.get('state')=='parking_turn':
        cv2.putText(image,'TURN ACTIVE: FREE LATCHED',(8,86),cv2.FONT_HERSHEY_SIMPLEX,.4,(255,255,0),1)
    age=time.monotonic()-received
    cv2.putText(image,f'age {age:.2f}s | hits {int(valid.sum())}',(8,PANEL_H-12),
                cv2.FONT_HERSHEY_SIMPLEX,.45,(0,0,255) if age>1. else (0,255,100),1)
    return image


def compose(camera, mask, telemetry, scan=None):
    """카메라·BEV·상태 글자를 한 장으로 합성. 입력이 없으면 빈 칸"""
    def fit(image):
        if image is None:
            return np.zeros((PANEL_H, PANEL_H, 3), np.uint8)
        scale = PANEL_H / image.shape[0]
        return cv2.resize(image, (int(image.shape[1] * scale), PANEL_H), interpolation=cv2.INTER_AREA)
    from ericar_driver.params import ANSWER_CONFIG
    overlay,white,segments=straight_overlay(camera,VIEW_CONFIG[1] if VIEW_CONFIG else ANSWER_CONFIG['parking'])
    left = fit(overlay)
    right = fit(cv2.cvtColor(white,cv2.COLOR_GRAY2BGR) if white is not None else None)
    top = np.hstack([left, right, draw_scan(scan,telemetry)])
    return np.vstack([top, text_panel(top.shape[1], telemetry)])


class State:
    def __init__(self):
        self.lock = threading.Condition()
        self.camera = self.mask = self.telemetry = self.scan = None
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
                    '<body style="margin:0;background:#111;color:#ddd">'
                    '<div>카메라: 초록=흰색 후보 화소, 노랑=검출 선분 | 가운데: 흰색 마스크 | '
                    'LiDAR 현재 관측: 색상별 물체, 주황=확인 부족, 초록=FREE, 빨강=점유. 하늘색 글자=우회전 진행. 하단 흰 선 상태는 실제 정지 조건</div>'
                    '<img src="/stream.mjpg" style="max-width:100%">')
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
                if time.time() - last < 1.0 / MAX_FPS:
                    continue
                last = time.time()
                ok, jpeg = cv2.imencode('.jpg', compose(camera, mask, telemetry, scan), [cv2.IMWRITE_JPEG_QUALITY, 70])
                if ok:
                    data = jpeg.tobytes()
                    self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '
                                     + str(len(data)).encode() + b'\r\n\r\n' + data + b'\r\n')
        except (BrokenPipeError, ConnectionResetError):
            pass


def main():
    import rclpy
    import yaml
    from pathlib import Path
    from ament_index_python.packages import get_package_share_directory
    from ericar_driver.params import ANSWER_CONFIG
    global VIEW_CONFIG
    config_path=Path(get_package_share_directory('ericar_driver'))/'config/robot.yaml'
    robot=yaml.safe_load(config_path.read_text())['robot']
    VIEW_CONFIG=(robot,ANSWER_CONFIG['parking'])
    cv2.setNumThreads(1)
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CompressedImage, Image, LaserScan
    from std_msgs.msg import String

    def on_camera(m):
        state.update(camera=cv2.imdecode(np.frombuffer(m.data, np.uint8), cv2.IMREAD_COLOR))

    def on_scan(m):
        ranges=np.asarray(m.ranges,float)
        angles=m.angle_min+np.arange(len(ranges))*m.angle_increment
        state.update(scan=(ranges,angles,m.range_min,m.range_max,time.monotonic()))

    def on_mask(m):
        state.update(mask=np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)[:, :m.width].copy())

    def on_telemetry(m):
        state.update(telemetry=json.loads(m.data))

    rclpy.init()
    node = rclpy.create_node('debug_view')
    node.create_subscription(CompressedImage, CAMERA_TOPIC, on_camera, qos_profile_sensor_data)
    node.create_subscription(Image, MASK_TOPIC, on_mask, qos_profile_sensor_data)
    node.create_subscription(LaserScan,'/scan',on_scan,qos_profile_sensor_data)
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
