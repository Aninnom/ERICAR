"""ROS 2 노드: 센서 수신, YOLO, 최종 명령 발행. 계산은 loop.py"""
# 출처: hello-osy/2026-orda-mentoring src/orda_sim/orda_sim/answer.py (613c9b4)
from pathlib import Path
import sys
import json
import cv2
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from ackermann_msgs.msg import AckermannDrive
from sensor_msgs.msg import Image, Imu, LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory

from .common import clip
from .traffic_lamp import NAMES as LAMP_NAMES, lamp_name
from .lidar import scan_sectors
from .loop import ControlLoop, Sensors, load_config
from .params import YOLO_CLASSES
from .section import START_SECTION


def load_detector(config, weights):
    """YOLO 모델 로드. 일반 8종·주차 단독 parking 클래스 검증"""
    c = config['answer']
    weights = Path(weights).expanduser()
    if not weights.is_file():
        raise SystemExit(f'YOLO 모델이 없습니다: {weights}\n두 번째 인자로 경로를 주거나 ~/models/best.pt에 두세요.')
    from ultralytics import YOLO
    detector = YOLO(str(weights))
    import torch
    threads = int(c['detector'].get('cpu_threads', config['environment'].get('cpu_threads', 2)))
    torch.set_num_threads(threads)
    detector.add_callback('on_predict_start', lambda _: torch.set_num_threads(threads))
    names=set(detector.names.values())
    if c['parking'].get('test_wait_for_sign',False):
        if 'parking' not in names:
            raise SystemExit('주차 단독 시험 모델에 parking 클래스 필요')
    elif names != set(YOLO_CLASSES):
        raise SystemExit(f'모델 클래스 {sorted(detector.names.values())}가 params.py의 YOLO_CLASSES 8종과 다름')
    return detector, threads


def stamp_of(message):
    return message.header.stamp.sec + message.header.stamp.nanosec / 1e9


class DriverNode(Node):
    def __init__(self, config, detector=None, threads=2):
        # 실차: 시스템 시계 사용 (시뮬 원본은 use_sim_time 고정)
        super().__init__('ericar_driver')
        self.config, self.c = config, config['answer']
        self.detector, self.threads = detector, threads
        cv2.setNumThreads(1)
        self.bridge = CvBridge()
        self.cancelled=False
        self.loop = ControlLoop(config)
        self.image = None
        self.image_stamp = -1e9
        self.scan = None
        self.scan_stamp = -1e9
        self.velocity = 0.0
        self.angular_velocity = 0.0
        self.odom_stamp = -1e9
        self.yaw_rate = 0.0
        self.imu_stamp = -1e9
        self.detector_failures = 0
        self.detections = []
        self.detection_stamp = -1e9
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        topics = config['topics']
        # 실차 드라이버는 /limo/ack_cmd의 AckermannDrive만 수신 (시뮬은 /drive, Stamped)
        self.publisher = self.create_publisher(AckermannDrive, topics['drive'], 10)
        self.status = self.create_publisher(String, topics.get('status', '/answer/status'), 10)
        self.debug = self.create_publisher(Image, topics.get('debug_image', '/answer/debug_image'), qos_profile_sensor_data)
        self.detection_output = self.create_publisher(String, topics.get('detections', '/answer/detections'), 10)
        self.create_subscription(Image, topics['image'], self.on_image, qos_profile_sensor_data)
        self.create_subscription(LaserScan, topics['scan'], self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, topics['odometry'], self.on_odometry, qos_profile_sensor_data)
        # 실측 회전(자이로). /odom 회전은 조향 명령으로 계산한 값이라 구간 판단·차선 변경·라바콘 방향에는 IMU 사용
        self.create_subscription(Imu, topics.get('imu', '/imu'), self.on_imu, qos_profile_sensor_data)
        self.create_subscription(Bool, '/parking/cancel', self.on_parking_cancel, 10)
        self.timer = self.create_timer(1.0 / self.c['rate_hz'], self.control)
        if detector is not None:
            self.worker = threading.Thread(target=self.detect, daemon=True)
            self.worker.start()
        self.get_logger().info('정답 코드 시작: YOLO + camera lane + lidar; absolute pose is unused.')

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_image(self, message):
        try:
            frame = self.bridge.imgmsg_to_cv2(message, 'bgr8')
            with self.lock:
                self.image = frame
                self.image_stamp = stamp_of(message)
        except Exception as error:
            self.get_logger().error(f'image conversion failed: {error}')

    def on_scan(self, message):
        self.scan = scan_sectors(message.ranges, message.angle_min, message.angle_increment,
                                 message.range_min, message.range_max, self.c['lidar'])
        self.scan_stamp = stamp_of(message)

    def on_odometry(self, message):
        self.velocity = message.twist.twist.linear.x
        self.angular_velocity = message.twist.twist.angular.z
        self.odom_stamp = stamp_of(message)

    def on_parking_cancel(self,message):
        if message.data and (self.loop.driver.state.startswith('parking_') or self.loop.driver.state=='parked' or self.c['parking'].get('test_wait_for_sign',False)):
            self.cancelled=True
            self.loop.driver.parking.cancel()
            self.loop.driver.state='parking_cancelled'
            self.publish(0.,0.)

    def on_imu(self, message):
        self.yaw_rate = message.angular_velocity.z
        self.imu_stamp = self.now()   # 헤더 시각이 비어 있는 드라이버가 있어 수신 시각 사용

    def detect(self):
        import torch
        c = self.c
        last_stamp = None
        while not self.stop_event.is_set():
            if self.loop.driver.state.startswith('parking_') or self.loop.driver.state=='parked':
                self.stop_event.wait(.2)
                continue
            with self.lock:
                frame = None if self.image is None else self.image.copy()
                stamp = self.image_stamp
            if frame is not None and stamp != last_stamp:
                last_stamp = stamp
                try:
                    results = self.detector.predict(frame, conf=c['detector']['confidence'],
                                                    imgsz=c['detector']['image_size'],
                                                    device=c['detector']['device'], verbose=False)[0]
                    torch.set_num_threads(self.threads)
                    objects = []
                    h, w = frame.shape[:2]
                    for box in results.boxes:
                        x0, y0, x1, y1 = box.xyxy[0].cpu().tolist()
                        margin = c['detector'].get('edge_margin', .01)
                        truncated = x0 <= margin*w or x1 >= (1-margin)*w or y0 <= margin*h or y1 >= (1-margin)*h
                        name = results.names[int(box.cls[0])]
                        extra = {}
                        if (name in LAMP_NAMES and c['detector'].get('lamp_check', True) and
                                (x1-x0)*(y1-y0)/(w*h) >= c['detector'].get('lamp_min_area', .005)):
                            # 같은 프레임에서 켜진 램프 위치로 색 재확인. 판정되면 YOLO 이름 대신 사용 (traffic_lamp.py)
                            lamp = lamp_name(frame, (x0, y0, x1, y1), c['detector'])
                            if lamp is not None and lamp != name:
                                extra, name = dict(yolo_name=name), lamp
                        objects.append(dict(name=name, **extra,
                                            bbox=[x0/w, y0/h, x1/w, y1/h], truncated=truncated,
                                            confidence=float(box.conf[0]),
                                            area=(x1 - x0) * (y1 - y0) / (w * h),
                                            center=((x0 + x1) / (2 * w), (y0 + y1) / (2 * h))))
                    with self.lock:
                        self.detections = objects
                        self.detection_stamp = stamp
                    self.detector_failures = 0
                except Exception as error:
                    # 예전: 예외 1회로 추론 종료 → 센서 대기로 영구 정지. 지금: 기록만 하고 다음 영상에서 재시도
                    # 검출이 끊긴 동안은 loop.py가 표지판 미션만 끄고 주행 계속
                    self.detector_failures += 1
                    if self.detector_failures <= c['detector']['failure_limit']:
                        self.get_logger().error(f'YOLO failed ({self.detector_failures}): {error}')
            self.stop_event.wait(1.0 / c['detector']['rate_hz'])

    def publish(self, speed, steering):
        # 최종 한계 적용: 개별 제어기의 실차 속도·조향 한계 초과 방지
        # 조향 영점 보정은 판단 결과(조향 0 = 직진)에 더한 뒤 한계 적용
        robot = self.config['robot']
        message = AckermannDrive()
        message.speed = float(clip(speed, -robot['max_speed'], robot['max_speed']))
        steering += robot.get('steering_offset', 0.0)
        message.steering_angle = float(clip(steering, -robot['max_steering'], robot['max_steering']))
        self.publisher.publish(message)

    def control(self):
        with self.lock:
            sensors = Sensors(now=self.now(), image=self.image, image_stamp=self.image_stamp,
                              detections=self.detections, detection_stamp=self.detection_stamp,
                              detector_ok=self.detector_failures < self.c['detector']['failure_limit'],
                              scan=self.scan, scan_stamp=self.scan_stamp, velocity=self.velocity,
                              angular_velocity=self.angular_velocity, odom_stamp=self.odom_stamp,
                              yaw_rate=self.yaw_rate, imu_stamp=self.imu_stamp)
        out = self.loop.step(sensors)
        elapsed=self.now()-sensors.now
        fresh=0<=elapsed<self.c['parking']['command_deadline']
        speed=out.speed if fresh and not self.cancelled else 0.
        steering=out.steering if fresh and not self.cancelled else 0.
        self.publish(speed,steering)
        if out.telemetry is not None:
            packet=json.loads(out.telemetry)
            packet['runtime']=dict(mode='parking_auto1',compute_ms=elapsed*1000,
                                   published_speed=speed,command_fresh=fresh)
            packet['control']['published_speed']=speed
            out.telemetry=json.dumps(packet)
        if out.lane_mask is not None:
            message = self.bridge.cv2_to_imgmsg(out.lane_mask, 'mono8')
            message.header.stamp.sec = int(out.lane_mask_stamp)
            message.header.stamp.nanosec = int((out.lane_mask_stamp-int(out.lane_mask_stamp))*1e9)
            self.debug.publish(message)
        if out.telemetry is not None:
            self.detection_output.publish(String(data=out.telemetry))
        if out.status is not None:
            self.get_logger().info(out.status)
            self.status.publish(String(data=out.status))


def parse_args(argv):
    """인자: [설정 yaml] [모델 .pt] [--yolo true|false] [--start-stage 미션] [--parking-test true|false]. --ros-args 이후는 ROS 몫"""
    args = argv[1:argv.index('--ros-args')] if '--ros-args' in argv else argv[1:]
    options = {'--yolo': 'true', '--start-stage': '', '--parking-test': 'false'}
    for flag in options:
        if flag in args:
            i = args.index(flag)
            if i + 1 >= len(args):
                raise SystemExit(f'{flag} 뒤에 값이 필요합니다')
            options[flag] = args[i + 1]
            args = args[:i] + args[i + 2:]
    yolo = options['--yolo'].lower() not in ('false', '0', 'no', 'off')
    parking_test=options['--parking-test'].lower() not in ('false','0','no','off')
    return args, yolo, options['--start-stage'], parking_test


def main(config_path=None):
    # 생략 시 패키지 config/robot.yaml, ~/models/best.pt, YOLO 사용, 첫 구간(S0)부터 시작
    args, yolo, start_stage, parking_test = parse_args(sys.argv)
    if config_path is None:
        config_path = args[0] if args else str(Path(get_package_share_directory('ericar_driver')) / 'config' / 'robot.yaml')
    config = load_config(config_path)
    c = config['answer']
    if start_stage:
        if start_stage not in START_SECTION:
            raise SystemExit(f'start_stage "{start_stage}"는 {list(START_SECTION)} 중 하나여야 합니다')
        c['missions']['start_stage'] = start_stage
    c['parking']['test_wait_for_sign']=parking_test
    c['parking']['test_drive_until_sign']=parking_test
    if parking_test:
        if not yolo:raise SystemExit('parking_test는 parking 표지판을 포함한 YOLO 모델 필요')
        c['missions']['start_stage']='parking'
    if not yolo:
        # 차선 시험용: 표지판 미션 없이 주행. 센서 상태 확인에서도 YOLO 제외
        c['detector']['enabled'] = False
    detector, threads = None, 2
    if c['detector']['enabled']:
        detector, threads = load_detector(config, args[1] if len(args) > 1 else '~/models/best.pt')

    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = DriverNode(config, detector, threads)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_event.set()
        node.loop.driver.parking.close()
        if rclpy.ok():
            node.publish(0.0, 0.0)
        if hasattr(node, 'worker'):
            node.worker.join(timeout=c['detector'].get('shutdown_timeout', 5.0))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
