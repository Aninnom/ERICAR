"""ROS 2 노드: 센서 수신, YOLO, 최종 명령 발행. 계산은 loop.py"""
# 출처: hello-osy/2026-orda-mentoring src/orda_sim/orda_sim/answer.py (613c9b4)
from pathlib import Path
import sys
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from ackermann_msgs.msg import AckermannDrive
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory

from .common import clip
from .lidar import scan_sectors
from .loop import ControlLoop, Sensors, load_config
from .params import YOLO_CLASSES


def load_detector(config, weights):
    """YOLO 모델 로드. 클래스 이름이 YOLO_CLASSES와 다르면 종료"""
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
    if set(detector.names.values()) != set(YOLO_CLASSES):
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
        self.bridge = CvBridge()
        self.loop = ControlLoop(config)
        self.image = None
        self.image_stamp = -1e9
        self.scan = None
        self.scan_stamp = -1e9
        self.velocity = 0.0
        self.angular_velocity = 0.0
        self.odom_stamp = -1e9
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

    def detect(self):
        import torch
        c = self.c
        last_stamp = None
        while not self.stop_event.is_set():
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
                        objects.append(dict(name=results.names[int(box.cls[0])],
                                            bbox=[x0/w, y0/h, x1/w, y1/h], truncated=truncated,
                                            confidence=float(box.conf[0]),
                                            area=(x1 - x0) * (y1 - y0) / (w * h),
                                            center=((x0 + x1) / (2 * w), (y0 + y1) / (2 * h))))
                    with self.lock:
                        self.detections = objects
                        self.detection_stamp = stamp
                except Exception as error:
                    self.get_logger().error(f'YOLO failed: {error}')
                    self.stop_event.set()
            self.stop_event.wait(1.0 / c['detector']['rate_hz'])

    def publish(self, speed, steering):
        # 최종 한계 적용: 주차 시간표 등 개별 제어기가 실차 한계를 넘는 경우 방지
        robot = self.config['robot']
        message = AckermannDrive()
        message.speed = float(clip(speed, -robot['max_speed'], robot['max_speed']))
        message.steering_angle = float(clip(steering, -robot['max_steering'], robot['max_steering']))
        self.publisher.publish(message)

    def control(self):
        with self.lock:
            sensors = Sensors(now=self.now(), image=self.image, image_stamp=self.image_stamp,
                              detections=self.detections, detection_stamp=self.detection_stamp,
                              detector_ok=not self.stop_event.is_set(),
                              scan=self.scan, scan_stamp=self.scan_stamp, velocity=self.velocity,
                              angular_velocity=self.angular_velocity, odom_stamp=self.odom_stamp)
        out = self.loop.step(sensors)
        self.publish(out.speed, out.steering)
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
    """인자: [설정 yaml] [모델 .pt] [--yolo true|false]. --ros-args 이후는 ROS 몫"""
    args = argv[1:argv.index('--ros-args')] if '--ros-args' in argv else argv[1:]
    yolo = True
    if '--yolo' in args:
        i = args.index('--yolo')
        yolo = args[i + 1].lower() not in ('false', '0', 'no', 'off')
        args = args[:i] + args[i + 2:]
    return args, yolo


def main(config_path=None):
    # 생략 시 패키지 config/robot.yaml, ~/models/best.pt, YOLO 사용
    args, yolo = parse_args(sys.argv)
    if config_path is None:
        config_path = args[0] if args else str(Path(get_package_share_directory('ericar_driver')) / 'config' / 'robot.yaml')
    config = load_config(config_path)
    c = config['answer']
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
        if rclpy.ok():
            node.publish(0.0, 0.0)
        if hasattr(node, 'worker'):
            node.worker.join(timeout=c['detector'].get('shutdown_timeout', 5.0))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
