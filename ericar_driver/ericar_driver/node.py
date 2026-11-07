"""ROS 2 노드: 센서 수신, YOLO, 최종 명령 발행"""
# 출처: hello-osy/2026-orda-mentoring src/orda_sim/orda_sim/answer.py (613c9b4)
import json
import math
from pathlib import Path
import sys
import threading

import yaml

from .common import clip
from .driver import Driver, sensor_status
from .lane_vision import LaneVision
from .lidar import scan_sectors
from .params import ANSWER_CONFIG, YOLO_CLASSES


def main(config_path=None):
    # Keep ROS imports here so pure perception/control tests run without ROS installed.
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from ackermann_msgs.msg import AckermannDriveStamped
    from sensor_msgs.msg import Image, LaserScan
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from cv_bridge import CvBridge

    if config_path is None:
        if len(sys.argv) != 2:
            raise SystemExit('사용법: scripts/answer.sh configs/config.yaml')
        config_path = sys.argv[1]
    path = Path(config_path).resolve()
    config = yaml.safe_load(path.read_text())
    config['answer'] = ANSWER_CONFIG
    c = config['answer']
    detector = None
    if c['detector']['enabled']:
        weights = Path(__file__).with_name('best.pt')
        if not weights.is_file():
            raise SystemExit(f'YOLO 모델이 없습니다: {weights}\nanswer.py 옆에 best.pt 파일을 넣으세요.')
        from ultralytics import YOLO
        detector = YOLO(str(weights))
        import torch
        threads = int(c['detector'].get('cpu_threads', config['environment'].get('cpu_threads', 2)))
        torch.set_num_threads(threads)
        detector.add_callback('on_predict_start', lambda _: torch.set_num_threads(threads))
        expected = set(YOLO_CLASSES)
        if set(detector.names.values()) != expected:
            raise SystemExit('모델의 클래스 이름은 answer.py의 YOLO_CLASSES 8종과 같아야 합니다. best.pt를 확인하세요.')

    class AnswerNode(Node):
        def __init__(self):
            super().__init__('orda_answer', parameter_overrides=[Parameter('use_sim_time', value=True)])
            self.bridge = CvBridge()
            self.driver = Driver(config)
            self.vision = LaneVision(c['lane'], config['robot']['camera'], config['robot']['front_axle_x'])
            self.image = None
            self.image_stamp = -1e9
            self.lane_stamp = None
            self.lane_bias = None
            self.lane_cache = None
            self.scan = None
            self.scan_stamp = -1e9
            self.velocity = 0.0
            self.angular_velocity = 0.0
            self.odom_stamp = -1e9
            self.detections = []
            self.detection_stamp = -1e9
            self.lock = threading.Lock()
            self.stop_event = threading.Event()
            self.last_state = ''
            topics = config['topics']
            self.publisher = self.create_publisher(AckermannDriveStamped, topics['drive'], 10)
            self.status = self.create_publisher(String, topics.get('status', '/answer/status'), 10)
            self.debug = self.create_publisher(Image, topics.get('debug_image', '/answer/debug_image'), qos_profile_sensor_data)
            self.debug_stamp = -1e9
            self.detection_debug_stamp = -1e9
            self.detection_output = self.create_publisher(String, topics.get('detections', '/answer/detections'), 10)
            self.create_subscription(Image, topics['image'], self.on_image, qos_profile_sensor_data)
            self.create_subscription(LaserScan, topics['scan'], self.on_scan, qos_profile_sensor_data)
            self.create_subscription(Odometry, topics['odometry'], self.on_odometry, qos_profile_sensor_data)
            self.timer = self.create_timer(1.0 / c['rate_hz'], self.control)
            if detector is not None:
                self.worker = threading.Thread(target=self.detect, daemon=True)
                self.worker.start()
            self.get_logger().info('정답 코드 시작: YOLO + camera lane + lidar; absolute pose is unused.')

        def now(self):
            return self.get_clock().now().nanoseconds / 1e9

        def stamp(self, message):
            return message.header.stamp.sec + message.header.stamp.nanosec / 1e9

        def on_image(self, message):
            try:
                frame = self.bridge.imgmsg_to_cv2(message, 'bgr8')
                with self.lock:
                    self.image = frame
                    self.image_stamp = self.stamp(message)
            except Exception as error:
                self.get_logger().error(f'image conversion failed: {error}')

        def on_scan(self, message):
            self.scan = scan_sectors(message.ranges, message.angle_min, message.angle_increment,
                                     message.range_min, message.range_max, c['lidar'])
            self.scan_stamp = self.stamp(message)

        def on_odometry(self, message):
            self.velocity = message.twist.twist.linear.x
            self.angular_velocity = message.twist.twist.angular.z
            self.odom_stamp = self.stamp(message)

        def detect(self):
            last_stamp = None
            while not self.stop_event.is_set():
                with self.lock:
                    frame = None if self.image is None else self.image.copy()
                    stamp = self.image_stamp
                if frame is not None and stamp != last_stamp:
                    last_stamp = stamp
                    try:
                        results = detector.predict(frame, conf=c['detector']['confidence'],
                                                   imgsz=c['detector']['image_size'],
                                                   device=c['detector']['device'], verbose=False)[0]
                        torch.set_num_threads(threads)
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
            message = AckermannDriveStamped()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = 'base_link'
            message.drive.speed = float(speed)
            message.drive.steering_angle = float(steering)
            self.publisher.publish(message)

        def control(self):
            now = self.now()
            with self.lock:
                frame = self.image
                image_stamp = self.image_stamp
                detections = self.detections
                detection_stamp = self.detection_stamp
            ages = dict(image=now-image_stamp, scan=now-self.scan_stamp,
                        odometry=now-self.odom_stamp, detector=now-detection_stamp)
            available = dict(image=frame is not None, scan=self.scan is not None,
                             lane=self.lane_cache is not None, detector=not self.stop_event.is_set())
            sensor_ready, required_camera = sensor_status(c, self.driver.state, ages, available)
            self.driver.debug['required_camera'] = required_camera
            if not sensor_ready or not math.isfinite(self.velocity) or not math.isfinite(self.angular_velocity):
                self.publish(0.0, 0.0)
                self.driver.debug['published_speed'] = 0.0
                self.driver.debug['speed'] = 0.0
                speed, steering = 0.0, 0.0
                self.driver.sensor_failure(now)
                state = 'waiting for fresh sensors / detector'
            else:
                if required_camera and (image_stamp != self.lane_stamp or self.driver.fork_bias != self.lane_bias):
                    self.lane_cache = self.vision.analyze(frame, self.driver.fork_bias)
                    self.lane_cache['stamp'] = image_stamp
                    self.lane_stamp = image_stamp
                    self.lane_bias = self.driver.fork_bias
                lane = self.lane_cache
                was_avoiding = self.driver.avoiding
                speed, steering, state = self.driver.update(lane, self.scan, detections,
                                                             detection_stamp, self.velocity, now, self.angular_velocity)
                if was_avoiding and not self.driver.avoiding:
                    self.vision.previous_fits = {}
                    self.vision.previous_boundaries = {}
                    self.vision.previous_center = None
                    self.lane_stamp = None
                if self.driver.state != 'parking_sequence':
                    age = max(now-image_stamp, .001)
                    limit = c.get('max_image_travel', .04) / age
                    speed = clip(speed, -limit, limit)
                self.publish(speed, steering)
                self.driver.debug['published_speed'] = speed
                self.driver.debug['speed'] = speed
                self.driver.debug['required_camera'] = required_camera
                if now - self.debug_stamp >= 1.0 / c['debug_rate_hz']:
                    message = self.bridge.cv2_to_imgmsg(lane['lane_mask'], 'mono8')
                    stamp = float(lane['stamp'])
                    message.header.stamp.sec = int(stamp)
                    message.header.stamp.nanosec = int((stamp-int(stamp))*1e9)
                    self.debug.publish(message)
                    self.debug_stamp = now
            # Telemetry follows the actual published command, including safety stops.
            if now - self.detection_debug_stamp >= 1.0 / c['debug_rate_hz']:
                lane = self.lane_cache or {}
                keys = ('valid', 'error', 'heading', 'center_px', 'forward', 'windows',
                        'fits', 'fit_spans', 'center_fit', 'fit_near', 'fit_far')
                control = dict(self.driver.debug, steering=steering, speed=speed,
                               state=state, sensor_ready=sensor_ready,
                               controller_state=self.driver.state)
                packet = dict(stamp=detection_stamp, age=now-detection_stamp, objects=detections,
                              control=control, image_age=now-image_stamp, scan_age=now-self.scan_stamp,
                              image_stamp=image_stamp, lane_stamp=lane.get('stamp'),
                              lane={key: lane[key] for key in keys if key in lane},
                              display=dict(lane=c['lane'], pid=c['pid'], detector=dict(timeout=c['detector']['timeout']),
                                           sensor_timeout=c['sensor_timeout']))
                self.detection_output.publish(String(data=json.dumps(packet)))
                self.detection_debug_stamp = now
            if state != self.last_state:
                self.get_logger().info(state)
                self.status.publish(String(data=state))
                self.last_state = state

    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = AnswerNode()
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
