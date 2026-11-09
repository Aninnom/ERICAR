"""제어 주기 1회 계산: 센서 상태 확인 → 차선 → 판단 → 발행할 값. ROS 의존 없음"""
# 출처: hello-osy/2026-orda-mentoring src/orda_sim/orda_sim/answer.py (613c9b4)의 AnswerNode.control
from dataclasses import dataclass
import json
import math
from pathlib import Path

import yaml

from .common import clip
from .driver import Driver
from .lane_vision import LaneVision
from .params import ANSWER_CONFIG

REQUIRED_KEYS = {
    'topics': ['image', 'scan', 'odometry', 'drive'],
    'robot': ['wheelbase', 'width', 'length', 'max_speed', 'max_steering', 'front_axle_x', 'camera', 'lidar'],
    'robot.camera': ['hfov_deg', 'x', 'z', 'pitch'],
    'robot.lidar': ['x'],
}


def validate(config):
    """필수 설정 키 검사. 누락 시 실행 중 KeyError 대신 시작 시 종료"""
    missing = []
    for section, keys in REQUIRED_KEYS.items():
        node = config
        for part in section.split('.'):
            node = node.get(part, {}) if isinstance(node, dict) else {}
        missing += [f'{section}.{key}' for key in keys if key not in node]
    if missing:
        raise SystemExit('robot.yaml 필수 키 누락: ' + ', '.join(missing))


def load_config(path):
    config = yaml.safe_load(Path(path).expanduser().resolve().read_text())
    validate(config)
    config['answer'] = ANSWER_CONFIG
    # 실차 라이다(0 = 무반사) 대응. robot.yaml의 robot.lidar.zero_is_no_return, 없으면 기존 동작.
    ANSWER_CONFIG['lidar']['zero_is_no_return'] = bool(config['robot']['lidar'].get('zero_is_no_return', False))
    return config


def sensor_status(config, state, ages, available):
    """Require the inputs used by this state, keeping lidar/odometry mandatory."""
    timed_parking = state in ('parking_sequence', 'parked')
    required_camera = not timed_parking or config['parking'].get('require_camera_during_sequence', False)
    ready = (available['scan'] and 0 <= ages['scan'] < config['sensor_timeout'] and
             0 <= ages['odometry'] < config['odometry_timeout'])
    if required_camera:
        ready = ready and available['image'] and 0 <= ages['image'] < config['sensor_timeout']
        if config['detector']['enabled']:
            ready = ready and available['detector'] and 0 <= ages['detector'] < config['detector']['timeout']
    else:
        # The timed sequence must have been entered through camera alignment.
        ready = ready and available['lane']
    return bool(ready), required_camera


@dataclass
class Sensors:
    """제어 주기 시점의 최신 센서 값 (시간은 초)"""
    now: float
    image: object
    image_stamp: float
    detections: list
    detection_stamp: float
    detector_ok: bool
    scan: dict
    scan_stamp: float
    velocity: float
    angular_velocity: float
    odom_stamp: float


@dataclass
class Output:
    """발행할 값. None이면 이번 주기 발행 없음"""
    speed: float
    steering: float
    lane_mask: object = None
    lane_mask_stamp: float = 0.0
    telemetry: str = None
    status: str = None


class ControlLoop:
    def __init__(self, config):
        self.c = config['answer']
        self.driver = Driver(config)
        self.vision = LaneVision(self.c['lane'], config['robot']['camera'], config['robot']['front_axle_x'])
        self.lane_stamp = None
        self.lane_bias = None
        self.lane_cache = None
        self.debug_stamp = -1e9
        self.detection_debug_stamp = -1e9
        self.last_state = ''

    def step(self, s):
        c, now = self.c, s.now
        out = Output(0.0, 0.0)
        ages = dict(image=now-s.image_stamp, scan=now-s.scan_stamp,
                    odometry=now-s.odom_stamp, detector=now-s.detection_stamp)
        available = dict(image=s.image is not None, scan=s.scan is not None,
                         lane=self.lane_cache is not None, detector=s.detector_ok)
        sensor_ready, required_camera = sensor_status(c, self.driver.state, ages, available)
        self.driver.debug['required_camera'] = required_camera
        if not sensor_ready or not math.isfinite(s.velocity) or not math.isfinite(s.angular_velocity):
            self.driver.debug['published_speed'] = 0.0
            self.driver.debug['speed'] = 0.0
            speed, steering = 0.0, 0.0
            self.driver.sensor_failure(now)
            state = 'waiting for fresh sensors / detector'
        else:
            if required_camera and (s.image_stamp != self.lane_stamp or self.driver.fork_bias != self.lane_bias):
                self.lane_cache = self.vision.analyze(s.image, self.driver.fork_bias)
                self.lane_cache['stamp'] = s.image_stamp
                self.lane_stamp = s.image_stamp
                self.lane_bias = self.driver.fork_bias
            lane = self.lane_cache
            was_avoiding = self.driver.avoiding
            speed, steering, state = self.driver.update(lane, s.scan, s.detections,
                                                         s.detection_stamp, s.velocity, now, s.angular_velocity)
            if was_avoiding and not self.driver.avoiding:
                self.vision.memory = {}
                self.vision.previous_center = None
                self.lane_stamp = None
            if self.driver.state != 'parking_sequence':
                age = max(now-s.image_stamp, .001)
                limit = c.get('max_image_travel', .04) / age
                speed = clip(speed, -limit, limit)
            out.speed, out.steering = speed, steering
            self.driver.debug['published_speed'] = speed
            self.driver.debug['speed'] = speed
            self.driver.debug['required_camera'] = required_camera
            if now - self.debug_stamp >= 1.0 / c['debug_rate_hz']:
                out.lane_mask, out.lane_mask_stamp = lane['lane_mask'], float(lane['stamp'])
                self.debug_stamp = now
        # Telemetry follows the actual published command, including safety stops.
        if now - self.detection_debug_stamp >= 1.0 / c['debug_rate_hz']:
            lane = self.lane_cache or {}
            keys = ('valid', 'error', 'heading', 'center_px', 'forward', 'windows',
                    'fits', 'fit_spans', 'pursuit', 'curvature')
            control = dict(self.driver.debug, steering=steering, speed=speed,
                           state=state, sensor_ready=sensor_ready,
                           controller_state=self.driver.state)
            packet = dict(stamp=s.detection_stamp, age=now-s.detection_stamp, objects=s.detections,
                          control=control, image_age=now-s.image_stamp, scan_age=now-s.scan_stamp,
                          image_stamp=s.image_stamp, lane_stamp=lane.get('stamp'),
                          lane={key: lane[key] for key in keys if key in lane},
                          display=dict(lane=c['lane'], pid=c['pid'], detector=dict(timeout=c['detector']['timeout']),
                                       sensor_timeout=c['sensor_timeout']))
            out.telemetry = json.dumps(packet)
            self.detection_debug_stamp = now
        if state != self.last_state:
            out.status = state
            self.last_state = state
        return out
