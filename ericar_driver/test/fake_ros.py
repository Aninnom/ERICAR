"""ROS 없는 환경에서 노드를 돌리기 위한 최소 가짜 모듈. 회귀 시험 전용"""
import sys
import types


class Clock:
    now_s = 0.0


class _Time:
    def __init__(self, seconds):
        self.nanoseconds = int(round(seconds * 1e9))

    def to_msg(self):
        return Stamp(seconds=self.nanoseconds / 1e9)


class Stamp:
    def __init__(self, seconds=0.0):
        self.sec = int(seconds)
        self.nanosec = int(round((seconds - int(seconds)) * 1e9))


class Header:
    def __init__(self, seconds=0.0):
        self.stamp = Stamp(seconds)
        self.frame_id = ''


class Msg:
    def __init__(self, **kw):
        self.header = Header()
        self.__dict__.update(kw)


class Publisher:
    def __init__(self, topic, log):
        self.topic, self.log = topic, log

    def publish(self, message):
        self.log.append((self.topic, message))


class Logger:
    def __init__(self, log):
        self.log = log

    def info(self, text):
        self.log.append(('log', text))

    warn = warning = error = info


class Node:
    def __init__(self, name, **kwargs):
        self.published, self.subscriptions, self.timers = [], {}, []

    def create_publisher(self, msg_type, topic, qos):
        return Publisher(topic, self.published)

    def create_subscription(self, msg_type, topic, callback, qos):
        self.subscriptions[topic] = callback

    def create_timer(self, period, callback):
        self.timers.append(callback)

    def get_clock(self):
        return types.SimpleNamespace(now=lambda: _Time(Clock.now_s))

    def get_logger(self):
        return Logger(self.published)

    def destroy_node(self):
        pass


class CvBridge:
    def imgmsg_to_cv2(self, message, encoding):
        return message.array

    def cv2_to_imgmsg(self, array, encoding):
        return Msg(array=array, encoding=encoding)


def install(spin_hook):
    """sys.modules에 가짜 rclpy 등 등록. spin_hook(node)이 rclpy.spin 대신 실행됨"""
    def module(name, **attrs):
        m = types.ModuleType(name)
        m.__dict__.update(attrs)
        sys.modules[name] = m
        return m
    state = {'ok': True}
    module('rclpy', init=lambda **kw: None, ok=lambda: state['ok'],
           shutdown=lambda: state.update(ok=False), spin=spin_hook)
    module('rclpy.node', Node=Node)
    module('rclpy.qos', qos_profile_sensor_data='sensor_data')
    module('rclpy.signals', SignalHandlerOptions=types.SimpleNamespace(NO=0))
    module('rclpy.parameter', Parameter=lambda *a, **k: None)
    for pkg, names in {'ackermann_msgs': ['AckermannDrive', 'AckermannDriveStamped'],
                       'sensor_msgs': ['Image', 'Imu', 'LaserScan','CameraInfo'], 'nav_msgs': ['Odometry'],
                       'std_msgs': ['String','Bool']}.items():
        module(pkg)
        module(pkg + '.msg', **{n: Msg for n in names})
    module('cv_bridge', CvBridge=CvBridge)
    module('ament_index_python')
    module('ament_index_python.packages', get_package_share_directory=lambda name: '.')
