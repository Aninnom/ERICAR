import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

from autorace_lidar.lidar_common import box_mask, scan_to_xy


class EstopNode(Node):
    """전방 위험 영역에 장애물이 있으면 E-stop 신호를 발행하는 노드.

    - 차량 앞 직사각형 영역(위험 영역)에 라이다 점이 들어오면 E-stop을 건다.
    - 오탐을 줄이기 위해 연속으로 여러 번 감지될 때만 걸고,
      해제는 더 넓은 영역이 연속으로 비어 있을 때만 한다(히스테리시스).
    - 스캔이 일정 시간 끊기면 라이다 고장으로 보고 E-stop을 건다(fail-safe).
    - 상태는 일정한 주기로 계속 발행한다(하트비트).

    출력 토픽 형식(estop_topic)은 임시로 std_msgs/Bool이다.
    팀의 topic_spec.md가 정해지면 그 메시지로 바꿔야 한다.
    """

    def __init__(self):
        super().__init__('estop_node')

        # 파라미터: 값은 autorace_bringup의 YAML에서 덮어쓸 수 있다.
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('estop_topic', '/estop')
        self.declare_parameter('angle_offset_deg', 0.0)
        self.declare_parameter('stop_dist', 0.35)      # 정지 거리 (m)
        self.declare_parameter('release_dist', 0.50)   # 해제 거리 (m)
        self.declare_parameter('half_width', 0.20)     # 위험 영역 반폭 (m)
        self.declare_parameter('min_x', 0.05)          # 차체 반사 제외 (m)
        self.declare_parameter('trigger_count', 2)     # 연속 감지 횟수
        self.declare_parameter('release_count', 5)     # 연속 비감지 횟수
        self.declare_parameter('scan_timeout', 0.5)    # 스캔 끊김 판정 (s)
        self.declare_parameter('publish_rate', 20.0)   # 발행 주기 (Hz)

        scan_topic = self.get_parameter('scan_topic').value
        estop_topic = self.get_parameter('estop_topic').value
        rate = self.get_parameter('publish_rate').value

        self._estop_active = False      # 장애물 때문에 걸린 E-stop 상태
        self._trigger_hits = 0
        self._clear_hits = 0
        self._last_scan_time = None
        self._last_output = None
        self._last_reason = ''

        self._pub = self.create_publisher(Bool, estop_topic, 10)
        self.create_subscription(LaserScan, scan_topic, self._on_scan, 10)
        self.create_timer(1.0 / rate, self._publish_state)

        self.get_logger().info(
            f'estop_node started: scan={scan_topic}, out={estop_topic}')

    def _on_scan(self, msg: LaserScan):
        """스캔이 들어올 때마다 위험 영역 안의 장애물 여부를 갱신한다."""
        self._last_scan_time = self.get_clock().now()

        offset = self.get_parameter('angle_offset_deg').value
        stop_dist = self.get_parameter('stop_dist').value
        release_dist = max(
            self.get_parameter('release_dist').value, stop_dist + 0.05)
        half_width = self.get_parameter('half_width').value
        min_x = self.get_parameter('min_x').value
        trigger_count = self.get_parameter('trigger_count').value
        release_count = self.get_parameter('release_count').value

        x, y = scan_to_xy(msg, angle_offset_deg=offset, max_range=release_dist)
        in_stop = box_mask(x, y, min_x, stop_dist, -half_width, half_width)
        in_release = box_mask(
            x, y, min_x, release_dist, -half_width, half_width)

        if not self._estop_active:
            # 정지 영역에 연속으로 감지되면 E-stop을 건다.
            if in_stop.any():
                self._trigger_hits += 1
                if self._trigger_hits >= trigger_count:
                    self._estop_active = True
                    self._clear_hits = 0
            else:
                self._trigger_hits = 0
        else:
            # 해제 영역이 연속으로 비어 있어야 E-stop을 푼다.
            if not in_release.any():
                self._clear_hits += 1
                if self._clear_hits >= release_count:
                    self._estop_active = False
                    self._trigger_hits = 0
            else:
                self._clear_hits = 0

    def _publish_state(self):
        """E-stop 상태를 일정 주기로 발행한다. 스캔이 끊기면 정지로 본다."""
        timeout = self.get_parameter('scan_timeout').value
        if self._last_scan_time is None:
            scan_lost = True
        else:
            age = (self.get_clock().now() - self._last_scan_time).nanoseconds
            scan_lost = age * 1e-9 > timeout

        output = self._estop_active or scan_lost
        reason = 'scan lost' if scan_lost else 'obstacle'

        # 상태가 바뀔 때만 로그를 남긴다.
        if output != self._last_output or (output and reason != self._last_reason):
            if output:
                self.get_logger().warn(f'E-STOP ON ({reason})')
            else:
                self.get_logger().info('E-STOP OFF')
        self._last_output = output
        self._last_reason = reason

        out = Bool()
        out.data = output
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = EstopNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
