import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        self.get_logger().info('control_node started')


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
