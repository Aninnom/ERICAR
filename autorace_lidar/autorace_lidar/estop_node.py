import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class EstopNode(Node):
    def __init__(self):
        super().__init__('estop_node')
        self.get_logger().info('estop_node started')


def main(args=None):
    rclpy.init(args=args)
    node = EstopNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
