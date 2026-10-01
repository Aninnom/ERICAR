import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class DecisionNode(Node):
    def __init__(self):
        super().__init__('decision_node')
        self.get_logger().info('decision_node started')


def main(args=None):
    rclpy.init(args=args)
    node = DecisionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
