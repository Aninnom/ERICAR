import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class SimNode(Node):
    def __init__(self):
        super().__init__('sim_node')
        self.get_logger().info('sim_node started')


def main(args=None):
    rclpy.init(args=args)
    node = SimNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
