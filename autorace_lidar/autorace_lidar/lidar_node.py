import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class LidarNode(Node):
    def __init__(self):
        super().__init__('lidar_node')
        self.get_logger().info('lidar_node started')


def main(args=None):
    rclpy.init(args=args)
    node = LidarNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
