import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.get_logger().info('camera_node started')


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
