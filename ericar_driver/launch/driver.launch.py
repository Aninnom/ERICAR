# 주행 노드 실행. 센서·차량 드라이버는 별도 실행
# 사용: ros2 launch ericar_driver driver.launch.py model:=/home/wego/models/best.pt
# 차선만 시험: ros2 launch ericar_driver driver.launch.py yolo:=false
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(get_package_share_directory('ericar_driver'), 'config', 'robot.yaml')
    config = LaunchConfiguration('config')
    model = LaunchConfiguration('model')
    yolo = LaunchConfiguration('yolo')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config),
        DeclareLaunchArgument('model', default_value=os.path.expanduser('~/models/best.pt')),
        DeclareLaunchArgument('yolo', default_value='true', description='false면 YOLO 없이 차선·라이다만'),
        Node(
            package='ericar_driver',
            executable='driver',
            arguments=[config, model, '--yolo', yolo],
            output='screen',
        ),
    ])
