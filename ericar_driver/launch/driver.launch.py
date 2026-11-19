# 주행 노드 실행. 센서·차량 드라이버는 별도 실행
# 사용: ros2 launch ericar_driver driver.launch.py model:=/home/wego/models/best.pt
# 차선만 시험: ros2 launch ericar_driver driver.launch.py yolo:=false
# 주차 A/B: ros2 launch ericar_driver driver.launch.py parking_profile:=a  (기본 b)
# 재출발: ros2 launch ericar_driver driver.launch.py start_stage:=fork  (README 표 참고)
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
    start_stage = LaunchConfiguration('start_stage')
    parking_test = LaunchConfiguration('parking_test')
    parking_profile = LaunchConfiguration('parking_profile')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config),
        DeclareLaunchArgument('model', default_value=os.path.expanduser('~/models/best.pt')),
        DeclareLaunchArgument('yolo', default_value='true', description='false면 YOLO 없이 차선·라이다만'),
        DeclareLaunchArgument('parking_test', default_value='false', description='true면 주차 단독 시험: 0.12 m/s 차선 주행, parking 표지판 뒤 빈칸 탐색'),
        DeclareLaunchArgument('parking_profile', default_value='b',
                              description='주차 A/B 시험: b(기본 0.54/±0.05) | a(비교용 0.45/±0.20)'),
        DeclareLaunchArgument('start_stage', default_value='slow',
                              description='시작 구간의 미션: slow(S0) | pedestrian(S1) | fork(S2) | cones(S3) | traffic·parking(S4)'),
        Node(
            package='ericar_driver',
            executable='driver',
            arguments=[config, model, '--yolo', yolo, '--start-stage', start_stage, '--parking-test', parking_test, '--parking-profile', parking_profile],
            output='screen',
        ),
    ])
