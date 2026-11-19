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
    arrow_vote, arrow_area, arrow_confidence = (LaunchConfiguration(n) for n in ('arrow_vote', 'arrow_area', 'arrow_confidence'))
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config),
        DeclareLaunchArgument('model', default_value=os.path.expanduser('~/models/best.pt')),
        DeclareLaunchArgument('yolo', default_value='true', description='false면 YOLO 없이 차선·라이다만'),
        DeclareLaunchArgument('parking_test', default_value='false', description='true면 주차 단독 시험: 0.12 m/s 차선 주행, parking 표지판 뒤 빈칸 탐색'),
        DeclareLaunchArgument('parking_profile', default_value='b',
                              description='주차 A/B 시험: b(기본 0.54/±0.05) | a(비교용 0.45/±0.20)'),
        # 갈림길 화살표 확정 (default = params.py 그대로, 빈 문자열은 launch가 인자를 빼먹을 수 있어 안 씀). 예: arrow_vote:=false arrow_area:=0.006 arrow_confidence:=0.85
        DeclareLaunchArgument('arrow_vote', default_value='default', description='화살표 다수결 확정 true|false'),
        DeclareLaunchArgument('arrow_area', default_value='default', description='화살표 집계 최소 면적(화면 비율, 예 0.006)'),
        DeclareLaunchArgument('arrow_confidence', default_value='default', description='arrow_area보다 작은 박스를 세는 최소 신뢰도(예 0.85)'),
        DeclareLaunchArgument('start_stage', default_value='slow',
                              description='시작 구간의 미션: slow(S0) | pedestrian(S1) | fork(S2) | cones(S3) | traffic·parking(S4)'),
        Node(
            package='ericar_driver',
            executable='driver',
            arguments=[config, model, '--yolo', yolo, '--start-stage', start_stage, '--parking-test', parking_test, '--parking-profile', parking_profile,
                       '--arrow-vote', arrow_vote, '--arrow-area', arrow_area, '--arrow-confidence', arrow_confidence],
            output='screen',
        ),
    ])
