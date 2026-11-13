# 차 한 번에 실행: 바퀴(아커만)·라이다·카메라(+노출 고정)·주행 노드·디버그 화면·(선택) 녹화
# 사용 (차, 환경: source /opt/ros/humble/setup.bash && source ~/wego_ws/install/setup.bash
#            && source ~/ericar_dev_ws/install/setup.bash):
#   ros2 launch ericar_driver bringup.launch.py                 # 전체 (YOLO 포함)
#   ros2 launch ericar_driver bringup.launch.py yolo:=false     # 차선·라이다만
#   ros2 launch ericar_driver bringup.launch.py record:=true    # ~/bags/run_<시각> 녹화 추가
#   ros2 launch ericar_driver bringup.launch.py driver:=false   # 센서만 (주행 노드 없이)
# 디버그 화면: 차 http://localhost:8080, 맥은 ssh -L 8080:localhost:8080 wego@<차> 후 http://localhost:8080
# 바퀴는 limo_base_ackerman 사용 (wego teleop_launch는 차동 모드라 아커만 odom 미계산)
# 주행 노드 인자(config·model·yolo 등)는 driver.launch.py로 그대로 전달
import os
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

# /imu: 실측 자이로. /odom 회전은 바퀴 드라이버가 보고 조향각으로 계산한 값(v·tan(조향)/축간거리)이라 실제 회전 확인용 (11/9)
BAG_TOPICS = ['/camera/color/image_raw/compressed', '/answer/detections', '/answer/status',
              '/odom', '/imu', '/limo/ack_cmd', '/limo_status', '/scan',
              '/camera/color/camera_info', '/parking/cancel']


def include(package, launch_file, arguments=None):
    path = os.path.join(get_package_share_directory(package), 'launch', launch_file)
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(path),
                                    launch_arguments=(arguments or {}).items())


def generate_launch_description():
    share = get_package_share_directory('ericar_driver')
    exposure = LaunchConfiguration('exposure')
    # 카메라 노출: 자동 노출은 노란 선 색이 날아감 → 수동 고정. 재실행마다 초기화되므로 서비스로 한 번 더 설정
    set_exposure = (
        'until ros2 service list | grep -q /camera/set_color_exposure; do sleep 1; done; '
        'ros2 service call /camera/set_color_auto_exposure std_srvs/srv/SetBool "{data: false}" > /dev/null; '
        'ros2 service call /camera/set_color_exposure orbbec_camera_msgs/srv/SetInt32 "{data: $0}" > /dev/null; '
        'echo "[bringup] 카메라 노출: $(ros2 service call /camera/get_color_exposure '
        'orbbec_camera_msgs/srv/GetInt32 | grep -o "data=[0-9]*")"')
    return LaunchDescription([
        DeclareLaunchArgument('driver', default_value='true', description='false면 센서만'),
        DeclareLaunchArgument('debug', default_value='true', description='디버그 화면 (포트 8080)'),
        DeclareLaunchArgument('record', default_value='false', description='true면 rosbag 녹화'),
        DeclareLaunchArgument('bag', default_value=os.path.expanduser(time.strftime('~/bags/run_%m%d_%H%M'))),
        DeclareLaunchArgument('exposure', default_value='600', description='카메라 수동 노출'),        #노출값 300 -> 600 수정
        include('limo_base', 'limo_base_ackerman.launch.py'),
        include('ydlidar_ros2_driver', 'ydlidar.launch.py'),
        include('orbbec_camera', 'astra_stereo_u3.launch.py',
                {'enable_color_auto_exposure': 'false', 'color_exposure': exposure}),
        ExecuteProcess(cmd=['bash', '-c', set_exposure, exposure], output='screen'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'driver.launch.py')),
                                 condition=IfCondition(LaunchConfiguration('driver'))),
        ExecuteProcess(cmd=['python3', os.path.join(share, 'tools', 'debug_view.py'), '8080'],
                       condition=IfCondition(LaunchConfiguration('debug')), output='log'),
        ExecuteProcess(cmd=['ros2', 'bag', 'record', '-o', LaunchConfiguration('bag')] + BAG_TOPICS,
                       condition=IfCondition(LaunchConfiguration('record')), output='log'),
    ])
