"""fast_parking1 목표 거리 기반 감속·흰 선 소실 즉시 정지 주차 전용. 센서 bringup driver=false와 함께 사용"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    path=os.path.join(get_package_share_directory('ericar_driver'),'launch','driver.launch.py')
    return LaunchDescription([
        DeclareLaunchArgument('model',default_value=os.path.expanduser('~/models/best.pt')),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(path),launch_arguments={
            'model':LaunchConfiguration('model'),'yolo':'true','parking_test':'true','start_stage':'parking'}.items()),
    ])
