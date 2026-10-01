import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('autorace_bringup'), 'config', 'race.yaml')

    return LaunchDescription([
        Node(
            package='autorace_camera',
            executable='camera_node',
            name='camera_node',
            parameters=[params],
        ),
        Node(
            package='autorace_lidar',
            executable='lidar_node',
            name='lidar_node',
            parameters=[params],
        ),
        Node(
            package='autorace_lidar',
            executable='estop_node',
            name='estop_node',
            parameters=[params],
        ),
        Node(
            package='autorace_decision',
            executable='decision_node',
            name='decision_node',
            parameters=[params],
        ),
        Node(
            package='autorace_control',
            executable='control_node',
            name='control_node',
            parameters=[params],
        ),
    ])
