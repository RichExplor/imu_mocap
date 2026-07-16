"""AHRS 姿态解算节点启动文件（单节点）

仅启动 ahrs_orientation_node，参数从统一配置文件加载。

如需启动全部节点（标定 + 姿态解算），请使用:
    ros2 launch imu_mocap imu_mocap.launch.py

用法:
    ros2 launch imu_mocap ahrs_orientation.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('imu_mocap'), 'config')
    params_file = os.path.join(config_dir, 'imu_mocap_params.yaml')

    return LaunchDescription([
        Node(
            package='imu_mocap',
            executable='ahrs_orientation_node',
            name='ahrs_orientation_node',
            output='screen',
            parameters=[params_file],
        ),
    ])
