"""imu_mocap 工程统一启动文件

启动全部节点：
  1. mocap_imu_calibration_node  - IMU 原始数据标定
  2. ahrs_orientation_node        - AHRS 姿态解算

所有可配置参数从 config/imu_mocap_params.yaml 统一加载并下发给各节点。

用法:
    ros2 launch imu_mocap imu_mocap.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # 定位统一参数配置文件
    config_dir = os.path.join(get_package_share_directory('imu_mocap'), 'config')
    params_file = os.path.join(config_dir, 'imu_mocap_params.yaml')

    return LaunchDescription([
        # ------------------------------------------------------------------
        # IMU 标定节点：解析原始 JSON 数据并发布 ImuDataArray
        # ------------------------------------------------------------------
        Node(
            package='imu_mocap',
            executable='mocap_imu_calibration_node',
            name='mocap_imu_calibration_node',
            output='screen',
            parameters=[params_file],
        ),

        # ------------------------------------------------------------------
        # AHRS 姿态解算节点：订阅 ImuDataArray，解算姿态并发布 ImuOrientationArray
        # ------------------------------------------------------------------
        Node(
            package='imu_mocap',
            executable='ahrs_orientation_node',
            name='ahrs_orientation_node',
            output='screen',
            parameters=[params_file],
        ),
    ])