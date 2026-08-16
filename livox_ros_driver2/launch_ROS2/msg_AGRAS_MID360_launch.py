#!/usr/bin/env python3
# ============================================================
# Agras MID360 (定制版) 点云发布 launch 文件
# ============================================================
# 与标准 MID360 的差异:
#   - 使用 AGRAS_MID360_config.json 配置端口 (60001/60003)
#   - 驱动自动检测 protocol version=1 并启用 tag 线号提取
#   - --ext-imu 参数控制使用内置/外置 IMU
# ============================================================

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ── 可配置参数 ──────────────────────────────────────────
    ext_imu_enable_arg = DeclareLaunchArgument(
        'ext_imu', default_value='false',
        description='启用外部串口IMU (L431_ADI @ ttyACM0), 默认使用内置Livox IMU')

    ext_imu_enable_lc = LaunchConfiguration('ext_imu')

    xfer_format   = 1
    multi_topic   = 0
    data_src      = 0
    publish_freq  = 10.0
    output_type   = 0
    frame_id      = 'livox_frame'
    lvx_file_path = '/home/livox/livox_test.lvx'
    cmdline_bd_code = 'livox0000000001'

    # ── 外部 IMU 桥接器参数 ─────────────────────────────────
    ext_imu_port          = '/dev/ttyS0'
    ext_imu_baudrate      = 921600
    ext_imu_gyro_unit     = 0          # 0=rad/s, 1=deg/s
    ext_imu_accel_unit    = 0          # 0=m/s², 1=G
    ext_imu_topic         = '/livox/imu'
    ext_imu_frame_id      = 'livox_frame'
    ext_imu_publish_rate  = 500.0       # ★ 外置IMU 500Hz, 逐采样发布

    cur_path = os.path.split(os.path.realpath(__file__))[0] + '/'
    cur_config_path = cur_path + '../config'
    user_config_path = os.path.join(cur_config_path, 'AGRAS_MID360_config.json')

    livox_ros2_params = [
        {"xfer_format": xfer_format},
        {"multi_topic": multi_topic},
        {"data_src": data_src},
        {"publish_freq": publish_freq},
        {"output_data_type": output_type},
        {"frame_id": frame_id},
        {"lvx_file_path": lvx_file_path},
        {"user_config_path": user_config_path},
        {"cmdline_input_bd_code": cmdline_bd_code},
        # ── 外部 IMU 参数 ──
        {"ext_imu_enable":        ext_imu_enable_lc},
        {"ext_imu_port":          ext_imu_port},
        {"ext_imu_baudrate":      ext_imu_baudrate},
        {"ext_imu_gyro_unit":     ext_imu_gyro_unit},
        {"ext_imu_accel_unit":    ext_imu_accel_unit},
        {"ext_imu_topic":         ext_imu_topic},
        {"ext_imu_frame_id":      ext_imu_frame_id},
        {"ext_imu_publish_rate":  ext_imu_publish_rate},
    ]

    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=livox_ros2_params
    )

    return LaunchDescription([
        ext_imu_enable_arg,
        livox_driver,
    ])
