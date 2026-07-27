#!/usr/bin/env python3
# ============================================================
# Agras MID360 (定制版) 点云发布 launch 文件
# ============================================================
# 与标准 MID360 的差异:
#   - 使用 AGRAS_MID360_config.json 配置端口 (60001/60003)
#   - 驱动自动检测 protocol version=1 并启用 tag 线号提取
# ============================================================

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import launch

################### user configure parameters for ros2 start ###################
xfer_format   = 1    # 1=customized pointcloud format (CustomMsg, required by FASTLIO2)
multi_topic   = 0    # 0-All LiDARs share the same topic, 1-One LiDAR one topic
data_src      = 0    # 0-lidar, others-Invalid data src
publish_freq  = 10.0 # freqency of publish, 5.0, 10.0, 20.0, 50.0, etc.
output_type   = 0
frame_id      = 'livox_frame'
lvx_file_path = '/home/livox/livox_test.lvx'
cmdline_bd_code = 'livox0000000001'

# ── 外部 IMU 桥接器参数 (L431_ADI 协议, 串口 ttyACM0) ────────────
# 设为 True 以启用外部 ADIS16500 IMU, 与点云同步发布
ext_imu_enable        = True       # ★ 启用外部 IMU 桥接器
ext_imu_port          = '/dev/ttyACM0'
ext_imu_baudrate      = 921600
ext_imu_gyro_unit     = 0          # 0=rad/s (Livox/FAST_LIO 默认), 1=deg/s
ext_imu_accel_unit    = 0          # 0=m/s² (Livox/FAST_LIO 默认), 1=G
ext_imu_topic         = '/livox/imu'
ext_imu_frame_id      = 'livox_frame'
ext_imu_publish_rate  = 200.0      # Hz, IMU 数据源 1000Hz, 降频发布

cur_path = os.path.split(os.path.realpath(__file__))[0] + '/'
cur_config_path = cur_path + '../config'
user_config_path = os.path.join(cur_config_path, 'AGRAS_MID360_config.json')
################### user configure parameters for ros2 end #####################

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
    {"ext_imu_enable":        ext_imu_enable},
    {"ext_imu_port":          ext_imu_port},
    {"ext_imu_baudrate":      ext_imu_baudrate},
    {"ext_imu_gyro_unit":     ext_imu_gyro_unit},
    {"ext_imu_accel_unit":    ext_imu_accel_unit},
    {"ext_imu_topic":         ext_imu_topic},
    {"ext_imu_frame_id":      ext_imu_frame_id},
    {"ext_imu_publish_rate":  ext_imu_publish_rate},
]


def generate_launch_description():
    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=livox_ros2_params
        )

    return LaunchDescription([
        livox_driver,
    ])
