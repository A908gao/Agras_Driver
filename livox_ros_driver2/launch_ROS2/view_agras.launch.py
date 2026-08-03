#!/usr/bin/env python3
# ============================================================
# Agras MID360 — 纯点云显示 (无 SLAM 算法)
# 用法: ros2 launch livox_ros_driver2 view_agras.launch.py
# ============================================================

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('livox_ros_driver2')
    rviz_cfg = os.path.join(pkg_dir, 'config', 'agras_minimal.rviz')
    user_cfg = os.path.join(pkg_dir, 'config', 'AGRAS_MID360_config.json')

    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=[{
            'xfer_format': 0,              # 0=PointCloud2 (RViz可直接显示)
            'multi_topic': 0,
            'data_src': 0,
            'publish_freq': 10.0,
            'output_data_type': 0,
            'frame_id': 'livox_frame',
            'user_config_path': user_cfg,
            'cmdline_input_bd_code': 'livox0000000001',
        }]
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'livox_frame']
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg]
    )

    return LaunchDescription([livox_driver, static_tf, rviz])
