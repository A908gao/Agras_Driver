"""FC(MAVROS) vs 外置IMU 姿态/空间位移验证 — 节点 + RViz."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('fc_ext_imu_verify')
    rviz_config = os.path.join(pkg_share, 'config', 'verify.rviz')

    verify_node = Node(
        package='fc_ext_imu_verify',
        executable='verify_node',
        name='fc_ext_imu_verify',
        output='screen',
        parameters=[{
            'fc_imu_topic': LaunchConfiguration('fc_imu_topic'),
            'fc_pose_topic': LaunchConfiguration('fc_pose_topic'),
            'ext_imu_topic': LaunchConfiguration('ext_imu_topic'),
            'ext_imu_offset': LaunchConfiguration('ext_imu_offset'),
        }],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='log',
    )

    return LaunchDescription([
        DeclareLaunchArgument('fc_imu_topic', default_value='/mavros/imu/data',
                              description='MAVROS 飞控姿态话题 (EKF)'),
        DeclareLaunchArgument('fc_pose_topic', default_value='/mavros/local_position/pose',
                              description='MAVROS 飞控位置话题'),
        DeclareLaunchArgument('ext_imu_topic', default_value='/livox/imu',
                              description='外置 IMU 话题 (MAVLink HIGHRES_IMU → ExtImuBridge)'),
        DeclareLaunchArgument('ext_imu_offset', default_value='[0.0, -0.05, 0.0]',
                              description='外置 IMU 在飞控系 (ENU) 的安装偏移, 默认右侧 5cm'),
        verify_node,
        rviz_node,
    ])
