import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Agras MID360 LI-Init 外参+时间偏移标定

    话题: /livox/lidar (CustomMsg) + /livox/imu (sensor_msgs/Imu)
    需驱动已在运行 (start_agras.sh init 会自动先起驱动)。
    """
    package_share = get_package_share_directory('lidar_imu_init')
    config_file = os.path.join(package_share, 'config', 'agras_mid360.yaml')
    rviz_config = os.path.join(package_share, 'rviz_cfg', 'fast_lo.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),

        Node(
            package='lidar_imu_init',
            executable='li_init',
            name='laserMapping',
            output='screen',
            parameters=[
                config_file,
                {'point_filter_num': 2},
                {'max_iteration': 5},
                {'cube_side_length': 2000.0},
            ],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
    ])
