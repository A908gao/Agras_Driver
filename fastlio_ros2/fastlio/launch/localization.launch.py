import os.path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node


def generate_launch_description():
    """Agras 定位模式: FAST-LIO (激光惯性里程计) + localizer (ICP 重定位)

    在已有 PCD 地图中进行定位:
      - FAST-LIO 发布 /cloud_registered_body + /Odometry
      - localizer 通过 /localizer/relocalize 服务加载地图,
        输出 map -> odom TF 定位结果 (local_frame 自动取里程计 header.frame_id)

    用法:
      ros2 launch fast_lio localization.launch.py
      ros2 launch fast_lio localization.launch.py config_file:=agras_mid360_ext.yaml
      ros2 launch fast_lio localization.launch.py rviz:=false
    """
    fast_lio_pkg = get_package_share_directory('fast_lio')
    localizer_pkg = get_package_share_directory('localizer')

    config_path = LaunchConfiguration('config_path')
    config_file = LaunchConfiguration('config_file')
    rviz_use = LaunchConfiguration('rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_config_path_cmd = DeclareLaunchArgument(
        'config_path', default_value=os.path.join(fast_lio_pkg, 'config'),
        description='FAST-LIO Yaml config directory'
    )
    declare_config_file_cmd = DeclareLaunchArgument(
        'config_file', default_value='agras_mid360.yaml',
        description='FAST-LIO config file'
    )
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Use RViz to monitor localization result'
    )

    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='fastlio_mapping',
        output='screen',
        parameters=[
            PathJoinSubstitution([config_path, config_file]),
            {'use_sim_time': use_sim_time},
            # 定位模式: 不发布 FAST-LIO 自建地图, 减小带宽/内存压力
            {'publish.map_en': False},
        ],
    )

    # baselink -> livox_frame 静态安装偏移 (与建图模式一致;
    # 缺它则 livox_frame 无父节点, 固定帧 map 下 /cloud_registered 无法显示)
    static_tf_b2l_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'baselink', 'livox_frame'],
    )

    localizer_node = Node(
        package='localizer',
        namespace='localizer',
        executable='localizer_node',
        name='localizer_node',
        output='screen',
        parameters=[
            {'config_path': os.path.join(
                localizer_pkg, 'config', 'agras_localizer.yaml')}
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(
            localizer_pkg, 'rviz', 'agras_localizer.rviz')],
        condition=IfCondition(rviz_use),
    )

    ld = LaunchDescription()
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_config_path_cmd)
    ld.add_action(declare_config_file_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(fast_lio_node)
    ld.add_action(static_tf_b2l_node)
    ld.add_action(localizer_node)
    ld.add_action(rviz_node)
    return ld
