import launch
import launch_ros.actions
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """FASTLIO2 LiDAR-IMU 外参标定 — Agras MID360"""

    rviz_cfg = PathJoinSubstitution(
        [FindPackageShare("fastlio2"), "rviz", "fastlio2.rviz"]
    )

    config_path = PathJoinSubstitution(
        [FindPackageShare("fastlio2"), "config", "agras_calib.yaml"]
    )

    return launch.LaunchDescription(
        [
            # Bridge livox_frame → lidar (驱动发布 livox_frame, FASTLIO2 使用 lidar)
            launch_ros.actions.Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=["0", "0", "0", "0", "0", "0", "lidar", "livox_frame"],
            ),
            launch_ros.actions.Node(
                package="fastlio2",
                executable="lio_node",
                name="laserMapping",
                output="screen",
                parameters=[{"config_path": config_path}],
            ),
            launch_ros.actions.Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_cfg],
            ),
        ]
    )
