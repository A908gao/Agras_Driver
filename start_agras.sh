#!/bin/bash
# ============================================================
# Agras MID360 — ROS2 一键启动脚本
# 用法: cd ~/Videos/livov_ws && ./start_agras.sh [mapping|calib|diag|check|init] [--built-in]
#   mapping   - FAST-LIO 建图 (默认)
#   built-in  - 单独使用时: 选择内置IMU配置 (默认用外置)
#   不带 --built-in 时使用外置IMU
# ============================================================

WS_DIR="/home/b/Videos/livov_ws"
ROS_SETUP="/opt/ros/humble/setup.bash"
WS_SETUP="$WS_DIR/install/setup.bash"

MODE="mapping"
USE_EXT_IMU="true"
CONFIG_FILE="agras_mid360.yaml"

for arg in "$@"; do
    case $arg in
        mapping|calib|view|diag|check|init) MODE="$arg" ;;
        --built-in) USE_EXT_IMU="false"; CONFIG_FILE="agras_mid360.yaml" ;;
        --ext-imu)  USE_EXT_IMU="true";  CONFIG_FILE="agras_mid360_ext.yaml" ;;
    esac
done

# 外置IMU使用专用配置
if [ "$USE_EXT_IMU" = "true" ]; then
    CONFIG_FILE="agras_mid360_ext.yaml"
else
    CONFIG_FILE="agras_mid360.yaml"
fi

SOURCE_CMD="source $ROS_SETUP && source $WS_SETUP"

# 终端1: Livox 驱动
DRIVER_CMD="$SOURCE_CMD && echo '=== Agras MID360 Driver (IMU: ' && if [ \"$USE_EXT_IMU\" = \"true\" ]; then echo 'External) ==='; else echo 'Built-in) ==='; fi && ros2 launch livox_ros_driver2 msg_AGRAS_MID360_launch.py ext_imu:=$USE_EXT_IMU; exec bash"

# 终端2: 应用
if [ "$MODE" == "view" ]; then
    APP_CMD="$SOURCE_CMD && sleep 2 && echo '=== PointCloud Viewer ===' && ros2 launch livox_ros_driver2 view_agras.launch.py; exec bash"
elif [ "$MODE" == "calib" ]; then
    APP_CMD="$SOURCE_CMD && sleep 2 && echo '=== FASTLIO2 LiDAR-IMU 外参标定 ===' && ros2 launch fastlio2 lio_agras_calib_launch.py; exec bash"
elif [ "$MODE" == "diag" ]; then
    APP_CMD="$SOURCE_CMD && sleep 2 && echo '=== IMU-LiDAR 同步诊断 ===' && ros2 run fast_lio imu_lidar_sync_diag.py; exec bash"
elif [ "$MODE" == "check" ]; then
    APP_CMD="$SOURCE_CMD && sleep 2 && echo '=== FAST-LIO Mapping ===' && ros2 launch fast_lio mapping.launch.py config_file:=$CONFIG_FILE rviz:=false; exec bash"
    CHECK_CMD="$SOURCE_CMD && sleep 6 && echo '=== IMU/LiDAR 方向一致性验证 ===' && ros2 run fast_lio imu_lidar_direction_check.py; exec bash"
elif [ "$MODE" == "init" ]; then
    APP_CMD="$SOURCE_CMD && sleep 2 && echo '=== LI-Init 外参+时间偏移标定 ===' && ros2 launch lidar_imu_init agras_mid360.launch.py; exec bash"
else
    APP_CMD="$SOURCE_CMD && sleep 2 && echo '=== FAST-LIO Mapping (IMU: ' && if [ \"$USE_EXT_IMU\" = \"true\" ]; then echo 'External) ==='; else echo 'Built-in) ==='; fi && ros2 launch fast_lio mapping.launch.py config_file:=$CONFIG_FILE; exec bash"
fi

# 检测可用终端模拟器
if command -v gnome-terminal &> /dev/null; then
    gnome-terminal -- bash -c "$DRIVER_CMD" &
    sleep 0.5
    gnome-terminal -- bash -c "$APP_CMD" &
    if [ "$MODE" == "check" ]; then
        sleep 1
        gnome-terminal -- bash -c "$CHECK_CMD" &
    fi
elif command -v xterm &> /dev/null; then
    xterm -hold -e bash -c "$DRIVER_CMD" &
    sleep 0.5
    xterm -hold -e bash -c "$APP_CMD" &
    if [ "$MODE" == "check" ]; then
        sleep 1
        xterm -hold -e bash -c "$CHECK_CMD" &
    fi
elif command -v konsole &> /dev/null; then
    konsole -e bash -c "$DRIVER_CMD" &
    sleep 0.5
    konsole -e bash -c "$APP_CMD" &
    if [ "$MODE" == "check" ]; then
        sleep 1
        konsole -e bash -c "$CHECK_CMD" &
    fi
else
    echo "未找到 gnome-terminal/xterm/konsole，请手动开两个终端运行:"
    echo ""
    echo "  终端1 (驱动):"
    echo "    source $ROS_SETUP && source $WS_SETUP"
    echo "    ros2 launch livox_ros_driver2 msg_AGRAS_MID360_launch.py"
    echo ""
    if [ "$MODE" == "calib" ]; then
        echo "  终端2 (标定):"
        echo "    source $ROS_SETUP && source $WS_SETUP"
        echo "    ros2 launch fastlio2 lio_agras_calib_launch.py"
    else
        echo "  终端2 (建图):"
        echo "    source $ROS_SETUP && source $WS_SETUP"
        echo "    ros2 launch fast_lio mapping.launch.py config_file:=agras_mid360.yaml"
    fi
    exit 1
fi

echo "已启动:"
echo "  终端1: Livox 驱动 (端口60001/60003)"
if [ "$MODE" == "calib" ]; then
    echo "  终端2: FASTLIO2 LiDAR-IMU 外参标定"
    echo ""
    echo "  ╔══════════════════════════════════════════╗"
    echo "  ║  标定步骤:                              ║"
    echo "  ║  1. 保持雷达静止 5-10 秒 (初始化)       ║"
    echo "  ║  2. 充分移动雷达: 平移 + 旋转           ║"
    echo "  ║  3. 标定收敛后将 r_il/t_il 填入配置文件 ║"
    echo "  ╚══════════════════════════════════════════╝"
elif [ "$MODE" == "diag" ]; then
    echo "  终端2: IMU-LiDAR 同步诊断"
    echo ""
    echo "  诊断项目:"
    echo "    - IMU 时间戳连续性 (500Hz → Δt≈2ms)"
    echo "    - LiDAR 发布频率 (10Hz → Δt≈100ms)"
    echo "    - 每扫描 IMU 帧数 (预期 ~50帧)"
    echo "    - IMU 时间戳是否覆盖 LiDAR 扫描区间"
elif [ "$MODE" == "check" ]; then
    echo "  终端2: FAST-LIO (无 RViz)"
    echo "  终端3: IMU/LiDAR 方向一致性验证"
    echo ""
    echo "  依次向前/左/上推传感器, 验证 extrinsic_R 矩阵正确性"
elif [ "$MODE" == "init" ]; then
    echo "  终端2: LI-Init 外参+时间偏移自动标定"
    echo ""
    echo "  ╔══════════════════════════════════════════╗"
    echo "  ║  标定步骤:                              ║"
    echo "  ║  1. 传感器静置 5 秒                     ║"
    echo "  ║  2. 充分移动: 平移+旋转 (脚本会提示)     ║"
    echo "  ║  3. 标定完成后结果写入 result/ 目录      ║"
    echo "  ╚══════════════════════════════════════════╝"
else
    echo "  终端2: FAST-LIO (agras_mid360.yaml)"
fi
