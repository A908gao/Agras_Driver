#!/bin/bash
# ============================================================
# Agras MID360 — ROS2 一键启动脚本
# 用法: cd ~/Videos/livov_ws && ./start_agras.sh [mapping|calib]
#   mapping  - FAST-LIO 建图 (默认)
#   calib    - FASTLIO2 LiDAR-IMU 外参标定
# ============================================================

WS_DIR="/home/b/Videos/livov_ws"
ROS_SETUP="/opt/ros/humble/setup.bash"
WS_SETUP="$WS_DIR/install/setup.bash"

MODE="${1:-mapping}"

SOURCE_CMD="source $ROS_SETUP && source $WS_SETUP"

# 终端1: Livox 驱动 (后台运行，不阻塞)
DRIVER_CMD="$SOURCE_CMD && echo '=== Agras MID360 Driver ===' && ros2 launch livox_ros_driver2 msg_AGRAS_MID360_launch.py; exec bash"

# 终端2: 建图 或 标定
if [ "$MODE" == "calib" ]; then
    APP_CMD="$SOURCE_CMD && sleep 2 && echo '=== FASTLIO2 LiDAR-IMU 外参标定 ===' && ros2 launch fastlio2 lio_agras_calib_launch.py; exec bash"
else
    APP_CMD="$SOURCE_CMD && sleep 2 && echo '=== FAST-LIO Mapping ===' && ros2 launch fast_lio mapping.launch.py config_file:=agras_mid360.yaml; exec bash"
fi

# 检测可用终端模拟器
if command -v gnome-terminal &> /dev/null; then
    gnome-terminal -- bash -c "$DRIVER_CMD" &
    sleep 0.5
    gnome-terminal -- bash -c "$APP_CMD" &
elif command -v xterm &> /dev/null; then
    xterm -hold -e bash -c "$DRIVER_CMD" &
    sleep 0.5
    xterm -hold -e bash -c "$APP_CMD" &
elif command -v konsole &> /dev/null; then
    konsole -e bash -c "$DRIVER_CMD" &
    sleep 0.5
    konsole -e bash -c "$APP_CMD" &
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
else
    echo "  终端2: FAST-LIO (agras_mid360.yaml)"
fi
