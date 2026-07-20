#!/bin/bash
# ============================================================
# Agras MID360 + FAST-LIO 一键启动脚本
# 用法: cd ~/Videos/livov_ws && ./start_agras.sh
# ============================================================

WS_DIR="/home/b/Videos/livov_ws"
ROS_SETUP="/opt/ros/humble/setup.bash"
WS_SETUP="$WS_DIR/install/setup.bash"

SOURCE_CMD="source $ROS_SETUP && source $WS_SETUP"

# 终端1: Livox 驱动 (后台运行，不阻塞)
DRIVER_CMD="$SOURCE_CMD && echo '=== Agras MID360 Driver ===' && ros2 launch livox_ros_driver2 msg_AGRAS_MID360_launch.py; exec bash"

# 终端2: FAST-LIO (延迟2秒等驱动就绪)
FASTLIO_CMD="$SOURCE_CMD && sleep 2 && echo '=== FAST-LIO Mapping ===' && ros2 launch fast_lio mapping.launch.py config_file:=agras_mid360.yaml; exec bash"

# 检测可用终端模拟器
if command -v gnome-terminal &> /dev/null; then
    gnome-terminal -- bash -c "$DRIVER_CMD" &
    sleep 0.5
    gnome-terminal -- bash -c "$FASTLIO_CMD" &
elif command -v xterm &> /dev/null; then
    xterm -hold -e bash -c "$DRIVER_CMD" &
    sleep 0.5
    xterm -hold -e bash -c "$FASTLIO_CMD" &
elif command -v konsole &> /dev/null; then
    konsole -e bash -c "$DRIVER_CMD" &
    sleep 0.5
    konsole -e bash -c "$FASTLIO_CMD" &
else
    echo "未找到 gnome-terminal/xterm/konsole，请手动开两个终端运行:"
    echo ""
    echo "  终端1 (驱动):"
    echo "    source $ROS_SETUP && source $WS_SETUP"
    echo "    ros2 launch livox_ros_driver2 msg_AGRAS_MID360_launch.py"
    echo ""
    echo "  终端2 (FAST-LIO):"
    echo "    source $ROS_SETUP && source $WS_SETUP"
    echo "    ros2 launch fast_lio mapping.launch.py config_file:=agras_mid360.yaml"
    exit 1
fi

echo "已启动:"
echo "  终端1: Livox 驱动 (端口60001/60003)"
echo "  终端2: FAST-LIO (agras_mid360.yaml)"
