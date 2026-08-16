#!/bin/bash
# ============================================================
# Agras MID360 — ROS2 一键启动脚本 (FAST-LIO)
# 用法: ./start_agras.sh [模式] [选项...]
#
#   模式:
#     build     - 编译整个工程
#     mapping   - SLAM 建图 (默认)
#     init      - LI-Init 外参+时间偏移自动标定
#     view      - 纯点云可视化
#     check     - 建图 + 方向一致性验证
#     diag      - IMU-LiDAR 同步诊断
#
#   选项:
#     --ext-imu    使用外置 IMU (默认内置 IMU)
#     --built-in   使用内置 IMU (默认)
# ============================================================

WS_DIR="/home/b/Agras_Driver/agras_ws"
ROS_SETUP="/opt/ros/humble/setup.bash"
WS_SETUP="$WS_DIR/install/setup.bash"

MODE="mapping"
USE_EXT_IMU="false"

for arg in "$@"; do
    case $arg in
        build|clean|mapping|view|diag|check|init) MODE="$arg" ;;
        --built-in) USE_EXT_IMU="false" ;;
        --ext-imu)  USE_EXT_IMU="true"  ;;
    esac
done

# ── 清理 + 编译模式 ──
if [ "$MODE" == "clean" ]; then
    echo "=== 清理编译缓存 ==="
    cd "$WS_DIR" && rm -rf build install log
    echo "=== 已清理 ==="
    exit 0
fi
if [ "$MODE" == "build" ]; then
    echo "=== 清理旧缓存 + 编译工程 ==="
    cd "$WS_DIR" && rm -rf build install log
    source "$ROS_SETUP"
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
    echo "=== 编译完成 ==="
    exit $?
fi

if [ "$USE_EXT_IMU" = "true" ]; then
    CONFIG_FILE="agras_mid360_ext.yaml"
    IMU_LABEL="External"
else
    CONFIG_FILE="agras_mid360.yaml"
    IMU_LABEL="Built-in"
fi

# 本板 (RK3588 / Mali-G610) 缺少 Rockchip 的 rknpu DRI 驱动,
# 强制 Mesa 使用 llvmpipe 软件渲染, 避免 "MESA-LOADER: failed to open rknpu" 报错
SOURCE_CMD="export LIBGL_ALWAYS_SOFTWARE=1 && source $ROS_SETUP && source $WS_SETUP"

DRIVER_CMD="$SOURCE_CMD && echo '=== Agras MID360 Driver (IMU: $IMU_LABEL) ===' && ros2 launch livox_ros_driver2 msg_AGRAS_MID360_launch.py ext_imu:=$USE_EXT_IMU; exec bash"

case "$MODE" in
    view)
        APP_CMD="$SOURCE_CMD && sleep 2 && echo '=== PointCloud Viewer ===' && ros2 launch livox_ros_driver2 view_agras.launch.py; exec bash"
        ;;
    diag)
        APP_CMD="$SOURCE_CMD && sleep 2 && echo '=== IMU-LiDAR 同步诊断 ===' && ros2 run fast_lio imu_lidar_sync_diag.py; exec bash"
        ;;
    check)
        APP_CMD="$SOURCE_CMD && sleep 2 && echo '=== FAST-LIO Mapping (IMU: $IMU_LABEL) ===' && ros2 launch fast_lio mapping.launch.py config_file:=$CONFIG_FILE rviz:=false; exec bash"
        CHECK_CMD="$SOURCE_CMD && sleep 6 && echo '=== IMU/LiDAR 方向一致性验证 ===' && ros2 run fast_lio imu_lidar_direction_check.py; exec bash"
        ;;
    init)
        APP_CMD="$SOURCE_CMD && sleep 2 && echo '=== LI-Init 外参+时间偏移标定 ===' && ros2 launch lidar_imu_init agras_mid360.launch.py; exec bash"
        ;;
    *)
        APP_CMD="$SOURCE_CMD && sleep 2 && echo '=== FAST-LIO Mapping (IMU: $IMU_LABEL) ===' && ros2 launch fast_lio mapping.launch.py config_file:=$CONFIG_FILE; exec bash"
        ;;
esac

LAUNCHED=false
if command -v gnome-terminal &> /dev/null; then
    gnome-terminal -- bash -c "$DRIVER_CMD" &
    sleep 0.5
    gnome-terminal -- bash -c "$APP_CMD" &
    [ "$MODE" == "check" ] && { sleep 1; gnome-terminal -- bash -c "$CHECK_CMD" & }
    LAUNCHED=true
elif command -v xterm &> /dev/null; then
    xterm -hold -e bash -c "$DRIVER_CMD" &
    sleep 0.5
    xterm -hold -e bash -c "$APP_CMD" &
    [ "$MODE" == "check" ] && { sleep 1; xterm -hold -e bash -c "$CHECK_CMD" & }
    LAUNCHED=true
elif command -v konsole &> /dev/null; then
    konsole -e bash -c "$DRIVER_CMD" &
    sleep 0.5
    konsole -e bash -c "$APP_CMD" &
    [ "$MODE" == "check" ] && { sleep 1; konsole -e bash -c "$CHECK_CMD" & }
    LAUNCHED=true
fi

if ! $LAUNCHED; then
    echo "未找到终端模拟器，请手动运行:"
    echo "  终端1: source $ROS_SETUP && source $WS_SETUP && ros2 launch livox_ros_driver2 msg_AGRAS_MID360_launch.py ext_imu:=$USE_EXT_IMU"
    case "$MODE" in
        init) echo "  终端2: ros2 launch lidar_imu_init agras_mid360.launch.py" ;;
        *)    echo "  终端2: ros2 launch fast_lio mapping.launch.py config_file:=$CONFIG_FILE" ;;
    esac
    exit 1
fi

echo "已启动:"
echo "  终端1: Livox 驱动 (端口60001/60003)"
case "$MODE" in
    diag)   echo "  终端2: IMU-LiDAR 同步诊断" ;;
    check)  echo "  终端2: FAST-LIO 建图 (无RViz)"; echo "  终端3: IMU/LiDAR 方向一致性验证" ;;
    init)   echo "  终端2: LI-Init 外参+时间偏移标定"; echo "  步骤: 1.静止5s → 2.充分移动 → 3.结果写入 result/" ;;
    *)      echo "  终端2: FAST-LIO 建图 ($CONFIG_FILE)" ;;
esac
