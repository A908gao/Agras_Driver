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
#     stream    - MID360 点云 UDP 推流到 QGC (分析→MID360 Point Cloud 页面)
#     check     - 建图 + 方向一致性验证
#     verify    - 飞控(MAVROS) vs 外置IMU 姿态/位移验证 (RViz)
#     diag      - IMU-LiDAR 同步诊断
#
#   选项:
#     --ext-imu    使用外置 IMU (默认内置 IMU)
#     --built-in   使用内置 IMU (默认)
#     --gcs-ip=IP  QGC 所在设备 IP (stream 模式, 留空则 UDP 广播)
# ============================================================

# 工作区路径: 以脚本自身所在目录为准 (开发机/机载板通用)
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP="/opt/ros/humble/setup.bash"
WS_SETUP="$WS_DIR/install/setup.bash"

MODE="mapping"
USE_EXT_IMU="false"
GCS_IP=""

for arg in "$@"; do
    case $arg in
        build|clean|mapping|view|stream|diag|check|verify|init) MODE="$arg" ;;
        --built-in) USE_EXT_IMU="false" ;;
        --ext-imu)  USE_EXT_IMU="true"  ;;
        --gcs-ip=*) GCS_IP="${arg#*=}" ;;
    esac
done

# 验证模式必须使用外置 IMU (与飞控姿态对比)
[ "$MODE" == "verify" ] && USE_EXT_IMU="true"

# ── 清理 + 编译模式 ──
if [ "$MODE" == "clean" ]; then
    echo "=== 清理编译缓存 ==="
    cd "$WS_DIR" && rm -rf build install log
    echo "=== 已清理 ==="
    exit 0
fi
if [ "$MODE" == "build" ]; then
    echo "=== 清理旧缓存 + 编译工程 ($WS_DIR) ==="
    cd "$WS_DIR" && rm -rf build install log
    source "$ROS_SETUP"
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
    echo "=== 编译完成 ==="
    exit $?
fi

if [ ! -f "$WS_SETUP" ]; then
    echo "错误: 未找到 $WS_SETUP"
    echo "工作区尚未编译, 请先执行: ./start_agras.sh build"
    exit 1
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
    stream)
        BRIDGE_ARGS=""
        [ -n "$GCS_IP" ] && BRIDGE_ARGS="gcs_ip:=$GCS_IP"
        APP_CMD="$SOURCE_CMD && sleep 2 && echo '=== MID360 -> QGC UDP Stream (port 57120) ===' && ros2 launch mid360_udp_bridge mid360_udp_bridge.launch.py $BRIDGE_ARGS; exec bash"
        ;;
    diag)
        APP_CMD="$SOURCE_CMD && sleep 2 && echo '=== IMU-LiDAR 同步诊断 ===' && ros2 run fast_lio imu_lidar_sync_diag.py; exec bash"
        ;;
    check)
        APP_CMD="$SOURCE_CMD && sleep 2 && echo '=== FAST-LIO Mapping (IMU: $IMU_LABEL) ===' && ros2 launch fast_lio mapping.launch.py config_file:=$CONFIG_FILE rviz:=false; exec bash"
        CHECK_CMD="$SOURCE_CMD && sleep 6 && echo '=== IMU/LiDAR 方向一致性验证 ===' && ros2 run fast_lio imu_lidar_direction_check.py; exec bash"
        ;;
    verify)
        APP_CMD="$SOURCE_CMD && echo '=== MAVROS: 飞控 /dev/ttyACM0 ===' && ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0:2000000; exec bash"
        CHECK_CMD="$SOURCE_CMD && sleep 4 && echo '=== FC vs 外置IMU 姿态/位移验证 (RViz) ===' && ros2 launch fc_ext_imu_verify verify.launch.py; exec bash"
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
    { [ "$MODE" == "check" ] || [ "$MODE" == "verify" ]; } && { sleep 1; gnome-terminal -- bash -c "$CHECK_CMD" & }
    LAUNCHED=true
elif command -v xterm &> /dev/null; then
    xterm -hold -e bash -c "$DRIVER_CMD" &
    sleep 0.5
    xterm -hold -e bash -c "$APP_CMD" &
    { [ "$MODE" == "check" ] || [ "$MODE" == "verify" ]; } && { sleep 1; xterm -hold -e bash -c "$CHECK_CMD" & }
    LAUNCHED=true
elif command -v konsole &> /dev/null; then
    konsole -e bash -c "$DRIVER_CMD" &
    sleep 0.5
    konsole -e bash -c "$APP_CMD" &
    { [ "$MODE" == "check" ] || [ "$MODE" == "verify" ]; } && { sleep 1; konsole -e bash -c "$CHECK_CMD" & }
    LAUNCHED=true
fi

if ! $LAUNCHED; then
    echo "未找到终端模拟器，请手动运行:"
    echo "  终端1: source $ROS_SETUP && source $WS_SETUP && ros2 launch livox_ros_driver2 msg_AGRAS_MID360_launch.py ext_imu:=$USE_EXT_IMU"
    case "$MODE" in
        init)   echo "  终端2: ros2 launch lidar_imu_init agras_mid360.launch.py" ;;
        verify) echo "  终端2: ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0:2000000"
                echo "  终端3: ros2 launch fc_ext_imu_verify verify.launch.py" ;;
        *)      echo "  终端2: ros2 launch fast_lio mapping.launch.py config_file:=$CONFIG_FILE" ;;
    esac
    exit 1
fi

echo "已启动:"
echo "  终端1: Livox 驱动 (端口60001/60003)"
case "$MODE" in
    diag)   echo "  终端2: IMU-LiDAR 同步诊断" ;;
    check)  echo "  终端2: FAST-LIO 建图 (无RViz)"; echo "  终端3: IMU/LiDAR 方向一致性验证" ;;
    verify) echo "  终端2: MAVROS (飞控 /dev/ttyACM0)"; echo "  终端3: FC vs 外置IMU 姿态/位移验证 (RViz)" ;;
    init)   echo "  终端2: LI-Init 外参+时间偏移标定"; echo "  步骤: 1.静止5s → 2.充分移动 → 3.结果写入 result/" ;;
    *)      echo "  终端2: FAST-LIO 建图 ($CONFIG_FILE)" ;;
esac
