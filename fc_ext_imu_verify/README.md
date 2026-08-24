# fc_ext_imu_verify — 飞控 vs 外置 IMU 验证

验证 MAVROS 连接的飞控（PX4 7-nano）与 MAVLink 外置 IMU（L431_ADI/ADIS16500）的
**姿态**与**空间位移**，并在同一个 RViz 中对比显示。

## 数据流

```
飞控 (PX4, USB /dev/ttyACM0, extvision 250Hz HIGHRES_IMU)
  └─ mavros px4.launch ──► /mavros/imu/data (EKF姿态)  /mavros/local_position/pose (位置)

外置 IMU (L431_ADI, MAVLink v2 HIGHRES_IMU, /dev/ttyIMU)
  └─ livox_ros_driver2 ExtImuBridge ──► /livox/imu (角速度/加速度, 无姿态)

verify_node ──► 对外置IMU角速度积分得姿态(1s后与飞控航向对齐一次)
            ──► TF:  map/fc_pose_frame → fc_base → fc (绿轴)
                                           └→ ext_imu (黄轴, 右侧5cm)
            ──► Path: /verify/fc_path (绿)  /verify/ext_imu_path (黄)
            ──► /verify/attitude_diff (roll/pitch/yaw 差, 度)
```

## 使用

一键启动（三终端：驱动 / MAVROS / 验证+RViz）：

```bash
cd /home/b/Videos/livov_ws
./start_agras.sh verify
```

或手动：

```bash
# 终端1: 雷达驱动 + 外置IMU桥接
ros2 launch livox_ros_driver2 msg_AGRAS_MID360_launch.py ext_imu:=true
# 终端2: MAVROS (飞控)
ros2 launch mavros px4.launch fcu_url:=/dev/ttyACM0:2000000
# 终端3: 验证节点 + RViz
ros2 launch fc_ext_imu_verify verify.launch.py
```

## 参数（verify.launch.py 可覆盖）

| 参数 | 默认 | 说明 |
|---|---|---|
| `fc_imu_topic` | `/mavros/imu/data` | 飞控 EKF 姿态话题 |
| `fc_pose_topic` | `/mavros/local_position/pose` | 飞控位置话题 |
| `ext_imu_topic` | `/livox/imu` | 外置 IMU 话题（若用第二个 mavros 实例读取外置IMU可改为如 `/mavros_ext/imu/data`） |
| `ext_imu_offset` | `[0.0, -0.05, 0.0]` | 外置 IMU 在飞控系 (ENU: x前 y左 z上) 的安装偏移；右侧 5cm → y 为负 |
| `publish_rate` | `50.0` | TF/Path 发布频率 |
| `align_initial_yaw` | `true` | 外置 IMU 积分姿态开机后与飞控航向对齐一次 |

## 判定标准

- TF 中 `fc` 与 `ext_imu` 两组坐标轴应基本平行（roll/pitch 差 < 数度；
  yaw 对齐后差应接近 0）。
- `/verify/attitude_diff` 三个分量持续在 0 附近。
- 两条 Path 应几乎重合（仅相差被机体姿态旋转的 5 cm 杆臂），
  即外置 IMU 确实刚性安装在飞控右侧。
