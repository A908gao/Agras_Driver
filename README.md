# Agras MID360 Driver

基于 Livox MID360 激光雷达定制的 Agras 农业无人机驱动方案，包含驱动层修改、FAST-LIO 适配、SDK 层优化。

---

## ⚠️ 注意事项（使用前必读）

### 网络配置（不可更改）
Agras 雷达的 IP 和端口由固件固定，**不可修改**：

| 配置项 | 固定值 |
|--------|--------|
| 雷达 IP | **192.168.1.10** |
| 主机 IP | **192.168.1.20**（必须设置为此 IP） |
| 点云端口 | **60001** |
| IMU 端口 | **60003** |
| 命令端口 | **60000** |

> 使用前请将主机网卡 IP 设置为 `192.168.1.20`，子网掩码 `255.255.255.0`。

### 帧率提升导致点云视觉差异
Agras 固件将扫描帧率从标准 MID360 的 **21.9Hz 提升至 ~33Hz**。这带来：

- ✅ 每秒有效点数从 5.4 万提升至 **11.7 万**（2.2 倍）
- ⚠️ 单帧扫描时间从 45ms 缩短至 30ms，**单帧内点云看起来更稀疏**
- ⚠️ 更高帧率意味着 RViz 中旧点被更快替换，视觉上"闪烁"感更强

> 这是正常现象，不影响 SLAM 精度。如需视觉上更稠密，可降低 RViz 的 Decay Time 或降低驱动 `publish_freq`。

### FOV 缩减
Agras 固件屏蔽了右后方约 **60°** 扇区（方位角 [-180°, -130°] 及 [170°, 180°]），实际 FOV 约 **300°**。这是农业无人机机身遮挡区域的正常设计。

### 200m Sentinel 点
Agras 固件在无激光回波时生成距离恰好 **200m**、反射率为 **0** 的占位点（标准 MID360 使用全零坐标）。驱动已设置 `>50m` 过滤丢弃这些点。

### 适用场景
本驱动仅适配 **Agras MID360（定制固件）** 和 **标准 MID360**。其他 Livox 型号（HAP、MID360s 等）已移除支持，如需使用请自行甄别和适配。

---

## 模块改动汇总

### 1. livox_ros_driver2

#### 点云解析适配
| 文件 | 改动 |
|------|------|
| `src/comm/pub_handler.cpp` | **移除 Agras 的 tag 分线逻辑**，统一使用 `i % line_num` 轮转分配 4 条激光线 |
| `src/comm/pub_handler.cpp` | **>50m 距离点直接丢弃**（Agras 固件在 200m 处放置无回波 sentinel） |
| `src/comm/pub_handler.cpp` | 移除 `use_tag_for_line = true`（version=1 检测逻辑） |
| `src/comm/pub_handler.cpp` | 移除 `[DEBUG ROS]` / `[FILTER]` 等调试输出 |
| `src/comm/comm.h` | 移除 `kLineNumberHAP`，精简为 MID360 专用 |

#### IMU 数据修复（关键）
| 文件 | 改动 |
|------|------|
| `src/lddc.cpp` | **加速度 G → m/s² 转换（×9.8）**，修复 FAST-LIO 重力补偿错误导致的定位漂移 |
| `src/lddc.cpp` | 移除 `[DEBUG ROS] Distribute skip` 调试输出 |

#### 配置与启动文件
| 文件 | 说明 |
|------|------|
| `config/AGRAS_MID360_config.json` | **新增** Agras 网络配置（端口 60001/60003，IP 192.168.1.10） |
| `config/MID360_config.json` | 保留原版 MID360 配置 |
| `config/agras_pointcloud.rviz` | **新增** RViz 点云可视化配置 |
| `config/agras_minimal.rviz` | **新增** RViz 精简配置 |
| `config/agras_tf.rviz` | **新增** RViz TF 显示配置 |
| `launch_ROS2/msg_AGRAS_MID360_launch.py` | **新增** Agras 驱动启动（无 RViz） |
| `launch_ROS2/rviz_AGRAS_MID360_launch.py` | **新增** Agras 驱动启动 + RViz2 + 静态 TF |

#### 清理删除
- `config/HAP_config.json`, `MID360s_config.json`, `mixed_HAP_MID360_config.json`
- `launch_ROS2/msg_HAP_launch.py`, `msg_MID360s_launch.py`
- `launch_ROS2/rviz_HAP_launch.py`, `rviz_MID360s_launch.py`, `rviz_mixed.py`
- `config/agras_mid360_params.yaml`

#### Agras vs MID360 差异
| 参数 | MID360 | Agras |
|------|--------|-------|
| version | 0 | **1** |
| 端口 | 56300/56400 | **60001/60003** |
| 帧率 | 21.9Hz 固定 | **25~33Hz 动态** |
| FOV | 360°全覆盖 | **~300°（右后方 60°屏蔽）** |
| 无回波标记 | tag=0x00 + (0,0,0) | **200m sentinel（ref=0）** |
| tag 编码 | 位掩码 | **线号+回波号** |
| IMU 频率 | 200Hz | **~184Hz** |
| IMU 加速度单位 | G（驱动转换 m/s²） | G（驱动转换 m/s²） |

---

### 2. FAST_LIO_ROS2

| 文件 | 改动 |
|------|------|
| `config/agras_mid360.yaml` | **新增** Agras 参数：`lidar_type: 1`、`scan_rate: 10`、`det_range: 50m`、`blind: 0.3m`、**`b_gyr_cov: 0.001`（适配陀螺偏置）** |
| `config/mid360.yaml` | 保留原版配置 |
| `src/laserMapping.cpp` | **8 处 `camera_init` → `livox_frame`**，修复 TF 帧名不匹配 |
| `src/preprocess.cpp` | 适配 Agras 点云格式和 timestamp 处理 |
| `rviz/fastlio.rviz` | 修复 Fixed Frame `camera_init` → `livox_frame` |
| `launch/mapping.launch.py` | 添加 `map → livox_frame` 静态 TF 发布 |

---

### 3. Livox-SDK2

| 文件 | 改动 |
|------|------|
| `sdk_core/device_manager.cpp` | 添加 Agras 端口识别（60001/60003），移除 `[DEBUG SDK]` 输出 |
| `sdk_core/data_handler/data_handler.cpp` | 移除 `[DEBUG SDK] DataHandler::Handle` 输出 |
| `sdk_core/comm/define.h` | 添加 Agras 端口常量 |
| `sdk_core/command_handler/mid360_command_handler.cpp` | 适配 Agras version=1 协议 |
| `sdk_core/params_check.cpp` | 适配 Agras 参数校验 |

---

## 编译 & 运行

```bash
# 编译 SDK
cd Livox-SDK2 && mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_POLICY_VERSION_MINIMUM=3.5
make -j
sudo cp sdk_core/liblivox_lidar_sdk_static.a sdk_core/liblivox_lidar_sdk_shared.so /usr/local/lib/

# 编译驱动 + FAST-LIO
cd ../.. && source /opt/ros/humble/setup.bash
colcon build --packages-select livox_ros_driver2 fast_lio --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# 一键启动（自动开两个终端）
./start_agras.sh

# 或手动分别启动：
# 终端1 - 驱动
ros2 launch livox_ros_driver2 msg_AGRAS_MID360_launch.py

# 终端2 - FAST-LIO
ros2 launch fast_lio mapping.launch.py config_file:=agras_mid360.yaml
```

> **启动后务必静止 3 秒**，让 FAST-LIO 完成 IMU 初始化和陀螺偏置估计。
