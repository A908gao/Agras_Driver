#!/usr/bin/env python3
"""
imu_lidar_direction_check.py — IMU/LiDAR 方向一致性在线验证
===========================================================
同时订阅 /livox/imu 和 /cloud_registered, 实时显示 IMU 加速度在
LiDAR 坐标系下的投影, 推动传感器即可验证 extrinsic_R 是否正确。

用法:
  ros2 run fast_lio imu_lidar_direction_check.py
  python3 imu_lidar_direction_check.py --imu /livox/imu --lidar /cloud_registered

验证方法:
  - 传感器平放静止 → IMU Z (LiDAR 坐标系) 应 ≈ +9.8 m/s² (重力朝上)
  - 快速向前推 → LiDAR X 轴应出现正向加速度尖峰
  - 快速向左推 → LiDAR Y 轴应出现正向加速度尖峰
  - 向上举 → LiDAR Z 轴加速度应明显增大

依赖: rclpy, sensor_msgs, numpy
"""

import sys
import time
import argparse
import math
from collections import deque
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Imu, PointCloud2
import numpy as np

# ═══════════════════════════════════════════════════════════════════════
# 当前 extrinsic_R (应与 agras_mid360.yaml 一致)
# 格式: column-major → column j = LiDAR axis j in IMU coordinates
# ═══════════════════════════════════════════════════════════════════════
EXTRINSIC_R = np.array([
    [0.054884, 0.998303, -0.019466],
    [0.995874, -0.056140, -0.071290],
    [0.072262, 0.015473, 0.997266],
], dtype=np.float64)  # column j = LiDAR axis j in IMU frame (LI-Init, Z flipped)
# v_imu = R @ v_lidar,  v_lidar = R.T @ v_imu

B = '\033[1m'; D = '\033[2m'; G = '\033[92m'; Y = '\033[93m'; C = '\033[96m'; R = '\033[0m'


class ImuLidarChecker(Node):
    def __init__(self, imu_topic: str, lidar_topic: str):
        super().__init__('imu_lidar_direction_check')
        self._R = EXTRINSIC_R  # v_imu = R @ v_lidar
        self._R_inv = self._R.T  # v_lidar = R.T @ v_imu (R is orthogonal)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)

        self._imu_sub = self.create_subscription(Imu, imu_topic, self._imu_cb, qos)
        self._lidar_sub = self.create_subscription(PointCloud2, lidar_topic, self._pcl_cb, qos)

        # 滑动窗口
        self._imu_acc = deque(maxlen=5)   # (ax, ay, az) in IMU frame
        self._imu_gyro = deque(maxlen=5)
        self._lidar_ts = deque(maxlen=3)

        # 统计
        self._last_print = time.time()
        self._imu_count = 0
        self._lidar_count = 0
        self._rest_acc = [0.0, 0.0, 0.0]  # 静止基准

        # 阈值
        self._move_thresh = 2.0   # m/s², 超过此值认为有运动

        self.get_logger().info("=== IMU/LiDAR Direction Check Started ===")
        self.get_logger().info(f"extrinsic_R (v_imu = R @ v_lidar):\n{self._R}")
        self.get_logger().info("操作说明: 平放3秒标定静止基准, 然后依次向前/左/上推动传感器")

    def _imu_cb(self, msg: Imu):
        self._imu_count += 1
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        self._imu_acc.append((ax, ay, az))
        self._imu_gyro.append((
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
        ))

    def _pcl_cb(self, msg: PointCloud2):
        self._lidar_count += 1
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._lidar_ts.append(ts)

    def spin_once(self):
        rclpy.spin_once(self, timeout_sec=0.1)

    def get_imu_acc(self) -> Tuple[float, float, float]:
        """返回 IMU 系下的平均加速度"""
        if not self._imu_acc:
            return (0, 0, 0)
        arr = np.array(self._imu_acc)
        return tuple(np.mean(arr, axis=0))

    def get_lidar_acc(self) -> Tuple[float, float, float]:
        """将 IMU 加速度转换到 LiDAR 坐标系"""
        ax, ay, az = self.get_imu_acc()
        v_imu = np.array([ax, ay, az])
        v_lidar = self._R_inv @ v_imu  # R.T @ v_imu
        return tuple(v_lidar)

    def get_lidar_hz(self) -> float:
        if len(self._lidar_ts) < 2:
            return 0.0
        dt = self._lidar_ts[-1] - self._lidar_ts[0]
        return (len(self._lidar_ts) - 1) / dt if dt > 0 else 0.0

    def _lidar_acc_from_rest(self) -> Tuple[float, float, float]:
        """将静止基准转换到 LiDAR 坐标系"""
        v = np.array(self._rest_acc)
        v_lidar = self._R_inv @ v
        return tuple(v_lidar)


def fmt_vec(x, y, z, fmt_str="{:+.2f}"):
    return f"[{fmt_str.format(x)}, {fmt_str.format(y)}, {fmt_str.format(z)}]"


def main():
    parser = argparse.ArgumentParser(description="IMU/LiDAR 方向一致性在线验证")
    parser.add_argument('--imu', default='/livox/imu')
    parser.add_argument('--lidar', default='/cloud_registered')
    args = parser.parse_args()

    rclpy.init()
    checker = ImuLidarChecker(args.imu, args.lidar)

    # ── 阶段1: 即时基准采集 ──────────────────────────────────
    print(f"\n{B}{'═' * 60}{R}")
    print(f"{B}  阶段1: 采集静止基准 (取第一帧){R}")
    print(f"{B}{'═' * 60}{R}")
    # 取第一帧IMU数据作为基准, 无延迟
    for _ in range(50):  # 最多等 5 秒
        checker.spin_once()
        if checker._imu_acc:
            checker._rest_acc = checker.get_imu_acc()
            break
        time.sleep(0.1)
    ax, ay, az = checker._rest_acc
    mag = math.sqrt(ax**2 + ay**2 + az**2)
    print(f"  静止基准 (IMU系):   {fmt_vec(ax, ay, az)} m/s²,  |g|={mag:.2f}")
    lx, ly, lz = checker._lidar_acc_from_rest()
    print(f"  静止基准 (LiDAR系):  {fmt_vec(lx, ly, lz)} m/s²")
    print()

    # ── 阶段2: 运动验证 ──────────────────────────────────────────
    print(f"{B}{'═' * 60}{R}")
    print(f"{B}  阶段2: 运动方向验证 (Ctrl+C 退出){R}")
    print(f"{B}{'═' * 60}{R}")
    print(f"  依次执行以下动作, 观察 LiDAR 系加速度哪个轴响应最大:")
    print(f"  {C}1. 快速向前推{R} → LiDAR X 应出现正向尖峰")
    print(f"  {C}2. 快速向左推{R} → LiDAR Y 应出现正向尖峰")
    print(f"  {C}3. 快速向上举{R} → LiDAR Z 应出现正向尖峰")
    print()
    print(f"  {'Time':>6s}  {'IMU acc (m/s²)':>35s}  {'LiDAR acc (m/s²)':>35s}  {'Motion'}")
    print(f"  {'-'*6}  {'-'*35}  {'-'*35}  {'-'*16}")

    last_dir = ""
    try:
        while rclpy.ok():
            checker.spin_once()
            now = time.time()
            if now - checker._last_print < 0.1:
                continue
            checker._last_print = now

            iax, iay, iaz = checker.get_imu_acc()
            lax, lay, laz = checker.get_lidar_acc()

            # 检测运动方向 (LiDAR系)
            lidar_vec = np.array([lax, lay, laz])
            rest_lidar = checker._lidar_acc_from_rest()
            delta = lidar_vec - np.array(rest_lidar)
            delta_mag = np.linalg.norm(delta)

            direction = "静止"
            if delta_mag > checker._move_thresh:
                max_axis = np.argmax(np.abs(delta))
                axis_names = ['X(前)', 'Y(左)', 'Z(上)']
                sign = '+' if delta[max_axis] > 0 else '-'
                direction = f"→ LiDAR {axis_names[max_axis]}{sign}  {delta_mag:.1f}"

            # 颜色: 运动轴高亮
            def color_val(val, ref, is_lidar=False):
                s = f"{val:+.2f}"
                if abs(val - ref) > checker._move_thresh:
                    return f"{G}{s}{R}"
                return s

            rlx, rly, rlz = rest_lidar
            print(f"  {now % 100:>6.1f}  {fmt_vec(iax, iay, iaz):>35s}  "
                  f"[{color_val(lax, rlx)}, {color_val(lay, rly)}, {color_val(laz, rlz)}]  "
                  f"{direction}")

            if direction != last_dir and direction != "静止":
                print(f"  {Y}▲ 检测到运动: {direction}{R}")
                last_dir = direction

    except KeyboardInterrupt:
        pass
    finally:
        print(f"\n{B}  验证结束。{R}")
        print(f"  IMU 总帧数:  {checker._imu_count}")
        print(f"  LiDAR 总帧数: {checker._lidar_count}")
        print(f"  LiDAR 频率:  {checker.get_lidar_hz():.1f} Hz")
        checker.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
