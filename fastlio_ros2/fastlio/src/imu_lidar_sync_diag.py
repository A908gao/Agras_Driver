#!/usr/bin/env python3
"""
imu_lidar_sync_diag.py — IMU-LiDAR 时间戳连续性与同步诊断节点
==============================================================
功能:
  1. 实时监控 /livox/imu (500Hz) 时间戳连续性 (间隔 ~2ms)
  2. 实时监控 /livox/lidar (10Hz) 发布频率 (间隔 ~100ms)
  3. 统计每个 LiDAR 扫描周期内收到的 IMU 数量 (预期 ~50帧/扫描)
  4. 检查 IMU 时间戳是否覆盖 LiDAR 扫描区间

用法:
  ros2 run fast_lio imu_lidar_sync_diag.py                    # 使用默认topic
  ros2 run fast_lio imu_lidar_sync_diag.py --imu /livox/imu --lidar /livox/lidar
  ros2 run fast_lio imu_lidar_sync_diag.py --csv diag.csv    # 导出CSV

依赖: rclpy, sensor_msgs, livox_ros_driver2 (CustomMsg)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import argparse
import math
import sys
import time
from collections import deque
from typing import List, Optional

import numpy as np
from sensor_msgs.msg import Imu, PointCloud2

# 尝试导入 CustomMsg (Livox MID360 格式)
try:
    from livox_ros_driver2.msg import CustomMsg
    HAS_CUSTOM_MSG = True
except ImportError:
    HAS_CUSTOM_MSG = False


# ═══════════════════════════════════════════════════════════════════════
# 统计工具
# ═══════════════════════════════════════════════════════════════════════

class StatsRing:
    """固定窗口统计"""
    def __init__(self, maxlen: int = 200):
        self._data = deque(maxlen=maxlen)

    def add(self, v: float):
        self._data.append(v)

    @property
    def mean(self) -> float:
        return float(np.mean(self._data)) if self._data else 0.0

    @property
    def std(self) -> float:
        return float(np.std(self._data)) if len(self._data) > 1 else 0.0

    @property
    def min(self) -> float:
        return float(np.min(self._data)) if self._data else 0.0

    @property
    def max(self) -> float:
        return float(np.max(self._data)) if self._data else 0.0

    @property
    def count(self) -> int:
        return len(self._data)

    @property
    def latest(self) -> float:
        return self._data[-1] if self._data else 0.0


class ImuLidarSyncDiag(Node):
    """IMU-LiDAR 同步诊断节点"""

    def __init__(self, imu_topic: str, lidar_topic: str, csv_path: str = ""):
        super().__init__('imu_lidar_sync_diag')

        self._imu_topic = imu_topic
        self._lidar_topic = lidar_topic

        # ── QoS ──────────────────────────────────────────────────────
        imu_qos = QoSProfile(depth=100,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)
        lidar_qos = QoSProfile(depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)

        # ── 订阅 ──────────────────────────────────────────────────────
        self._imu_sub = self.create_subscription(Imu, imu_topic, self._imu_cb, imu_qos)
        if HAS_CUSTOM_MSG:
            self._lidar_sub = self.create_subscription(CustomMsg, lidar_topic, self._lidar_cb, lidar_qos)
            self.get_logger().info(f"Subscribed to {lidar_topic} (CustomMsg)")
        else:
            self._lidar_sub = self.create_subscription(PointCloud2, lidar_topic, self._lidar_pc2_cb, lidar_qos)
            self.get_logger().info(f"Subscribed to {lidar_topic} (PointCloud2)")

        self.get_logger().info(f"Subscribed to {imu_topic} (Imu)")

        # ── 统计变量 ──────────────────────────────────────────────────
        # IMU
        self._imu_ts_ring = deque(maxlen=1000)     # 最近1000帧时间戳
        self._imu_interval = StatsRing(200)          # IMU帧间隔 (s)
        self._imu_count_total = 0
        self._imu_last_ts = None
        self._imu_discont_count = 0                  # 时间戳跳变次数
        self._imu_discont_thresh = 0.005             # >5ms 视为不连续

        # LiDAR
        self._lidar_ts_ring = deque(maxlen=200)
        self._lidar_interval = StatsRing(50)
        self._lidar_count_total = 0
        self._lidar_last_ts = None

        # 每个LiDAR扫描内的IMU计数
        self._scan_imu_counts = StatsRing(50)        # IMU数/扫描
        self._current_scan_imu_count = 0
        self._scan_start_ts = None
        self._scan_end_ts = None

        # ── CSV 导出 ──────────────────────────────────────────────────
        self._csv_path = csv_path
        self._csv_fd = None
        if csv_path:
            self._csv_fd = open(csv_path, 'w')
            self._csv_fd.write("scan_idx,scan_ts,scan_dt,imu_count,imu_rate_hz,"
                               "imu_interval_ms,imu_interval_std_ms,"
                               "lidar_interval_ms,lidar_interval_std_ms,"
                               "imu_discont,imu_first_ts,imu_last_ts,"
                               "imu_coverage_ok\n")
            self.get_logger().info(f"CSV output: {csv_path}")

        # ── 定时报告 ──────────────────────────────────────────────────
        self._report_timer = self.create_timer(5.0, self._report_cb)
        self._last_report_time = time.time()
        self._scan_idx = 0

        self.get_logger().info("=== IMU-LiDAR Sync Diagnostic Node Started ===")
        self.get_logger().info(f"Expected: IMU 500Hz (dt=2.0ms), LiDAR 10Hz (dt=100ms)")
        self.get_logger().info(f"Expected IMU/LiDAR ratio: ~50 frames per scan")

    # ── 回调 ──────────────────────────────────────────────────────────
    def _imu_cb(self, msg: Imu):
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._imu_ts_ring.append(ts)
        self._imu_count_total += 1

        if self._imu_last_ts is not None:
            dt = ts - self._imu_last_ts
            self._imu_interval.add(dt)

            # 时间戳不连续检测
            if dt > self._imu_discont_thresh:
                self._imu_discont_count += 1
                if self._imu_discont_count <= 10:  # 只打印前10次
                    self.get_logger().warn(
                        f"⚠ IMU discontinuity: dt={dt*1000:.2f}ms "
                        f"(prev={self._imu_last_ts:.6f}, cur={ts:.6f})")

        self._imu_last_ts = ts

        # 当前LiDAR扫描周期内计数
        if self._scan_start_ts is not None:
            self._current_scan_imu_count += 1

    def _lidar_cb(self, msg: CustomMsg):
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._process_lidar(ts)

    def _lidar_pc2_cb(self, msg: PointCloud2):
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._process_lidar(ts)

    def _process_lidar(self, ts: float):
        """处理 LiDAR 扫描"""
        # 完成上一个扫描周期的统计
        if self._scan_start_ts is not None:
            self._scan_imu_counts.add(self._current_scan_imu_count)
            self._scan_idx += 1

            # 计算覆盖检查
            imu_coverage_ok = False
            imu_first_in_scan = 0.0
            imu_last_in_scan = 0.0
            if self._imu_ts_ring:
                imu_first = None
                imu_last = None
                for t in self._imu_ts_ring:
                    if self._scan_start_ts <= t <= ts:
                        if imu_first is None:
                            imu_first = t
                        imu_last = t
                if imu_first is not None and imu_last is not None:
                    imu_first_in_scan = imu_first
                    imu_last_in_scan = imu_last
                    # IMU覆盖至少80%的扫描区间
                    coverage = (imu_last - imu_first) / (ts - self._scan_start_ts) if ts > self._scan_start_ts else 0
                    imu_coverage_ok = coverage >= 0.8

            # CSV 记录
            if self._csv_fd:
                dt_scan = ts - self._scan_start_ts
                self._csv_fd.write(
                    f"{self._scan_idx},{ts:.6f},{dt_scan*1000:.2f},"
                    f"{self._current_scan_imu_count},"
                    f"{self._current_scan_imu_count/dt_scan if dt_scan > 0 else 0:.1f},"
                    f"{self._imu_interval.mean*1000:.2f},"
                    f"{self._imu_interval.std*1000:.2f},"
                    f"{self._lidar_interval.mean*1000:.2f},"
                    f"{self._lidar_interval.std*1000:.2f},"
                    f"{self._imu_discont_count},"
                    f"{imu_first_in_scan:.6f},{imu_last_in_scan:.6f},"
                    f"{int(imu_coverage_ok)}\n"
                )

        # 开始新扫描周期
        if self._lidar_last_ts is not None:
            dt = ts - self._lidar_last_ts
            self._lidar_interval.add(dt)

        self._lidar_last_ts = ts
        self._lidar_ts_ring.append(ts)
        self._lidar_count_total += 1
        self._scan_start_ts = ts
        self._current_scan_imu_count = 0

    # ── 定时报告 ──────────────────────────────────────────────────────
    def _report_cb(self):
        now = time.time()
        elapsed = now - self._last_report_time
        self._last_report_time = now

        imu_rate = self._imu_count_total / elapsed if elapsed > 0 else 0

        print()
        print("=" * 72)
        print(f"  IMU-LiDAR Sync Diagnostic  ─  {self._imu_topic} / {self._lidar_topic}")
        print("=" * 72)
        print(f"  IMU  总帧数: {self._imu_count_total:>8}  |  平均频率: {imu_rate:>6.1f} Hz")
        print(f"  IMU  间隔  均值: {self._imu_interval.mean*1000:>6.2f} ms  "
              f"标准差: {self._imu_interval.std*1000:>6.2f} ms  "
              f"范围: [{self._imu_interval.min*1000:.2f}, {self._imu_interval.max*1000:.2f}] ms")
        print(f"  IMU  时间戳跳变次数: {self._imu_discont_count}")

        lidar_rate = self._lidar_count_total / elapsed if elapsed > 0 else 0
        print(f"  ─────────────────────────────────────────────")
        print(f"  LiDAR 总帧数: {self._lidar_count_total:>8}  |  平均频率: {lidar_rate:>6.1f} Hz")
        print(f"  LiDAR 间隔  均值: {self._lidar_interval.mean*1000:>6.2f} ms  "
              f"标准差: {self._lidar_interval.std*1000:>6.2f} ms  "
              f"范围: [{self._lidar_interval.min*1000:.2f}, {self._lidar_interval.max*1000:.2f}] ms")
        print(f"  ─────────────────────────────────────────────")
        print(f"  IMU帧/扫描 均值: {self._scan_imu_counts.mean:>6.1f}  "
              f"标准差: {self._scan_imu_counts.std:>6.1f}  "
              f"范围: [{self._scan_imu_counts.min:.0f}, {self._scan_imu_counts.max:.0f}]")
        expected = 50.0  # 500Hz / 10Hz
        ratio = self._scan_imu_counts.mean / expected * 100 if expected > 0 else 0
        print(f"  预期 50帧/扫描 → 实际 {ratio:.0f}%")
        print("─" * 72)

        # 关键判定
        issues = []
        if self._imu_interval.count > 10:
            if abs(self._imu_interval.mean - 0.002) > 0.0005:
                issues.append(f"IMU频率偏差: {self._imu_interval.mean*1000:.2f}ms ≠ 2.00ms")
            if self._imu_interval.std > 0.001:
                issues.append(f"IMU间隔抖动大: σ={self._imu_interval.std*1000:.2f}ms")
            if self._imu_discont_count > 0:
                issues.append(f"IMU时间戳不连续: {self._imu_discont_count}次跳变")

        if self._lidar_interval.count > 5:
            if abs(self._lidar_interval.mean - 0.100) > 0.02:
                issues.append(f"LiDAR频率偏差: {self._lidar_interval.mean*1000:.0f}ms ≠ 100ms")

        if self._scan_imu_counts.count > 5:
            if abs(self._scan_imu_counts.mean - 50) > 10:
                issues.append(f"每扫描IMU数偏差: {self._scan_imu_counts.mean:.0f} ≠ 50")

        if issues:
            print("  ⚠ 发现问题:")
            for issue in issues:
                print(f"    - {issue}")
        else:
            print("  ✅ 所有指标正常")
        print("=" * 72)

    def destroy_node(self):
        if self._csv_fd:
            self._csv_fd.close()
            self.get_logger().info(f"CSV saved to {self._csv_path}")
        super().destroy_node()


# ═══════════════════════════════════════════════════════════════════════
def main():
    parser = argparse.ArgumentParser(description="IMU-LiDAR 时间戳连续性与同步诊断")
    parser.add_argument('--imu', default='/livox/imu', help='IMU topic')
    parser.add_argument('--lidar', default='/livox/lidar', help='LiDAR topic')
    parser.add_argument('--csv', default='', help='CSV output path')
    args = parser.parse_args()

    rclpy.init()
    node = ImuLidarSyncDiag(args.imu, args.lidar, args.csv)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
