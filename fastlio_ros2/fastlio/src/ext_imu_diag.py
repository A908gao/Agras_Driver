#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ext_imu_diag.py — 外置 IMU (L431_ADI / ADIS16500) 稳定性与 LIO 耦合性诊断节点
================================================================================
数据链路:
  L431_ADI MCU (500Hz HIGHRES_IMU, MAVLink v2)
    → /dev/ttyS0 → ext_imu_bridge → /livox/imu (sensor_msgs/Imu)

验证外置 IMU 是否满足接入 FAST-LIO 的稳定性要求:

  [1] 时间戳连续性     帧间隔跳变(丢帧/串口断流) / 乱序 / 重复时间戳
  [2] 最近1秒窗口最大时间波动   窗口内 dt_max - dt_min (峰峰波动) /
                       最大绝对偏差 / RMS 抖动
  [3] 频率稳定性       实测频率 vs 期望频率(默认500Hz) + 丢帧率估计
  [4] 静态零偏与噪声   静止段: 陀螺σ / 加表σ / 重力模长偏差 / 残余陀螺零偏
                       (LIO 零速修正与重力对齐的直接依据)
  [5] 数据有效性       NaN / 饱和 / 数据卡死(连续重复读数) / 端到端时延

用法:
  ros2 run fast_lio ext_imu_diag.py                     # 默认 /livox/imu @500Hz
  ros2 run fast_lio ext_imu_diag.py --rate 200          # 自定义期望频率
  ros2 run fast_lio ext_imu_diag.py --duration 30       # 运行30s自动退出, 退出码=判定(0通过/1失败)
  ros2 run fast_lio ext_imu_diag.py --csv imu_diag.csv  # 导出CSV
"""

import argparse
import math
import sys
from collections import deque
from typing import List, Optional, Tuple

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Imu

G_STD = 9.80665  # 标准重力 m/s²

# ── 静止段判定阈值 (检测用, 非判定用) ──
STATIC_GYRO_NORM_MAX = 0.15   # rad/s  静止时陀螺均值模长上限
STATIC_ACCEL_STD_MAX = 0.5    # m/s²   静止时加表模长σ上限
STATIC_GRAV_WIN = 1.0         # m/s²   静止时重力模长容差


class StatsRing:
    """固定窗口统计"""
    def __init__(self, maxlen: int = 200):
        self._data = deque(maxlen=maxlen)

    def add(self, v: float) -> None:
        self._data.append(v)

    @property
    def count(self) -> int:
        return len(self._data)

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


class ExtImuDiag(Node):
    """外置 IMU 稳定性诊断节点"""

    def __init__(self, imu_topic: str, rate: float, duration: float,
                 report_interval: float, csv_path: str,
                 gap_ms: float, rate_tol: float, spread_max_ms: float,
                 jitter_max_ms: float, loss_max_pct: float,
                 gyro_sigma_max: float, accel_sigma_max: float,
                 grav_err_max: float, gyro_norm_static_max: float,
                 latency_max_ms: float, frozen_max: int,
                 gyro_sat: float, accel_sat: float):
        super().__init__('ext_imu_diag')

        self._imu_topic = imu_topic
        self._exp_rate = float(rate)
        self._exp_dt = 1.0 / self._exp_rate
        # 跳变阈值 = max(用户设定, 3倍期望间隔)
        self._gap_thresh = max(gap_ms / 1000.0, 3.0 * self._exp_dt)

        # ── 判定阈值 ──
        self._rate_tol = rate_tol
        self._spread_max = spread_max_ms / 1000.0
        self._jitter_max = jitter_max_ms / 1000.0
        self._loss_max = loss_max_pct / 100.0
        self._gyro_sigma_max = gyro_sigma_max
        self._accel_sigma_max = accel_sigma_max
        self._grav_err_max = grav_err_max
        self._gyro_norm_static_max = gyro_norm_static_max
        self._latency_max = latency_max_ms / 1000.0
        self._frozen_max = frozen_max
        self._gyro_sat = gyro_sat
        self._accel_sat = accel_sat

        # ── 订阅 (BEST_EFFORT 兼容 RELIABLE/BEST_EFFORT 发布端) ──
        qos = QoSProfile(depth=500,
                         reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self._sub = self.create_subscription(Imu, imu_topic, self._cb, qos)

        # ── 滚动窗口: (stamp, dt, gx, gy, gz, ax, ay, az) ──
        self._win: deque = deque()
        self._last_stamp: Optional[float] = None
        self._last_vec: Optional[Tuple[float, ...]] = None
        self._consec_same = 0
        self._max_same_streak = 0

        # ── 累计计数 ──
        self._n_total = 0
        self._n_gap = 0        # 时间戳跳变(大间隔)
        self._n_ooo = 0        # 乱序
        self._n_dup = 0        # 重复时间戳
        self._n_nan = 0
        self._n_sat = 0
        self._dt_max_ever = 0.0
        self._dt_min_ever = float('inf')

        self._latency = StatsRing(100)   # ms

        # ── 静止段噪声 ──
        self._static_latest: Optional[dict] = None
        self._static_best: Optional[dict] = None
        self._static_windows = 0

        # ── 状态 ──
        self._done = False
        self._exit_code = 1
        self._reported_final = False

        # ── CSV ──
        self._csv_path = csv_path
        self._csv_fd = None
        if csv_path:
            self._csv_fd = open(csv_path, 'w')
            self._csv_fd.write(
                "t_sec,n_win,span_s,rate_hz,dt_mean_ms,dt_std_ms,dt_min_ms,dt_max_ms,"
                "spread_ms,max_dev_ms,win_gaps,"
                "ooo_cum,dup_cum,gap_cum,lost,loss_pct,static,"
                "gyro_sig_max,accel_sig_max,gyro_norm_mean,grav_err,accel_norm_std,"
                "nan_cum,sat_cum,max_same_streak,latency_max_ms,stall_ms,pass\n")
            self.get_logger().info(f"CSV output: {csv_path}")

        # ── 定时器 ──
        self._report_timer = self.create_timer(report_interval, self._report_cb)
        if duration > 0:
            self._end_timer = self.create_timer(duration, self._finalize_cb)

        self.get_logger().info(f"Subscribed to {imu_topic} (Imu, BEST_EFFORT)")
        self.get_logger().info(
            f"Expected rate: {self._exp_rate:.0f} Hz (dt={self._exp_dt*1000:.2f} ms)")
        self.get_logger().info(
            f"Gap threshold: {self._gap_thresh*1000:.1f} ms | "
            f"1s-window spread limit: {spread_max_ms:.1f} ms")
        self.get_logger().info("=== External IMU Stability Diagnostic Started ===")
        self.get_logger().info("提示: 保持设备静止数秒可自动评估零偏/噪声")

    # ── 对外状态 ──────────────────────────────────────────────
    @property
    def done(self) -> bool:
        return self._done

    @property
    def exit_code(self) -> int:
        return self._exit_code

    # ── 回调 ──────────────────────────────────────────────────
    def _cb(self, msg: Imu):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        now_s = self.get_clock().now().nanoseconds * 1e-9
        self._latency.add((now_s - stamp) * 1000.0)

        self._n_total += 1
        dt = None
        if self._last_stamp is not None:
            d = stamp - self._last_stamp
            if d < 0:
                self._n_ooo += 1
            elif d == 0.0:
                self._n_dup += 1
            else:
                dt = d
                self._dt_max_ever = max(self._dt_max_ever, d)
                self._dt_min_ever = min(self._dt_min_ever, d)
                if d > self._gap_thresh:
                    self._n_gap += 1
                    if self._n_gap <= 10:
                        self.get_logger().warn(
                            f"⚠ timestamp gap: dt={d*1000:.2f}ms > "
                            f"{self._gap_thresh*1000:.2f}ms")
        self._last_stamp = stamp

        gx, gy, gz = (msg.angular_velocity.x, msg.angular_velocity.y,
                      msg.angular_velocity.z)
        ax, ay, az = (msg.linear_acceleration.x, msg.linear_acceleration.y,
                      msg.linear_acceleration.z)

        self._win.append((stamp, dt, gx, gy, gz, ax, ay, az))

        # ── 有效性 ──
        if any(map(math.isnan, (gx, gy, gz, ax, ay, az))):
            self._n_nan += 1
        if (max(abs(gx), abs(gy), abs(gz)) > self._gyro_sat or
                max(abs(ax), abs(ay), abs(az)) > self._accel_sat):
            self._n_sat += 1

        # ── 卡死检测: 连续完全相同的读数 (健康IMU几乎不会逐帧重复) ──
        vec = (gx, gy, gz, ax, ay, az)
        if self._last_vec is not None and vec == self._last_vec:
            self._consec_same += 1
            self._max_same_streak = max(self._max_same_streak, self._consec_same)
        else:
            self._consec_same = 0
        self._last_vec = vec

    # ── 窗口与统计 ────────────────────────────────────────────
    def _window(self) -> deque:
        """按最新时间戳截取最近1秒窗口"""
        if self._win:
            latest = self._win[-1][0]
            while self._win and latest - self._win[0][0] > 1.0:
                self._win.popleft()
        return self._win

    def _compute(self) -> Optional[dict]:
        if self._n_total == 0:
            return None

        win = self._window()
        latest = win[-1][0]
        n_win = len(win)
        span = latest - win[0][0] if n_win > 1 else 0.0
        dts = [e[1] for e in win if e[1] is not None]
        n_dt = len(dts)
        rate = n_dt / span if span > 1e-9 else 0.0
        expected_n = span / self._exp_dt if span > 1e-9 else 0.0
        lost = max(0, int(round(expected_n)) - n_dt)
        loss_pct = lost / expected_n * 100.0 if expected_n > 1e-9 else 0.0

        if dts:
            dt_mean = float(np.mean(dts))
            dt_std = float(np.std(dts)) if len(dts) > 1 else 0.0
            dt_min = float(np.min(dts))
            dt_max = float(np.max(dts))
        else:
            dt_mean = dt_std = dt_min = dt_max = 0.0

        spread = dt_max - dt_min
        max_dev = max(abs(d - self._exp_dt) for d in dts) if dts else 0.0
        win_gaps = sum(1 for d in dts if d > self._gap_thresh)

        # ── 窗口内静止段分析 ──
        g = np.array([[e[2], e[3], e[4]] for e in win], dtype=float)
        a = np.array([[e[5], e[6], e[7]] for e in win], dtype=float)
        gyro_mean = g.mean(axis=0)
        gyro_sig = g.std(axis=0)
        accel_sig = a.std(axis=0)
        accel_norms = np.linalg.norm(a, axis=1)
        accel_norm_mean = float(accel_norms.mean())
        accel_norm_std = float(accel_norms.std())
        gyro_norm_mean = float(np.linalg.norm(gyro_mean))

        static = (gyro_norm_mean < STATIC_GYRO_NORM_MAX and
                  accel_norm_std < STATIC_ACCEL_STD_MAX and
                  abs(accel_norm_mean - G_STD) < STATIC_GRAV_WIN)
        if static and span > 0.5:
            snap = {
                'gyro_sig': gyro_sig.tolist(),
                'accel_sig': accel_sig.tolist(),
                'gyro_norm_mean': float(gyro_norm_mean),
                'accel_norm_mean': float(accel_norm_mean),
                'accel_norm_std': float(accel_norm_std),
                'grav_err': float(accel_norm_mean - G_STD),
            }
            self._static_latest = snap
            self._static_windows += 1
            if (self._static_best is None or
                    max(snap['gyro_sig']) < max(self._static_best['gyro_sig'])):
                self._static_best = snap

        # ── 数据流停滞 (时间戳 vs 当前时钟) ──
        now_s = self.get_clock().now().nanoseconds * 1e-9
        stall_ms = max(0.0, (now_s - latest) * 1e3)

        return {
            'latest': latest, 'n_win': n_win, 'span': span, 'rate': rate,
            'dt_mean': dt_mean, 'dt_std': dt_std, 'dt_min': dt_min,
            'dt_max': dt_max, 'spread': spread, 'max_dev': max_dev,
            'win_gaps': win_gaps, 'lost': lost, 'loss_pct': loss_pct,
            'static': self._static_latest, 'stall_ms': stall_ms,
        }

    # ── 判定 ──────────────────────────────────────────────────
    def _evaluate(self, c: dict) -> List[Tuple[str, str, str]]:
        res: List[Tuple[str, str, str]] = []

        def add(name: str, ok: bool, detail: str, skip: bool = False):
            status = 'skip' if skip else ('ok' if ok else 'fail')
            res.append((name, status, detail))

        # 1. 数据到达
        add('数据到达', c['n_win'] > 0, f"最近1秒 {c['n_win']} 帧")

        # 2. 频率稳定性
        if c['span'] > 0.2 and c['n_win'] > 2:
            dev = (c['rate'] - self._exp_rate) / self._exp_rate
            add('频率稳定性', abs(dev) <= self._rate_tol,
                f"实测 {c['rate']:.1f} Hz vs 期望 {self._exp_rate:.0f} Hz "
                f"(偏差 {dev*100:+5.1f}%)")
        else:
            add('频率稳定性', False, "窗口数据不足, 等待1秒...", skip=True)

        # 3. 时间戳连续性
        if c['n_win'] > 2:
            add('时间戳连续性',
                c['win_gaps'] == 0 and self._n_ooo == 0 and self._n_dup == 0,
                f"窗口跳变 {c['win_gaps']} | 累计乱序 {self._n_ooo} | 重复 {self._n_dup}")
        else:
            add('时间戳连续性', False, "数据不足", skip=True)

        # 4. 最近1秒窗口最大时间波动
        if c['n_win'] > 2:
            ok = (c['spread'] <= self._spread_max and
                  c['dt_std'] <= self._jitter_max)
            add('1秒窗口波动', ok,
                f"峰峰 {c['spread']*1e3:.2f}ms(限{self._spread_max*1e3:.1f}) | "
                f"最大偏差 {c['max_dev']*1e3:.2f}ms | "
                f"RMS {c['dt_std']*1e3:.2f}ms(限{self._jitter_max*1e3:.1f})")
        else:
            add('1秒窗口波动', False, "数据不足", skip=True)

        # 5. 丢帧率
        if c['span'] > 0.2:
            add('丢帧率', c['loss_pct'] <= self._loss_max * 100.0,
                f"丢失 {c['lost']} 帧 / {c['loss_pct']:.1f}% "
                f"(限 {self._loss_max*100:.1f}%)")
        else:
            add('丢帧率', False, "数据不足", skip=True)

        # 6-9. 静态零偏/噪声
        s = c['static']
        if s is None:
            add('陀螺噪声σ', False, "未观测到静止段", skip=True)
            add('加表噪声σ', False, "未观测到静止段", skip=True)
            add('重力模长', False, "未观测到静止段", skip=True)
            add('残余零偏', False, "未观测到静止段", skip=True)
        else:
            gs = max(s['gyro_sig'])
            as_ = max(s['accel_sig'])
            add('陀螺噪声σ', gs <= self._gyro_sigma_max,
                f"max {gs:.4f} rad/s (限 {self._gyro_sigma_max:.4f})")
            add('加表噪声σ', as_ <= self._accel_sigma_max,
                f"max {as_:.4f} m/s² (限 {self._accel_sigma_max:.3f})")
            add('重力模长', abs(s['grav_err']) <= self._grav_err_max,
                f"偏差 {s['grav_err']:+.4f} m/s² (限 ±{self._grav_err_max:.2f})")
            add('残余零偏', s['gyro_norm_mean'] <= self._gyro_norm_static_max,
                f"{s['gyro_norm_mean']:.5f} rad/s "
                f"(限 {self._gyro_norm_static_max:.4f})")

        # 10. 数据有效性
        valid = (self._n_nan == 0 and self._n_sat == 0 and
                 self._max_same_streak <= self._frozen_max)
        add('数据有效性', valid,
            f"NaN {self._n_nan} | 饱和 {self._n_sat} | "
            f"连续重复 {self._max_same_streak}(限{self._frozen_max})")

        # 11. 数据流停滞
        add('数据流停滞', c['stall_ms'] < 200.0,
            f"滞后 {c['stall_ms']:.1f} ms (限 200.0)")

        # 12. 端到端时延
        if self._latency.count > 10:
            add('端到端时延', self._latency.max <= self._latency_max * 1000.0,
                f"均值 {self._latency.mean:.2f} ms | "
                f"最大 {self._latency.max:.2f} ms "
                f"(限 {self._latency_max*1000:.0f})")
        else:
            add('端到端时延', False, "数据不足", skip=True)

        return res

    # ── 报告 ──────────────────────────────────────────────────
    def _print_report(self, final: bool):
        if final:
            self._reported_final = True

        c = self._compute()
        if c is None:
            if final:
                print("=" * 74)
                print("  ❌ 未收到任何 IMU 数据 — 外置 IMU 检测失败")
                print("  请检查: 1) 驱动是否以 ext_imu:=true 启动")
                print("          2) /dev/ttyS0 是否存在 (ls -l /dev/ttyS0)")
                print("          3) MCU 供电 / USB 连接 / 波特率 (921600)")
                print("          4) ros2 topic list | grep imu")
                print("=" * 74)
                self._exit_code = 1
            else:
                print(f"⏳ 等待 IMU 数据... ({self._imu_topic}, "
                      f"期望 {self._exp_rate:.0f}Hz)")
            return

        res = self._evaluate(c)
        ok_count = sum(1 for _, st, _ in res if st == 'ok')
        fail_count = sum(1 for _, st, _ in res if st == 'fail')
        skip_count = sum(1 for _, st, _ in res if st == 'skip')
        passed = fail_count == 0 and ok_count > 0

        print()
        print("=" * 74)
        print(f"  外置 IMU 稳定性诊断 ─ {self._imu_topic}  "
              f"(期望 {self._exp_rate:.0f} Hz)")
        print("=" * 74)
        print(f"  [数据流] 累计帧数 {self._n_total} | 最近1秒 {c['n_win']} 帧 | "
              f"实测频率 {c['rate']:.1f} Hz")
        print(f"  [1秒窗口] 帧间隔: 均值 {c['dt_mean']*1e3:.2f} ms | "
              f"最大 {c['dt_max']*1e3:.2f} ms | 最小 {c['dt_min']*1e3:.2f} ms")
        print(f"            最大时间波动(峰峰) {c['spread']*1e3:.2f} ms | "
              f"最大绝对偏差 {c['max_dev']*1e3:.2f} ms | "
              f"RMS抖动 {c['dt_std']*1e3:.2f} ms")
        print(f"            丢帧估计 {c['lost']} 帧 ({c['loss_pct']:.1f}%)")
        print(f"  [时间戳] 跳变(>{self._gap_thresh*1e3:.1f}ms) 累计 {self._n_gap} | "
              f"乱序 {self._n_ooo} | 重复 {self._n_dup} | "
              f"历史间隔 [{self._dt_min_ever*1e3:.2f}, "
              f"{self._dt_max_ever*1e3:.2f}] ms")

        s = c['static']
        if s is not None:
            print(f"  [静态噪声] 最近 ({self._static_windows} 个静止窗口观测)")
            print(f"    陀螺σ [{s['gyro_sig'][0]:.4f}, {s['gyro_sig'][1]:.4f}, "
                  f"{s['gyro_sig'][2]:.4f}] rad/s")
            print(f"    加表σ [{s['accel_sig'][0]:.4f}, {s['accel_sig'][1]:.4f}, "
                  f"{s['accel_sig'][2]:.4f}] m/s²")
            print(f"    重力模长偏差 {s['grav_err']:+.4f} m/s² | "
                  f"残余陀螺零偏 {s['gyro_norm_mean']:.5f} rad/s | "
                  f"振动 {s['accel_norm_std']:.4f} m/s²")
            if self._static_best is not None and self._static_best is not s:
                b = self._static_best
                print(f"  [静态噪声] 最佳 (最低陀螺σ)")
                print(f"    陀螺σ max {max(b['gyro_sig']):.4f} rad/s | "
                      f"加表σ max {max(b['accel_sig']):.4f} m/s² | "
                      f"重力偏差 {b['grav_err']:+.4f} m/s²")
        else:
            print("  [静态噪声] 未观测到静止段 — 保持 IMU 静止约1秒即可自动评估")

        print(f"  [有效性] NaN {self._n_nan} | 饱和 {self._n_sat} | "
              f"连续重复读数 {self._max_same_streak} | "
              f"时延最大 {self._latency.max:.2f} ms | "
              f"数据流滞后 {c['stall_ms']:.1f} ms")
        print("-" * 74)

        icons = {'ok': '✅', 'fail': '❌', 'skip': '⏭ '}
        for name, status, detail in res:
            print(f"  {icons[status]} {name:<12} {detail}")

        print("-" * 74)
        if passed:
            print(f"  ✅ 综合判定: PASS ({ok_count} 项通过"
                  f"{', 跳过 ' + str(skip_count) if skip_count else ''})"
                  f" — 外置 IMU 满足 LIO 耦合稳定性要求")
        else:
            print(f"  ❌ 综合判定: FAIL — 通过 {ok_count}, "
                  f"失败 {fail_count}, 跳过 {skip_count}")
        print("=" * 74)

        self._write_csv(c, passed)

        if final:
            self._exit_code = 0 if passed else 1

    def _write_csv(self, c: dict, passed: bool):
        if not self._csv_fd:
            return
        s = c['static']
        row = (f"{c['latest']:.6f},{c['n_win']},{c['span']:.6f},{c['rate']:.1f},"
               f"{c['dt_mean']*1e3:.3f},{c['dt_std']*1e3:.3f},"
               f"{c['dt_min']*1e3:.3f},{c['dt_max']*1e3:.3f},"
               f"{c['spread']*1e3:.3f},{c['max_dev']*1e3:.3f},{c['win_gaps']},"
               f"{self._n_ooo},{self._n_dup},{self._n_gap},"
               f"{c['lost']},{c['loss_pct']:.2f},{1 if s else 0},")
        if s:
            row += (f"{max(s['gyro_sig']):.5f},{max(s['accel_sig']):.5f},"
                    f"{s['gyro_norm_mean']:.5f},{s['grav_err']:.5f},"
                    f"{s['accel_norm_std']:.5f},")
        else:
            row += ",,,,"
        row += (f"{self._n_nan},{self._n_sat},{self._max_same_streak},"
                f"{self._latency.max:.3f},{c['stall_ms']:.1f},"
                f"{1 if passed else 0}\n")
        self._csv_fd.write(row)
        self._csv_fd.flush()

    # ── 定时回调 ──────────────────────────────────────────────
    def _report_cb(self):
        self._print_report(final=False)

    def _finalize_cb(self):
        self.get_logger().info("诊断时长结束, 输出最终报告...")
        self._print_report(final=True)
        self._done = True

    def destroy_node(self):
        if not self._reported_final:
            self._print_report(final=True)
        if self._csv_fd:
            self._csv_fd.close()
            self.get_logger().info(f"CSV saved to {self._csv_path}")
        super().destroy_node()


def main():
    parser = argparse.ArgumentParser(
        description='外置IMU稳定性与LIO耦合性诊断 (时间戳连续性/1秒窗口波动/频率/零偏噪声)')
    parser.add_argument('--imu', default='/livox/imu', help='IMU topic')
    parser.add_argument('--rate', type=float, default=500.0,
                        help='期望频率 Hz (默认500)')
    parser.add_argument('--duration', type=float, default=0.0,
                        help='运行时长s, 0=无限 (默认). 到时自动输出最终判定并退出')
    parser.add_argument('--report-interval', type=float, default=2.0,
                        help='报告间隔s (默认2)')
    parser.add_argument('--csv', default='', help='CSV输出路径')
    parser.add_argument('--gap-ms', type=float, default=6.0,
                        help='时间戳跳变阈值ms (默认6, 且不小于3倍期望间隔)')
    parser.add_argument('--rate-tol', type=float, default=0.05,
                        help='频率容差 (默认0.05=±5%%)')
    parser.add_argument('--spread-max-ms', type=float, default=2.0,
                        help='最近1秒窗口最大时间波动(峰峰)上限 ms (默认2.0)')
    parser.add_argument('--jitter-max-ms', type=float, default=0.8,
                        help='帧间隔RMS抖动上限 ms (默认0.8)')
    parser.add_argument('--loss-max-pct', type=float, default=2.0,
                        help='丢帧率上限 %% (默认2.0)')
    parser.add_argument('--gyro-sigma-max', type=float, default=0.02,
                        help='静止陀螺σ上限 rad/s (默认0.02)')
    parser.add_argument('--accel-sigma-max', type=float, default=0.15,
                        help='静止加表σ上限 m/s² (默认0.15)')
    parser.add_argument('--grav-err-max', type=float, default=0.15,
                        help='重力模长偏差上限 m/s² (默认0.15)')
    parser.add_argument('--gyro-norm-static-max', type=float, default=0.02,
                        help='静止残余陀螺零偏上限 rad/s (默认0.02)')
    parser.add_argument('--latency-max-ms', type=float, default=20.0,
                        help='端到端时延上限 ms (默认20)')
    parser.add_argument('--frozen-max', type=int, default=50,
                        help='卡死判定: 连续重复读数帧数上限 (默认50)')
    parser.add_argument('--gyro-sat', type=float, default=30.0,
                        help='陀螺饱和判定阈值 rad/s (默认30)')
    parser.add_argument('--accel-sat', type=float, default=150.0,
                        help='加表饱和判定阈值 m/s² (默认150)')
    args = parser.parse_args()

    rclpy.init()
    node = ExtImuDiag(
        imu_topic=args.imu, rate=args.rate, duration=args.duration,
        report_interval=args.report_interval, csv_path=args.csv,
        gap_ms=args.gap_ms, rate_tol=args.rate_tol,
        spread_max_ms=args.spread_max_ms, jitter_max_ms=args.jitter_max_ms,
        loss_max_pct=args.loss_max_pct, gyro_sigma_max=args.gyro_sigma_max,
        accel_sigma_max=args.accel_sigma_max, grav_err_max=args.grav_err_max,
        gyro_norm_static_max=args.gyro_norm_static_max,
        latency_max_ms=args.latency_max_ms, frozen_max=args.frozen_max,
        gyro_sat=args.gyro_sat, accel_sat=args.accel_sat)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        code = node.exit_code
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
