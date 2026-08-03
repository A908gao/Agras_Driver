#!/usr/bin/env python3
"""
imu_orientation_check.py — IMU 坐标系诊断 (内置工具) v2.0
==========================================================
逐步引导确定 ADIS16500 外置 IMU 相对 LiDAR 的安装方向。
输出 FAST-LIO 的 extrinsic_R, 支持任意正交安装角度。

用法:  python3 imu_orientation_check.py --port /dev/ttyIMU

协议: MAVLink v2 HIGHRES_IMU (pymavlink)

流程:
  步骤1: 静止3秒 → 重力方向
  步骤2-4: 分别绕 LiDAR Z/X/Y 轴旋转 → 脚本自动检测响应最大的 IMU 轴
  最终: 输出 3×3 extrinsic_R + 可选自动写入 agras_mid360.yaml
"""

import sys
import time
import math
import threading
from typing import Optional, Tuple, List

from pymavlink import mavutil

# ═══════════════════════════════════════════════════════════════════════
DEG2RAD = math.pi / 180.0
RAD2DEG = 180.0 / math.pi
MAVLINK_MSG_ID_HIGHRES_IMU = 105


class ImuReader:
    """基于 pymavlink 的 MAVLink HIGHRES_IMU 读取器"""

    def __init__(self):
        self._mav = mavutil.mavlink.MAVLink(None)
        self.accel = (0.0, 0.0, 0.0)
        self.gyro = (0.0, 0.0, 0.0)
        self.temp = 0.0
        self.counter = 0
        self.ready = threading.Event()
        # MAVLink 数据已是 SI 单位, 不需要标定系数
        self.gyro_scale = 1.0   # 占位, 兼容旧流程
        self.accel_scale = 1.0

    def feed(self, b: int) -> None:
        """喂入单字节, pymavlink 内部自动帧同步"""
        try:
            msg = self._mav.parse_char(bytes([b]))
        except Exception:
            return
        if msg is None:
            return
        if msg.get_msgId() == MAVLINK_MSG_ID_HIGHRES_IMU:
            self.counter = (self.counter + 1) % 65536
            # MAVLink: gyro = rad/s, accel = m/s²
            self.gyro = (
                msg.xgyro * RAD2DEG,
                msg.ygyro * RAD2DEG,
                msg.zgyro * RAD2DEG,
            )
            self.accel = (
                msg.xacc,
                msg.yacc,
                msg.zacc,
            )
            self.temp = msg.temperature
            self.ready.set()

# ═══════════════════════════════════════════════════════════════════════
def collect(reader: ImuReader, duration: float) -> Tuple[List, List]:
    gy, ac = [], []
    reader.ready.clear()
    t0 = time.time()
    while time.time() - t0 < duration:
        if reader.ready.wait(timeout=0.5):
            reader.ready.clear()
            gy.append(reader.gyro)
            ac.append(reader.accel)
    return gy, ac

def mean(samples: List[Tuple]) -> Tuple[float, float, float]:
    if not samples: return (0.0, 0.0, 0.0)
    n = len(samples)
    return (sum(s[0] for s in samples)/n, sum(s[1] for s in samples)/n, sum(s[2] for s in samples)/n)

def peak_of(samples: List[Tuple]) -> Tuple[float, float, float]:
    """返回各轴最大绝对值所在样本的值 (保留符号)."""
    if not samples: return (0.0, 0.0, 0.0)
    p = max(samples, key=lambda s: abs(s[0]) + abs(s[1]) + abs(s[2]))
    return p

# ═══════════════════════════════════════════════════════════════════════
B = '\033[1m'; D = '\033[2m'; G = '\033[92m'; Y = '\033[93m'; C = '\033[96m'; R = '\033[0m'

def ask(prompt: str) -> str:
    print(f"\n{G}  > {prompt}{R}", end='', flush=True)
    return input().strip().lower()

def step(n: int, title: str, instr: str) -> None:
    print(f"\n{C}┌─ 步骤 {n}: {title}{R}")
    for line in instr.strip().split('\n'):
        print(f"{C}│{R}  {D}{line}{R}")

def show(label: str, x: float, y: float, z: float, unit: str) -> None:
    print(f"  {label}:  X={x:10.3f}  Y={y:10.3f}  Z={z:10.3f}  {unit}")

def fmt_mat(r: List[float]) -> str:
    return (f"    [{r[0]:.6g}, {r[1]:.6g}, {r[2]:.6g},\n"
            f"     {r[3]:.6g}, {r[4]:.6g}, {r[5]:.6g},\n"
            f"     {r[6]:.6g}, {r[7]:.6g}, {r[8]:.6g}]")

def write_yaml(extR: List[float]) -> None:
    """将 extrinsic_R 写入 agras_mid360.yaml, 并确保 extrinsic_est_en: true."""
    import os, re
    candidates = [
        'FAST_LIO_ROS2/config/agras_mid360.yaml',
        'FASTLIO2_ROS2/fastlio2/config/agras_calib.yaml',
        'src/FAST_LIO_ROS2/config/agras_mid360.yaml',
    ]
    yaml_path = None
    for c in candidates:
        if os.path.exists(c):
            yaml_path = c
            break
    if not yaml_path:
        print(f"  {Y}⚠ 未找到 agras_mid360.yaml, 请手动写入。{R}")
        return

    with open(yaml_path, 'r') as f:
        content = f.read()

    backup = yaml_path + '.bak'
    with open(backup, 'w') as f:
        f.write(content)

    new_R = (f"extrinsic_R: [{extR[0]:.6g}, {extR[1]:.6g}, {extR[2]:.6g}, "
             f"{extR[3]:.6g}, {extR[4]:.6g}, {extR[5]:.6g}, "
             f"{extR[6]:.6g}, {extR[7]:.6g}, {extR[8]:.6g}]")
    content = re.sub(r'extrinsic_R:\s*\[.*?\]', new_R, content)
    content = re.sub(r'extrinsic_est_en:\s*false', 'extrinsic_est_en:  true', content)

    with open(yaml_path, 'w') as f:
        f.write(content)

    print(f"  {G}✓ 已写入: {yaml_path}{R}")
    print(f"  {D}  备份:   {backup}{R}")


# ═══════════════════════════════════════════════════════════════════════

def run(port: str, baudrate: int = 921600) -> None:
    try:
        import serial
    except ImportError:
        print("请先安装 pyserial: pip install pyserial"); sys.exit(1)

    reader = ImuReader()
    stop = threading.Event()

    def serial_thread():
        try:
            with serial.Serial(port, baudrate, timeout=1.0) as ser:
                ser.reset_input_buffer()
                while not stop.is_set():
                    n = ser.in_waiting
                    if n > 0:
                        for b in ser.read(n): reader.feed(b)
                    else:
                        b = ser.read(1)
                        if b: reader.feed(b[0])
        except Exception as e:
            if not stop.is_set(): print(f"\n串口错误: {e}")

    th = threading.Thread(target=serial_thread, daemon=True)
    th.start()

    print(f"\n{B}  ADIS16500 IMU 坐标系诊断 (MAVLink){R}")
    print(f"  串口: {port}")
    print(f"  ✓ 已连接\n")

    # ── 步骤 1: 静止测试 ─────────────────────────────────────────
    step(1, "静止测试 — 哪个轴朝上?",
        "将传感器静置在水平桌面上, 不要触碰。")

    while True:
        ans = ask("传感器已平放? 按 Enter 开始 3 秒采集")
        if ans in ('q','quit','exit'): stop.set(); return

        print(f"  {D}采集 3 秒...{R}", end='', flush=True)
        _, ac = collect(reader, 3.0)
        if not ac:
            print(f"\r{' '*30}\r  {Y}⚠ 无数据{R}"); continue

        avg_a = mean(ac)
        mag = math.sqrt(avg_a[0]**2 + avg_a[1]**2 + avg_a[2]**2)
        print(f"\r{' '*30}\r", end='')
        show("静止平均值", avg_a[0], avg_a[1], avg_a[2], "m/s²")
        print(f"  合加速度: {mag:.3f} m/s²")

        if abs(mag - 9.8) > 2.0:
            print(f"  {Y}⚠ 数据异常, 请确保传感器静止{R}")
            if ask("重试? (y / q=退出)").startswith('q'): stop.set(); return
            continue

        axs = sorted([('X',avg_a[0]),('Y',avg_a[1]),('Z',avg_a[2])], key=lambda x: abs(x[1]), reverse=True)
        up_a, up_v = axs[0]
        up_s = '+' if up_v > 0 else '-'
        print(f"\n  {B}★ 重力方向: IMU {up_a}{up_s} 读数最大 ({up_v:+.3f} m/s²){R}")
        print(f"    重力加速度 ~9.8 m/s² 沿 IMU {up_a}{up_s}")

        if ask("确认? (y / n=重试 / q=退出)").startswith('q'): stop.set(); return
        if ask("确认? (y / n=重试)") in ('y',''): break

    # ── 旋转测试: 找出 LiDAR 各轴对应 IMU 哪个轴 ──────────────────
    # 返回 (imu_axis: 0=X,1=Y,2=Z, sign: +1/-1, ok: bool)
    def rotation_test(num: int, lidar_axis: str, direction_hint: str):
        """
        用户绕 LiDAR 的 lidar_axis 旋转, 脚本自动找出响应的 IMU 轴和符号。
        返回 (imu_axis_idx, sign, ok).
        imu_axis_idx: 0=X, 1=Y, 2=Z
        sign: +1 表示同向, -1 表示反向
        ok: False 表示用户退出
        """
        axis_names = ['X', 'Y', 'Z']
        step(num, f"绕 LiDAR {lidar_axis} 轴旋转",
            direction_hint + f"\n脚本将自动检测哪个 IMU 轴响应最大。")
        while True:
            ans = ask(f"准备好绕 LiDAR {lidar_axis} 轴旋转? 按 Enter 开始")
            if ans.startswith('q'): return (0, 1, False)

            print(f"  {D}采集 3 秒... 立即旋转!{R}", end='', flush=True)
            gy, _ = collect(reader, 3.0)
            if not gy:
                print(f"\r{' '*30}\r  {Y}⚠ 无数据{R}"); continue

            px, py, pz = peak_of(gy)
            print(f"\r{' '*30}\r", end='')
            show(f"旋转峰值 (LiDAR {lidar_axis})", px, py, pz, "deg/s")

            # 找出绝对值最大的轴
            vals = [(0, px, 'X'), (1, py, 'Y'), (2, pz, 'Z')]
            vals.sort(key=lambda v: abs(v[1]), reverse=True)
            best_idx, best_val, best_name = vals[0]
            second_idx, second_val, second_name = vals[1]

            if abs(best_val) < 30:
                print(f"  {Y}⚠ 所有轴响应都太小 (max={best_val:.1f} deg/s), 请用力旋转{R}")
                if ask("重试? (y / q=退出)").startswith('q'): return (0, 1, False)
                continue

            # 检查响应是否集中在单一轴上
            ratio = abs(second_val) / max(abs(best_val), 1.0)
            if ratio > 0.6:
                print(f"  {Y}⚠ 多个轴同时响应 (第2大={second_name}:{second_val:.1f}), 请尽量绕单一轴旋转{R}")
                if ask("重试? (y / q=退出)").startswith('q'): return (0, 1, False)
                continue

            sign = 1 if best_val > 0 else -1
            sign_str = '+' if sign > 0 else '-'
            print(f"  {G}★ LiDAR {lidar_axis} 旋转 → IMU {best_name}{sign_str} 响应最大 ({best_val:.1f} deg/s){R}")
            print(f"    即: LiDAR {lidar_axis} 方向 = IMU {best_name}{sign_str} 方向")

            if ask(f"确认? (y / n=重试 / q=退出)").startswith('q'):
                return (0, 1, False)
            if ask("确认? (y/n)") in ('y', ''):
                return (best_idx, sign, True)

    # ── 步骤 2: 绕 Z 轴旋转 ──────────────────────────────────────
    z_idx, z_sign, ok = rotation_test(2, "Z",
        "右手定则: 拇指朝 LiDAR Z+ (上), 四指弯曲=正角速度。\n"
        "将传感器平放, 从上方逆时针快速旋转约 90°。")
    if not ok: stop.set(); return

    # ── 步骤 3: 绕 X 轴旋转 ──────────────────────────────────────
    x_idx, x_sign, ok = rotation_test(3, "X",
        "右手定则: 拇指朝 LiDAR X+ (前), 四指弯曲=正角速度。\n"
        "握住传感器, 快速向前点头翻滚。")
    if not ok: stop.set(); return

    # ── 步骤 4: 绕 Y 轴旋转 ──────────────────────────────────────
    y_idx, y_sign, ok = rotation_test(4, "Y",
        "右手定则: 拇指朝 LiDAR Y+ (左), 四指弯曲=正角速度。\n"
        "握住传感器, 快速向左侧翻。")
    if not ok: stop.set(); return

    # ── 检查轴映射是否正交 (X,Y,Z 应对应不同 IMU 轴) ─────────────
    if len({x_idx, y_idx, z_idx}) < 3:
        print(f"\n  {Y}⚠ 检测到的轴映射有冲突:")
        print(f"    LiDAR X → IMU {['X','Y','Z'][x_idx]}")
        print(f"    LiDAR Y → IMU {['X','Y','Z'][y_idx]}")
        print(f"    LiDAR Z → IMU {['X','Y','Z'][z_idx]}")
        print(f"    三个 LiDAR 轴应对应三个不同的 IMU 轴, 请重新运行。{R}")
        stop.set(); return

    # ═══════════════════════════════════════════════════════════════
    # 构建 extrinsic_R: v_lidar = R * v_imu
    # R 的列 j 是 LiDAR 轴 j 在 IMU 坐标系中的单位向量
    # ═══════════════════════════════════════════════════════════════
    stop.set()

    # 先构建 IMU→LiDAR 的映射矩阵 (每行对应一个 LiDAR 轴)
    # mapping[lidar_axis] = (imu_axis, sign)
    mapping = {0: (x_idx, x_sign), 1: (y_idx, y_sign), 2: (z_idx, z_sign)}

    # 旋转矩阵: column j = LiDAR axis j in IMU coordinates
    extR = [0.0] * 9
    for lidar_j in range(3):
        imu_i, sign = mapping[lidar_j]
        extR[imu_i * 3 + lidar_j] = float(sign)

    print(f"\n{B}{'═' * 55}{R}")
    print(f"{B}  轴映射诊断结果{R}")
    print(f"{B}{'═' * 55}{R}")
    print()
    print(f"  LiDAR X (前)  →  IMU {['X','Y','Z'][x_idx]}{'+' if x_sign>0 else '-'}")
    print(f"  LiDAR Y (左)  →  IMU {['X','Y','Z'][y_idx]}{'+' if y_sign>0 else '-'}")
    print(f"  LiDAR Z (上)  →  IMU {['X','Y','Z'][z_idx]}{'+' if z_sign>0 else '-'}")
    print()

    # 验证: 静止时重力应沿 LiDAR Z- 方向
    # R * [0,0,-9.8] 应对应到静止加速度方向
    # 也就是 R 的第 2 列 (Z) 应指向上方, 与重力方向相反
    imu_grav = (avg_a[0], avg_a[1], avg_a[2])
    grav_imu_axis = max(range(3), key=lambda i: abs(imu_grav[i]))
    print(f"  重力验证: 静止时 IMU {['X','Y','Z'][grav_imu_axis]} 轴读数最大 ({imu_grav[grav_imu_axis]:+.3f} m/s²)")
    if grav_imu_axis == z_idx:
        print(f"  {G}✓ 与旋转测试一致 (LiDAR Z → IMU {['X','Y','Z'][z_idx]}){R}")
    else:
        print(f"  {Y}⚠ 重力轴 ({['X','Y','Z'][grav_imu_axis]}) 与旋转 Z 轴 ({['X','Y','Z'][z_idx]}) 不一致, 请检查{R}")

    print()
    print(f"  {B}extrinsic_R — 写入 agras_mid360.yaml 的 mapping 段:{R}")
    print(fmt_mat(extR))
    print()
    print(f"  {D}# 同时建议: extrinsic_est_en: true  (让 FAST-LIO 在线精化){R}")

    # ── 自动写入 YAML ─────────────────────────────────────────────
    ans = ask("是否自动写入 agras_mid360.yaml? (y=写入 / n=跳过)")
    if ans in ('y', ''):
        write_yaml(extR)
    else:
        print(f"  {D}跳过。可手动复制上面的 extrinsic_R 到配置文件。{R}")

    print(f"\n  {B}{G}✅ 诊断完成!{R}")


if __name__ == '__main__':
    import argparse
    ap = argparse.ArgumentParser(description='IMU 坐标系交互式诊断')
    ap.add_argument('--port','-p', default='/dev/ttyIMU', help='串口路径')
    ap.add_argument('--baudrate','-b', type=int, default=921600, help='波特率')
    args = ap.parse_args()
    try:
        run(args.port, args.baudrate)
    except KeyboardInterrupt:
        print(f"\n中断。")
