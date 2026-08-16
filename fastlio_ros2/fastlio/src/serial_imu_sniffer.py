#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
serial_imu_sniffer.py — 外置 IMU 原始串口帧率/时序分析工具
============================================================
绕过 ROS 驱动, 直接读取 /dev/ttyS0 原始字节流, 解析 MAVLink v2
HIGHRES_IMU 帧, 定位丢帧发生在 MCU / 串口链路 / 主机驱动哪一层。

每秒统计:
  - MAVLink 总帧数 / HIGHRES_IMU 帧数 (实际到达频率)
  - CRC 错误 / 重新同步次数 (字节损坏或丢字节 → 链路/电气/主机读延迟)
  - SEQ 跳变数 (链路丢失的帧, MCU 已发出但主机没收到)
  - time_usec 增量 min/max/mean/≠2000us 计数 (MCU 采样节拍抖动, 直接可见)

判定逻辑:
  - 到达频率 ≈500Hz 且 time_usec 增量恒 2000us → MCU 正常, 问题在主机/驱动
  - 到达频率 <490Hz, CRC错误=0, time_usec 增量恒 2000us → MCU 每帧都发了
    但数量不足 (MCU 发送节拍/调度问题)
  - CRC 错误>0 或 SEQ 跳变>0 → 串口链路丢字节 (电气噪声/USB FIFO 溢出/主机读延迟)
  - time_usec 增量出现非 2000us → MCU 采样/发送节拍抖动 (MCU 固件问题)

用法:
  python3 serial_imu_sniffer.py --seconds 30               # 默认 /dev/ttyS0
  python3 serial_imu_sniffer.py --port /dev/ttyUSB0 --seconds 10
  python3 serial_imu_sniffer.py --self-test                # CRC/解析自检
"""

import argparse
import os
import struct
import sys
import termios
import time

STX = 0xFD
HIGHRES_IMU_MSGID = 105
HIGHRES_IMU_PAYLOAD_LEN = 63
# MAVLink 各消息的 crc_extra 字节 (来自 mavlink_msg_highres_imu.h:
#   MAVLINK_MSG_ID_HIGHRES_IMU_CRC 93 = 0x5D)
CRC_EXTRA_BY_MSGID = {HIGHRES_IMU_MSGID: 0x5D}
EXPECTED_US_500HZ = 2000


# ── MAVLink X.25 CRC-16 ───────────────────────────────────────────
def crc16_x25(data: bytes, seed: int = 0xFFFF) -> int:
    crc = seed & 0xFFFF
    for b in data:
        tmp = (crc ^ b) & 0xFF
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = (((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF)
    return crc


class MavlinkStreamParser:
    """流式 MAVLink v2 帧解析 (支持签名, 坏帧自动重同步)"""

    def __init__(self):
        self.buf = bytearray()
        self.resync_events = 0   # 丢弃乱码前缀/非法帧的次数

    def feed(self, chunk: bytes):
        """返回 [(frame_bytes, crc_ok, seq, msgid), ...]"""
        self.buf.extend(chunk)
        out = []
        while True:
            i = self.buf.find(STX)
            if i < 0:
                self.buf.clear()
                break
            if i > 0:
                self.resync_events += 1
                del self.buf[:i]
                continue
            if len(self.buf) < 10:
                break
            plen = self.buf[1]
            incompat = self.buf[2]
            total = 10 + plen + 2
            sig = 13 if (incompat & 0x01) else 0
            if total < 12 or total > 280:      # 非法帧长 → 丢弃 STX 重新同步
                self.resync_events += 1
                del self.buf[0]
                continue
            if len(self.buf) < total + sig:
                break
            frame = bytes(self.buf[:total])
            seq = frame[4]
            msgid = frame[7] | (frame[8] << 8) | (frame[9] << 16)
            extra = CRC_EXTRA_BY_MSGID.get(msgid)
            if extra is None:
                ok = None   # 非目标消息, 不校验 CRC
            else:
                calc = crc16_x25(frame[1:total - 2] + bytes([extra]))
                got = frame[total - 2] | (frame[total - 1] << 8)
                ok = calc == got
            out.append((frame, ok, seq, msgid))
            del self.buf[:total + sig]
        return out


# ── HIGHRES_IMU 载荷解析 (仅取 time_usec, 无需浮点字段) ──────────
def parse_highres_time_usec(payload: bytes):
    if len(payload) < 8:
        return None
    return struct.unpack('<Q', payload[:8])[0]


def build_highres_frame(seq: int, time_usec: int) -> bytes:
    payload = struct.pack('<Q', time_usec)          # time_usec
    payload += struct.pack('<9f', *([0.0] * 9))     # acc/gyro/mag 9 floats
    payload += struct.pack('<4f', *([0.0] * 4))     # pressure/temp 4 floats
    payload += struct.pack('<H', 0)                 # fields_updated
    payload += struct.pack('<B', 0)                 # id
    assert len(payload) == HIGHRES_IMU_PAYLOAD_LEN
    head = bytes([STX, HIGHRES_IMU_PAYLOAD_LEN, 0x00, 0x00, seq & 0xFF,
                  0x00, 0x00, HIGHRES_IMU_MSGID & 0xFF,
                  (HIGHRES_IMU_MSGID >> 8) & 0xFF, (HIGHRES_IMU_MSGID >> 16) & 0xFF])
    crc = crc16_x25(head[1:] + payload +
                    bytes([CRC_EXTRA_BY_MSGID[HIGHRES_IMU_MSGID]]))
    return head + payload + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def self_test():
    print("=== CRC / 解析自检 ===")
    # 1. 构造 3 帧 → 解析应全部通过
    stream = b''.join(build_highres_frame(i, i * EXPECTED_US_500HZ) for i in range(3))
    p = MavlinkStreamParser()
    frames = p.feed(stream)
    assert len(frames) == 3 and all(f[1] for f in frames), "构造帧解析失败"
    ts = [parse_highres_time_usec(f[0][10:10 + HIGHRES_IMU_PAYLOAD_LEN]) for f in frames]
    assert ts == [0, 2000, 4000], f"time_usec 解析错误: {ts}"
    # 2. CRC 篡改 → 应判为坏帧
    bad = bytearray(build_highres_frame(7, 14000))
    bad[-1] ^= 0xFF
    frames = p.feed(bytes(bad))
    assert len(frames) == 1 and not frames[0][1], "CRC 篡改未被检出"
    # 3. 帧间插入垃圾字节 → 重同步应恢复
    junk = b'\x12\x34' + build_highres_frame(8, 16000)
    frames = p.feed(junk)
    assert len(frames) == 1 and frames[0][1], "重同步失败"
    print("  3/3 通过: 帧解析 / CRC 检出 / 重同步")
    print("=== 自检完成 ===")


def open_raw_serial(port: str, baud: int):
    fd = os.open(port, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
    attrs = termios.tcgetattr(fd)
    attrs[2] &= ~termios.CSTOPB
    attrs[2] &= ~termios.PARENB
    attrs[2] &= ~termios.CSIZE
    attrs[2] |= termios.CS8
    attrs[2] &= ~termios.CRTSCTS
    attrs[3] &= ~(termios.ICANON | termios.ECHO | termios.ISIG)
    attrs[4] &= ~termios.OPOST
    baud_const = getattr(termios, f'B{baud}', None)
    if baud_const is None:
        print(f"不支持的波特率常量 B{baud}")
        sys.exit(1)
    attrs[4] = baud_const   # ISPEED
    attrs[5] = baud_const   # OSPEED
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    termios.tcflush(fd, termios.TCIOFLUSH)
    return fd


def main():
    ap = argparse.ArgumentParser(description='外置 IMU 原始串口帧率/时序分析')
    ap.add_argument('--port', default='/dev/ttyS0', help='串口设备 (默认 /dev/ttyS0)')
    ap.add_argument('--baud', type=int, default=921600, help='波特率 (默认 921600)')
    ap.add_argument('--seconds', type=float, default=0.0, help='运行时长s, 0=无限')
    ap.add_argument('--save', default='', help='原始字节流保存路径 (供事后分析)')
    ap.add_argument('--self-test', action='store_true', help='CRC/解析自检')
    args = ap.parse_args()

    if args.self_test:
        self_test()
        return

    if not os.path.exists(args.port):
        print(f"❌ 设备不存在: {args.port}")
        print("   请先安装 udev 规则或使用 --port /dev/ttyUSBx")
        sys.exit(1)

    fd = open_raw_serial(args.port, args.baud)
    print(f"监听 {args.port} @ {args.baud} 波特 (Ctrl+C 停止)")
    if args.save:
        print(f"原始字节流保存: {args.save}")
    raw_saved = bytearray() if args.save else None

    parser = MavlinkStreamParser()
    last_seq = None
    last_us = None

    win_start = time.time()
    n_total = n_imu = n_bad_crc = n_seq_gap = n_other = 0
    us_deltas = []          # 本窗口 time_usec 增量
    us_odd = 0              # ≠2000us 的次数
    all_us_odd = 0          # 累计
    all_bad_crc = 0
    all_seq_gap = 0
    all_total = 0

    def flush_window(end=False):
        nonlocal win_start, n_total, n_imu, n_bad_crc, n_seq_gap, n_other
        nonlocal us_deltas, us_odd
        el = time.time() - win_start
        if el <= 0:
            return
        rate = n_imu / el
        line = (f"[t={time.time()-args.seconds:>5.1f}s] MAVLink帧 {n_total:>5} | "
                f"HIGHRES_IMU {n_imu:>5} ({rate:>5.1f} Hz) | CRC坏 {n_bad_crc:>3} | "
                f"SEQ跳 {n_seq_gap:>3} | 其他msg {n_other:>3} | 重同步 {parser.resync_events}")
        print(line)
        if us_deltas:
            mn = min(us_deltas)
            mx = max(us_deltas)
            mean = sum(us_deltas) / len(us_deltas)
            print(f"   time_usec增量: n={len(us_deltas)} mean={mean:.0f}us "
                  f"min={mn}us max={mx}us  ≠2000us: {us_odd}")
        win_start = time.time()
        n_total = n_imu = n_bad_crc = n_seq_gap = n_other = 0
        us_deltas = []
        us_odd = 0

    t0 = time.time()
    try:
        while True:
            try:
                chunk = os.read(fd, 4096)
            except BlockingIOError:
                chunk = b''
            if chunk:
                if raw_saved is not None:
                    raw_saved.extend(chunk)
                for frame, ok, seq, msgid in parser.feed(chunk):
                    n_total += 1
                    all_total += 1
                    if ok is False:
                        n_bad_crc += 1
                        all_bad_crc += 1
                        continue
                    if ok is None:
                        n_other += 1
                        continue
                    if last_seq is not None and seq != ((last_seq + 1) & 0xFF):
                        n_seq_gap += 1
                        all_seq_gap += 1
                    last_seq = seq
                    if msgid == HIGHRES_IMU_MSGID:
                        n_imu += 1
                        us = parse_highres_time_usec(
                            frame[10:10 + HIGHRES_IMU_PAYLOAD_LEN])
                        if us is not None:
                            if last_us is not None and us >= last_us:
                                d = us - last_us
                                us_deltas.append(d)
                                if d != EXPECTED_US_500HZ:
                                    us_odd += 1
                                    all_us_odd += 1
                            last_us = us
            if time.time() - win_start >= 1.0:
                flush_window()
            if args.seconds > 0 and time.time() - t0 >= args.seconds:
                break
    except KeyboardInterrupt:
        print("\n手动停止")
    finally:
        os.close(fd)
        if raw_saved is not None:
            with open(args.save, 'wb') as fp:
                fp.write(raw_saved)
            print(f"已保存 {len(raw_saved)} 字节 → {args.save}")

    print("=" * 66)
    print(f"汇总: 总帧 {all_total} | 累计CRC坏 {all_bad_crc} | "
          f"累计SEQ跳 {all_seq_gap} | 累计节拍异常 {all_us_odd}")
    if all_total == 0:
        print("⚠ 未收到任何数据 — 检查设备/波特率/驱动占用")
    elif all_bad_crc == 0 and all_seq_gap == 0 and all_us_odd == 0:
        print("✅ 串口链路干净: 到达频率即 MCU 实际发送频率")
        print("   → 若频率 <490Hz, 问题在 MCU 发送节拍; 若 ≈500Hz, 问题在主机/ROS驱动")
    elif all_us_odd > 0:
        print("⚠ 检测到 MCU 采样节拍抖动 (time_usec 增量 ≠ 2000us) → MCU 固件调度问题")
    if all_bad_crc or all_seq_gap:
        print("⚠ 串口链路存在丢字节 (CRC坏/SEQ跳) → 电气噪声 / USB FIFO溢出 / 主机读延迟")
    print("=" * 66)


if __name__ == '__main__':
    main()
