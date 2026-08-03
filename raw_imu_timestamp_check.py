#!/usr/bin/env python3
"""
raw_imu_timestamp_check.py — MCU串口原始IMU时间戳连续性验证 (v2.0)
==================================================================
使用 pymavlink 官方库解析 MAVLink v2 HIGHRES_IMU 帧,
验证 time_usec 时间戳是否严格连续 (500Hz -> dt = 2000us).

用法:
  python3 raw_imu_timestamp_check.py                          # 默认 /dev/ttyIMU
  python3 raw_imu_timestamp_check.py --port /dev/ttyACM0     # 指定端口
  python3 raw_imu_timestamp_check.py --duration 30           # 运行30秒
  python3 raw_imu_timestamp_check.py --csv ts_dump.csv       # 导出CSV

依赖: pyserial, pymavlink
  pip install pyserial pymavlink
"""

import sys
import time
import argparse
from collections import deque
from typing import Optional

import serial
import numpy as np
from pymavlink import mavutil

# MAVLink message IDs
MAVLINK_MSG_ID_HIGHRES_IMU = 105


def parse_highres_imu(msg) -> dict:
    """Extract HIGHRES_IMU fields from pymavlink message object."""
    return {
        'time_usec': msg.time_usec,
        'xgyro': msg.xgyro, 'ygyro': msg.ygyro, 'zgyro': msg.zgyro,
        'xacc':  msg.xacc,  'yacc':  msg.yacc,  'zacc':  msg.zacc,
        'temperature': msg.temperature,
    }


def main():
    parser = argparse.ArgumentParser(description="MCU serial IMU timestamp continuity check")
    parser.add_argument('--port', default='/dev/ttyIMU', help='Serial port path')
    parser.add_argument('--baudrate', type=int, default=921600, help='Baud rate')
    parser.add_argument('--duration', type=float, default=0, help='Run duration (s), 0=forever')
    parser.add_argument('--csv', default='', help='CSV output path')
    args = parser.parse_args()

    print(f"Opening {args.port} @ {args.baudrate} ...")
    ser = serial.Serial(args.port, args.baudrate, timeout=0.1)
    print(f"Serial port opened: {ser.name}")

    # Create pymavlink parser (no file binding, manual feed)
    mav = mavutil.mavlink.MAVLink(None)

    # CSV output
    csv_fd = None
    if args.csv:
        csv_fd = open(args.csv, 'w')
        csv_fd.write("idx,time_usec,dt_us,dt_ms,gyro_x,gyro_y,gyro_z,acc_x,acc_y,acc_z,temp\n")

    # Statistics
    last_ts = None
    dt_ring = deque(maxlen=1000)
    discont_count = 0
    frame_count = 0
    imu_count = 0
    other_count = 0
    start_time = time.time()

    print("Reading IMU data via pymavlink... (Ctrl+C to stop)")
    print(f"{'Idx':>6}  {'time_usec':>14}  {'dt_us':>8}  {'dt_ms':>8}  "
          f"{'gyro_x':>10}  {'gyro_y':>10}  {'gyro_z':>10}")
    print("-" * 80)

    try:
        while True:
            if args.duration > 0 and time.time() - start_time > args.duration:
                break

            raw = ser.read(4096)
            if not raw:
                continue

            for byte in raw:
                # pymavlink parse_char expects bytes, not int (Py3 bytes[n] -> int)
                try:
                    msg = mav.parse_char(bytes([byte]))
                except Exception:
                    continue  # CRC mismatch or malformed frame, skip silently
                if msg is None:
                    continue

                frame_count += 1

                if msg.get_msgId() == MAVLINK_MSG_ID_HIGHRES_IMU:
                    imu_count += 1
                    data = parse_highres_imu(msg)
                    ts = data['time_usec']

                    dt_us = 0
                    if last_ts is not None:
                        dt_us = ts - last_ts
                        # Filter obvious parse errors: negative dt or absurdly large jump (>1s)
                        if dt_us < 0 or dt_us > 1000000:
                            discont_count += 1
                            last_ts = ts  # reset baseline, don't poison stats
                            dt_us = 0
                        elif dt_us > 5000:
                            if discont_count < 20:
                                print(f"  !! LARGE dt: {dt_us} us ({dt_us/1000:.1f} ms)")
                            discont_count += 1
                            dt_ring.append(dt_us)
                        else:
                            dt_ring.append(dt_us)
                    last_ts = ts

                    if imu_count <= 30 or imu_count % 200 == 0:
                        print(f"{imu_count:>6}  {ts:>14}  {dt_us:>8}  {dt_us/1000:>8.2f}  "
                              f"{data['xgyro']:>10.6f}  {data['ygyro']:>10.6f}  {data['zgyro']:>10.6f}")

                    if csv_fd:
                        csv_fd.write(
                            f"{imu_count},{ts},{dt_us},{dt_us/1000:.3f},"
                            f"{data['xgyro']},{data['ygyro']},{data['zgyro']},"
                            f"{data['xacc']},{data['yacc']},{data['zacc']},"
                            f"{data['temperature']}\n"
                        )
                else:
                    other_count += 1
                    if other_count <= 5:
                        msglen = getattr(msg, 'get_msglen', lambda: '?')()
                        print(f"  [non-IMU] msgid={msg.get_msgId()} len={msglen}")

    except KeyboardInterrupt:
        pass
    finally:
        elapsed = time.time() - start_time
        print()
        print("=" * 72)
        print("  Raw Serial IMU Timestamp Continuity Check (pymavlink)")
        print("=" * 72)
        print(f"  Duration:     {elapsed:.1f}s")
        if elapsed > 0:
            print(f"  MAVLink frames: {frame_count} ({frame_count/elapsed:.0f} Hz)")
            print(f"  IMU frames:     {imu_count} ({imu_count/elapsed:.0f} Hz)")
        print(f"  Other frames:   {other_count}")

        if dt_ring:
            arr = np.array(dt_ring, dtype=np.float64)
            actual_hz = 1e6 / np.mean(arr) if np.mean(arr) > 0 else 0
            print(f"  " + "-" * 44)
            print(f"  dt mean:  {np.mean(arr):.1f} us ({np.mean(arr)/1000:.2f} ms) -> {actual_hz:.0f} Hz")
            print(f"  dt std:   {np.std(arr):.1f} us")
            print(f"  dt range: [{np.min(arr):.0f}, {np.max(arr):.0f}] us")
            print(f"  Discontinuities: {discont_count}")
            expected_us = 2000  # 500Hz
            if abs(np.mean(arr) - expected_us) < 100:
                print(f"  >> dt matches expected 500Hz (2000us)")
            elif abs(np.mean(arr) - expected_us) < 300:
                print(f"  >> dt slightly off: {np.mean(arr):.0f}us ({actual_hz:.0f}Hz)")
            else:
                print(f"  ** dt DEVIATION: expected 2000us (500Hz), got {np.mean(arr):.0f}us ({actual_hz:.0f}Hz)")
        else:
            print("  ** NO IMU data received!")
            print("     Check: does MCU firmware output MAVLink HIGHRES_IMU (msgid=105)?")

        print("=" * 72)

        ser.close()
        if csv_fd:
            csv_fd.close()
            print(f"CSV saved to {args.csv}")


if __name__ == '__main__':
    main()
