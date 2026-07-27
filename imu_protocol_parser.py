#!/usr/bin/env python3
"""
L431_ADI 二进制通信协议解析器
==============================
协议版本 v2.0  |  物理层: UART 921600-8-N-1  |  帧头: AA 55

功能:
  - 从串口 (TTY USB) 实时读取并解析 IMU 数据流
  - 支持 6 种帧类型: IMU_DATA, STATUS, HEARTBEAT, DEBUG, COMMAND, CMD_RESPONSE
  - CRC-8/DALLAS 校验, 状态机帧同步
  - 原始传感器值 → 物理量自动转换 (需收到 STATUS 帧的标定参数)
  - 可选 CSV 导出
  - 可向上位机发送标定命令

用法:
  python imu_protocol_parser.py                          # 自动检测 /dev/ttyUSB*
  python imu_protocol_parser.py --port /dev/ttyUSB0     # 指定串口
  python imu_protocol_parser.py --port /dev/ttyUSB0 --csv data.csv
  python imu_protocol_parser.py --port /dev/ttyUSB0 --cmd 0x10 0x01
  python imu_protocol_parser.py --list                   # 列出可用串口
  python imu_protocol_parser.py --self-test              # CRC 自检

依赖: pyserial
  pip install pyserial
"""

import sys
import struct
import time
import argparse
import math
import os
import glob
from dataclasses import dataclass
from typing import Optional, Tuple, Callable, Dict, List

# ═══════════════════════════════════════════════════════════════════════
# CRC-8/DALLAS — 多项式 0x31, 初始值 0xFF, 无反射
# 查找表与 MCU 端 protocol.c 完全一致
# ═══════════════════════════════════════════════════════════════════════

CRC8_TABLE: List[int] = [
    0x00,0x31,0x62,0x53,0xC4,0xF5,0xA6,0x97,0xB9,0x88,0xDB,0xEA,0x7D,0x4C,0x1F,0x2E,
    0x43,0x72,0x21,0x10,0x87,0xB6,0xE5,0xD4,0xFA,0xCB,0x98,0xA9,0x3E,0x0F,0x5C,0x6D,
    0x86,0xB7,0xE4,0xD5,0x42,0x73,0x20,0x11,0x3F,0x0E,0x5D,0x6C,0xFB,0xCA,0x99,0xA8,
    0xC5,0xF4,0xA7,0x96,0x01,0x30,0x63,0x52,0x7C,0x4D,0x1E,0x2F,0xB8,0x89,0xDA,0xEB,
    0x3D,0x0C,0x5F,0x6E,0xF9,0xC8,0x9B,0xAA,0x84,0xB5,0xE6,0xD7,0x40,0x71,0x22,0x13,
    0x7E,0x4F,0x1C,0x2D,0xBA,0x8B,0xD8,0xE9,0xC7,0xF6,0xA5,0x94,0x03,0x32,0x61,0x50,
    0xBB,0x8A,0xD9,0xE8,0x7F,0x4E,0x1D,0x2C,0x02,0x33,0x60,0x51,0xC6,0xF7,0xA4,0x95,
    0xF8,0xC9,0x9A,0xAB,0x3C,0x0D,0x5E,0x6F,0x41,0x70,0x23,0x12,0x85,0xB4,0xE7,0xD6,
    0x7A,0x4B,0x18,0x29,0xBE,0x8F,0xDC,0xED,0xC3,0xF2,0xA1,0x90,0x07,0x36,0x65,0x54,
    0x39,0x08,0x5B,0x6A,0xFD,0xCC,0x9F,0xAE,0x80,0xB1,0xE2,0xD3,0x44,0x75,0x26,0x17,
    0xFC,0xCD,0x9E,0xAF,0x38,0x09,0x5A,0x6B,0x45,0x74,0x27,0x16,0x81,0xB0,0xE3,0xD2,
    0xBF,0x8E,0xDD,0xEC,0x7B,0x4A,0x19,0x28,0x06,0x37,0x64,0x55,0xC2,0xF3,0xA0,0x91,
    0x47,0x76,0x25,0x14,0x83,0xB2,0xE1,0xD0,0xFE,0xCF,0x9C,0xAD,0x3A,0x0B,0x58,0x69,
    0x04,0x35,0x66,0x57,0xC0,0xF1,0xA2,0x93,0xBD,0x8C,0xDF,0xEE,0x79,0x48,0x1B,0x2A,
    0xC1,0xF0,0xA3,0x92,0x05,0x34,0x67,0x56,0x78,0x49,0x1A,0x2B,0xBC,0x8D,0xDE,0xEF,
    0x82,0xB3,0xE0,0xD1,0x46,0x77,0x24,0x15,0x3B,0x0A,0x59,0x68,0xFF,0xCE,0x9D,0xAC,
]


def crc8(data: bytes) -> int:
    """CRC-8/DALLAS — 与 MCU 端 protocol.c 完全一致."""
    c = 0xFF
    for b in data:
        c = CRC8_TABLE[c ^ b]
    return c


# ═══════════════════════════════════════════════════════════════════════
# 协议常量
# ═══════════════════════════════════════════════════════════════════════

SYNC0, SYNC1 = 0xAA, 0x55
SYNC_BYTES = bytes([SYNC0, SYNC1])

TYPE_DEBUG     = 0x00
TYPE_IMU_DATA  = 0x01
TYPE_HEARTBEAT = 0x02
TYPE_STATUS    = 0x03
TYPE_COMMAND   = 0x04
TYPE_CMD_RESP  = 0x05

MAX_PAYLOAD = 128
RAD2DEG = 180.0 / math.pi

TYPE_NAMES: Dict[int, str] = {
    0x00: "DEBUG", 0x01: "IMU_DATA", 0x02: "HEARTBEAT",
    0x03: "STATUS", 0x04: "COMMAND", 0x05: "CMD_RESPONSE",
}

STATE_NAMES: Dict[int, str] = {
    0x00: "GYRO_CAL", 0x01: "RUNNING", 0x02: "ACAL_WAIT",
    0x03: "ACAL_RUN", 0x04: "ACAL_OK", 0x05: "ACAL_FAIL",
}

CMD_RESULT: Dict[int, str] = {
    0x00: "OK", 0x01: "ERR_BUSY", 0x02: "ERR_STATE", 0x03: "ERR_UNKNOWN_CMD",
}


# ═══════════════════════════════════════════════════════════════════════
# 数据结构
# ═══════════════════════════════════════════════════════════════════════

@dataclass
class ImuRaw:
    """IMU_DATA 帧 — 原始传感器值 (28 字节载荷)."""
    counter: int    # uint16 BE
    gx: int         # int32 LE, X轴陀螺
    gy: int         # int32 LE, Y轴陀螺
    gz: int         # int32 LE, Z轴陀螺
    ax: int         # int32 LE, X轴加速度计
    ay: int         # int32 LE, Y轴加速度计
    az: int         # int32 LE, Z轴加速度计
    temp: int       # int16 LE, 温度


@dataclass
class ImuPhys:
    """IMU 数据 — 物理单位."""
    counter: int
    gx_dps: float     # deg/s
    gy_dps: float
    gz_dps: float
    ax_ms2: float     # m/s²
    ay_ms2: float
    az_ms2: float
    temp_c: float     # °C


@dataclass
class Calib:
    """标定参数 (由 STATUS 帧更新)."""
    gyro_scale: float = 0.0
    accel_scale: float = 0.0
    temp_scale: float = 0.1
    gbias: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    aoffs: Tuple[float, float, float] = (0.0, 0.0, 0.0)


# ═══════════════════════════════════════════════════════════════════════
# 帧解析器 — 字节级状态机
# ═══════════════════════════════════════════════════════════════════════

class FrameParser:
    """L431_ADI 协议帧解析器.

    用法:
        parser = FrameParser()
        parser.on_imu_phys = lambda imu: print(f"gx={imu.gx_dps:.3f}")
        for byte in serial_stream:
            parser.feed(byte)
    """

    def __init__(self):
        self._state = 'SYNC0'
        self._buf = bytearray()
        self._typ = 0
        self._len = 0

        self.calib = Calib()

        # 统计
        self.frame_count = 0
        self.crc_err_count = 0
        self.sync_loss_count = 0
        self.type_counts: Dict[int, int] = {}
        self.byte_count = 0
        self.t0 = time.time()

        # 用户回调
        self.on_imu_raw: Optional[Callable[[ImuRaw], None]] = None
        self.on_imu_phys: Optional[Callable[[ImuPhys], None]] = None
        self.on_status: Optional[Callable[[dict], None]] = None
        self.on_heartbeat: Optional[Callable[[], None]] = None
        self.on_debug: Optional[Callable[[str], None]] = None
        self.on_cmd_resp: Optional[Callable[[int, int, str], None]] = None
        self.on_bad_frame: Optional[Callable[[bytes], None]] = None
        self.on_any_frame: Optional[Callable[[bytes], None]] = None

    # ── 喂入字节 ──────────────────────────────────────────────────

    def feed(self, byte: int) -> None:
        b = byte
        if self._state == 'SYNC0':
            if b == SYNC0:
                self._state = 'SYNC1'
        elif self._state == 'SYNC1':
            if b == SYNC1:
                self._state = 'TYPE'
            elif b != SYNC0:
                self._state = 'SYNC0'
        elif self._state == 'TYPE':
            self._typ = b
            self._buf = bytearray([b])
            self._state = 'LEN'
        elif self._state == 'LEN':
            if b > MAX_PAYLOAD:
                self.sync_loss_count += 1
                self._state = 'SYNC0'
                return
            self._len = b
            self._buf.append(b)
            self._state = 'CRC' if b == 0 else 'PAYLOAD'
        elif self._state == 'PAYLOAD':
            self._buf.append(b)
            if len(self._buf) - 2 >= self._len:
                self._state = 'CRC'
        elif self._state == 'CRC':
            self._on_crc(b)
            self._state = 'SYNC0'

    def feed_bytes(self, data: bytes) -> None:
        for b in data:
            self.feed(b)

    # ── CRC 校验 & 分发 ───────────────────────────────────────────

    def _on_crc(self, crc_rcv: int) -> None:
        crc_calc = crc8(bytes(self._buf))
        payload = bytes(self._buf[2:])

        self.frame_count += 1
        self.byte_count += 5 + self._len
        self.type_counts[self._typ] = self.type_counts.get(self._typ, 0) + 1

        raw_frame = SYNC_BYTES + bytes(self._buf) + bytes([crc_rcv])

        if self.on_any_frame:
            self.on_any_frame(raw_frame)

        if crc_calc != crc_rcv:
            self.crc_err_count += 1
            if self.on_bad_frame:
                self.on_bad_frame(raw_frame)
            return

        self._dispatch(self._typ, payload)

    def _dispatch(self, typ: int, payload: bytes) -> None:
        try:
            if typ == TYPE_IMU_DATA and len(payload) >= 28:
                raw = self._parse_imu(payload)
                if self.on_imu_raw:
                    self.on_imu_raw(raw)
                if self.on_imu_phys:
                    self.on_imu_phys(self._raw_to_phys(raw))

            elif typ == TYPE_STATUS and len(payload) >= 40:
                st = self._parse_status(payload)
                self.calib.gyro_scale = st['gyro_scale']
                self.calib.accel_scale = st['accel_scale']
                self.calib.temp_scale = st['temp_scale']
                self.calib.gbias = (st['gbias_x'], st['gbias_y'], st['gbias_z'])
                self.calib.aoffs = (st['aoffs_x'], st['aoffs_y'], st['aoffs_z'])
                if self.on_status:
                    self.on_status(st)

            elif typ == TYPE_HEARTBEAT:
                if self.on_heartbeat:
                    self.on_heartbeat()

            elif typ == TYPE_DEBUG:
                text = payload.decode('ascii', errors='replace').rstrip('\x00')
                if self.on_debug:
                    self.on_debug(text)

            elif typ == TYPE_CMD_RESP and len(payload) >= 2:
                if self.on_cmd_resp:
                    self.on_cmd_resp(payload[0], payload[1],
                                     CMD_RESULT.get(payload[1], f"0x{payload[1]:02X}"))
        except Exception:
            pass

    # ── 各类型解析器 ──────────────────────────────────────────────

    @staticmethod
    def _parse_imu(payload: bytes) -> ImuRaw:
        return ImuRaw(
            counter = struct.unpack('>H',  payload[0:2])[0],
            gx      = struct.unpack('<i',  payload[2:6])[0],
            gy      = struct.unpack('<i',  payload[6:10])[0],
            gz      = struct.unpack('<i',  payload[10:14])[0],
            ax      = struct.unpack('<i',  payload[14:18])[0],
            ay      = struct.unpack('<i',  payload[18:22])[0],
            az      = struct.unpack('<i',  payload[22:26])[0],
            temp    = struct.unpack('<h',  payload[26:28])[0],
        )

    @staticmethod
    def _parse_status(payload: bytes) -> dict:
        return {
            'state':        payload[0],
            'state_name':   STATE_NAMES.get(payload[0], f"0x{payload[0]:02X}"),
            'cal_progress': payload[1],
            'cpu_load':     payload[2],
            'gyro_scale':   struct.unpack('<f', payload[3:7])[0],
            'accel_scale':  struct.unpack('<f', payload[7:11])[0],
            'temp_scale':   struct.unpack('<f', payload[11:15])[0],
            'gbias_x':      struct.unpack('<f', payload[15:19])[0],
            'gbias_y':      struct.unpack('<f', payload[19:23])[0],
            'gbias_z':      struct.unpack('<f', payload[23:27])[0],
            'aoffs_x':      struct.unpack('<f', payload[27:31])[0],
            'aoffs_y':      struct.unpack('<f', payload[31:35])[0],
            'aoffs_z':      struct.unpack('<f', payload[35:39])[0],
            'moving':       payload[39] != 0,
        }

    def _raw_to_phys(self, raw: ImuRaw) -> ImuPhys:
        c = self.calib
        return ImuPhys(
            counter = raw.counter,
            gx_dps  = raw.gx * c.gyro_scale * RAD2DEG - c.gbias[0],
            gy_dps  = raw.gy * c.gyro_scale * RAD2DEG - c.gbias[1],
            gz_dps  = raw.gz * c.gyro_scale * RAD2DEG - c.gbias[2],
            ax_ms2  = raw.ax * c.accel_scale - c.aoffs[0],
            ay_ms2  = raw.ay * c.accel_scale - c.aoffs[1],
            az_ms2  = raw.az * c.accel_scale - c.aoffs[2],
            temp_c  = raw.temp * c.temp_scale,
        )

    @property
    def elapsed(self) -> float:
        return time.time() - self.t0

    @property
    def fps(self) -> float:
        e = self.elapsed
        return self.frame_count / e if e > 0 else 0


# ═══════════════════════════════════════════════════════════════════════
# 帧构建 (上位机 → MCU 命令)
# ═══════════════════════════════════════════════════════════════════════

def build_frame(typ: int, payload: bytes = b'') -> bytes:
    """构建完整帧 (含 SYNC + CRC)."""
    header = bytes([typ, len(payload)])
    return SYNC_BYTES + header + payload + bytes([crc8(header + payload)])


def build_command(cmd_id: int, param: int = 0) -> bytes:
    """构建 COMMAND 帧 (7 字节)."""
    return build_frame(TYPE_COMMAND, bytes([cmd_id, param]))


# ═══════════════════════════════════════════════════════════════════════
# 串口工具
# ═══════════════════════════════════════════════════════════════════════

def list_serial_ports() -> List[str]:
    """列出可用的串口设备."""
    ports = []
    for pat in ['/dev/ttyUSB*', '/dev/ttyACM*', '/dev/ttyS*',
                '/dev/tty.usb*', '/dev/cu.usb*']:
        ports.extend(glob.glob(pat))
    return sorted(set(ports))


def auto_detect_port() -> Optional[str]:
    """自动检测第一个可用 TTY USB 串口."""
    ports = list_serial_ports()
    if ports:
        return ports[0]
    return None


# ═══════════════════════════════════════════════════════════════════════
# 终端显示
# ═══════════════════════════════════════════════════════════════════════

class Display:
    """格式化终端输出 — 紧凑模式: STATUS 为单行, 详细块仅在状态变化时打印."""

    def __init__(self, show_raw: bool = False, show_phys: bool = True,
                 rate_hz: float = 10.0, show_hex: bool = False, quiet: bool = False,
                 verbose_status: bool = False):
        self.show_raw = show_raw
        self.show_phys = show_phys
        self.rate_hz = rate_hz
        self.show_hex = show_hex
        self.quiet = quiet
        self.verbose_status = verbose_status
        self._last = 0.0
        self._ival = 1.0 / rate_hz if rate_hz > 0 else 0.0
        self._skip = 0
        self._hb = 0

        # 跟踪上一次 STATUS 的关键字段, 变化时打印详细块
        self._last_state = -1
        self._last_cal = -1
        self._last_moving = -1
        self._status_printed_once = False

        # 攒一行 IMU+status 合并输出
        self._pending_imu: Optional[ImuPhys] = None
        self._pending_status: Optional[dict] = None

    def on_imu_raw(self, raw: ImuRaw) -> None:
        if not self.show_raw or self.quiet or not self._throttle():
            return
        print(f" RAW[{raw.counter:5d}] "
              f"g=({raw.gx:>12d},{raw.gy:>12d},{raw.gz:>12d}) "
              f"a=({raw.ax:>12d},{raw.ay:>12d},{raw.az:>12d}) "
              f"t={raw.temp:>6d}")

    def on_imu_phys(self, imu: ImuPhys) -> None:
        if not self.show_phys or self.quiet:
            return
        if not self._throttle():
            return
        self._pending_imu = imu
        self._flush()

    def on_status(self, st: dict) -> None:
        if self.quiet:
            return
        self._pending_status = st

        # 检测状态变化 → 打印详细块
        changed = (st['state'] != self._last_state or
                   st['cal_progress'] != self._last_cal or
                   st['moving'] != self._last_moving)
        if changed or not self._status_printed_once or self.verbose_status:
            self._print_status_block(st)
            self._last_state = st['state']
            self._last_cal = st['cal_progress']
            self._last_moving = st['moving']
            self._status_printed_once = True

        self._flush()

    def on_heartbeat(self) -> None:
        self._hb += 1
        if self.quiet:
            return
        c = '\033[2m'
        if self._hb % 10 == 0:
            print(f"{c}♥{self._hb}\033[0m", end='', flush=True)
        else:
            print(f"{c}.\033[0m", end='', flush=True)

    def on_debug(self, text: str) -> None:
        if self.quiet:
            return
        print(f"\n\033[92m[DEBUG] {text}\033[0m")

    def on_cmd_resp(self, cmd_id: int, result: int, name: str) -> None:
        c = '\033[92m' if result == 0 else '\033[91m'
        print(f"\n{c}[CMD_RESP] cmd=0x{cmd_id:02X} → {name}\033[0m")

    # ── 内部 ──────────────────────────────────────────────────────

    def _flush(self) -> None:
        """合并 IMU + STATUS 为一行输出."""
        if self._pending_imu is None:
            return
        imu = self._pending_imu
        st = self._pending_status
        self._pending_imu = None
        self._pending_status = None

        extra = f" \033[2m(+{self._skip})\033[0m" if self._skip else ""
        self._skip = 0

        # 基础 IMU 行
        line = (f" IMU[{imu.counter:5d}] "
                f"g=[\033[96m{imu.gx_dps:8.3f}\033[0m,"
                f"\033[96m{imu.gy_dps:8.3f}\033[0m,"
                f"\033[96m{imu.gz_dps:8.3f}\033[0m] "
                f"a=[\033[93m{imu.ax_ms2:7.3f}\033[0m,"
                f"\033[93m{imu.ay_ms2:7.3f}\033[0m,"
                f"\033[93m{imu.az_ms2:7.3f}\033[0m] "
                f"t={imu.temp_c:5.1f}°C")

        # 附加紧凑 STATUS 信息
        if st:
            cal_str = "idle" if st['cal_progress'] == 255 else f"{st['cal_progress']}%"
            move_str = "\033[91mMOV\033[0m" if st['moving'] else "stl"
            line += (f"  \033[2m│\033[0m "
                     f"\033[94m{st['state_name']}\033[0m "
                     f"cpu:{st['cpu_load']}% "
                     f"cal:{cal_str} "
                     f"[{move_str}]")

        line += extra
        print(line)

    def _print_status_block(self, st: dict) -> None:
        """打印详细 STATUS 块 (状态变化时)."""
        m = "\033[91mMOVING\033[0m" if st['moving'] else "still"
        cal_str = "idle" if st['cal_progress'] == 255 else f"{st['cal_progress']}%"
        print(f"\n\033[1m── STATUS ────────────────────────────────────\033[0m")
        print(f"  State: \033[94m{st['state_name']}\033[0m  "
              f"Cal: {cal_str}  CPU: {st['cpu_load']}%  [{m}]")
        print(f"  Gyro scale:  {st['gyro_scale']:.6e} rad/s/LSB")
        print(f"  Accel scale: {st['accel_scale']:.6e} m/s²/LSB")
        print(f"  Gyro bias:   ({st['gbias_x']:+.6f}, {st['gbias_y']:+.6f}, {st['gbias_z']:+.6f}) deg/s")
        print(f"  Accel offs:  ({st['aoffs_x']:+.6f}, {st['aoffs_y']:+.6f}, {st['aoffs_z']:+.6f}) m/s²")
        print(f"\033[2m──────────────────────────────────────────────\033[0m\n")

    def _throttle(self) -> bool:
        now = time.time()
        if self._ival <= 0:
            return True
        if now - self._last >= self._ival:
            self._skip = 0
            self._last = now
            return True
        self._skip += 1
        return False


# ═══════════════════════════════════════════════════════════════════════
# CSV 输出
# ═══════════════════════════════════════════════════════════════════════

class CsvWriter:
    """IMU 数据 CSV 记录器."""

    HEADER = ("timestamp,counter,"
              "gx_dps,gy_dps,gz_dps,"
              "ax_ms2,ay_ms2,az_ms2,"
              "temp_c,"
              "raw_gx,raw_gy,raw_gz,"
              "raw_ax,raw_ay,raw_az,"
              "raw_temp")

    def __init__(self, path: str, append: bool = False):
        exists = os.path.exists(path) and os.path.getsize(path) > 0
        self._f = open(path, 'a' if append else 'w')
        if not append or not exists:
            self._f.write(self.HEADER + '\n')
        self._t0 = time.time()
        self._last_raw: Optional[ImuRaw] = None

    def on_imu_raw(self, raw: ImuRaw) -> None:
        self._last_raw = raw

    def on_imu_phys(self, phys: ImuPhys) -> None:
        r = self._last_raw
        row = (f"{time.time() - self._t0:.6f},{phys.counter},"
               f"{phys.gx_dps:.6f},{phys.gy_dps:.6f},{phys.gz_dps:.6f},"
               f"{phys.ax_ms2:.6f},{phys.ay_ms2:.6f},{phys.az_ms2:.6f},"
               f"{phys.temp_c:.2f}")
        if r:
            row += f",{r.gx},{r.gy},{r.gz},{r.ax},{r.ay},{r.az},{r.temp}"
        self._f.write(row + '\n')

    def close(self) -> None:
        self._f.close()


# ═══════════════════════════════════════════════════════════════════════
# 主逻辑
# ═══════════════════════════════════════════════════════════════════════

def run_serial(port: str, baudrate: int = 921600,
               csv_path: Optional[str] = None,
               show_raw: bool = False, show_phys: bool = True,
               rate_hz: float = 10.0, show_hex: bool = False,
               quiet: bool = False, timeout: float = 1.0,
               verbose_status: bool = False) -> None:
    """从串口读取数据流, 解析并打印."""

    try:
        import serial
    except ImportError:
        print("\033[91m缺少 pyserial 库.\033[0m 请运行: pip install pyserial", file=sys.stderr)
        sys.exit(1)

    display = Display(show_raw=show_raw, show_phys=show_phys,
                      rate_hz=rate_hz, show_hex=show_hex, quiet=quiet,
                      verbose_status=verbose_status)

    parser = FrameParser()
    parser.on_imu_raw = display.on_imu_raw
    parser.on_imu_phys = display.on_imu_phys
    parser.on_status = display.on_status
    parser.on_heartbeat = display.on_heartbeat
    parser.on_debug = display.on_debug
    parser.on_cmd_resp = display.on_cmd_resp

    if show_hex:
        def _hex(raw: bytes) -> None:
            print(f"  HEX: {raw.hex(' ')}")
        parser.on_any_frame = _hex

    csv = None
    if csv_path:
        csv = CsvWriter(csv_path)
        # 保留显示回调的同时也写 CSV
        orig_raw = parser.on_imu_raw
        orig_phys = parser.on_imu_phys
        parser.on_imu_raw = lambda r: (csv.on_imu_raw(r), orig_raw(r) if orig_raw else None)
        parser.on_imu_phys = lambda p: (csv.on_imu_phys(p), orig_phys(p) if orig_phys else None)

    print(f"╔══════════════════════════════════════════════╗")
    print(f"║  L431_ADI 协议解析器 v2.0                    ║")
    print(f"╠══════════════════════════════════════════════╣")
    print(f"║  Port: {port:<37s} ║")
    print(f"║  Baud: {baudrate:<37d} ║")
    if csv_path:
        print(f"║  CSV:  {csv_path:<37s} ║")
    print(f"╚══════════════════════════════════════════════╝")
    print(f"  等待数据… (按 Ctrl+C 停止)\n")

    try:
        with serial.Serial(port, baudrate, timeout=timeout) as ser:
            ser.reset_input_buffer()
            while True:
                n = ser.in_waiting
                if n > 0:
                    parser.feed_bytes(ser.read(n))
                else:
                    b = ser.read(1)
                    if b:
                        parser.feed(b[0])

    except KeyboardInterrupt:
        print(f"\n\n⏹  用户停止.")
    except serial.SerialException as e:
        print(f"\n\033[91m串口错误:\033[0m {e}", file=sys.stderr)
        sys.exit(1)
    finally:
        if csv:
            csv.close()
        _print_stats(parser)


def send_cmd(port: str, cmd_id: int, param: int = 0,
             baudrate: int = 921600, timeout: float = 3.0) -> None:
    """发送命令并等待应答."""
    try:
        import serial
    except ImportError:
        print("\033[91m缺少 pyserial.\033[0m 请运行: pip install pyserial", file=sys.stderr)
        sys.exit(1)

    resp: list = []

    def on_resp(cid, res, name):
        resp.append((cid, res, name))

    parser = FrameParser()
    parser.on_cmd_resp = on_resp

    frame = build_command(cmd_id, param)
    print(f"发送: cmd_id=0x{cmd_id:02X} param=0x{param:02X}")
    print(f"帧:   {frame.hex(' ').upper()}")

    with serial.Serial(port, baudrate, timeout=0.1) as ser:
        ser.reset_input_buffer()
        ser.write(frame)
        ser.flush()

        deadline = time.time() + timeout
        while time.time() < deadline:
            n = ser.in_waiting
            if n:
                parser.feed_bytes(ser.read(n))
                if resp:
                    cid, res, name = resp[0]
                    mark = '\033[92m✓\033[0m' if res == 0 else '\033[91m✗\033[0m'
                    print(f"{mark} 应答: cmd=0x{cid:02X} result={name}")
                    return
            time.sleep(0.01)

    print("\033[93m⏱ 超时:\033[0m 未收到应答")


def _print_stats(p: FrameParser) -> None:
    e = p.elapsed
    ok = p.frame_count - p.crc_err_count
    print(f"\n{'─' * 50}")
    print(f"  统计")
    print(f"{'─' * 50}")
    print(f"  用时:        {e:.1f}s")
    print(f"  总帧:        {p.frame_count}  (有效: {ok}, CRC错误: {p.crc_err_count})")
    print(f"  总字节:      {p.byte_count:,}")
    if e > 0:
        print(f"  帧率:        {p.fps:.1f} fps")
        print(f"  带宽:        {p.byte_count * 8 / e / 1000:.1f} kbps")
    if p.type_counts:
        print(f"  帧类型分布:")
        for t, n in sorted(p.type_counts.items()):
            name = TYPE_NAMES.get(t, f"0x{t:02X}")
            pct = n / p.frame_count * 100 if p.frame_count else 0
            print(f"    0x{t:02X} {name:<12s} {n:>8d} ({pct:5.1f}%)")
    print(f"{'─' * 50}")


# ═══════════════════════════════════════════════════════════════════════
# 自检
# ═══════════════════════════════════════════════════════════════════════

def self_test() -> bool:
    """验证 CRC 实现和帧解析正确性."""
    ok = True

    # 1. CRC 表 vs 位运算
    for i in range(256):
        # 位运算 CRC-8: poly=0x31, init=0xFF, 单字节
        crc = 0xFF
        crc ^= i
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x31) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
        # 查表法: 单字节 CRC 即 table[init ^ byte]
        if CRC8_TABLE[0xFF ^ i] != crc:
            print(f"  ❌ CRC 表不一致 @ i=0x{i:02X}")
            ok = False
            break
    else:
        print("  ✓ CRC-8 查找表验证通过")

    # 2. 帧构建 → 解析 往返
    if ok:
        parser = FrameParser()
        results: List[ImuPhys] = []
        parser.on_imu_phys = lambda p: results.append(p)

        raw = ImuRaw(counter=42, gx=1000, gy=-2000, gz=3000,
                     ax=40000, ay=-50000, az=60000, temp=250)
        pl = (struct.pack('>H', raw.counter) +
              struct.pack('<i', raw.gx) + struct.pack('<i', raw.gy) +
              struct.pack('<i', raw.gz) + struct.pack('<i', raw.ax) +
              struct.pack('<i', raw.ay) + struct.pack('<i', raw.az) +
              struct.pack('<h', raw.temp))
        frame = build_frame(TYPE_IMU_DATA, pl)
        parser.feed_bytes(frame)

        if len(results) != 1:
            print(f"  ❌ 帧解析失败: 预期 1, 实际 {len(results)}")
            ok = False
        else:
            print(f"  ✓ 帧往返测试通过 (counter={results[0].counter})")

    # 3. CRC 错误检测
    if ok:
        p2 = FrameParser()
        bad_cnt = [0]
        p2.on_bad_frame = lambda _: bad_cnt.__setitem__(0, bad_cnt[0] + 1)
        bad = bytearray(frame)
        bad[-1] ^= 0x01
        p2.feed_bytes(bytes(bad))
        if bad_cnt[0] != 1:
            print(f"  ❌ CRC 错误检测失败")
            ok = False
        else:
            print(f"  ✓ CRC 错误检测正常")

    # 4. 同步恢复 — 在合法帧前后插入垃圾字节
    if ok:
        p3 = FrameParser()
        cnt = [0]
        p3.on_imu_phys = lambda _: cnt.__setitem__(0, cnt[0] + 1)
        # 垃圾: 不含完整 AA 55 序列, 不会触发假 TYPE/LEN
        garbage = bytes([0x00, 0xAA, 0x00, 0x55, 0xFF, 0xAA, 0xCC, 0xDD])
        p3.feed_bytes(garbage)
        p3.feed_bytes(frame)          # 合法帧 1
        p3.feed_bytes(bytes([0xEE, 0x11, 0x22]))  # 帧间垃圾
        p3.feed_bytes(frame)          # 合法帧 2
        if cnt[0] != 2:
            print(f"  ❌ 同步恢复失败: 预期 2, 实际 {cnt[0]}")
            ok = False
        else:
            print(f"  ✓ 同步恢复正常")

    if ok:
        print(f"\n  ✅ 所有自检通过!")
    else:
        print(f"\n  ❌ 自检失败!")
    return ok


# ═══════════════════════════════════════════════════════════════════════
# CLI 入口
# ═══════════════════════════════════════════════════════════════════════

def main():
    ap = argparse.ArgumentParser(
        description="L431_ADI 二进制协议解析器 — IMU 数据流实时解析",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  %(prog)s                                  自动检测 /dev/ttyUSB*
  %(prog)s --port /dev/ttyUSB0             指定串口
  %(prog)s --port /dev/ttyUSB0 --csv d.csv 保存 CSV
  %(prog)s --port /dev/ttyUSB0 --raw       显示原始传感器值
  %(prog)s --port /dev/ttyUSB0 --rate 1    1Hz 终端打印
  %(prog)s --port /dev/ttyUSB0 --cmd 0x10 0x01  发送标定命令
  %(prog)s --list                          列出可用串口
  %(prog)s --self-test                     CRC & 帧解析自检
        """,
    )

    ap.add_argument('--port', '-p', type=str, default=None,
                    help='串口路径 (如 /dev/ttyUSB0), 不指定则自动检测')
    ap.add_argument('--baudrate', '-b', type=int, default=921600,
                    help='波特率 (默认 921600)')
    ap.add_argument('--csv', '-c', type=str, default=None,
                    help='CSV 输出路径')
    ap.add_argument('--raw', '-r', action='store_true',
                    help='打印原始传感器值')
    ap.add_argument('--no-phys', '-n', action='store_true',
                    help='不打印物理量')
    ap.add_argument('--rate', type=float, default=10.0,
                    help='IMU 终端打印频率 Hz (默认 10, 0=全打)')
    ap.add_argument('--hex', action='store_true',
                    help='打印每帧原始十六进制')
    ap.add_argument('--quiet', '-q', action='store_true',
                    help='安静模式 (仅结束时打印统计)')
    ap.add_argument('--verbose-status', '-v', action='store_true',
                    help='每次收到 STATUS 都打印详细块 (默认仅在变化时打印)')
    ap.add_argument('--cmd', type=str, nargs=2, metavar=('CMD_ID', 'PARAM'),
                    help='发送命令后退出 (如 0x10 0x01)')
    ap.add_argument('--list', action='store_true',
                    help='列出可用串口并退出')
    ap.add_argument('--self-test', action='store_true',
                    help='运行自检并退出')

    args = ap.parse_args()

    if args.self_test:
        ok = self_test()
        sys.exit(0 if ok else 1)

    if args.list:
        ports = list_serial_ports()
        if ports:
            print("可用串口:")
            for p in ports:
                print(f"  {p}")
        else:
            print("未检测到串口设备")
            print("(Linux 常见: /dev/ttyUSB0, /dev/ttyACM0)")
        return

    port = args.port or auto_detect_port()
    if not port:
        print("\033[91m未指定串口且无法自动检测.\033[0m")
        print("请使用 --port 指定 或 --list 列出可用串口")
        sys.exit(1)

    if args.cmd:
        try:
            cid = int(args.cmd[0], 16) if 'x' in args.cmd[0].lower() else int(args.cmd[0])
            par = int(args.cmd[1], 16) if 'x' in args.cmd[1].lower() else int(args.cmd[1])
        except ValueError:
            print("\033[91mCMD_ID/PARAM 格式错误\033[0m (如 0x10 0x01)")
            sys.exit(1)
        send_cmd(port, cid, par, baudrate=args.baudrate)
        return

    run_serial(port=port, baudrate=args.baudrate,
               csv_path=args.csv,
               show_raw=args.raw,
               show_phys=not args.no_phys,
               rate_hz=args.rate if not args.quiet else 0,
               show_hex=args.hex,
               quiet=args.quiet,
               verbose_status=args.verbose_status)


if __name__ == '__main__':
    main()
