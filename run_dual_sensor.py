#!/usr/bin/env python3
"""
Windows 原生串口版：M1616M 压力 + TB100 IMU 解包与可视化（简化版）

- 仅支持 Windows COM 口
- 压力：16x16 实时热力图（matplotlib）
- IMU：解析加速度与四元数
- 同步：按主机接收时间最近邻对齐
"""

from __future__ import annotations

import argparse
import csv
import os
import struct
import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Deque, List, Optional, Tuple

import matplotlib.pyplot as plt
import numpy as np
import serial


# ---------------------------- 数据结构 ----------------------------

@dataclass
class PressureFrame:
    host_ts: float
    values: np.ndarray  # (16,16), g
    checksum_ok: bool


@dataclass
class ImuFrame:
    host_ts: float
    device_ts_us: int
    qos: int
    quat_wxyz: Tuple[float, float, float, float]
    acc_g: Tuple[float, float, float]
    crc_ok: bool
    simple_sum_ok: bool


# ---------------------------- 工具 ----------------------------

def modbus_crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if (crc & 1) else (crc >> 1)
    return crc & 0xFFFF


def orient_pressure(mat: np.ndarray, transpose: bool, flipud: bool, fliplr: bool) -> np.ndarray:
    out = mat
    if transpose:
        out = out.T
    if flipud:
        out = np.flipud(out)
    if fliplr:
        out = np.fliplr(out)
    return out


def nearest_imu(ts: float, imu_buf: Deque[ImuFrame], max_dt_sec: float) -> Optional[ImuFrame]:
    if not imu_buf:
        return None
    best = min(imu_buf, key=lambda x: abs(x.host_ts - ts))
    return best if abs(best.host_ts - ts) <= max_dt_sec else None


def pressure_range_to_setf(pressure_range: str) -> Optional[str]:
    mapping = {
        "1kg": "SETF=5",
        "3kg": "SETF=1",
        "5kg": "SETF=6",
        "10kg": "SETF=7",
        "20kg": "SETF=2",
        "30kg": "SETF=4",
        "50kg": "SETF=3",
    }
    if pressure_range == "skip":
        return None
    return mapping[pressure_range]


def configure_pressure_sensor_range(ser: serial.Serial, pressure_range: str, suffix: bytes) -> None:
    cmd_setf = pressure_range_to_setf(pressure_range)
    if cmd_setf is None:
        print("[PRESSURE] Skip range config")
        return

    cmds = [cmd_setf.encode("ascii") + suffix, b"SET=OK" + suffix]
    for cmd in cmds:
        ser.write(cmd)
        ser.flush()
        time.sleep(0.12)

    print(f"[PRESSURE] Range set request sent: {pressure_range} ({cmd_setf}) + SET=OK")


# ---------------------------- 协议解析 ----------------------------

class M1616MParser:
    HEADER = b"\xAA\xAB\xAC"
    FRAME_LEN = 516

    def __init__(self) -> None:
        self.buf = bytearray()

    def feed(self, data: bytes, host_ts: float) -> List[PressureFrame]:
        self.buf.extend(data)
        out: List[PressureFrame] = []

        while True:
            idx = self.buf.find(self.HEADER)
            if idx < 0:
                if len(self.buf) > 2:
                    self.buf = self.buf[-2:]
                break

            if idx > 0:
                del self.buf[:idx]

            if len(self.buf) < self.FRAME_LEN:
                break

            frame = bytes(self.buf[: self.FRAME_LEN])
            ok = (sum(frame[:515]) & 0xFF) == frame[515]
            if not ok:
                # 错位重同步
                del self.buf[:1]
                continue

            vals = np.frombuffer(frame[3:515], dtype=">u2").astype(np.float32).reshape(16, 16)
            out.append(PressureFrame(host_ts=host_ts, values=vals, checksum_ok=True))
            del self.buf[: self.FRAME_LEN]

        return out


class TB100Parser:
    HEADER = b"\xAA\x55"

    def __init__(self) -> None:
        self.buf = bytearray()

    def _parse_payload64(self, payload: bytes, host_ts: float, crc_ok: bool) -> ImuFrame:
        fmt = "<I H h h H 4i 3i 3i 3h b B H H"
        (
            ts_us,
            status,
            _roll_i,
            _pitch_i,
            _yaw_i,
            qw_i,
            qx_i,
            qy_i,
            qz_i,
            _gx_i,
            _gy_i,
            _gz_i,
            ax_i,
            ay_i,
            az_i,
            _mx_i,
            _my_i,
            _mz_i,
            _temp_i,
            _out_rate_i,
            _reserved_i,
            simple_sum_i,
        ) = struct.unpack(fmt, payload)

        simple_ok = (sum(payload[:62]) & 0xFFFF) == simple_sum_i

        return ImuFrame(
            host_ts=host_ts,
            device_ts_us=ts_us,
            qos=(status & 0x7),
            quat_wxyz=(qw_i * 1e-7, qx_i * 1e-7, qy_i * 1e-7, qz_i * 1e-7),
            acc_g=(ax_i * 1e-5, ay_i * 1e-5, az_i * 1e-5),
            crc_ok=crc_ok,
            simple_sum_ok=simple_ok,
        )

    def feed(self, data: bytes, host_ts: float) -> List[ImuFrame]:
        self.buf.extend(data)
        out: List[ImuFrame] = []

        while True:
            idx = self.buf.find(self.HEADER)
            if idx < 0:
                if len(self.buf) > 1:
                    self.buf = self.buf[-1:]
                break

            if idx > 0:
                del self.buf[:idx]

            if len(self.buf) < 5:
                break

            payload_len = self.buf[2]
            frame_len = 2 + 1 + payload_len + 2
            if len(self.buf) < frame_len:
                break

            frame = bytes(self.buf[:frame_len])
            crc_rx = int.from_bytes(frame[-2:], "little")
            crc_calc = modbus_crc16(frame[2:-2])
            crc_ok = crc_rx == crc_calc

            if payload_len >= 68:
                content = frame[3 : 3 + payload_len]  # 4字节信息字 + 64字节载荷
                payload64 = content[4:68]
                if len(payload64) == 64:
                    try:
                        out.append(self._parse_payload64(payload64, host_ts, crc_ok))
                    except struct.error:
                        pass

            del self.buf[:frame_len]

        return out


# ---------------------------- 串口线程 ----------------------------

class SerialReaderThread(threading.Thread):
    def __init__(
        self,
        name: str,
        port: str,
        baud: int,
        parser,
        callback,
        stop_event: threading.Event,
        on_open: Optional[Callable[[serial.Serial], None]] = None,
        timeout: float = 0.02,
    ):
        super().__init__(daemon=True)
        self.name = name
        self.port = port
        self.baud = baud
        self.parser = parser
        self.callback = callback
        self.stop_event = stop_event
        self.on_open = on_open
        self.timeout = timeout
        self.ser: Optional[serial.Serial] = None

    def run(self) -> None:
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=self.timeout)
            print(f"[{self.name}] Opened {self.port} @ {self.baud}")
            if self.on_open is not None:
                self.on_open(self.ser)
        except Exception as e:
            print(f"[{self.name}] Failed to open {self.port}: {e}")
            return

        try:
            while not self.stop_event.is_set():
                chunk = self.ser.read(self.ser.in_waiting or 1)
                if not chunk:
                    continue
                ts = time.perf_counter()
                frames = self.parser.feed(chunk, host_ts=ts)
                for f in frames:
                    self.callback(f)
        finally:
            if self.ser and self.ser.is_open:
                self.ser.close()
            print(f"[{self.name}] Closed")


# ---------------------------- 主程序 ----------------------------

def main() -> None:
    if os.name != "nt":
        raise RuntimeError("This script is Windows-only. Please run in native Windows terminal.")

    ap = argparse.ArgumentParser(description="Windows dual sensor parser (M1616M + TB100)")
    ap.add_argument("--pressure-port", default="COM8")
    ap.add_argument("--pressure-baud", type=int, default=115200)
    ap.add_argument("--pressure-range", default="1kg", choices=["1kg", "3kg", "5kg", "10kg", "20kg", "30kg", "50kg", "skip"])
    ap.add_argument("--pressure-cmd-suffix", default="cr", choices=["none", "cr", "lf", "crlf"])

    ap.add_argument("--imu-port", default="COM4")
    ap.add_argument("--imu-baud", type=int, default=115200)
    ap.add_argument("--disable-imu", action="store_true")

    ap.add_argument("--sync-max-dt-ms", type=float, default=200.0)
    ap.add_argument("--save-csv", default="")

    ap.add_argument("--transpose", action="store_true")
    ap.add_argument("--flipud", action="store_true")
    ap.add_argument("--fliplr", action="store_true")
    ap.add_argument("--vmax", type=float, default=1000.0)
    args = ap.parse_args()

    stop_event = threading.Event()
    lock = threading.Lock()

    latest_pressure: Optional[PressureFrame] = None
    latest_imu: Optional[ImuFrame] = None
    imu_buffer: Deque[ImuFrame] = deque(maxlen=8000)

    csv_writer = None
    csv_fp = None
    if args.save_csv:
        out = Path(args.save_csv)
        out.parent.mkdir(parents=True, exist_ok=True)
        csv_fp = out.open("w", newline="", encoding="utf-8")
        csv_writer = csv.writer(csv_fp)
        header = ["host_ts", "sync_lag_ms", "pressure_checksum_ok"] + [f"p_{i}" for i in range(256)]
        header += ["imu_ok", "imu_crc_ok", "imu_simple_ok", "imu_ax_g", "imu_ay_g", "imu_az_g", "imu_qw", "imu_qx", "imu_qy", "imu_qz"]
        csv_writer.writerow(header)

    suffix_map = {"none": b"", "cr": b"\r", "lf": b"\n", "crlf": b"\r\n"}

    def on_pressure_open(ser: serial.Serial):
        configure_pressure_sensor_range(ser, args.pressure_range, suffix_map[args.pressure_cmd_suffix])

    def on_pressure(frame: PressureFrame):
        nonlocal latest_pressure
        with lock:
            latest_pressure = frame
            imu = None if args.disable_imu else nearest_imu(frame.host_ts, imu_buffer, args.sync_max_dt_ms / 1000.0)

            if csv_writer is not None:
                flat = frame.values.reshape(-1).tolist()
                lag_ms = (frame.host_ts - imu.host_ts) * 1000.0 if imu else float("nan")
                row = [frame.host_ts, lag_ms, int(frame.checksum_ok)] + flat
                if imu is None:
                    row += [0, 0, 0, "", "", "", "", "", "", ""]
                else:
                    qw, qx, qy, qz = imu.quat_wxyz
                    ax, ay, az = imu.acc_g
                    row += [1, int(imu.crc_ok), int(imu.simple_sum_ok), ax, ay, az, qw, qx, qy, qz]
                csv_writer.writerow(row)

    def on_imu(frame: ImuFrame):
        nonlocal latest_imu
        with lock:
            latest_imu = frame
            imu_buffer.append(frame)

    pressure_thr = SerialReaderThread(
        name="PRESSURE",
        port=args.pressure_port,
        baud=args.pressure_baud,
        parser=M1616MParser(),
        callback=on_pressure,
        stop_event=stop_event,
        on_open=on_pressure_open,
    )

    imu_thr = None
    if not args.disable_imu:
        imu_thr = SerialReaderThread(
            name="IMU",
            port=args.imu_port,
            baud=args.imu_baud,
            parser=TB100Parser(),
            callback=on_imu,
            stop_event=stop_event,
        )

    pressure_thr.start()
    if imu_thr is not None:
        imu_thr.start()

    print("Running... Press Ctrl+C to stop")

    plt.ion()
    fig = plt.figure(figsize=(12, 6))
    ax1 = fig.add_subplot(1, 2, 1)
    ax2 = fig.add_subplot(1, 2, 2)
    ax2.axis("off")

    im = ax1.imshow(np.zeros((16, 16), dtype=np.float32), cmap="jet", vmin=0, vmax=args.vmax, interpolation="nearest")
    fig.colorbar(im, ax=ax1)
    info = ax2.text(0.01, 0.99, "Waiting...", va="top", ha="left", family="monospace", fontsize=11)

    try:
        while True:
            with lock:
                p = latest_pressure
                i = latest_imu

            if p is not None:
                im.set_data(orient_pressure(p.values, args.transpose, args.flipud, args.fliplr))

            lines = []
            if p is None:
                lines.append("Pressure: N/A")
            else:
                lines.append(f"Pressure mean/max: {float(np.mean(p.values)):.1f}/{float(np.max(p.values)):.1f} g")

            if args.disable_imu:
                lines.append("IMU: DISABLED")
            elif i is None:
                lines.append("IMU: N/A")
            else:
                ax, ay, az = i.acc_g
                qw, qx, qy, qz = i.quat_wxyz
                lines.append(f"IMU acc[g]: ({ax:+.5f}, {ay:+.5f}, {az:+.5f})")
                lines.append(f"IMU quat : ({qw:+.6f}, {qx:+.6f}, {qy:+.6f}, {qz:+.6f})")
                lines.append(f"IMU crc/sum ok: {i.crc_ok}/{i.simple_sum_ok}")

            info.set_text("\n".join(lines))
            fig.canvas.draw_idle()
            fig.canvas.flush_events()
            time.sleep(0.03)

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        stop_event.set()
        pressure_thr.join(timeout=2.0)
        if imu_thr is not None:
            imu_thr.join(timeout=2.0)
        if csv_fp is not None:
            csv_fp.close()
        plt.ioff()
        plt.close(fig)
        print("Stopped")


if __name__ == "__main__":
    main()
