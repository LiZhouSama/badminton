#!/usr/bin/env python3
"""
Windows 楂樻€ц兘鐗堬細M1616M + TB100 鍙岃澶囧悓姝ユ姄鍙?瑙ｅ寘

璁捐鐩爣锛?- 浠?Windows 鍘熺敓 COM
- 涓插彛閲囬泦涓庤В鍖呭垎绾跨▼锛堥槦鍒楄В鑰︼級
- pyqtgraph 楂樺埛鏂板彲瑙嗗寲锛堝彲 OpenGL锛?- 鍘嬪姏 + IMU 鎸変富鏈烘椂閽熸渶杩戦偦鍚屾
"""

from __future__ import annotations

import argparse
import csv
import os
import queue
import struct
import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Deque, List, Optional, Tuple

import numpy as np
import serial


# ---------------------------- 鏁版嵁缁撴瀯 ----------------------------

@dataclass
class PressureFrame:
    host_ts: float
    values: np.ndarray
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


# ---------------------------- 鍗忚杈呭姪 ----------------------------

def modbus_crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if (crc & 1) else (crc >> 1)
    return crc & 0xFFFF


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
    for cmd in [cmd_setf.encode("ascii") + suffix, b"SET=OK" + suffix]:
        ser.write(cmd)
        ser.flush()
        time.sleep(0.12)
    print(f"[PRESSURE] Range set request sent: {pressure_range} ({cmd_setf}) + SET=OK")


# ---------------------------- 瑙ｆ瀽鍣?----------------------------

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
            crc_ok = int.from_bytes(frame[-2:], "little") == modbus_crc16(frame[2:-2])

            if payload_len >= 68:
                content = frame[3 : 3 + payload_len]
                payload64 = content[4:68]
                if len(payload64) == 64:
                    try:
                        out.append(self._parse_payload64(payload64, host_ts, crc_ok))
                    except struct.error:
                        pass

            del self.buf[:frame_len]

        return out


# ---------------------------- 楂樻€ц兘骞跺彂 ----------------------------

class SerialIngestThread(threading.Thread):
    def __init__(
        self,
        name: str,
        port: str,
        baud: int,
        out_q: queue.Queue,
        stop_event: threading.Event,
        on_open: Optional[Callable[[serial.Serial], None]] = None,
        read_chunk: int = 4096,
        timeout: float = 0.01,
        stats: Optional[dict] = None,
    ):
        super().__init__(daemon=True)
        self.name = name
        self.port = port
        self.baud = baud
        self.out_q = out_q
        self.stop_event = stop_event
        self.on_open = on_open
        self.read_chunk = read_chunk
        self.timeout = timeout
        self.stats = stats if stats is not None else {}
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
                n = self.ser.in_waiting
                chunk = self.ser.read(min(self.read_chunk, n) if n > 0 else 1)
                if not chunk:
                    continue
                item = (time.perf_counter(), chunk)
                try:
                    self.out_q.put_nowait(item)
                except queue.Full:
                    self.stats[f"{self.name.lower()}_drop_chunks"] = self.stats.get(f"{self.name.lower()}_drop_chunks", 0) + 1
        finally:
            if self.ser and self.ser.is_open:
                self.ser.close()
            print(f"[{self.name}] Closed")


class ParserThread(threading.Thread):
    def __init__(
        self,
        name: str,
        in_q: queue.Queue,
        parser,
        on_frame: Callable,
        stop_event: threading.Event,
        stats: Optional[dict] = None,
    ):
        super().__init__(daemon=True)
        self.name = name
        self.in_q = in_q
        self.parser = parser
        self.on_frame = on_frame
        self.stop_event = stop_event
        self.stats = stats if stats is not None else {}

    def run(self) -> None:
        while not self.stop_event.is_set():
            try:
                ts, chunk = self.in_q.get(timeout=0.05)
            except queue.Empty:
                continue

            frames = self.parser.feed(chunk, host_ts=ts)
            if frames:
                self.stats[f"{self.name.lower()}_parsed_frames"] = self.stats.get(f"{self.name.lower()}_parsed_frames", 0) + len(frames)
                for f in frames:
                    self.on_frame(f)


# ---------------------------- 涓荤▼搴?----------------------------

def nearest_imu(ts: float, imu_buf: Deque[ImuFrame], max_dt_sec: float) -> Optional[ImuFrame]:
    if not imu_buf:
        return None
    best = min(imu_buf, key=lambda x: abs(x.host_ts - ts))
    return best if abs(best.host_ts - ts) <= max_dt_sec else None


def orient_pressure(mat: np.ndarray, transpose: bool, flipud: bool, fliplr: bool) -> np.ndarray:
    out = mat
    if transpose:
        out = out.T
    if flipud:
        out = np.flipud(out)
    if fliplr:
        out = np.fliplr(out)
    return out


def build_bilinear_plan(h: int, w: int, scale: int):
    if scale <= 1:
        return None
    out_h, out_w = h * scale, w * scale
    y = np.linspace(0, h - 1, out_h)
    x = np.linspace(0, w - 1, out_w)
    y0 = np.floor(y).astype(np.int32)
    x0 = np.floor(x).astype(np.int32)
    y1 = np.clip(y0 + 1, 0, h - 1)
    x1 = np.clip(x0 + 1, 0, w - 1)
    wy = (y - y0).astype(np.float32)
    wx = (x - x0).astype(np.float32)
    return y0, y1, wy, x0, x1, wx


def bilinear_resize_with_plan(mat: np.ndarray, plan) -> np.ndarray:
    if plan is None:
        return mat
    y0, y1, wy, x0, x1, wx = plan
    top = mat[y0[:, None], x0[None, :]] * (1.0 - wx)[None, :] + mat[y0[:, None], x1[None, :]] * wx[None, :]
    bottom = mat[y1[:, None], x0[None, :]] * (1.0 - wx)[None, :] + mat[y1[:, None], x1[None, :]] * wx[None, :]
    out = top * (1.0 - wy)[:, None] + bottom * wy[:, None]
    return out.astype(np.float32, copy=False)


def pressure_for_display(
    mat: np.ndarray,
    transpose: bool,
    flipud: bool,
    fliplr: bool,
    rotate_ccw90: bool,
    interp_plan,
) -> np.ndarray:
    out = orient_pressure(mat, transpose, flipud, fliplr)
    if rotate_ccw90:
        out = np.rot90(out, k=1)
    out = bilinear_resize_with_plan(out, interp_plan)
    return out


def quat_wxyz_to_rotmat(quat_wxyz: Tuple[float, float, float, float]) -> np.ndarray:
    w, x, y, z = quat_wxyz
    n = float(np.sqrt(w * w + x * x + y * y + z * z))
    if n < 1e-12:
        return np.eye(3, dtype=np.float32)
    w, x, y, z = w / n, x / n, y / n, z / n
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=np.float32,
    )


def imu_rot_sensor_to_world(quat_wxyz: Tuple[float, float, float, float], quat_world_to_sensor: bool) -> np.ndarray:
    # TB100 quaternion convention may differ by firmware; provide runtime switch.
    rot = quat_wxyz_to_rotmat(quat_wxyz)
    return rot.T if quat_world_to_sensor else rot


def build_racket_model_sensor_frame() -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    # Sensor is at butt cap; butt -> head points to sensor -Z.
    shaft_len = 0.53
    head_center_z = -0.58
    head_rx = 0.10
    head_rz = 0.14
    face_normal_len = 0.12

    shaft = np.array([[0.0, 0.0, 0.0], [0.0, 0.0, -shaft_len]], dtype=np.float32)
    theta = np.linspace(0.0, 2.0 * np.pi, 73, dtype=np.float32)
    head = np.column_stack(
        [
            head_rx * np.cos(theta),
            np.zeros_like(theta, dtype=np.float32),
            head_center_z + head_rz * np.sin(theta),
        ]
    ).astype(np.float32)
    face_normal = np.array([[0.0, 0.0, head_center_z], [0.0, face_normal_len, head_center_z]], dtype=np.float32)
    return shaft, head, face_normal


def rotate_points(points: np.ndarray, rot: np.ndarray) -> np.ndarray:
    return (rot @ points.T).T.astype(np.float32, copy=False)


def main() -> None:
    if os.name != "nt":
        raise RuntimeError("This script is Windows-only. Please run in native Windows terminal.")

    ap = argparse.ArgumentParser(description="Windows high-performance dual sensor parser")
    ap.add_argument("--pressure-port", default="COM8")
    ap.add_argument("--pressure-baud", type=int, default=115200)
    ap.add_argument("--pressure-range", default="1kg", choices=["1kg", "3kg", "5kg", "10kg", "20kg", "30kg", "50kg", "skip"])
    ap.add_argument("--pressure-cmd-suffix", default="cr", choices=["none", "cr", "lf", "crlf"])

    ap.add_argument("--imu-port", default="COM4")
    ap.add_argument("--imu-baud", type=int, default=115200)
    ap.add_argument("--disable-imu", action="store_true")
    ap.add_argument("--imu-quat-world-to-sensor", action="store_true")

    ap.add_argument("--sync-max-dt-ms", type=float, default=200.0)
    ap.add_argument("--save-csv", default="")

    ap.add_argument("--fps", type=float, default=60.0)
    ap.add_argument("--no-opengl", action="store_true")
    ap.add_argument("--headless", action="store_true")
    ap.add_argument("--status-interval", type=float, default=1.0)
    ap.add_argument("--duration-sec", type=float, default=0.0)

    ap.add_argument("--transpose", action="store_true")
    ap.add_argument("--flipud", action="store_true")
    ap.add_argument("--fliplr", action="store_true")
    ap.add_argument("--no-rotate-ccw90", dest="no_rotate_ccw90", action="store_true")
    ap.add_argument("--no-rotate-cw90", dest="no_rotate_ccw90", action="store_true", help=argparse.SUPPRESS)
    ap.add_argument("--interp-scale", type=int, default=4)
    ap.add_argument("--vmax", type=float, default=1000.0)
    ap.add_argument("--queue-size", type=int, default=4096)
    ap.add_argument("--read-chunk", type=int, default=4096)
    args = ap.parse_args()

    stop_event = threading.Event()
    stats = {
        "pressure_drop_chunks": 0,
        "imu_drop_chunks": 0,
        "pressure_parsed_frames": 0,
        "imu_parsed_frames": 0,
        "pressure_frames": 0,
        "pressure_bad_checksum": 0,
        "imu_frames": 0,
        "imu_bad_crc": 0,
        "imu_bad_simple": 0,
        "synced": 0,
        "unsynced": 0,
    }

    lock = threading.Lock()
    latest_pressure: Optional[PressureFrame] = None
    latest_imu: Optional[ImuFrame] = None
    imu_buffer: Deque[ImuFrame] = deque(maxlen=20000)
    interp_scale = max(1, args.interp_scale)
    display_hw = 16 * interp_scale
    pressure_interp_plan = build_bilinear_plan(16, 16, interp_scale)
    latest_pressure_display: Optional[np.ndarray] = None
    latest_pressure_display_ts: Optional[float] = None

    # Zero calibration state.
    zero_offset = np.zeros((16, 16), dtype=np.float32)
    zero_enabled = False
    zero_calibrating = False
    zero_start_ts = 0.0
    zero_duration_sec = 5.0
    zero_samples: List[np.ndarray] = []
    zero_status_msg = "闆剁偣锛氭湭鏍″噯"

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

    def on_pressure(frame: PressureFrame):
        nonlocal latest_pressure, latest_pressure_display, latest_pressure_display_ts
        nonlocal zero_enabled, zero_calibrating, zero_start_ts, zero_samples, zero_offset, zero_status_msg

        with lock:
            raw_vals = frame.values

            # 1) Collect baseline samples while zero calibration is active.
            if zero_calibrating:
                zero_samples.append(raw_vals.copy())
                elapsed = time.perf_counter() - zero_start_ts
                if elapsed >= zero_duration_sec:
                    if len(zero_samples) >= 3:
                        stack = np.stack(zero_samples, axis=0).astype(np.float32)  # (N,16,16)
                        # Robust per-cell baseline by trimming to 10%~90% quantile.
                        p10 = np.percentile(stack, 10, axis=0)
                        p90 = np.percentile(stack, 90, axis=0)
                        valid = (stack >= p10[None, ...]) & (stack <= p90[None, ...])
                        clipped = np.where(valid, stack, np.nan)
                        zero_map = np.nanmean(clipped, axis=0)
                        fallback = np.mean(stack, axis=0)
                        zero_map = np.where(np.isnan(zero_map), fallback, zero_map).astype(np.float32)

                        zero_offset = zero_map
                        zero_enabled = True
                        zero_status_msg = f"闆剁偣锛氬凡鏍″噯锛堟牱鏈?{len(zero_samples)} 甯э級"
                    else:
                        zero_status_msg = "Zero calibration failed (not enough samples)"

                    zero_calibrating = False
                    zero_samples.clear()

            # 2) Apply zero-offset map for visualization/sync/CSV.
            if zero_enabled:
                corrected = raw_vals - zero_offset
                corrected = np.maximum(corrected, 0.0)
            else:
                corrected = raw_vals

            display_vals = pressure_for_display(
                corrected,
                args.transpose,
                args.flipud,
                args.fliplr,
                rotate_ccw90=(not args.no_rotate_ccw90),
                interp_plan=pressure_interp_plan,
            )
            latest_pressure = PressureFrame(host_ts=frame.host_ts, values=corrected, checksum_ok=frame.checksum_ok)
            latest_pressure_display = display_vals
            latest_pressure_display_ts = frame.host_ts
            stats["pressure_frames"] += 1
            if not frame.checksum_ok:
                stats["pressure_bad_checksum"] += 1

            imu = None if args.disable_imu else nearest_imu(frame.host_ts, imu_buffer, args.sync_max_dt_ms / 1000.0)
            if not args.disable_imu:
                if imu is None:
                    stats["unsynced"] += 1
                else:
                    stats["synced"] += 1

            if csv_writer is not None:
                lag_ms = (frame.host_ts - imu.host_ts) * 1000.0 if imu else float("nan")
                row = [frame.host_ts, lag_ms, int(frame.checksum_ok)] + corrected.reshape(-1).tolist()
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
            stats["imu_frames"] += 1
            if not frame.crc_ok:
                stats["imu_bad_crc"] += 1
            if not frame.simple_sum_ok:
                stats["imu_bad_simple"] += 1

    suffix_map = {"none": b"", "cr": b"\r", "lf": b"\n", "crlf": b"\r\n"}

    def on_pressure_open(ser: serial.Serial):
        configure_pressure_sensor_range(ser, args.pressure_range, suffix_map[args.pressure_cmd_suffix])

    q_pressure: queue.Queue = queue.Queue(maxsize=args.queue_size)
    q_imu: queue.Queue = queue.Queue(maxsize=args.queue_size)

    pressure_ingest = SerialIngestThread(
        name="PRESSURE",
        port=args.pressure_port,
        baud=args.pressure_baud,
        out_q=q_pressure,
        stop_event=stop_event,
        on_open=on_pressure_open,
        read_chunk=args.read_chunk,
        stats=stats,
    )
    pressure_parse = ParserThread(
        name="PRESSURE",
        in_q=q_pressure,
        parser=M1616MParser(),
        on_frame=on_pressure,
        stop_event=stop_event,
        stats=stats,
    )

    imu_ingest = None
    imu_parse = None
    if not args.disable_imu:
        imu_ingest = SerialIngestThread(
            name="IMU",
            port=args.imu_port,
            baud=args.imu_baud,
            out_q=q_imu,
            stop_event=stop_event,
            read_chunk=args.read_chunk,
            stats=stats,
        )
        imu_parse = ParserThread(
            name="IMU",
            in_q=q_imu,
            parser=TB100Parser(),
            on_frame=on_imu,
            stop_event=stop_event,
            stats=stats,
        )

    # 鍚姩绾跨▼
    pressure_ingest.start()
    pressure_parse.start()
    if imu_ingest is not None:
        imu_ingest.start()
    if imu_parse is not None:
        imu_parse.start()

    imu_desc = "DISABLED" if args.disable_imu else f"{args.imu_port}@{args.imu_baud}"
    print(
        f"Running fast mode | Pressure {args.pressure_port}@{args.pressure_baud} range={args.pressure_range} "
        f"| IMU {imu_desc} | OpenGL={not args.no_opengl}"
    )

    start_ts = time.perf_counter()

    def should_stop_by_duration() -> bool:
        return args.duration_sec > 0 and (time.perf_counter() - start_ts) >= args.duration_sec

    try:
        if args.headless:
            last_print = 0.0
            while not stop_event.is_set():
                if should_stop_by_duration():
                    break
                now = time.perf_counter()
                if now - last_print >= args.status_interval:
                    with lock:
                        p = latest_pressure
                        i = latest_imu
                        st = dict(stats)
                    elapsed = max(1e-6, now - start_ts)
                    line = (
                        f"[HEADLESS] P={st['pressure_frames']} ({st['pressure_frames']/elapsed:.1f} fps) "
                        f"drop={st['pressure_drop_chunks']} | IMU={st['imu_frames']} ({st['imu_frames']/elapsed:.1f} fps) "
                        f"drop={st['imu_drop_chunks']} | sync={st['synced']}/{st['unsynced']}"
                    )
                    if p is not None:
                        line += f" | Pmean/Pmax={float(np.mean(p.values)):.1f}/{float(np.max(p.values)):.1f}g"
                    if i is not None:
                        ax, ay, az = i.acc_g
                        line += f" | acc=({ax:+.4f},{ay:+.4f},{az:+.4f})"
                    print(line)
                    last_print = now
                time.sleep(0.01)
        else:
            import pyqtgraph as pg
            from pyqtgraph.Qt import QtCore, QtWidgets

            pg.setConfigOptions(antialias=False, useOpenGL=(not args.no_opengl), imageAxisOrder="row-major")
            app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

            win = QtWidgets.QWidget()
            win.setWindowTitle("Dual Sensor SAGE (Windows High Performance)")
            vbox = QtWidgets.QVBoxLayout(win)

            top_row = QtWidgets.QHBoxLayout()
            vbox.addLayout(top_row, stretch=1)

            glw = pg.GraphicsLayoutWidget()
            top_row.addWidget(glw, stretch=2)
            vb = glw.addViewBox(lockAspect=True)
            vb.setMouseEnabled(x=False, y=False)
            img = pg.ImageItem()
            vb.addItem(img)
            vb.setRange(QtCore.QRectF(0, 0, display_hw, display_hw), padding=0)
            cmap = pg.colormap.get("viridis")
            img.setLookupTable(cmap.getLookupTable(0.0, 1.0, 256))
            img.setLevels((0, args.vmax))
            img.setImage(np.zeros((display_hw, display_hw), dtype=np.float32), autoLevels=False)

            imu3d_enabled = False
            imu3d_error_msg = ""
            imu_init_rot_world: Optional[np.ndarray] = None
            last_drawn_imu_ts: Optional[float] = None
            sensor0_to_view = np.array(
                [
                    [1.0, 0.0, 0.0],
                    [0.0, 0.0, 1.0],
                    [0.0, -1.0, 0.0],
                ],
                dtype=np.float32,
            )
            racket_shaft_sensor, racket_head_sensor, racket_face_sensor = build_racket_model_sensor_frame()
            racket_shaft_item = None
            racket_head_item = None
            racket_face_item = None

            if not args.disable_imu:
                try:
                    import pyqtgraph.opengl as gl

                    imu3d_enabled = True
                    imu_view = gl.GLViewWidget()
                    imu_view.setMinimumWidth(420)
                    imu_view.opts["distance"] = 2.0
                    imu_view.opts["elevation"] = 18.0
                    imu_view.opts["azimuth"] = -40.0
                    top_row.addWidget(imu_view, stretch=1)

                    grid = gl.GLGridItem()
                    grid.scale(0.25, 0.25, 0.25)
                    imu_view.addItem(grid)

                    axis = gl.GLAxisItem()
                    axis.setSize(0.25, 0.25, 0.25)
                    imu_view.addItem(axis)

                    racket_shaft_item = gl.GLLinePlotItem(
                        pos=rotate_points(racket_shaft_sensor, sensor0_to_view),
                        color=(1.0, 0.9, 0.2, 1.0),
                        width=4.0,
                        antialias=(not args.no_opengl),
                        mode="line_strip",
                    )
                    racket_head_item = gl.GLLinePlotItem(
                        pos=rotate_points(racket_head_sensor, sensor0_to_view),
                        color=(0.1, 0.8, 1.0, 1.0),
                        width=2.0,
                        antialias=(not args.no_opengl),
                        mode="line_strip",
                    )
                    racket_face_item = gl.GLLinePlotItem(
                        pos=rotate_points(racket_face_sensor, sensor0_to_view),
                        color=(1.0, 0.3, 0.3, 1.0),
                        width=2.0,
                        antialias=(not args.no_opengl),
                        mode="line_strip",
                    )
                    imu_view.addItem(racket_shaft_item)
                    imu_view.addItem(racket_head_item)
                    imu_view.addItem(racket_face_item)
                except Exception as e:
                    imu3d_error_msg = str(e)

            if (not args.disable_imu) and (not imu3d_enabled):
                imu_warn = QtWidgets.QLabel("IMU 3D unavailable:\n" + (imu3d_error_msg or "OpenGL backend not available"))
                imu_warn.setWordWrap(True)
                imu_warn.setStyleSheet("font-family: Consolas, monospace; font-size: 11px;")
                top_row.addWidget(imu_warn, stretch=1)

            lbl = QtWidgets.QLabel("Waiting data...")
            lbl.setStyleSheet("font-family: Consolas, monospace; font-size: 12px;")
            vbox.addWidget(lbl)

            # 浜や簰鎸夐挳锛氶浂鐐规牎鍑?/ 娑堥櫎闆剁偣
            btn_row = QtWidgets.QHBoxLayout()
            btn_zero_calib = QtWidgets.QPushButton("闆剁偣鏍″噯")
            btn_zero_clear = QtWidgets.QPushButton("娑堥櫎闆剁偣")
            btn_row.addWidget(btn_zero_calib)
            btn_row.addWidget(btn_zero_clear)
            btn_imu_reset = QtWidgets.QPushButton("Reset IMU Zero")
            btn_row.addWidget(btn_imu_reset)
            btn_row.addStretch(1)
            vbox.addLayout(btn_row)

            timer = QtCore.QTimer()
            timer.setInterval(max(1, int(1000 / max(1.0, args.fps))))
            try:
                timer.setTimerType(QtCore.Qt.PreciseTimer)
            except Exception:
                pass
            last_drawn_pressure_ts: Optional[float] = None
            imu_zero_status_msg = "IMU zero: waiting first frame"

            def start_zero_calibration():
                nonlocal zero_enabled, zero_calibrating, zero_start_ts, zero_samples, zero_status_msg
                with lock:
                    zero_enabled = False
                    zero_calibrating = True
                    zero_start_ts = time.perf_counter()
                    zero_samples = []
                    zero_status_msg = "Zero calibration running (5.0s)"
                print("[PRESSURE] Zero calibration started (5s)")

            def clear_zero_calibration():
                nonlocal zero_enabled, zero_calibrating, zero_samples, zero_offset, zero_status_msg
                with lock:
                    zero_enabled = False
                    zero_calibrating = False
                    zero_samples.clear()
                    zero_offset = np.zeros((16, 16), dtype=np.float32)
                    zero_status_msg = "闆剁偣锛氬凡娓呴櫎"
                print("[PRESSURE] Zero calibration cleared")

            def reset_imu_zero_pose():
                nonlocal imu_init_rot_world, imu_zero_status_msg, last_drawn_imu_ts
                imu_init_rot_world = None
                last_drawn_imu_ts = None
                imu_zero_status_msg = "IMU zero: waiting next frame"
                print("[IMU] Zero pose reset")

            btn_zero_calib.clicked.connect(start_zero_calibration)
            btn_zero_clear.clicked.connect(clear_zero_calibration)
            btn_imu_reset.clicked.connect(reset_imu_zero_pose)

            def tick():
                nonlocal last_drawn_pressure_ts, last_drawn_imu_ts, imu_init_rot_world, imu_zero_status_msg
                if should_stop_by_duration():
                    app.quit()
                    return

                with lock:
                    p = latest_pressure
                    p_disp = latest_pressure_display
                    p_disp_ts = latest_pressure_display_ts
                    i = latest_imu
                    st = dict(stats)
                    z_enabled = zero_enabled
                    z_calib = zero_calibrating
                    z_status = zero_status_msg
                    z_remaining = max(0.0, zero_duration_sec - (time.perf_counter() - zero_start_ts)) if zero_calibrating else 0.0
                    z_samples_n = len(zero_samples)

                if (p_disp is not None) and (p_disp_ts is not None) and (last_drawn_pressure_ts != p_disp_ts):
                    img.setImage(p_disp, autoLevels=False)
                    last_drawn_pressure_ts = p_disp_ts

                if imu3d_enabled and (i is not None) and (last_drawn_imu_ts != i.host_ts):
                    rot_sw = imu_rot_sensor_to_world(i.quat_wxyz, args.imu_quat_world_to_sensor)
                    if imu_init_rot_world is None:
                        imu_init_rot_world = rot_sw
                        imu_zero_status_msg = 'IMU zero: locked'
                    rot_rel = imu_init_rot_world.T @ rot_sw
                    rot_view = sensor0_to_view @ rot_rel

                    if racket_shaft_item is not None:
                        racket_shaft_item.setData(pos=rotate_points(racket_shaft_sensor, rot_view))
                    if racket_head_item is not None:
                        racket_head_item.setData(pos=rotate_points(racket_head_sensor, rot_view))
                    if racket_face_item is not None:
                        racket_face_item.setData(pos=rotate_points(racket_face_sensor, rot_view))
                    last_drawn_imu_ts = i.host_ts

                elapsed = max(1e-6, time.perf_counter() - start_ts)
                lines = [
                    f"P={st['pressure_frames']} ({st['pressure_frames']/elapsed:.1f} fps) parsed={st['pressure_parsed_frames']} drop={st['pressure_drop_chunks']}",
                    f"IMU={st['imu_frames']} ({st['imu_frames']/elapsed:.1f} fps) parsed={st['imu_parsed_frames']} drop={st['imu_drop_chunks']}",
                    f"sync={st['synced']}/{st['unsynced']}",
                ]
                if z_calib:
                    lines.append(f"{z_status} | remain {z_remaining:0.1f}s | samples {z_samples_n}")
                else:
                    lines.append(f"{z_status} | {'enabled' if z_enabled else 'disabled'}")

                if args.disable_imu:
                    lines.append('IMU 3D: disabled')
                elif imu3d_enabled:
                    lines.append(f"IMU 3D: on | {imu_zero_status_msg}")
                else:
                    err_short = imu3d_error_msg.splitlines()[0] if imu3d_error_msg else 'OpenGL backend not available'
                    lines.append(f"IMU 3D: unavailable ({err_short})")

                if p is not None:
                    lines.append(f"P mean/max = {float(np.mean(p.values)):.1f}/{float(np.max(p.values)):.1f} g")
                if i is not None:
                    ax, ay, az = i.acc_g
                    qw, qx, qy, qz = i.quat_wxyz
                    lines.append(f"acc[g]=({ax:+.5f}, {ay:+.5f}, {az:+.5f})")
                    lines.append(f"quat =({qw:+.6f}, {qx:+.6f}, {qy:+.6f}, {qz:+.6f})")

                btn_zero_calib.setEnabled(not z_calib)
                btn_zero_clear.setEnabled(z_calib or z_enabled)
                btn_imu_reset.setEnabled((not args.disable_imu) and imu3d_enabled)
                lbl.setText("\n".join(lines))
            timer.timeout.connect(tick)
            timer.start()
            win.resize(1100, 760)
            win.show()
            app.exec()

    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        pressure_ingest.join(timeout=2.0)
        pressure_parse.join(timeout=2.0)
        if imu_ingest is not None:
            imu_ingest.join(timeout=2.0)
        if imu_parse is not None:
            imu_parse.join(timeout=2.0)
        if csv_fp is not None:
            csv_fp.close()
        print("Stopped")


if __name__ == "__main__":
    main()

