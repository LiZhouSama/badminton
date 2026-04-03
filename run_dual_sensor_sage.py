#!/usr/bin/env python3
"""
Windows high-performance dual sensor runner.

Features:
- M1616M pressure stream parser (16x16 matrix)
- TB100 IMU stream parser (quat + acceleration)
- Multi-threaded serial ingest + parse
- Pressure heatmap visualization with interpolation
- Optional IMU 3D visualization with OpenGL fallback strategy
- Optional OBJ racket model rendering and real-time pose sync
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import queue
import struct
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Deque, List, Optional, Tuple

import numpy as np
import serial


OPENGL_INSTALL_HINT = "pip install PyOpenGL PyOpenGL_accelerate"
DEFAULT_RACKET_OBJ = Path("obj/badminton_racket/Racket.obj")

UI_TEXT = {
    "title": "Dual Sensor SAGE (Windows High Performance)",
    "waiting": "Waiting data...",
    "zero_btn_start": "Start Zero Calibration",
    "zero_btn_clear": "Clear Zero Calibration",
    "imu_btn_reset": "Reset IMU Zero",
    "imu3d_unavailable_prefix": "IMU 3D unavailable:",
    "zero_not_calibrated": "Zero calibration: not calibrated",
    "zero_running": "Zero calibration: running",
    "zero_enabled": "enabled",
    "zero_disabled": "disabled",
    "zero_cleared": "Zero calibration: cleared",
}


class OpenGLAutoRetryRequested(RuntimeError):
    """Signal to restart process once with software OpenGL."""


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


@dataclass
class ObjMesh:
    vertices: np.ndarray  # (N,3) float32
    faces: np.ndarray  # (M,3) int32
    texcoords: Optional[np.ndarray] = None  # (N,2) float32
    texture_path: Optional[Path] = None
    vertex_colors: Optional[np.ndarray] = None  # (N,4) float32


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
                    key = f"{self.name.lower()}_drop_chunks"
                    self.stats[key] = self.stats.get(key, 0) + 1
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
            if not frames:
                continue
            key = f"{self.name.lower()}_parsed_frames"
            self.stats[key] = self.stats.get(key, 0) + len(frames)
            for frame in frames:
                self.on_frame(frame)


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
    rot = quat_wxyz_to_rotmat(quat_wxyz)
    return rot.T if quat_world_to_sensor else rot


def rotate_points(points: np.ndarray, rot: np.ndarray) -> np.ndarray:
    return (rot @ points.T).T.astype(np.float32, copy=False)


def euler_deg_to_rotmat(roll_deg: float, pitch_deg: float, yaw_deg: float) -> np.ndarray:
    rx = math.radians(roll_deg)
    ry = math.radians(pitch_deg)
    rz = math.radians(yaw_deg)
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)

    rot_x = np.array([[1.0, 0.0, 0.0], [0.0, cx, -sx], [0.0, sx, cx]], dtype=np.float32)
    rot_y = np.array([[cy, 0.0, sy], [0.0, 1.0, 0.0], [-sy, 0.0, cy]], dtype=np.float32)
    rot_z = np.array([[cz, -sz, 0.0], [sz, cz, 0.0], [0.0, 0.0, 1.0]], dtype=np.float32)
    return (rot_z @ rot_y @ rot_x).astype(np.float32)


def safe_normalize(v: np.ndarray, fallback: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(v))
    if n < 1e-8:
        return fallback.astype(np.float32)
    return (v / n).astype(np.float32)


def parse_obj_index(index_text: str, n_items: int) -> int:
    idx = int(index_text)
    if idx > 0:
        idx -= 1
    else:
        idx = n_items + idx
    return idx


def parse_obj_face_token(token: str, n_vertices: int, n_texcoords: int) -> Tuple[int, int]:
    parts = token.split("/")
    if len(parts) == 0 or parts[0] == "":
        raise ValueError("invalid face token")

    v_idx = parse_obj_index(parts[0], n_vertices)
    vt_idx = -1
    if len(parts) >= 2 and parts[1] != "":
        vt_idx = parse_obj_index(parts[1], n_texcoords)
    return v_idx, vt_idx


def parse_obj_mtllib_entries(line_rest: str) -> List[str]:
    rest = line_rest.strip()
    if not rest:
        return []
    entries = [rest]
    for token in rest.split():
        if token not in entries:
            entries.append(token)
    return entries


def resolve_mtl_texture(obj_path: Path, mtllib_entries: List[str]) -> Optional[Path]:
    obj_dir = obj_path.parent
    for entry in mtllib_entries:
        mtl_path = (obj_dir / entry).resolve()
        if not mtl_path.exists():
            continue
        try:
            with mtl_path.open("r", encoding="utf-8", errors="ignore") as fp:
                for raw in fp:
                    line = raw.strip()
                    if not line or line.startswith("#"):
                        continue
                    if not line.lower().startswith("map_kd "):
                        continue
                    tex_rel = line.split(None, 1)[1].strip().strip('"').strip("'")
                    cand = (mtl_path.parent / tex_rel).resolve()
                    if cand.exists():
                        return cand
                    # Fallback for option-style lines where path is the tail token.
                    tail = tex_rel.split()[-1]
                    cand_tail = (mtl_path.parent / tail).resolve()
                    if cand_tail.exists():
                        return cand_tail
        except Exception:
            continue

    # Heuristic fallback for exports that include texture files but no map_Kd bindings.
    tex_dir = obj_dir / "Textures"
    if tex_dir.exists():
        images: List[Path] = []
        for pattern in ("*.png", "*.jpg", "*.jpeg", "*.PNG", "*.JPG", "*.JPEG"):
            images.extend(sorted(tex_dir.glob(pattern)))
        if images:
            def texture_rank(p: Path) -> Tuple[int, int, str]:
                name = p.name.lower()
                uv_score = 0
                if "uv" in name:
                    uv_score += 3
                if "texture" in name:
                    uv_score += 2
                if "handle" in name:
                    uv_score -= 1
                return (-uv_score, len(name), name)

            images.sort(key=texture_rank)
            return images[0].resolve()
    return None


def sample_vertex_colors_from_texture(texcoords: np.ndarray, texture_path: Path, QtGui) -> Optional[np.ndarray]:
    if texcoords is None or texcoords.size == 0:
        return None
    img = QtGui.QImage(str(texture_path))
    if img.isNull():
        return None
    img = img.convertToFormat(QtGui.QImage.Format_RGBA8888)
    w = img.width()
    h = img.height()
    if w <= 1 or h <= 1:
        return None

    colors = np.empty((texcoords.shape[0], 4), dtype=np.float32)
    for i in range(texcoords.shape[0]):
        u = float(texcoords[i, 0]) % 1.0
        v = float(texcoords[i, 1]) % 1.0
        x = int(round(u * (w - 1)))
        y = int(round((1.0 - v) * (h - 1)))
        c = img.pixelColor(x, y)
        colors[i, 0] = c.redF()
        colors[i, 1] = c.greenF()
        colors[i, 2] = c.blueF()
        colors[i, 3] = 1.0
    return colors


def load_obj_mesh(path: Path) -> ObjMesh:
    raw_vertices: List[Tuple[float, float, float]] = []
    raw_texcoords: List[Tuple[float, float]] = []
    out_vertices: List[Tuple[float, float, float]] = []
    out_texcoords: List[Tuple[float, float]] = []
    faces: List[Tuple[int, int, int]] = []
    vt_used_any = False
    remap: dict = {}
    mtllib_entries: List[str] = []

    with path.open("r", encoding="utf-8", errors="ignore") as fp:
        for raw in fp:
            line = raw.strip()
            if not line or line.startswith("#"):
                continue
            if line.startswith("mtllib "):
                mtllib_entries.extend(parse_obj_mtllib_entries(line.split(None, 1)[1]))
                continue
            if line.startswith("v "):
                parts = line.split()
                if len(parts) >= 4:
                    raw_vertices.append((float(parts[1]), float(parts[2]), float(parts[3])))
                continue
            if line.startswith("vt "):
                parts = line.split()
                if len(parts) >= 3:
                    raw_texcoords.append((float(parts[1]), float(parts[2])))
                continue
            if not line.startswith("f "):
                continue

            parts = line.split()[1:]
            if len(parts) < 3:
                continue
            corner_indices: List[int] = []
            for token in parts:
                try:
                    v_idx, vt_idx = parse_obj_face_token(token, len(raw_vertices), len(raw_texcoords))
                except Exception:
                    continue
                if not (0 <= v_idx < len(raw_vertices)):
                    continue
                if not (0 <= vt_idx < len(raw_texcoords)):
                    vt_idx = -1
                if vt_idx >= 0:
                    vt_used_any = True

                key = (v_idx, vt_idx)
                idx = remap.get(key)
                if idx is None:
                    idx = len(out_vertices)
                    remap[key] = idx
                    out_vertices.append(raw_vertices[v_idx])
                    if vt_idx >= 0:
                        out_texcoords.append(raw_texcoords[vt_idx])
                    else:
                        out_texcoords.append((0.0, 0.0))
                corner_indices.append(idx)
            if len(corner_indices) < 3:
                continue
            root = corner_indices[0]
            for i in range(1, len(corner_indices) - 1):
                faces.append((root, corner_indices[i], corner_indices[i + 1]))

    if not out_vertices or not faces:
        raise RuntimeError(f"Invalid OBJ mesh: {path}")

    texture_path = resolve_mtl_texture(path, mtllib_entries)
    texcoords = np.asarray(out_texcoords, dtype=np.float32) if vt_used_any else None
    return ObjMesh(
        vertices=np.asarray(out_vertices, dtype=np.float32),
        faces=np.asarray(faces, dtype=np.int32),
        texcoords=texcoords,
        texture_path=texture_path,
    )


def decimate_faces_stride(faces: np.ndarray, budget: int) -> np.ndarray:
    if budget <= 0 or faces.shape[0] <= budget:
        return faces
    stride = int(math.ceil(faces.shape[0] / budget))
    return faces[::stride].astype(np.int32, copy=False)


def compact_mesh(
    vertices: np.ndarray,
    faces: np.ndarray,
    texcoords: Optional[np.ndarray] = None,
    texture_path: Optional[Path] = None,
    vertex_colors: Optional[np.ndarray] = None,
) -> ObjMesh:
    used = np.unique(faces.reshape(-1))
    remap = np.full(vertices.shape[0], -1, dtype=np.int32)
    remap[used] = np.arange(used.size, dtype=np.int32)
    new_faces = remap[faces]
    new_vertices = vertices[used]
    new_texcoords = texcoords[used].astype(np.float32, copy=False) if texcoords is not None else None
    new_vertex_colors = vertex_colors[used].astype(np.float32, copy=False) if vertex_colors is not None else None
    return ObjMesh(
        vertices=new_vertices.astype(np.float32, copy=False),
        faces=new_faces.astype(np.int32, copy=False),
        texcoords=new_texcoords,
        texture_path=texture_path,
        vertex_colors=new_vertex_colors,
    )


def cross_section_spread(points: np.ndarray, axis_idx: int) -> float:
    if points.shape[0] < 8:
        return float("inf")
    ctr = points.mean(axis=0)
    centered = points - ctr[None, :]
    centered[:, axis_idx] = 0.0
    rad = np.linalg.norm(centered, axis=1)
    return float(np.mean(rad))


def auto_align_racket_vertices(vertices: np.ndarray) -> np.ndarray:
    bbox_min = np.min(vertices, axis=0)
    bbox_max = np.max(vertices, axis=0)
    ext = bbox_max - bbox_min
    axis_idx = int(np.argmax(ext))

    axis_vals = vertices[:, axis_idx]
    axis_min = float(np.min(axis_vals))
    axis_max = float(np.max(axis_vals))
    axis_span = max(1e-6, axis_max - axis_min)
    edge_ratio = 0.08
    low_thr = axis_min + edge_ratio * axis_span
    high_thr = axis_max - edge_ratio * axis_span
    low_pts = vertices[axis_vals <= low_thr]
    high_pts = vertices[axis_vals >= high_thr]
    if (low_pts.shape[0] < 32) or (high_pts.shape[0] < 32):
        edge_ratio = 0.12
        low_thr = axis_min + edge_ratio * axis_span
        high_thr = axis_max - edge_ratio * axis_span
        low_pts = vertices[axis_vals <= low_thr]
        high_pts = vertices[axis_vals >= high_thr]
    if (low_pts.shape[0] < 8) or (high_pts.shape[0] < 8):
        # Fallback for very sparse meshes.
        low_thr = float(np.percentile(axis_vals, 5))
        high_thr = float(np.percentile(axis_vals, 95))
        low_pts = vertices[axis_vals <= low_thr]
        high_pts = vertices[axis_vals >= high_thr]

    low_spread = cross_section_spread(low_pts, axis_idx)
    high_spread = cross_section_spread(high_pts, axis_idx)
    butt_pts = low_pts if low_spread <= high_spread else high_pts
    head_pts = high_pts if low_spread <= high_spread else low_pts

    butt_center = butt_pts.mean(axis=0)
    head_center = head_pts.mean(axis=0)
    centered = vertices - butt_center[None, :]

    z_model = safe_normalize(head_center - butt_center, fallback=np.array([1.0, 0.0, 0.0], dtype=np.float32))

    cov = np.cov(centered.T)
    eigvals, eigvecs = np.linalg.eigh(cov)
    y_model = eigvecs[:, int(np.argmin(eigvals))].astype(np.float32)
    y_model = y_model - float(np.dot(y_model, z_model)) * z_model
    y_model = safe_normalize(y_model, fallback=np.array([0.0, 1.0, 0.0], dtype=np.float32))
    x_model = safe_normalize(np.cross(y_model, z_model), fallback=np.array([0.0, 0.0, 1.0], dtype=np.float32))
    y_model = safe_normalize(np.cross(z_model, x_model), fallback=np.array([0.0, 1.0, 0.0], dtype=np.float32))

    model_basis = np.column_stack([x_model, y_model, z_model]).astype(np.float32)
    desired_x = np.array([-1.0, 0.0, 0.0], dtype=np.float32)
    desired_y = np.array([0.0, 1.0, 0.0], dtype=np.float32)
    desired_z = np.array([0.0, 0.0, -1.0], dtype=np.float32)
    desired_basis = np.column_stack([desired_x, desired_y, desired_z]).astype(np.float32)

    rot = desired_basis @ model_basis.T
    return (rot @ centered.T).T.astype(np.float32)


def estimate_handle_center(vertices: np.ndarray) -> np.ndarray:
    z = vertices[:, 2]
    z_min = float(np.min(z))
    z_max = float(np.max(z))
    z_span = max(1e-6, z_max - z_min)

    # Use butt-side slice to stabilize x/y center for the grip area.
    butt_slice = z >= (z_max - 0.10 * z_span)
    if int(np.count_nonzero(butt_slice)) < 100:
        butt_slice = z >= (z_max - 0.18 * z_span)
    if int(np.count_nonzero(butt_slice)) < 20:
        butt_slice = np.ones(vertices.shape[0], dtype=bool)
    butt_pts = vertices[butt_slice]
    center_xy = np.mean(butt_pts[:, :2], axis=0).astype(np.float32)

    # Place origin inside the handle instead of the butt end.
    # 9 cm is a typical badminton grip-center depth; for short models
    # clamp by ratio to avoid overshooting.
    handle_depth = min(0.09, 0.18 * z_span)
    handle_center_z = z_max - handle_depth
    return np.array([center_xy[0], center_xy[1], handle_center_z], dtype=np.float32)


def load_prepare_racket_mesh(
    obj_path: Path,
    face_budget: int,
    align_roll: float,
    align_pitch: float,
    align_yaw: float,
) -> ObjMesh:
    mesh = load_obj_mesh(obj_path)
    vertices_m = mesh.vertices * 0.01
    aligned = auto_align_racket_vertices(vertices_m)
    handle_center = estimate_handle_center(aligned)
    aligned = aligned - handle_center[None, :]
    manual_rot = euler_deg_to_rotmat(align_roll, align_pitch, align_yaw)
    aligned = rotate_points(aligned, manual_rot)
    # Stride decimation breaks surface continuity for textured meshes.
    # Keep full topology when UVs are present so texture can render correctly.
    if mesh.texcoords is not None:
        faces = mesh.faces
    else:
        faces = decimate_faces_stride(mesh.faces, face_budget)
    compact = compact_mesh(aligned, faces, texcoords=mesh.texcoords, texture_path=mesh.texture_path)
    return compact


def build_wireframe_racket_sensor_frame() -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
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
    # Keep handle center at origin for consistent IMU anchoring.
    handle_center_from_butt = 0.17 * shaft_len
    shift = np.array([0.0, 0.0, -handle_center_from_butt], dtype=np.float32)
    shaft = shaft - shift[None, :]
    head = head - shift[None, :]
    face_normal = face_normal - shift[None, :]
    return shaft, head, face_normal


def normalize_requested_opengl_mode(mode: str, no_opengl_flag: bool) -> str:
    if no_opengl_flag:
        return "none"
    return mode


def effective_qt_opengl_mode(requested_mode: str, retried: bool) -> str:
    if requested_mode == "auto":
        return "software" if retried else "desktop"
    return requested_mode


def configure_qt_opengl_env(qt_opengl_mode: str) -> None:
    if qt_opengl_mode == "desktop":
        os.environ["QT_OPENGL"] = "desktop"
    elif qt_opengl_mode == "software":
        os.environ["QT_OPENGL"] = "software"


def format_opengl_error(err: Exception) -> str:
    msg = str(err).strip()
    low = msg.lower()
    if "no module named 'opengl'" in low:
        return f"{msg}. Install dependency: {OPENGL_INSTALL_HINT}"
    return msg


def should_retry_in_software(
    requested_mode: str,
    qt_mode: str,
    already_retried: bool,
    err: Exception,
) -> bool:
    if requested_mode != "auto":
        return False
    if qt_mode != "desktop":
        return False
    if already_retried:
        return False

    msg = str(err).lower()
    if "no module named 'opengl'" in msg:
        return False

    tokens = [
        "opengl",
        "wgl",
        "glviewwidget",
        "qopengl",
        "failed to create context",
        "could not create",
        "failed creating",
    ]
    return any(t in msg for t in tokens)


def relaunch_with_software_opengl() -> None:
    script = str(Path(__file__).resolve())
    argv = sys.argv[1:]
    new_args: List[str] = []
    replaced_mode = False

    i = 0
    while i < len(argv):
        arg = argv[i]
        if arg == "--opengl-mode":
            replaced_mode = True
            new_args.extend(["--opengl-mode", "software"])
            i += 2
            continue
        if arg.startswith("--opengl-mode="):
            replaced_mode = True
            new_args.append("--opengl-mode=software")
            i += 1
            continue
        if arg == "--_opengl-retried":
            i += 1
            continue
        new_args.append(arg)
        i += 1

    if not replaced_mode:
        new_args.extend(["--opengl-mode", "software"])
    new_args.append("--_opengl-retried")

    env = os.environ.copy()
    env["QT_OPENGL"] = "software"
    print("[UI] Relaunching with software OpenGL...")
    os.execve(sys.executable, [sys.executable, script, *new_args], env)


def qmatrix_from_rot3(rot: np.ndarray, QtGui):
    m = QtGui.QMatrix4x4()
    m.setRow(0, QtGui.QVector4D(float(rot[0, 0]), float(rot[0, 1]), float(rot[0, 2]), 0.0))
    m.setRow(1, QtGui.QVector4D(float(rot[1, 0]), float(rot[1, 1]), float(rot[1, 2]), 0.0))
    m.setRow(2, QtGui.QVector4D(float(rot[2, 0]), float(rot[2, 1]), float(rot[2, 2]), 0.0))
    m.setRow(3, QtGui.QVector4D(0.0, 0.0, 0.0, 1.0))
    return m


def main() -> None:
    if os.name != "nt":
        raise RuntimeError("This script is Windows-only. Please run it in native Windows terminal.")

    ap = argparse.ArgumentParser(description="Windows high-performance dual sensor parser")
    ap.add_argument("--pressure-port", default="COM3")
    ap.add_argument("--pressure-baud", type=int, default=115200)
    ap.add_argument("--pressure-range", default="1kg", choices=["1kg", "3kg", "5kg", "10kg", "20kg", "30kg", "50kg", "skip"])
    ap.add_argument("--pressure-cmd-suffix", default="cr", choices=["none", "cr", "lf", "crlf"])

    ap.add_argument("--imu-port", default="COM5")
    ap.add_argument("--imu-baud", type=int, default=115200)
    ap.add_argument("--disable-imu", action="store_true")
    ap.add_argument("--imu-quat-world-to-sensor", action="store_true")

    ap.add_argument("--sync-max-dt-ms", type=float, default=200.0)
    ap.add_argument("--save-csv", default="")

    ap.add_argument("--fps", type=float, default=60.0)
    ap.add_argument("--opengl-mode", choices=["auto", "desktop", "software", "none"], default="auto")
    ap.add_argument("--no-opengl", action="store_true", help=argparse.SUPPRESS)
    ap.add_argument("--_opengl-retried", action="store_true", help=argparse.SUPPRESS)

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

    ap.add_argument("--racket-obj", default=str(DEFAULT_RACKET_OBJ))
    ap.add_argument("--no-racket-obj", action="store_true")
    ap.add_argument("--racket-face-budget", type=int, default=8000)
    ap.add_argument("--racket-align-roll", type=float, default=0.0)
    ap.add_argument("--racket-align-pitch", type=float, default=0.0)
    ap.add_argument("--racket-align-yaw", type=float, default=0.0)
    args = ap.parse_args()

    requested_opengl_mode = normalize_requested_opengl_mode(args.opengl_mode, args.no_opengl)
    qt_opengl_mode = effective_qt_opengl_mode(requested_opengl_mode, args._opengl_retried)
    if qt_opengl_mode in ("desktop", "software"):
        configure_qt_opengl_env(qt_opengl_mode)

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

    zero_offset = np.zeros((16, 16), dtype=np.float32)
    zero_enabled = False
    zero_calibrating = False
    zero_start_ts = 0.0
    zero_duration_sec = 5.0
    zero_samples: List[np.ndarray] = []
    zero_status_msg = UI_TEXT["zero_not_calibrated"]

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
            if zero_calibrating:
                zero_samples.append(raw_vals.copy())
                elapsed = time.perf_counter() - zero_start_ts
                if elapsed >= zero_duration_sec:
                    if len(zero_samples) >= 3:
                        stack = np.stack(zero_samples, axis=0).astype(np.float32)
                        p10 = np.percentile(stack, 10, axis=0)
                        p90 = np.percentile(stack, 90, axis=0)
                        valid = (stack >= p10[None, ...]) & (stack <= p90[None, ...])
                        clipped = np.where(valid, stack, np.nan)
                        zero_map = np.nanmean(clipped, axis=0)
                        fallback = np.mean(stack, axis=0)
                        zero_map = np.where(np.isnan(zero_map), fallback, zero_map).astype(np.float32)
                        zero_offset = zero_map
                        zero_enabled = True
                        zero_status_msg = f"Zero calibration: calibrated ({len(zero_samples)} samples)"
                    else:
                        zero_status_msg = "Zero calibration failed (not enough samples)"
                    zero_calibrating = False
                    zero_samples.clear()

            corrected = np.maximum(raw_vals - zero_offset, 0.0) if zero_enabled else raw_vals
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

    pressure_ingest.start()
    pressure_parse.start()
    if imu_ingest is not None:
        imu_ingest.start()
    if imu_parse is not None:
        imu_parse.start()

    imu_desc = "DISABLED" if args.disable_imu else f"{args.imu_port}@{args.imu_baud}"
    print(
        f"Running | Pressure {args.pressure_port}@{args.pressure_baud} range={args.pressure_range} "
        f"| IMU {imu_desc} | OpenGL request={requested_opengl_mode} qt={qt_opengl_mode}"
    )

    start_ts = time.perf_counter()
    request_software_relaunch = False
    software_relaunch_reason = ""

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

            pg.setConfigOptions(
                antialias=False,
                useOpenGL=(qt_opengl_mode != "none"),
                imageAxisOrder="row-major",
            )
            app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

            win = QtWidgets.QWidget()
            win.setWindowTitle(UI_TEXT["title"])
            vbox = QtWidgets.QVBoxLayout(win)

            top_row = QtWidgets.QHBoxLayout()
            vbox.addLayout(top_row, stretch=1)

            glw = pg.GraphicsLayoutWidget()
            top_row.addWidget(glw, stretch=1)
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
            imu3d_source = "none"
            imu_init_rot_world: Optional[np.ndarray] = None
            last_drawn_imu_ts: Optional[float] = None
            last_drawn_pressure_ts: Optional[float] = None
            imu_zero_status_msg = "IMU zero: waiting first frame"

            sensor0_to_view = np.array(
                [
                    [1.0, 0.0, 0.0],
                    [0.0, 0.0, 1.0],
                    [0.0, -1.0, 0.0],
                ],
                dtype=np.float32,
            )

            wire_shaft_sensor, wire_head_sensor, wire_face_sensor = build_wireframe_racket_sensor_frame()
            wire_shaft_item = None
            wire_head_item = None
            wire_face_item = None
            racket_mesh_item = None
            QtGui = None

            if (not args.disable_imu) and (qt_opengl_mode != "none"):
                try:
                    import pyqtgraph.opengl as gl
                    from pyqtgraph.Qt import QtGui as _QtGui

                    QtGui = _QtGui
                    try:
                        import OpenGL.GL  # noqa: F401
                    except Exception as dep_err:
                        raise RuntimeError(format_opengl_error(dep_err)) from dep_err

                    imu3d_enabled = True
                    imu_view = gl.GLViewWidget()
                    try:
                        imu_view.setBackgroundColor((255, 255, 255, 255))
                    except Exception:
                        pass
                    imu_view.opts["distance"] = 2.0
                    imu_view.opts["elevation"] = 18.0
                    imu_view.opts["azimuth"] = -40.0
                    top_row.addWidget(imu_view, stretch=1)

                    axis = gl.GLAxisItem()
                    axis.setSize(0.25, 0.25, 0.25)
                    imu_view.addItem(axis)

                    if not args.no_racket_obj:
                        obj_path = Path(args.racket_obj)
                        try:
                            mesh = load_prepare_racket_mesh(
                                obj_path=obj_path,
                                face_budget=max(100, args.racket_face_budget),
                                align_roll=args.racket_align_roll,
                                align_pitch=args.racket_align_pitch,
                                align_yaw=args.racket_align_yaw,
                            )
                            if mesh.texture_path is not None and mesh.texcoords is not None:
                                vcolors = sample_vertex_colors_from_texture(mesh.texcoords, mesh.texture_path, QtGui)
                                if vcolors is not None:
                                    mesh.vertex_colors = vcolors

                            mesh_kwargs = {
                                "vertexes": mesh.vertices,
                                "faces": mesh.faces,
                                "smooth": False,
                                "drawEdges": False,
                                "drawFaces": True,
                                "shader": "shaded",
                            }
                            if mesh.vertex_colors is not None:
                                # Use direct vertex colors for texture preview; avoid lighting darkening.
                                mesh_kwargs["vertexColors"] = mesh.vertex_colors
                                mesh_kwargs["smooth"] = True
                                mesh_kwargs["shader"] = None
                            else:
                                mesh_kwargs["color"] = (0.87, 0.87, 0.92, 1.0)

                            racket_mesh_item = gl.GLMeshItem(**mesh_kwargs)
                            imu_view.addItem(racket_mesh_item)
                            if mesh.vertex_colors is not None and mesh.texture_path is not None:
                                imu3d_source = f"obj:{obj_path.name} tex:{mesh.texture_path.name} faces={mesh.faces.shape[0]}"
                            else:
                                imu3d_source = f"obj:{obj_path.name} faces={mesh.faces.shape[0]}"
                        except Exception as mesh_err:
                            imu3d_error_msg = f"OBJ fallback: {mesh_err}"

                    if racket_mesh_item is None:
                        wire_shaft_item = gl.GLLinePlotItem(
                            pos=rotate_points(wire_shaft_sensor, sensor0_to_view),
                            color=(1.0, 0.9, 0.2, 1.0),
                            width=4.0,
                            antialias=True,
                            mode="line_strip",
                        )
                        wire_head_item = gl.GLLinePlotItem(
                            pos=rotate_points(wire_head_sensor, sensor0_to_view),
                            color=(0.1, 0.8, 1.0, 1.0),
                            width=2.0,
                            antialias=True,
                            mode="line_strip",
                        )
                        wire_face_item = gl.GLLinePlotItem(
                            pos=rotate_points(wire_face_sensor, sensor0_to_view),
                            color=(1.0, 0.3, 0.3, 1.0),
                            width=2.0,
                            antialias=True,
                            mode="line_strip",
                        )
                        imu_view.addItem(wire_shaft_item)
                        imu_view.addItem(wire_head_item)
                        imu_view.addItem(wire_face_item)
                        if imu3d_source == "none":
                            imu3d_source = "wireframe"
                except Exception as e:
                    if should_retry_in_software(requested_opengl_mode, qt_opengl_mode, args._opengl_retried, e):
                        raise OpenGLAutoRetryRequested(str(e)) from e
                    imu3d_enabled = False
                    imu3d_error_msg = format_opengl_error(e)

            if (not args.disable_imu) and (not imu3d_enabled):
                if qt_opengl_mode == "none":
                    imu3d_error_msg = "disabled by --opengl-mode none"
                elif not imu3d_error_msg:
                    imu3d_error_msg = f"OpenGL backend unavailable. Install dependency: {OPENGL_INSTALL_HINT}"
                imu_warn = QtWidgets.QLabel(f"{UI_TEXT['imu3d_unavailable_prefix']}\n{imu3d_error_msg}")
                imu_warn.setWordWrap(True)
                imu_warn.setStyleSheet("font-family: Consolas, monospace; font-size: 11px;")
                top_row.addWidget(imu_warn, stretch=1)

            lbl = QtWidgets.QLabel(UI_TEXT["waiting"])
            lbl.setStyleSheet("font-family: Consolas, monospace; font-size: 12px;")
            vbox.addWidget(lbl)

            btn_row = QtWidgets.QHBoxLayout()
            btn_zero_calib = QtWidgets.QPushButton(UI_TEXT["zero_btn_start"])
            btn_zero_clear = QtWidgets.QPushButton(UI_TEXT["zero_btn_clear"])
            btn_imu_reset = QtWidgets.QPushButton(UI_TEXT["imu_btn_reset"])
            btn_row.addWidget(btn_zero_calib)
            btn_row.addWidget(btn_zero_clear)
            btn_row.addWidget(btn_imu_reset)
            btn_row.addStretch(1)
            vbox.addLayout(btn_row)

            timer = QtCore.QTimer()
            timer.setInterval(max(1, int(1000 / max(1.0, args.fps))))
            try:
                timer.setTimerType(QtCore.Qt.PreciseTimer)
            except Exception:
                pass

            def start_zero_calibration():
                nonlocal zero_enabled, zero_calibrating, zero_start_ts, zero_samples, zero_status_msg
                with lock:
                    zero_enabled = False
                    zero_calibrating = True
                    zero_start_ts = time.perf_counter()
                    zero_samples = []
                    zero_status_msg = f"{UI_TEXT['zero_running']} (5.0s)"
                print("[PRESSURE] Zero calibration started (5s)")

            def clear_zero_calibration():
                nonlocal zero_enabled, zero_calibrating, zero_samples, zero_offset, zero_status_msg
                with lock:
                    zero_enabled = False
                    zero_calibrating = False
                    zero_samples.clear()
                    zero_offset = np.zeros((16, 16), dtype=np.float32)
                    zero_status_msg = UI_TEXT["zero_cleared"]
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
                        imu_zero_status_msg = "IMU zero: locked"
                    rot_rel = imu_init_rot_world.T @ rot_sw
                    rot_view = sensor0_to_view @ rot_rel

                    if racket_mesh_item is not None and QtGui is not None:
                        transform = qmatrix_from_rot3(rot_view, QtGui)
                        racket_mesh_item.setTransform(transform)
                    else:
                        if wire_shaft_item is not None:
                            wire_shaft_item.setData(pos=rotate_points(wire_shaft_sensor, rot_view))
                        if wire_head_item is not None:
                            wire_head_item.setData(pos=rotate_points(wire_head_sensor, rot_view))
                        if wire_face_item is not None:
                            wire_face_item.setData(pos=rotate_points(wire_face_sensor, rot_view))
                    last_drawn_imu_ts = i.host_ts

                elapsed = max(1e-6, time.perf_counter() - start_ts)
                lines = [
                    f"P={st['pressure_frames']} ({st['pressure_frames']/elapsed:.1f} fps) parsed={st['pressure_parsed_frames']} drop={st['pressure_drop_chunks']}",
                    f"IMU={st['imu_frames']} ({st['imu_frames']/elapsed:.1f} fps) parsed={st['imu_parsed_frames']} drop={st['imu_drop_chunks']}",
                    f"sync={st['synced']}/{st['unsynced']}",
                    f"OpenGL request={requested_opengl_mode} qt={qt_opengl_mode}",
                ]
                if z_calib:
                    lines.append(f"{z_status} | remain {z_remaining:0.1f}s | samples {z_samples_n}")
                else:
                    lines.append(f"{z_status} | {UI_TEXT['zero_enabled'] if z_enabled else UI_TEXT['zero_disabled']}")

                if args.disable_imu:
                    lines.append("IMU 3D: disabled")
                elif imu3d_enabled:
                    lines.append(f"IMU 3D: on | {imu_zero_status_msg} | source={imu3d_source}")
                else:
                    err_short = imu3d_error_msg.splitlines()[0] if imu3d_error_msg else "OpenGL backend unavailable"
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
            win.resize(1260, 780)
            win.show()
            app.exec()
    except OpenGLAutoRetryRequested as e:
        request_software_relaunch = True
        software_relaunch_reason = str(e)
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

    if request_software_relaunch:
        print(f"[UI] Desktop OpenGL failed, retrying with software backend: {software_relaunch_reason}")
        relaunch_with_software_opengl()


if __name__ == "__main__":
    main()
