#!/usr/bin/env python3
"""
Run Xsens MVN SMPL-H retargeting and SAGE dual-sensor capture in one Qt window.

The viewer uses one software timeline based on host time.perf_counter().
Each source can connect late or drop out independently; rendering samples the
nearest already-arrived frame for each source at now - sync_latency.
"""

from __future__ import annotations

import argparse
import csv
import os
import platform
import queue
import signal
import threading
import time
from collections import deque
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Deque, Dict, List, Optional, Tuple

# Windows conda stacks can load both LLVM OpenMP and Intel OpenMP when torch,
# numpy/scipy, and Qt/OpenGL are used in one process. This combined viewer is
# intentionally single-process, so set the compatibility escape hatch before
# importing numpy/torch-backed project modules.
os.environ.setdefault("KMP_DUPLICATE_LIB_OK", "TRUE")

import numpy as np

import capture_xsens_mvn_smplh_realtime as xsens_rt
from run_dual_sensor_sage import (
    CLOCK_SYNC_HEADER,
    DEFAULT_RACKET_OBJ,
    OPENGL_INSTALL_HINT,
    UI_TEXT,
    CorrectedImuFrame,
    ImuFrame,
    M1616MParser,
    ParserThread,
    PressureFrame,
    SerialIngestThread,
    TB100Parser,
    build_bilinear_plan,
    build_racket_init_to_human_rot,
    build_wireframe_racket_sensor_frame,
    configure_pressure_sensor_range,
    configure_qt_opengl_env,
    effective_qt_opengl_mode,
    format_opengl_error,
    host_clock_row,
    imu_rot_sensor_to_world,
    is_wsl,
    load_prepare_racket_mesh,
    normalize_requested_opengl_mode,
    pressure_for_csv,
    pressure_for_display,
    qmatrix_from_rot3,
    rotate_points,
    rotmat_to_quat_wxyz,
    sample_vertex_colors_from_texture,
)


HUMAN0_TO_VIEW = np.array(
    [
        [1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
        [0.0, -1.0, 0.0],
    ],
    dtype=np.float32,
)

# This is the exact coordinate conversion used by
# xsens_rt.smplh_vertices_to_pyqtgraph(). Use it for every object rendered in
# the SMPL-H view so racket positions and rotations share the body's frame.
SMPL_TO_VIEW = np.array(
    [
        [1.0, 0.0, 0.0],
        [0.0, 0.0, -1.0],
        [0.0, 1.0, 0.0],
    ],
    dtype=np.float32,
)


@dataclass
class TimelineItem:
    host_ts: float
    host_wall_ts_s: float
    data: Any


class TimelineBuffer:
    def __init__(self, maxlen: int) -> None:
        self.items: Deque[TimelineItem] = deque(maxlen=max(1, maxlen))

    def append(self, item: TimelineItem) -> None:
        if not np.isfinite(item.host_ts):
            return
        self.items.append(item)

    def latest(self) -> Optional[TimelineItem]:
        return self.items[-1] if self.items else None

    def sample_at_or_before(self, target_ts: float) -> Optional[TimelineItem]:
        if not self.items:
            return None
        for item in reversed(self.items):
            if item.host_ts <= target_ts:
                return item
        return self.items[0]


@dataclass
class PressureVisualData:
    display_values: np.ndarray
    record: PressureFrame
    interaction_active: bool
    interaction_pressure_g: float


@dataclass
class XsensVisualData:
    sample_counter: int
    vertices: np.ndarray
    right_palm_position: Optional[np.ndarray]


def pressure_interaction_state(
    was_active: bool,
    peak_pressure_g: float,
    attach_threshold_g: float,
    release_threshold_g: float,
) -> bool:
    if was_active:
        return peak_pressure_g > release_threshold_g
    return peak_pressure_g >= attach_threshold_g


def unique_dir(path: Path) -> Path:
    if not path.exists():
        return path
    for idx in range(1, 1000):
        candidate = path.with_name(f"{path.name}_{idx:02d}")
        if not candidate.exists():
            return candidate
    raise RuntimeError(f"Could not allocate unique output folder for {path}")


class DualSensorRealtimeRuntime:
    def __init__(self, args: argparse.Namespace, stop_event: threading.Event) -> None:
        self.args = args
        self.stop_event = stop_event
        self.lock = threading.Lock()
        self.stats: Dict[str, int] = {
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
        self.pressure_buffer = TimelineBuffer(maxlen=args.timeline_buffer_frames)
        self.imu_buffer = TimelineBuffer(maxlen=args.timeline_buffer_frames)
        self.pressure_valid_buffer: Deque[PressureFrame] = deque(maxlen=20000)
        self.sync_pending_imu: Deque[CorrectedImuFrame] = deque()

        self.interp_scale = max(1, args.interp_scale)
        self.display_hw = 16 * self.interp_scale
        self.pressure_interp_plan = build_bilinear_plan(16, 16, self.interp_scale)

        self.zero_offset = np.zeros((16, 16), dtype=np.float32)
        self.zero_enabled = False
        self.zero_calibrating = not args.disable_pressure
        self.zero_start_ts = time.perf_counter()
        self.zero_duration_sec = 3.0
        self.zero_samples: List[np.ndarray] = []
        self.zero_status_msg = (
            f"{UI_TEXT['zero_running']} (startup 3.0s)"
            if self.zero_calibrating
            else "Zero calibration: pressure disabled"
        )
        self.pressure_recording_enabled = args.disable_pressure
        self.startup_zero_pending = not args.disable_pressure
        self.interaction_active = False
        self.interaction_pressure_g = 0.0

        self.imu_init_rot_world: Optional[np.ndarray] = None
        self.imu_zero_status_msg = "IMU zero: waiting first frame"
        self.imu_zero_seq = 0
        self.racket_init_to_human = build_racket_init_to_human_rot()

        self.csv_dir: Optional[Path] = None
        self.csv_files: List[Any] = []
        self.pressure_csv_writer = None
        self.imu_csv_writer = None
        self.sync_csv_writer = None
        self.clock_sync_writer = None
        self.last_clock_sync_ts = time.perf_counter()
        if not args.no_dual_csv:
            self._open_csv_files(args.dual_csv_dir or (args.output_root / "dual_sensor"))

        self.q_pressure: queue.Queue = queue.Queue(maxsize=args.queue_size)
        self.q_imu: queue.Queue = queue.Queue(maxsize=args.queue_size)
        self.threads: List[threading.Thread] = []
        self._build_threads()

    def _open_csv_files(self, csv_dir: Path) -> None:
        self.csv_dir = unique_dir(csv_dir)
        self.csv_dir.mkdir(parents=True, exist_ok=False)
        pressure_fp = (self.csv_dir / "pressure.csv").open("w", newline="", encoding="utf-8")
        imu_fp = (self.csv_dir / "imu.csv").open("w", newline="", encoding="utf-8")
        sync_fp = (self.csv_dir / "sync.csv").open("w", newline="", encoding="utf-8")
        clock_fp = (self.csv_dir / "clock_sync.csv").open("w", newline="", encoding="utf-8")
        self.csv_files.extend([pressure_fp, imu_fp, sync_fp, clock_fp])
        self.pressure_csv_writer = csv.writer(pressure_fp)
        self.imu_csv_writer = csv.writer(imu_fp)
        self.sync_csv_writer = csv.writer(sync_fp)
        self.clock_sync_writer = csv.writer(clock_fp)

        pressure_header = ["pressure_host_ts", "pressure_host_wall_ts_s", "pressure_checksum_ok"] + [
            f"p_{i}" for i in range(256)
        ]
        imu_header = [
            "imu_host_ts",
            "imu_host_wall_ts_s",
            "imu_device_ts_us",
            "imu_qos",
            "imu_crc_ok",
            "imu_simple_ok",
            "imu_zero_seq",
            "imu_ax_g",
            "imu_ay_g",
            "imu_az_g",
            "imu_qw",
            "imu_qx",
            "imu_qy",
            "imu_qz",
        ]
        sync_header = (
            imu_header
            + [
                "pressure_matched",
                "pressure_host_ts",
                "pressure_host_wall_ts_s",
                "sync_lag_ms",
                "sync_lag_wall_ms",
                "pressure_checksum_ok",
            ]
            + [f"p_{i}" for i in range(256)]
        )
        self.pressure_csv_writer.writerow(pressure_header)
        self.imu_csv_writer.writerow(imu_header)
        self.sync_csv_writer.writerow(sync_header)
        self.clock_sync_writer.writerow(CLOCK_SYNC_HEADER)
        self.clock_sync_writer.writerow(host_clock_row("capture_start"))
        print(f"[DUAL] CSV output: {self.csv_dir}")

    def _build_threads(self) -> None:
        suffix_map = {"none": b"", "cr": b"\r", "lf": b"\n", "crlf": b"\r\n"}

        def on_pressure_open(ser: Any) -> None:
            configure_pressure_sensor_range(
                ser,
                self.args.pressure_range,
                suffix_map[self.args.pressure_cmd_suffix],
            )

        if not self.args.disable_pressure:
            self.threads.extend(
                [
                    SerialIngestThread(
                        name="PRESSURE",
                        port=self.args.pressure_port,
                        baud=self.args.pressure_baud,
                        out_q=self.q_pressure,
                        stop_event=self.stop_event,
                        on_open=on_pressure_open,
                        read_chunk=self.args.read_chunk,
                        stats=self.stats,
                    ),
                    ParserThread(
                        name="PRESSURE",
                        in_q=self.q_pressure,
                        parser=M1616MParser(),
                        on_frame=self.on_pressure,
                        stop_event=self.stop_event,
                        stats=self.stats,
                    ),
                ]
            )

        if not self.args.disable_imu:
            self.threads.extend(
                [
                    SerialIngestThread(
                        name="IMU",
                        port=self.args.imu_port,
                        baud=self.args.imu_baud,
                        out_q=self.q_imu,
                        stop_event=self.stop_event,
                        read_chunk=self.args.read_chunk,
                        stats=self.stats,
                    ),
                    ParserThread(
                        name="IMU",
                        in_q=self.q_imu,
                        parser=TB100Parser(),
                        on_frame=self.on_imu,
                        stop_event=self.stop_event,
                        stats=self.stats,
                    ),
                ]
            )

    def start(self) -> None:
        for thread in self.threads:
            thread.start()
        imu_desc = "DISABLED" if self.args.disable_imu else f"{self.args.imu_port} @ {self.args.imu_baud}"
        p_desc = "DISABLED" if self.args.disable_pressure else f"{self.args.pressure_port} @ {self.args.pressure_baud}"
        print(f"[DUAL] Running | Pressure {p_desc} range={self.args.pressure_range} | IMU {imu_desc}")

    def stop(self) -> None:
        self.stop_event.set()
        for thread in self.threads:
            if thread.ident is not None:
                thread.join(timeout=2.0)
        with self.lock:
            self.flush_sync_rows(float("inf"), force=True)
        if self.clock_sync_writer is not None:
            self.clock_sync_writer.writerow(host_clock_row("capture_stop"))
        for fp in self.csv_files:
            fp.close()
        print("[DUAL] Stopped")

    def build_imu_row(self, frame: CorrectedImuFrame) -> List[object]:
        qw, qx, qy, qz = frame.quat_human_wxyz
        ax, ay, az = frame.acc_human_g
        return [
            frame.host_ts,
            frame.host_wall_ts_s,
            frame.device_ts_us,
            frame.qos,
            int(frame.crc_ok),
            int(frame.simple_sum_ok),
            frame.zero_seq,
            ax,
            ay,
            az,
            qw,
            qx,
            qy,
            qz,
        ]

    @staticmethod
    def build_pressure_cells(frame: Optional[PressureFrame]) -> List[object]:
        if frame is None or frame.csv_values is None:
            return [""] * 256
        return frame.csv_values.reshape(-1).tolist()

    def flush_sync_rows(self, current_host_ts: float, force: bool = False) -> None:
        cutoff_ts = float("inf") if force else (current_host_ts - self.args.sync_max_dt_ms / 1000.0)
        while self.sync_pending_imu and (force or self.sync_pending_imu[0].host_ts <= cutoff_ts):
            imu_frame = self.sync_pending_imu.popleft()
            pressure_frame = self._nearest_pressure(imu_frame.host_ts, self.args.sync_max_dt_ms / 1000.0)
            if pressure_frame is None:
                self.stats["unsynced"] += 1
            else:
                self.stats["synced"] += 1
            if self.sync_csv_writer is not None:
                lag_ms = (pressure_frame.host_ts - imu_frame.host_ts) * 1000.0 if pressure_frame is not None else float("nan")
                lag_wall_ms = (
                    (pressure_frame.host_wall_ts_s - imu_frame.host_wall_ts_s) * 1000.0
                    if pressure_frame is not None
                    else float("nan")
                )
                row = self.build_imu_row(imu_frame)
                row += [
                    int(pressure_frame is not None),
                    pressure_frame.host_ts if pressure_frame is not None else "",
                    pressure_frame.host_wall_ts_s if pressure_frame is not None else "",
                    lag_ms,
                    lag_wall_ms,
                    int(pressure_frame.checksum_ok) if pressure_frame is not None else "",
                ]
                row += self.build_pressure_cells(pressure_frame)
                self.sync_csv_writer.writerow(row)

        if force:
            min_keep_ts = float("inf")
        elif self.sync_pending_imu:
            min_keep_ts = self.sync_pending_imu[0].host_ts - self.args.sync_max_dt_ms / 1000.0
        else:
            min_keep_ts = current_host_ts - self.args.sync_max_dt_ms / 1000.0
        while self.pressure_valid_buffer and self.pressure_valid_buffer[0].host_ts < min_keep_ts:
            self.pressure_valid_buffer.popleft()

    def _nearest_pressure(self, ts: float, max_dt_sec: float) -> Optional[PressureFrame]:
        if not self.pressure_valid_buffer:
            return None
        best = min(self.pressure_valid_buffer, key=lambda frame: abs(frame.host_ts - ts))
        return best if abs(best.host_ts - ts) <= max_dt_sec else None

    def on_pressure(self, frame: PressureFrame) -> None:
        with self.lock:
            raw_vals = frame.values
            if self.zero_calibrating:
                self.zero_samples.append(raw_vals.copy())
                elapsed = time.perf_counter() - self.zero_start_ts
                if elapsed >= self.zero_duration_sec:
                    if len(self.zero_samples) >= 3:
                        stack = np.stack(self.zero_samples, axis=0).astype(np.float32)
                        p10 = np.percentile(stack, 10, axis=0)
                        p90 = np.percentile(stack, 90, axis=0)
                        valid = (stack >= p10[None, ...]) & (stack <= p90[None, ...])
                        clipped = np.where(valid, stack, np.nan)
                        zero_map = np.nanmean(clipped, axis=0)
                        fallback = np.mean(stack, axis=0)
                        zero_map = np.where(np.isnan(zero_map), fallback, zero_map).astype(np.float32)
                        self.zero_offset = zero_map
                        self.zero_enabled = True
                        self.zero_status_msg = f"Zero calibration: calibrated ({len(self.zero_samples)} samples)"
                    else:
                        self.zero_status_msg = "Zero calibration failed (not enough samples)"
                        self.zero_offset = np.zeros((16, 16), dtype=np.float32)
                        self.zero_enabled = False
                    self.zero_calibrating = False
                    self.zero_samples.clear()
                    if self.startup_zero_pending:
                        self.pressure_valid_buffer.clear()
                        self.sync_pending_imu.clear()
                        self.pressure_recording_enabled = True
                        self.startup_zero_pending = False
                        print("[PRESSURE] Startup zero calibration completed; pressure recording enabled")

            corrected = np.maximum(raw_vals - self.zero_offset, 0.0) if self.zero_enabled else raw_vals
            self.interaction_pressure_g = float(np.max(corrected))
            if self.pressure_recording_enabled and frame.checksum_ok and not self.zero_calibrating:
                self.interaction_active = pressure_interaction_state(
                    self.interaction_active,
                    self.interaction_pressure_g,
                    self.args.interaction_pressure_threshold,
                    self.args.interaction_release_threshold,
                )
            elif self.zero_calibrating:
                self.interaction_active = False
            csv_vals = pressure_for_csv(
                corrected,
                self.args.transpose,
                self.args.flipud,
                self.args.fliplr,
                rotate_ccw90=(not self.args.no_rotate_ccw90),
            )
            display_vals = pressure_for_display(
                corrected,
                self.args.transpose,
                self.args.flipud,
                self.args.fliplr,
                rotate_ccw90=(not self.args.no_rotate_ccw90),
                interp_plan=self.pressure_interp_plan,
            )
            pressure_record = PressureFrame(
                host_ts=frame.host_ts,
                host_wall_ts_s=frame.host_wall_ts_s,
                values=corrected,
                checksum_ok=frame.checksum_ok,
                csv_values=csv_vals,
            )
            self.pressure_buffer.append(
                TimelineItem(
                    host_ts=frame.host_ts,
                    host_wall_ts_s=frame.host_wall_ts_s,
                    data=PressureVisualData(
                        display_values=display_vals,
                        record=pressure_record,
                        interaction_active=self.interaction_active,
                        interaction_pressure_g=self.interaction_pressure_g,
                    ),
                )
            )

            self.stats["pressure_frames"] += 1
            if not frame.checksum_ok:
                self.stats["pressure_bad_checksum"] += 1
            if self.pressure_recording_enabled and frame.checksum_ok:
                self.pressure_valid_buffer.append(pressure_record)
            if self.pressure_recording_enabled and self.pressure_csv_writer is not None:
                row = [frame.host_ts, frame.host_wall_ts_s, int(frame.checksum_ok)] + csv_vals.reshape(-1).tolist()
                self.pressure_csv_writer.writerow(row)
            if self.pressure_recording_enabled:
                self.flush_sync_rows(frame.host_ts)

    def on_imu(self, frame: ImuFrame) -> None:
        with self.lock:
            rot_sw = imu_rot_sensor_to_world(frame.quat_wxyz, self.args.imu_quat_world_to_sensor)
            if self.imu_init_rot_world is None:
                self.imu_init_rot_world = rot_sw
                self.imu_zero_seq += 1
                self.imu_zero_status_msg = "IMU zero: locked"
            rot_rel = self.imu_init_rot_world.T @ rot_sw
            rot_human = self.racket_init_to_human @ rot_rel
            quat_human = rotmat_to_quat_wxyz(rot_human)
            acc_human_np = (rot_human @ np.asarray(frame.acc_g, dtype=np.float32)).astype(np.float32)
            corrected_frame = CorrectedImuFrame(
                host_ts=frame.host_ts,
                host_wall_ts_s=frame.host_wall_ts_s,
                device_ts_us=frame.device_ts_us,
                qos=frame.qos,
                quat_human_wxyz=quat_human,
                acc_human_g=(float(acc_human_np[0]), float(acc_human_np[1]), float(acc_human_np[2])),
                rot_human=rot_human,
                crc_ok=frame.crc_ok,
                simple_sum_ok=frame.simple_sum_ok,
                zero_seq=self.imu_zero_seq,
            )
            self.imu_buffer.append(TimelineItem(frame.host_ts, frame.host_wall_ts_s, corrected_frame))
            self.stats["imu_frames"] += 1
            if not frame.crc_ok:
                self.stats["imu_bad_crc"] += 1
            if not frame.simple_sum_ok:
                self.stats["imu_bad_simple"] += 1
            if self.imu_csv_writer is not None:
                self.imu_csv_writer.writerow(self.build_imu_row(corrected_frame))
            if self.pressure_recording_enabled:
                self.sync_pending_imu.append(corrected_frame)
                self.flush_sync_rows(frame.host_ts)

    def start_zero_calibration(self, duration_sec: float = 5.0) -> None:
        with self.lock:
            self.zero_duration_sec = duration_sec
            self.zero_enabled = False
            self.zero_calibrating = True
            self.zero_start_ts = time.perf_counter()
            self.zero_samples = []
            self.zero_status_msg = f"{UI_TEXT['zero_running']} ({duration_sec:.1f}s)"
            self.interaction_active = False
        print(f"[PRESSURE] Zero calibration started ({duration_sec:.1f}s)")

    def clear_zero_calibration(self) -> None:
        with self.lock:
            self.zero_enabled = False
            self.zero_calibrating = False
            self.zero_samples.clear()
            self.zero_offset = np.zeros((16, 16), dtype=np.float32)
            self.zero_status_msg = UI_TEXT["zero_cleared"]
            self.interaction_active = False
        print("[PRESSURE] Zero calibration cleared")

    def reset_imu_zero_pose(self) -> None:
        with self.lock:
            self.imu_init_rot_world = None
            self.imu_zero_status_msg = "IMU zero: waiting next frame"
        print("[IMU] Zero pose reset")

    def sample_for_timeline(self, target_ts: float) -> Dict[str, Any]:
        with self.lock:
            pressure_item = self.pressure_buffer.sample_at_or_before(target_ts)
            imu_item = self.imu_buffer.sample_at_or_before(target_ts)
            return {
                "pressure": pressure_item,
                "imu": imu_item,
                "stats": dict(self.stats),
                "zero_enabled": self.zero_enabled,
                "zero_calibrating": self.zero_calibrating,
                "zero_status": self.zero_status_msg,
                "zero_remaining": max(0.0, self.zero_duration_sec - (time.perf_counter() - self.zero_start_ts))
                if self.zero_calibrating
                else 0.0,
                "zero_samples": len(self.zero_samples),
                "imu_zero_status": self.imu_zero_status_msg,
                "interaction_active": self.interaction_active,
                "interaction_pressure_g": self.interaction_pressure_g,
            }

    def maybe_write_clock_sync_sample(self) -> None:
        if self.clock_sync_writer is None or self.args.clock_sync_interval_sec <= 0:
            return
        now = time.perf_counter()
        if now - self.last_clock_sync_ts >= self.args.clock_sync_interval_sec:
            self.clock_sync_writer.writerow(host_clock_row("periodic"))
            self.last_clock_sync_ts = now


def build_xsens_args(args: argparse.Namespace, viewer_enabled: bool) -> argparse.Namespace:
    xsens_output_dir = args.xsens_output_dir or (args.output_root / "xsens_mvn_udp")
    return argparse.Namespace(
        bind_ip=args.xsens_bind_ip,
        port=args.xsens_port,
        output_dir=xsens_output_dir,
        pt_output=args.xsens_pt_output,
        accept_from=args.xsens_accept_from,
        duration_s=args.duration_sec if args.duration_sec > 0 else None,
        max_packets=args.xsens_max_packets,
        max_samples=args.xsens_max_samples,
        socket_buffer_bytes=args.xsens_socket_buffer_bytes,
        recv_bytes=args.xsens_recv_bytes,
        pending_timeout_s=args.xsens_pending_timeout_s,
        flush_every=args.xsens_flush_every,
        print_every=args.xsens_print_every,
        clock_sync_interval_s=args.clock_sync_interval_sec,
        segment_id_base=args.xsens_segment_id_base,
        body_model=args.xsens_body_model,
        device=args.xsens_device,
        num_betas=args.xsens_num_betas,
        arm_correction=args.xsens_arm_correction,
        arm_twist_filter=args.xsens_arm_twist_filter,
        viewer="pyqtgraph" if viewer_enabled else "none",
        display_fps=args.fps,
        pt_checkpoint_every_s=args.xsens_pt_checkpoint_every_s,
        pt_checkpoint_every_frames=args.xsens_pt_checkpoint_every_frames,
        retarget_queue_size=args.xsens_retarget_queue_size,
        keep_stale_retarget_frames=args.xsens_keep_stale_retarget_frames,
        status_every_s=args.status_interval,
    )


def source_lag_text(sample: Optional[TimelineItem], target_ts: float) -> str:
    if sample is None:
        return "none"
    return f"{(sample.host_ts - target_ts) * 1000.0:+.1f}ms"


def qmatrix_from_pose(rot: np.ndarray, translation: np.ndarray, QtGui):
    transform = qmatrix_from_rot3(rot, QtGui)
    transform.setColumn(
        3,
        QtGui.QVector4D(
            float(translation[0]),
            float(translation[1]),
            float(translation[2]),
            1.0,
        ),
    )
    return transform


def poll_xsens_mesh(
    runtime: Optional[xsens_rt.RealtimeSmplhRuntime],
    mesh_buffer: TimelineBuffer,
    last_seen: Dict[str, Optional[int]],
) -> None:
    if runtime is None:
        return
    with runtime.state.lock:
        sample_counter = runtime.state.latest_vertices_sample_counter
        host_ts = runtime.state.latest_vertices_host_perf_ts_s
        host_wall_ts_s = runtime.state.latest_vertices_host_wall_ts_s
        vertices = None if runtime.state.latest_vertices is None else runtime.state.latest_vertices.copy()
        right_palm_position = (
            None
            if runtime.state.latest_right_palm_position is None
            else runtime.state.latest_right_palm_position.copy()
        )
    if sample_counter is None or host_ts is None or vertices is None:
        return
    if last_seen.get("sample_counter") == sample_counter:
        return
    mesh_buffer.append(
        TimelineItem(
            host_ts=host_ts,
            host_wall_ts_s=float("nan") if host_wall_ts_s is None else host_wall_ts_s,
            data=XsensVisualData(
                sample_counter=sample_counter,
                vertices=vertices,
                right_palm_position=right_palm_position,
            ),
        )
    )
    last_seen["sample_counter"] = sample_counter


def make_stable_gl_view_class(gl):
    class StableGLViewWidget(gl.GLViewWidget):
        @staticmethod
        def _event_pos(event):
            return event.position() if hasattr(event, "position") else event.localPos()

        def mousePressEvent(self, event):
            self.mousePos = self._event_pos(event)
            event.accept()

        def mouseReleaseEvent(self, event):
            if hasattr(self, "mousePos"):
                del self.mousePos
            event.accept()

    return StableGLViewWidget


def run_combined_viewer(
    args: argparse.Namespace,
    xsens_runtime: Optional[xsens_rt.RealtimeSmplhRuntime],
    dual_runtime: Optional[DualSensorRealtimeRuntime],
    global_stop_event: threading.Event,
    requested_opengl_mode: str,
    qt_opengl_mode: str,
    startup_errors: List[str],
) -> None:
    import pyqtgraph as pg
    from pyqtgraph.Qt import QtCore, QtWidgets

    pg.setConfigOptions(
        antialias=False,
        useOpenGL=(qt_opengl_mode != "none"),
        imageAxisOrder="row-major",
    )

    gl = None
    QtGui = None
    gl_error = ""
    if qt_opengl_mode != "none":
        try:
            import pyqtgraph.opengl as _gl
            from pyqtgraph.Qt import QtGui as _QtGui

            gl = _gl
            QtGui = _QtGui
        except Exception as exc:
            gl_error = format_opengl_error(exc)

    StableGLViewWidget = make_stable_gl_view_class(gl) if gl is not None else None
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])

    def request_stop() -> None:
        global_stop_event.set()
        if xsens_runtime is not None:
            xsens_runtime.stop_event.set()
        if dual_runtime is not None:
            dual_runtime.stop_event.set()

    app.aboutToQuit.connect(request_stop)

    win = QtWidgets.QWidget()
    win.setWindowTitle("SAGE + Xsens Realtime Timeline Sync")
    layout = QtWidgets.QVBoxLayout(win)
    splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
    layout.addWidget(splitter, stretch=1)

    pressure_panel = QtWidgets.QWidget()
    pressure_layout = QtWidgets.QVBoxLayout(pressure_panel)
    pressure_label = QtWidgets.QLabel("Pressure")
    pressure_label.setStyleSheet("font-weight: 600;")
    pressure_layout.addWidget(pressure_label)
    pressure_widget = pg.GraphicsLayoutWidget()
    pressure_layout.addWidget(pressure_widget, stretch=1)
    pressure_vb = pressure_widget.addViewBox(lockAspect=True)
    pressure_vb.setMouseEnabled(x=False, y=False)
    pressure_img = pg.ImageItem()
    pressure_vb.addItem(pressure_img)
    display_hw = dual_runtime.display_hw if dual_runtime is not None else 64
    pressure_vb.setRange(QtCore.QRectF(0, 0, display_hw, display_hw), padding=0)
    cmap = pg.colormap.get("viridis")
    pressure_img.setLookupTable(cmap.getLookupTable(0.0, 1.0, 256))
    pressure_img.setLevels((0, args.vmax))
    pressure_img.setImage(np.zeros((display_hw, display_hw), dtype=np.float32), autoLevels=False)
    splitter.addWidget(pressure_panel)

    imu_state: Dict[str, Any] = {"enabled": False, "last_ts": None, "source": "none", "error": gl_error}
    if dual_runtime is None or args.disable_imu:
        imu_widget = QtWidgets.QLabel("IMU 3D disabled")
        imu_widget.setAlignment(QtCore.Qt.AlignCenter)
        splitter.addWidget(imu_widget)
    elif gl is None or QtGui is None:
        msg = gl_error or f"OpenGL backend unavailable. Install dependency: {OPENGL_INSTALL_HINT}"
        imu_widget = QtWidgets.QLabel(f"{UI_TEXT['imu3d_unavailable_prefix']}\n{msg}")
        imu_widget.setWordWrap(True)
        imu_widget.setAlignment(QtCore.Qt.AlignCenter)
        splitter.addWidget(imu_widget)
        imu_state["error"] = msg
    else:
        imu_view = StableGLViewWidget()
        try:
            imu_view.setBackgroundColor((255, 255, 255, 255))
        except Exception:
            pass
        imu_view.opts["distance"] = 2.0
        imu_view.opts["elevation"] = 18.0
        imu_view.opts["azimuth"] = -40.0
        splitter.addWidget(imu_view)

        axis_len = 0.25
        for axis_pts_human, axis_color in [
            (np.array([[0.0, 0.0, 0.0], [axis_len, 0.0, 0.0]], dtype=np.float32), (1.0, 0.0, 0.0, 1.0)),
            (np.array([[0.0, 0.0, 0.0], [0.0, axis_len, 0.0]], dtype=np.float32), (0.0, 1.0, 0.0, 1.0)),
            (np.array([[0.0, 0.0, 0.0], [0.0, 0.0, axis_len]], dtype=np.float32), (0.0, 0.0, 1.0, 1.0)),
        ]:
            imu_view.addItem(
                gl.GLLinePlotItem(
                    pos=rotate_points(axis_pts_human, HUMAN0_TO_VIEW),
                    color=axis_color,
                    width=3.0,
                    antialias=True,
                    mode="line_strip",
                )
            )

        racket_init_to_view = HUMAN0_TO_VIEW @ dual_runtime.racket_init_to_human
        wire_shaft_sensor, wire_head_sensor, wire_face_sensor = build_wireframe_racket_sensor_frame()
        wire_shaft_item = None
        wire_head_item = None
        wire_face_item = None
        racket_mesh_item = None
        racket_mesh_kwargs: Optional[Dict[str, Any]] = None

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
                mesh_kwargs: Dict[str, Any] = {
                    "vertexes": mesh.vertices,
                    "faces": mesh.faces,
                    "smooth": False,
                    "drawEdges": False,
                    "drawFaces": True,
                    "shader": "shaded",
                }
                if mesh.vertex_colors is not None:
                    mesh_kwargs["vertexColors"] = mesh.vertex_colors
                    mesh_kwargs["smooth"] = True
                    mesh_kwargs["shader"] = None
                else:
                    mesh_kwargs["color"] = (0.87, 0.87, 0.92, 1.0)
                racket_mesh_kwargs = mesh_kwargs
                racket_mesh_item = gl.GLMeshItem(**mesh_kwargs)
                imu_view.addItem(racket_mesh_item)
                racket_mesh_item.setTransform(qmatrix_from_rot3(racket_init_to_view, QtGui))
                imu_state["source"] = f"obj:{obj_path.name} faces={mesh.faces.shape[0]}"
            except Exception as mesh_err:
                imu_state["error"] = f"OBJ fallback: {mesh_err}"

        if racket_mesh_item is None:
            wire_shaft_item = gl.GLLinePlotItem(
                pos=rotate_points(wire_shaft_sensor, racket_init_to_view),
                color=(1.0, 0.9, 0.2, 1.0),
                width=4.0,
                antialias=True,
                mode="line_strip",
            )
            wire_head_item = gl.GLLinePlotItem(
                pos=rotate_points(wire_head_sensor, racket_init_to_view),
                color=(0.1, 0.8, 1.0, 1.0),
                width=2.0,
                antialias=True,
                mode="line_strip",
            )
            wire_face_item = gl.GLLinePlotItem(
                pos=rotate_points(wire_face_sensor, racket_init_to_view),
                color=(1.0, 0.3, 0.3, 1.0),
                width=2.0,
                antialias=True,
                mode="line_strip",
            )
            imu_view.addItem(wire_shaft_item)
            imu_view.addItem(wire_head_item)
            imu_view.addItem(wire_face_item)
            imu_state["source"] = "wireframe"

        imu_state.update(
            {
                "enabled": True,
                "racket_mesh_item": racket_mesh_item,
                "wire_shaft_item": wire_shaft_item,
                "wire_head_item": wire_head_item,
                "wire_face_item": wire_face_item,
                "wire_shaft_sensor": wire_shaft_sensor,
                "wire_head_sensor": wire_head_sensor,
                "wire_face_sensor": wire_face_sensor,
                "racket_mesh_kwargs": racket_mesh_kwargs,
            }
        )

    xsens_state: Dict[str, Any] = {"enabled": False, "last_sample": None, "error": gl_error}
    if xsens_runtime is None:
        xsens_widget = QtWidgets.QLabel("Xsens disabled")
        xsens_widget.setAlignment(QtCore.Qt.AlignCenter)
        splitter.addWidget(xsens_widget)
    elif gl is None:
        msg = gl_error or f"OpenGL backend unavailable. Install dependency: {OPENGL_INSTALL_HINT}"
        xsens_widget = QtWidgets.QLabel(f"Xsens SMPL-H unavailable:\n{msg}")
        xsens_widget.setWordWrap(True)
        xsens_widget.setAlignment(QtCore.Qt.AlignCenter)
        splitter.addWidget(xsens_widget)
        xsens_state["error"] = msg
    else:
        xsens_view = StableGLViewWidget()
        xsens_view.setBackgroundColor("#20262d")
        xsens_view.opts["distance"] = 3.0
        xsens_view.opts["elevation"] = 15.0
        xsens_view.opts["azimuth"] = -65.0
        splitter.addWidget(xsens_view)

        xsens_view.addItem(xsens_rt.make_checkerboard_floor(gl))
        xsens_state.update(
            {
                "enabled": True,
                "view": xsens_view,
                "mesh_item": None,
                "body_shader": xsens_rt.make_balanced_body_shader(),
                "racket_position_smpl": np.asarray(
                    args.human_racket_initial_position,
                    dtype=np.float32,
                ),
                "racket_interacting": False,
            }
        )

        # Keep the original standalone IMU racket view above. This is a second
        # racket instance rendered in the SMPL-H view.
        if imu_state.get("enabled"):
            human_racket_mesh_item = None
            human_wire_shaft_item = None
            human_wire_head_item = None
            human_wire_face_item = None
            initial_rot_view = SMPL_TO_VIEW @ dual_runtime.racket_init_to_human
            initial_pos_view = SMPL_TO_VIEW @ xsens_state["racket_position_smpl"]

            if imu_state.get("racket_mesh_kwargs") is not None:
                human_racket_mesh_item = gl.GLMeshItem(**imu_state["racket_mesh_kwargs"])
                human_racket_mesh_item.setTransform(
                    qmatrix_from_pose(initial_rot_view, initial_pos_view, QtGui)
                )
                xsens_view.addItem(human_racket_mesh_item)
            else:
                human_wire_shaft_item = gl.GLLinePlotItem(
                    pos=rotate_points(imu_state["wire_shaft_sensor"], initial_rot_view) + initial_pos_view,
                    color=(1.0, 0.9, 0.2, 1.0),
                    width=4.0,
                    antialias=True,
                    mode="line_strip",
                )
                human_wire_head_item = gl.GLLinePlotItem(
                    pos=rotate_points(imu_state["wire_head_sensor"], initial_rot_view) + initial_pos_view,
                    color=(0.1, 0.8, 1.0, 1.0),
                    width=2.0,
                    antialias=True,
                    mode="line_strip",
                )
                human_wire_face_item = gl.GLLinePlotItem(
                    pos=rotate_points(imu_state["wire_face_sensor"], initial_rot_view) + initial_pos_view,
                    color=(1.0, 0.3, 0.3, 1.0),
                    width=2.0,
                    antialias=True,
                    mode="line_strip",
                )
                xsens_view.addItem(human_wire_shaft_item)
                xsens_view.addItem(human_wire_head_item)
                xsens_view.addItem(human_wire_face_item)

            xsens_state.update(
                {
                    "racket_enabled": True,
                    "racket_mesh_item": human_racket_mesh_item,
                    "racket_wire_shaft_item": human_wire_shaft_item,
                    "racket_wire_head_item": human_wire_head_item,
                    "racket_wire_face_item": human_wire_face_item,
                    "racket_wire_shaft_sensor": imu_state["wire_shaft_sensor"],
                    "racket_wire_head_sensor": imu_state["wire_head_sensor"],
                    "racket_wire_face_sensor": imu_state["wire_face_sensor"],
                }
            )

    status_label = QtWidgets.QLabel("Waiting data...")
    status_label.setStyleSheet("font-family: Consolas, monospace; font-size: 12px;")
    layout.addWidget(status_label)

    btn_row = QtWidgets.QHBoxLayout()
    btn_zero_calib = QtWidgets.QPushButton(UI_TEXT["zero_btn_start"])
    btn_zero_clear = QtWidgets.QPushButton(UI_TEXT["zero_btn_clear"])
    btn_imu_reset = QtWidgets.QPushButton(UI_TEXT["imu_btn_reset"])
    btn_row.addWidget(btn_zero_calib)
    btn_row.addWidget(btn_zero_clear)
    btn_row.addWidget(btn_imu_reset)
    btn_row.addStretch(1)
    layout.addLayout(btn_row)

    if dual_runtime is not None:
        btn_zero_calib.clicked.connect(lambda: dual_runtime.start_zero_calibration(5.0))
        btn_zero_clear.clicked.connect(dual_runtime.clear_zero_calibration)
        btn_imu_reset.clicked.connect(dual_runtime.reset_imu_zero_pose)
    else:
        btn_zero_calib.setEnabled(False)
        btn_zero_clear.setEnabled(False)
        btn_imu_reset.setEnabled(False)

    xsens_mesh_buffer = TimelineBuffer(maxlen=args.timeline_buffer_frames)
    xsens_last_seen: Dict[str, Optional[int]] = {"sample_counter": None}
    start_ts = time.perf_counter()
    timer = QtCore.QTimer()
    timer.setInterval(max(1, int(1000.0 / max(1.0, args.fps))))
    try:
        timer.setTimerType(QtCore.Qt.PreciseTimer)
    except Exception:
        pass

    last_drawn_pressure_ts: Optional[float] = None
    last_drawn_imu_ts: Optional[float] = None

    def tick() -> None:
        nonlocal last_drawn_pressure_ts, last_drawn_imu_ts
        if args.duration_sec > 0 and (time.perf_counter() - start_ts) >= args.duration_sec:
            app.quit()
            return

        now = time.perf_counter()
        target_ts = now - args.sync_latency_ms / 1000.0
        poll_xsens_mesh(xsens_runtime, xsens_mesh_buffer, xsens_last_seen)
        xsens_item = xsens_mesh_buffer.sample_at_or_before(target_ts)
        dual_snapshot = dual_runtime.sample_for_timeline(target_ts) if dual_runtime is not None else None

        pressure_item = dual_snapshot["pressure"] if dual_snapshot is not None else None
        imu_item = dual_snapshot["imu"] if dual_snapshot is not None else None

        if pressure_item is not None and pressure_item.host_ts != last_drawn_pressure_ts:
            pressure_img.setImage(pressure_item.data.display_values, autoLevels=False)
            last_drawn_pressure_ts = pressure_item.host_ts

        if imu_state.get("enabled") and imu_item is not None and imu_item.host_ts != last_drawn_imu_ts:
            imu_frame: CorrectedImuFrame = imu_item.data
            rot_view = HUMAN0_TO_VIEW @ imu_frame.rot_human
            if imu_state.get("racket_mesh_item") is not None and QtGui is not None:
                imu_state["racket_mesh_item"].setTransform(qmatrix_from_rot3(rot_view, QtGui))
            else:
                if imu_state.get("wire_shaft_item") is not None:
                    imu_state["wire_shaft_item"].setData(pos=rotate_points(imu_state["wire_shaft_sensor"], rot_view))
                if imu_state.get("wire_head_item") is not None:
                    imu_state["wire_head_item"].setData(pos=rotate_points(imu_state["wire_head_sensor"], rot_view))
                if imu_state.get("wire_face_item") is not None:
                    imu_state["wire_face_item"].setData(pos=rotate_points(imu_state["wire_face_sensor"], rot_view))
            last_drawn_imu_ts = imu_item.host_ts

        if xsens_state.get("enabled") and xsens_item is not None:
            xsens_data: XsensVisualData = xsens_item.data
            if xsens_state.get("last_sample") != xsens_data.sample_counter:
                display_vertices = xsens_rt.smplh_vertices_to_pyqtgraph(xsens_data.vertices)
                if xsens_state.get("mesh_item") is None:
                    xsens_state["mesh_item"] = gl.GLMeshItem(
                        vertexes=display_vertices,
                        faces=xsens_runtime.retargeter.faces,
                        color=(0.26, 0.68, 0.96, 1.0),
                        smooth=True,
                        drawEdges=False,
                        drawFaces=True,
                        shader=xsens_state["body_shader"],
                    )
                    xsens_state["view"].addItem(xsens_state["mesh_item"])
                else:
                    xsens_state["mesh_item"].setMeshData(
                        vertexes=display_vertices,
                        faces=xsens_runtime.retargeter.faces,
                    )
                xsens_state["last_sample"] = xsens_data.sample_counter

        pressure_is_fresh = bool(
            pressure_item is not None
            and target_ts - pressure_item.host_ts
            <= args.interaction_pressure_timeout_ms / 1000.0
        )
        interaction_active = bool(
            pressure_is_fresh and pressure_item.data.interaction_active
        )
        if xsens_state.get("racket_enabled") and imu_item is not None:
            xsens_data = xsens_item.data if xsens_item is not None else None
            right_palm_position = (
                None if xsens_data is None else xsens_data.right_palm_position
            )
            if interaction_active and right_palm_position is not None:
                # The prepared racket's origin is the grip center, so assigning
                # the palm center here teleports the handle to the hand and
                # keeps the positions attached until contact ends.
                xsens_state["racket_position_smpl"] = right_palm_position.copy()

            imu_frame = imu_item.data
            racket_rot_view = SMPL_TO_VIEW @ imu_frame.rot_human
            racket_pos_view = SMPL_TO_VIEW @ xsens_state["racket_position_smpl"]
            if xsens_state.get("racket_mesh_item") is not None and QtGui is not None:
                xsens_state["racket_mesh_item"].setTransform(
                    qmatrix_from_pose(racket_rot_view, racket_pos_view, QtGui)
                )
            else:
                if xsens_state.get("racket_wire_shaft_item") is not None:
                    xsens_state["racket_wire_shaft_item"].setData(
                        pos=rotate_points(
                            xsens_state["racket_wire_shaft_sensor"],
                            racket_rot_view,
                        )
                        + racket_pos_view
                    )
                if xsens_state.get("racket_wire_head_item") is not None:
                    xsens_state["racket_wire_head_item"].setData(
                        pos=rotate_points(
                            xsens_state["racket_wire_head_sensor"],
                            racket_rot_view,
                        )
                        + racket_pos_view
                    )
                if xsens_state.get("racket_wire_face_item") is not None:
                    xsens_state["racket_wire_face_item"].setData(
                        pos=rotate_points(
                            xsens_state["racket_wire_face_sensor"],
                            racket_rot_view,
                        )
                        + racket_pos_view
                    )
            xsens_state["racket_interacting"] = interaction_active

        elapsed = max(1e-6, now - start_ts)
        lines = [
            "timeline target={:.6f} latency={:.0f}ms | lag P={} IMU={} Xsens={}".format(
                target_ts,
                args.sync_latency_ms,
                source_lag_text(pressure_item, target_ts),
                source_lag_text(imu_item, target_ts),
                source_lag_text(xsens_item, target_ts),
            )
        ]
        lines.extend(startup_errors)

        if xsens_runtime is not None:
            with xsens_runtime.state.lock:
                xsens_counters = dict(xsens_runtime.state.counters)
                xsens_sample = xsens_runtime.state.latest_sample_counter
                xsens_error = xsens_runtime.state.latest_error
            lines.append(
                "Xsens retargeted={} mesh={} queue={} sample={}".format(
                    xsens_counters.get("retargeted_frames", 0),
                    xsens_counters.get("mesh_frames", 0),
                    xsens_runtime.pose_queue.qsize(),
                    "none" if xsens_sample is None else xsens_sample,
                )
            )
            if xsens_error:
                lines.append(f"Xsens last error: {xsens_error}")
        else:
            lines.append("Xsens disabled")

        if dual_snapshot is not None:
            st = dual_snapshot["stats"]
            lines.append(
                "SAGE P={} ({:.1f}/s) drop={} | IMU={} ({:.1f}/s) drop={} | sync={}/{}".format(
                    st.get("pressure_frames", 0),
                    st.get("pressure_frames", 0) / elapsed,
                    st.get("pressure_drop_chunks", 0),
                    st.get("imu_frames", 0),
                    st.get("imu_frames", 0) / elapsed,
                    st.get("imu_drop_chunks", 0),
                    st.get("synced", 0),
                    st.get("unsynced", 0),
                )
            )
            if dual_snapshot["zero_calibrating"]:
                lines.append(
                    "{} | remain {:.1f}s | samples {}".format(
                        dual_snapshot["zero_status"],
                        dual_snapshot["zero_remaining"],
                        dual_snapshot["zero_samples"],
                    )
                )
            else:
                lines.append(
                    "{} | {}".format(
                        dual_snapshot["zero_status"],
                        UI_TEXT["zero_enabled"] if dual_snapshot["zero_enabled"] else UI_TEXT["zero_disabled"],
                    )
                )
            if imu_state.get("enabled"):
                lines.append(f"IMU 3D: on | {dual_snapshot['imu_zero_status']} | source={imu_state.get('source')}")
            elif args.disable_imu:
                lines.append("IMU 3D: disabled")
            elif imu_state.get("error"):
                lines.append(f"IMU 3D: unavailable ({str(imu_state['error']).splitlines()[0]})")
            if pressure_item is not None:
                p_record: PressureFrame = pressure_item.data.record
                lines.append(f"P mean/max = {float(np.mean(p_record.values)):.1f}/{float(np.max(p_record.values)):.1f} g")
                interaction_text = "ATTACHED" if interaction_active else "free"
                if interaction_active and (
                    xsens_item is None or xsens_item.data.right_palm_position is None
                ):
                    interaction_text = "waiting for right palm"
                lines.append(
                    "Human-view racket: {} | peak={:.1f}g | attach/release={:.1f}/{:.1f}g | rotation=IMU".format(
                        interaction_text,
                        pressure_item.data.interaction_pressure_g,
                        args.interaction_pressure_threshold,
                        args.interaction_release_threshold,
                    )
                )
            if imu_item is not None:
                imu_frame = imu_item.data
                ax, ay, az = imu_frame.acc_human_g
                qw, qx, qy, qz = imu_frame.quat_human_wxyz
                lines.append(f"acc[g]=({ax:+.5f}, {ay:+.5f}, {az:+.5f}) quat=({qw:+.5f}, {qx:+.5f}, {qy:+.5f}, {qz:+.5f})")
            dual_runtime.maybe_write_clock_sync_sample()
            btn_zero_calib.setEnabled((not args.disable_pressure) and (not dual_snapshot["zero_calibrating"]))
            btn_zero_clear.setEnabled((not args.disable_pressure) and (dual_snapshot["zero_calibrating"] or dual_snapshot["zero_enabled"]))
            btn_imu_reset.setEnabled((not args.disable_imu) and bool(imu_state.get("enabled")))
        else:
            lines.append("SAGE dual sensor disabled")

        if requested_opengl_mode != qt_opengl_mode:
            lines.append(f"OpenGL request={requested_opengl_mode} qt={qt_opengl_mode}")
        status_label.setText("\n".join(lines))

    timer.timeout.connect(tick)
    timer.start()
    win.resize(1720, 900)
    win.show()
    app.exec()


def run_headless_status(
    args: argparse.Namespace,
    xsens_runtime: Optional[xsens_rt.RealtimeSmplhRuntime],
    dual_runtime: Optional[DualSensorRealtimeRuntime],
    global_stop_event: threading.Event,
    startup_errors: List[str],
) -> None:
    start_ts = time.perf_counter()
    last_print = 0.0
    xsens_mesh_buffer = TimelineBuffer(maxlen=args.timeline_buffer_frames)
    xsens_last_seen: Dict[str, Optional[int]] = {"sample_counter": None}
    while not global_stop_event.is_set():
        now = time.perf_counter()
        if args.duration_sec > 0 and now - start_ts >= args.duration_sec:
            break
        if now - last_print >= args.status_interval:
            target_ts = now - args.sync_latency_ms / 1000.0
            poll_xsens_mesh(xsens_runtime, xsens_mesh_buffer, xsens_last_seen)
            xsens_item = xsens_mesh_buffer.sample_at_or_before(target_ts)
            dual_snapshot = dual_runtime.sample_for_timeline(target_ts) if dual_runtime is not None else None
            pressure_item = dual_snapshot["pressure"] if dual_snapshot is not None else None
            imu_item = dual_snapshot["imu"] if dual_snapshot is not None else None
            pieces = [
                "target={:.6f} lag P={} IMU={} Xsens={}".format(
                    target_ts,
                    source_lag_text(pressure_item, target_ts),
                    source_lag_text(imu_item, target_ts),
                    source_lag_text(xsens_item, target_ts),
                )
            ]
            pieces.extend(startup_errors)
            if xsens_runtime is not None:
                with xsens_runtime.state.lock:
                    counters = dict(xsens_runtime.state.counters)
                pieces.append(f"Xsens retargeted={counters.get('retargeted_frames', 0)} mesh={counters.get('mesh_frames', 0)}")
            if dual_snapshot is not None:
                st = dual_snapshot["stats"]
                pieces.append(f"SAGE P={st.get('pressure_frames', 0)} IMU={st.get('imu_frames', 0)} sync={st.get('synced', 0)}/{st.get('unsynced', 0)}")
                dual_runtime.maybe_write_clock_sync_sample()
            print("[SYNC] " + " | ".join(pieces), flush=True)
            last_print = now
        time.sleep(0.01)


def parse_args() -> argparse.Namespace:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    default_output_root = Path("output/realtime_timeline_sync") / stamp
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-root", type=Path, default=default_output_root)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--duration-sec", type=float, default=0.0)
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--sync-latency-ms", type=float, default=120.0)
    parser.add_argument("--timeline-buffer-frames", type=int, default=20000)
    parser.add_argument("--status-interval", type=float, default=1.0)
    parser.add_argument("--clock-sync-interval-sec", type=float, default=5.0)
    parser.add_argument("--opengl-mode", choices=["auto", "desktop", "software", "none"], default="auto")
    parser.add_argument("--no-opengl", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--disable-xsens", action="store_true")
    parser.add_argument("--disable-dual-sensor", action="store_true")

    dual = parser.add_argument_group("SAGE dual sensor")
    dual.add_argument("--pressure-port", default=None)
    dual.add_argument("--pressure-baud", type=int, default=115200)
    dual.add_argument("--pressure-range", default="1kg", choices=["1kg", "3kg", "5kg", "10kg", "20kg", "30kg", "50kg", "skip"])
    dual.add_argument("--pressure-cmd-suffix", default="cr", choices=["none", "cr", "lf", "crlf"])
    dual.add_argument("--disable-pressure", action="store_true")
    dual.add_argument("--imu-port", default=None)
    dual.add_argument("--imu-baud", type=int, default=115200)
    dual.add_argument("--disable-imu", action="store_true")
    dual.add_argument("--imu-quat-world-to-sensor", action="store_true")
    dual.add_argument("--sync-max-dt-ms", type=float, default=200.0)
    dual.add_argument("--no-dual-csv", action="store_true")
    dual.add_argument("--dual-csv-dir", type=Path, default=None)
    dual.add_argument("--transpose", action="store_true")
    dual.add_argument("--flipud", action="store_true")
    dual.add_argument("--fliplr", action="store_true")
    dual.add_argument("--no-rotate-ccw90", dest="no_rotate_ccw90", action="store_true")
    dual.add_argument("--no-rotate-cw90", dest="no_rotate_ccw90", action="store_true", help=argparse.SUPPRESS)
    dual.add_argument("--interp-scale", type=int, default=4)
    dual.add_argument("--vmax", type=float, default=1000.0)
    dual.add_argument("--queue-size", type=int, default=4096)
    dual.add_argument("--read-chunk", type=int, default=4096)
    dual.add_argument("--windows-host", default=None)
    dual.add_argument("--racket-obj", default=str(DEFAULT_RACKET_OBJ))
    dual.add_argument("--no-racket-obj", action="store_true")
    dual.add_argument("--racket-face-budget", type=int, default=8000)
    dual.add_argument("--racket-align-roll", type=float, default=0.0)
    dual.add_argument("--racket-align-pitch", type=float, default=0.0)
    dual.add_argument("--racket-align-yaw", type=float, default=0.0)
    dual.add_argument(
        "--interaction-pressure-threshold",
        type=float,
        default=100.0,
        help="Attach the human-view racket when the peak zero-corrected pressure reaches this value in grams.",
    )
    dual.add_argument(
        "--interaction-release-threshold",
        type=float,
        default=80.0,
        help="Release the human-view racket when peak pressure falls to this value in grams.",
    )
    dual.add_argument(
        "--interaction-pressure-timeout-ms",
        type=float,
        default=500.0,
        help="Release attachment when no fresh pressure frame has arrived within this interval.",
    )
    dual.add_argument(
        "--human-racket-initial-position",
        type=float,
        nargs=3,
        metavar=("X", "Y", "Z"),
        default=(0.0, 0.0, 0.0),
        help="Initial grip-center position in the SMPL-H world frame, in meters.",
    )

    xsens = parser.add_argument_group("Xsens MVN SMPL-H")
    xsens.add_argument("--xsens-bind-ip", default="0.0.0.0")
    xsens.add_argument("--xsens-port", type=int, default=xsens_rt.mvn_udp.DEFAULT_PORT)
    xsens.add_argument("--xsens-output-dir", type=Path, default=None)
    xsens.add_argument("--xsens-pt-output", type=Path, default=None)
    xsens.add_argument("--xsens-accept-from", default=None)
    xsens.add_argument("--xsens-max-packets", type=int, default=None)
    xsens.add_argument("--xsens-max-samples", type=int, default=None)
    xsens.add_argument("--xsens-socket-buffer-bytes", type=int, default=4 * 1024 * 1024)
    xsens.add_argument("--xsens-recv-bytes", type=int, default=65535)
    xsens.add_argument("--xsens-pending-timeout-s", type=float, default=1.0)
    xsens.add_argument("--xsens-flush-every", type=int, default=100)
    xsens.add_argument("--xsens-print-every", type=int, default=100)
    xsens.add_argument("--xsens-segment-id-base", choices=["auto", "zero", "one"], default="auto")
    xsens.add_argument("--xsens-body-model", type=Path, default=xsens_rt.DEFAULT_BODY_MODEL)
    xsens.add_argument("--xsens-device", choices=["auto", "cpu", "cuda"], default="cpu")
    xsens.add_argument("--xsens-num-betas", type=int, default=16)
    xsens.add_argument("--xsens-arm-correction", choices=["none", "xsens_ros2_plus90"], default="none")
    xsens.add_argument(
        "--xsens-arm-twist-filter",
        choices=["none", "preserve_hand_global", "keep_measured_wrist"],
        default="none",
    )
    xsens.add_argument("--xsens-pt-checkpoint-every-s", type=float, default=5.0)
    xsens.add_argument("--xsens-pt-checkpoint-every-frames", type=int, default=0)
    xsens.add_argument("--xsens-retarget-queue-size", type=int, default=1024)
    xsens.add_argument("--xsens-keep-stale-retarget-frames", action="store_true")
    args = parser.parse_args()
    if args.interaction_pressure_threshold < 0.0:
        parser.error("--interaction-pressure-threshold must be non-negative")
    if args.interaction_release_threshold < 0.0:
        parser.error("--interaction-release-threshold must be non-negative")
    if args.interaction_release_threshold > args.interaction_pressure_threshold:
        parser.error("--interaction-release-threshold must not exceed --interaction-pressure-threshold")
    if args.interaction_pressure_timeout_ms <= 0.0:
        parser.error("--interaction-pressure-timeout-ms must be positive")
    return args


def install_signal_handlers(
    global_stop_event: threading.Event,
    xsens_runtime: Optional[xsens_rt.RealtimeSmplhRuntime],
    dual_runtime: Optional[DualSensorRealtimeRuntime],
) -> None:
    def request_stop(_signum: int, _frame: object) -> None:
        global_stop_event.set()
        if xsens_runtime is not None:
            xsens_runtime.stop_event.set()
        if dual_runtime is not None:
            dual_runtime.stop_event.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)


def main() -> None:
    args = parse_args()
    if args.windows_host:
        os.environ["WSL_WINDOWS_HOST"] = args.windows_host
    if args.pressure_port is None:
        args.pressure_port = "wintcp://17010" if is_wsl() else "COM10"
    if args.imu_port is None:
        args.imu_port = "wintcp://17004" if is_wsl() else "COM4"

    args.output_root.mkdir(parents=True, exist_ok=True)
    requested_opengl_mode = normalize_requested_opengl_mode(args.opengl_mode, args.no_opengl)
    qt_opengl_mode = effective_qt_opengl_mode(requested_opengl_mode, False)
    if qt_opengl_mode in ("desktop", "software"):
        configure_qt_opengl_env(qt_opengl_mode)

    global_stop_event = threading.Event()
    xsens_runtime: Optional[xsens_rt.RealtimeSmplhRuntime] = None
    dual_runtime: Optional[DualSensorRealtimeRuntime] = None
    startup_errors: List[str] = []

    try:
        if not args.disable_dual_sensor:
            try:
                dual_stop_event = threading.Event()
                dual_runtime = DualSensorRealtimeRuntime(args, stop_event=dual_stop_event)
            except Exception as exc:
                msg = f"SAGE startup failed: {exc}"
                startup_errors.append(msg)
                print(f"[DUAL] {msg}", flush=True)

        install_signal_handlers(global_stop_event, xsens_runtime, dual_runtime)

        if dual_runtime is not None:
            try:
                dual_runtime.start()
            except Exception as exc:
                msg = f"SAGE start failed: {exc}"
                startup_errors.append(msg)
                print(f"[DUAL] {msg}", flush=True)
                dual_runtime.stop()
                dual_runtime = None

        if not args.disable_xsens:
            try:
                xsens_stop_event = threading.Event()
                xsens_args = build_xsens_args(args, viewer_enabled=(not args.headless and qt_opengl_mode != "none"))
                xsens_runtime = xsens_rt.create_realtime_runtime(xsens_args, stop_event=xsens_stop_event)
                xsens_rt.start_realtime_runtime(xsens_runtime, print_banner=True)
            except Exception as exc:
                msg = f"Xsens startup failed: {exc}"
                startup_errors.append(msg)
                print(f"[REALTIME] {msg}", flush=True)
                if xsens_runtime is not None:
                    xsens_rt.stop_realtime_runtime(xsens_runtime, print_final=False)
                xsens_runtime = None
        if platform.system() == "Windows":
            print("[SYNC] Windows online mode active")
        print(f"[SYNC] Output root: {args.output_root}")
        print(f"[SYNC] Timeline latency: {args.sync_latency_ms:.0f} ms")

        if args.headless:
            run_headless_status(args, xsens_runtime, dual_runtime, global_stop_event, startup_errors)
        else:
            run_combined_viewer(
                args,
                xsens_runtime,
                dual_runtime,
                global_stop_event,
                requested_opengl_mode,
                qt_opengl_mode,
                startup_errors,
            )
    finally:
        global_stop_event.set()
        if dual_runtime is not None:
            dual_runtime.stop()
        if xsens_runtime is not None:
            xsens_rt.stop_realtime_runtime(xsens_runtime, print_final=True)


if __name__ == "__main__":
    main()
