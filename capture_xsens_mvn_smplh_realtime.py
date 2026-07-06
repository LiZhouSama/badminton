#!/usr/bin/env python3
"""
Capture Xsens MVN UDP pose packets, retarget them to SMPL-H in realtime, and
optionally show the latest SMPL-H mesh with pyqtgraph.

The original CSV files are still written by the same decoder used in
capture_xsens_mvn_udp.py.  The realtime path consumes decoded MXTP02 pose rows
directly in memory so it does not need to poll the CSV file.
"""

from __future__ import annotations

import argparse
import os
import queue
import signal
import threading
import time
from collections import Counter
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
import torch

import capture_xsens_mvn_udp as mvn_udp
from xsens_mvn_csv_to_smplh import (
    DEFAULT_BODY_MODEL,
    HAND_JOINT_MVN_PAIRS,
    HAND_MVN_TO_SMPLH,
    MVN_BODY_TO_SMPL24,
    SMPL24_NAMES,
    SMPLH_52_NAMES,
    THUMB_SWING_POSITION_PAIRS,
    WRIST_TWIST_LIMIT_DEG,
    FramePose,
    RetargetConfig,
    SegmentPose,
    body_model_rest_joints,
    build_smpl24_global_rotmats,
    build_smplh_global_rotmats,
    check_rotation_orthogonality,
    compute_trans_from_pelvis,
    create_body_model,
    estimate_fps,
    global_to_local_rotmats,
    local_to_global_rotmats,
    matrix_to_axis_angle,
    position_m_from_row,
    quat_wxyz_to_rotmat,
    read_smplh_parents,
    remove_arm_twist_keep_measured_wrists,
    remove_arm_twist_preserve_measured_hand_global,
    select_device,
    validate_mvn_frames,
)


@dataclass
class RealtimeSmplhFrame:
    sample_counter: int
    time_code_ms: int
    pose: np.ndarray
    root_orient: np.ndarray
    pose_body: np.ndarray
    pose_hand: np.ndarray
    trans: np.ndarray
    betas: np.ndarray
    vertices: Optional[np.ndarray] = None
    host_wall_ts_s: float = float("nan")
    host_perf_ts_s: float = float("nan")


class SharedRealtimeState:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.counters: Counter = Counter()
        self.latest_vertices: Optional[np.ndarray] = None
        self.latest_sample_counter: Optional[int] = None
        self.latest_time_code_ms: Optional[int] = None
        self.latest_host_wall_ts_s: Optional[float] = None
        self.latest_error: str = ""
        self.capture_stats: Optional[Dict[str, int]] = None
        self.final_pt_path: Optional[Path] = None
        self.checkpoint_pt_path: Optional[Path] = None


class RealtimeSmplhRetargeter:
    def __init__(
        self,
        body_model_path: Path,
        num_betas: int,
        device_arg: str,
        config: RetargetConfig,
    ) -> None:
        self.body_model_path = body_model_path
        self.num_betas = num_betas
        self.device = select_device(device_arg)
        self.config = config
        self.body_model = create_body_model(body_model_path, num_betas, self.device)
        self.rest_joints = body_model_rest_joints(self.body_model, num_betas, self.device)
        self.rest_root = self.rest_joints[0]
        self.smplh_parents = read_smplh_parents(body_model_path)
        self.faces = self.body_model.f.detach().cpu().numpy().astype(np.int32)

    def retarget_frame(self, frame: FramePose, include_vertices: bool) -> RealtimeSmplhFrame:
        frames = [frame]
        smpl24_global = build_smpl24_global_rotmats(
            frames,
            arm_correction=self.config.arm_correction,
        )
        smplh_global = build_smplh_global_rotmats(
            frames=frames,
            smpl24_global=smpl24_global,
            smplh_parents=self.smplh_parents,
            rest_joints=self.rest_joints,
        )
        smplh_local_raw = global_to_local_rotmats(smplh_global, self.smplh_parents)
        if self.config.arm_twist_filter == "none":
            smplh_local = smplh_local_raw.copy()
        elif self.config.arm_twist_filter == "preserve_hand_global":
            smplh_local = remove_arm_twist_preserve_measured_hand_global(
                global_rotmats=smplh_global,
                local_rotmats=smplh_local_raw,
                parents=self.smplh_parents,
                rest_joints=self.rest_joints,
            )
        elif self.config.arm_twist_filter == "keep_measured_wrist":
            smplh_local = remove_arm_twist_keep_measured_wrists(
                global_rotmats=smplh_global,
                local_rotmats=smplh_local_raw,
                parents=self.smplh_parents,
                rest_joints=self.rest_joints,
            )
        else:
            raise ValueError(f"Unsupported arm twist filter mode: {self.config.arm_twist_filter!r}")

        smplh_global_reconstructed = local_to_global_rotmats(smplh_local, self.smplh_parents)
        check_rotation_orthogonality("smpl24_global", smpl24_global)
        check_rotation_orthogonality("smplh_global", smplh_global)
        check_rotation_orthogonality("smplh_local", smplh_local)
        check_rotation_orthogonality("smplh_global_reconstructed", smplh_global_reconstructed)

        root_orient = matrix_to_axis_angle(smplh_global_reconstructed[:, 0])
        local_axis = matrix_to_axis_angle(smplh_local)
        pose_body = local_axis[:, 1:22].reshape(1, 63).astype(np.float32)
        pose_hand = local_axis[:, 22:52].reshape(1, 90).astype(np.float32)
        trans = compute_trans_from_pelvis(frames, root_orient, self.rest_root)
        pose = np.concatenate([root_orient, pose_body, pose_hand], axis=1).astype(np.float32)
        betas = np.zeros((1, self.num_betas), dtype=np.float32)

        vertices = (
            self._forward_vertices(root_orient, pose_body, pose_hand, trans, betas)
            if include_vertices
            else None
        )
        return RealtimeSmplhFrame(
            sample_counter=frame.sample_counter,
            time_code_ms=frame.time_code_ms,
            pose=pose[0],
            root_orient=root_orient[0].astype(np.float32),
            pose_body=pose_body[0],
            pose_hand=pose_hand[0],
            trans=trans[0],
            betas=betas[0],
            vertices=vertices,
        )

    def _forward_vertices(
        self,
        root_orient: np.ndarray,
        pose_body: np.ndarray,
        pose_hand: np.ndarray,
        trans: np.ndarray,
        betas: np.ndarray,
    ) -> np.ndarray:
        with torch.no_grad():
            out = self.body_model(
                pose_body=torch.from_numpy(pose_body).to(self.device),
                pose_hand=torch.from_numpy(pose_hand).to(self.device),
                betas=torch.from_numpy(betas).to(self.device),
                root_orient=torch.from_numpy(root_orient.astype(np.float32)).to(self.device),
                trans=torch.from_numpy(trans).to(self.device),
            )
        return out.v[0].detach().cpu().numpy().astype(np.float32)

    def metadata(self, source_pose_csv: Path) -> Dict[str, object]:
        return {
            "source_mvn_dir": str(source_pose_csv.parent),
            "source_pose_csv": str(source_pose_csv),
            "retarget_mode": "xsens_global_segment_to_smplh_v2",
            "coord_mode": "xsens_zup_to_smpl",
            "body_mapping_profile": "mvnx_to_smpl",
            "hand_mode": "xsens_relative_fingers_thumb_position_swing",
            "thumb_retarget_mode": "manus_thumb_position_swing_for_cmc_mcp_distal_swing_only",
            "arm_correction": self.config.arm_correction,
            "arm_twist_filter": self.config.arm_twist_filter,
            "wrist_twist_limit_deg": float(WRIST_TWIST_LIMIT_DEG),
            "body_model_path": str(self.body_model_path),
            "smpl24_names": SMPL24_NAMES,
            "smplh_52_names": SMPLH_52_NAMES,
            "smplh_parents": torch.from_numpy(self.smplh_parents.astype(np.int64)),
            "body_mapping": dict(MVN_BODY_TO_SMPL24),
            "hand_mapping": dict(HAND_MVN_TO_SMPLH),
            "hand_joint_mvn_pairs": dict(HAND_JOINT_MVN_PAIRS),
            "thumb_swing_position_pairs": dict(THUMB_SWING_POSITION_PAIRS),
            "notes": (
                "Realtime Xsens UDP retargeting. CSV rows are written by capture_xsens_mvn_udp.py; "
                "SMPL-H pose/trans are accumulated in memory and checkpointed periodically."
            ),
        }


class SmplhPtAccumulator:
    def __init__(self, metadata: Dict[str, object], num_betas: int) -> None:
        self.metadata = metadata
        self.num_betas = num_betas
        self.pose: List[np.ndarray] = []
        self.root_orient: List[np.ndarray] = []
        self.pose_body: List[np.ndarray] = []
        self.pose_hand: List[np.ndarray] = []
        self.trans: List[np.ndarray] = []
        self.betas: List[np.ndarray] = []
        self.sample_counter: List[int] = []
        self.time_code_ms: List[int] = []
        self.host_wall_ts_s: List[float] = []
        self.host_perf_ts_s: List[float] = []

    @property
    def frame_count(self) -> int:
        return len(self.pose)

    def add(self, frame: RealtimeSmplhFrame) -> None:
        self.pose.append(frame.pose.astype(np.float32, copy=True))
        self.root_orient.append(frame.root_orient.astype(np.float32, copy=True))
        self.pose_body.append(frame.pose_body.astype(np.float32, copy=True))
        self.pose_hand.append(frame.pose_hand.astype(np.float32, copy=True))
        self.trans.append(frame.trans.astype(np.float32, copy=True))
        self.betas.append(frame.betas.astype(np.float32, copy=True))
        self.sample_counter.append(int(frame.sample_counter))
        self.time_code_ms.append(int(frame.time_code_ms))
        self.host_wall_ts_s.append(float(frame.host_wall_ts_s))
        self.host_perf_ts_s.append(float(frame.host_perf_ts_s))

    def to_pt_dict(self) -> Dict[str, object]:
        time_code = np.asarray(self.time_code_ms, dtype=np.int64)
        data: Dict[str, object] = dict(self.metadata)
        data.update(
            {
                "pose": self._stack_float(self.pose, 156),
                "trans": self._stack_float(self.trans, 3),
                "root_orient": self._stack_float(self.root_orient, 3),
                "pose_body": self._stack_float(self.pose_body, 63),
                "pose_hand": self._stack_float(self.pose_hand, 90),
                "betas": self._stack_float(self.betas, self.num_betas),
                "sample_counter": torch.from_numpy(np.asarray(self.sample_counter, dtype=np.int64)),
                "time_code_ms": torch.from_numpy(time_code),
                "host_wall_ts_s": torch.from_numpy(np.asarray(self.host_wall_ts_s, dtype=np.float64)),
                "host_perf_ts_s": torch.from_numpy(np.asarray(self.host_perf_ts_s, dtype=np.float64)),
                "fps": estimate_fps(time_code),
                "created_at": datetime.now().isoformat(timespec="seconds"),
                "save_mode": "realtime_periodic_checkpoint",
            }
        )
        return data

    @staticmethod
    def _stack_float(items: Sequence[np.ndarray], width: int) -> torch.Tensor:
        if not items:
            return torch.zeros((0, width), dtype=torch.float32)
        return torch.from_numpy(np.stack(items, axis=0).astype(np.float32, copy=False))

    def save_atomic(self, output: Path) -> None:
        output.parent.mkdir(parents=True, exist_ok=True)
        tmp_output = output.with_name(output.name + ".tmp")
        torch.save(self.to_pt_dict(), tmp_output)
        os.replace(str(tmp_output), str(output))


def parse_args() -> argparse.Namespace:
    default_output_dir = Path("output/xsens_mvn_udp") / datetime.now().strftime("%Y%m%d_%H%M%S_realtime")
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bind-ip", default="0.0.0.0", help="Local interface to bind.")
    parser.add_argument("--port", type=int, default=mvn_udp.DEFAULT_PORT, help="UDP port from MVN.")
    parser.add_argument("--output-dir", type=Path, default=default_output_dir)
    parser.add_argument("--pt-output", type=Path, default=None)
    parser.add_argument("--accept-from", default=None, help="Optional sender IP filter.")
    parser.add_argument("--duration-s", type=float, default=None, help="Stop after this many seconds.")
    parser.add_argument("--max-packets", type=int, default=None, help="Stop after this many UDP packets.")
    parser.add_argument("--max-samples", type=int, default=None, help="Stop after this many decoded samples.")
    parser.add_argument("--socket-buffer-bytes", type=int, default=4 * 1024 * 1024)
    parser.add_argument("--recv-bytes", type=int, default=65535)
    parser.add_argument("--pending-timeout-s", type=float, default=1.0)
    parser.add_argument("--flush-every", type=int, default=100)
    parser.add_argument("--print-every", type=int, default=100)
    parser.add_argument("--clock-sync-interval-s", type=float, default=5.0)
    parser.add_argument("--segment-id-base", choices=["auto", "zero", "one"], default="auto")
    parser.add_argument("--body-model", type=Path, default=DEFAULT_BODY_MODEL)
    parser.add_argument("--device", choices=["auto", "cpu", "cuda"], default="cpu")
    parser.add_argument("--num-betas", type=int, default=16)
    parser.add_argument(
        "--arm-correction",
        choices=["none", "xsens_ros2_plus90"],
        default="none",
        help="Arm segment post-rotation correction. Default keeps raw Xsens arm/hand segment frames.",
    )
    parser.add_argument(
        "--arm-twist-filter",
        choices=["none", "preserve_hand_global", "keep_measured_wrist"],
        default="none",
        help="Optional arm twist filter. Default exports raw local arm rotations.",
    )
    parser.add_argument("--viewer", choices=["pyqtgraph", "none"], default="pyqtgraph")
    parser.add_argument("--display-fps", type=float, default=30.0)
    parser.add_argument("--pt-checkpoint-every-s", type=float, default=5.0)
    parser.add_argument("--pt-checkpoint-every-frames", type=int, default=0)
    parser.add_argument("--retarget-queue-size", type=int, default=1024)
    parser.add_argument("--status-every-s", type=float, default=1.0)
    return parser.parse_args()


def frame_from_pose_rows(rows: Sequence[Dict[str, object]]) -> FramePose:
    if not rows:
        raise ValueError("empty pose rows")
    first = rows[0]
    if str(first.get("rotation_format", "")) != "quaternion_wxyz":
        raise ValueError(f"only quaternion pose packets are retargeted, got {first.get('rotation_format')!r}")
    if any(int(row.get("sample_complete", 1)) != 1 for row in rows):
        raise ValueError(f"sample {first.get('sample_counter')} is incomplete")

    sample_counter = int(float(first["sample_counter"]))
    time_code_ms = int(float(first["time_code_ms"]))
    frame = FramePose(sample_counter=sample_counter, time_code_ms=time_code_ms)

    for row in rows:
        if int(float(row["sample_counter"])) != sample_counter:
            raise ValueError("pose rows contain multiple sample counters")
        item_kind = str(row.get("item_kind", ""))
        item_name = str(row.get("item_name", ""))
        pose = SegmentPose(
            rotmat=quat_wxyz_to_rotmat(row),
            pos_m=position_m_from_row(row),
            coordinate_system=str(row.get("coordinate_system", "")),
        )
        if item_kind == "body":
            frame.body[item_name] = pose
        elif item_kind == "left_finger":
            frame.left_finger[item_name] = pose
        elif item_kind == "right_finger":
            frame.right_finger[item_name] = pose

    validate_mvn_frames([frame])
    return frame


def host_timestamps_from_pose_rows(rows: Sequence[Dict[str, object]]) -> Tuple[float, float]:
    first = rows[0] if rows else {}
    wall_raw = first.get("host_wall_ts_s", first.get("recv_time_s", "nan"))
    perf_raw = first.get("host_perf_ts_s", "nan")
    return float(wall_raw), float(perf_raw)


def build_capture_args(args: argparse.Namespace) -> argparse.Namespace:
    return argparse.Namespace(
        bind_ip=args.bind_ip,
        port=args.port,
        output_dir=args.output_dir,
        accept_from=args.accept_from,
        duration_s=args.duration_s,
        max_packets=args.max_packets,
        max_samples=args.max_samples,
        socket_buffer_bytes=args.socket_buffer_bytes,
        recv_bytes=args.recv_bytes,
        pending_timeout_s=args.pending_timeout_s,
        flush_every=args.flush_every,
        print_every=args.print_every,
        clock_sync_interval_s=args.clock_sync_interval_s,
        segment_id_base=args.segment_id_base,
    )


def make_status_lines(
    state: SharedRealtimeState,
    pose_queue: "queue.Queue[List[Dict[str, object]]]",
    start_s: float,
) -> List[str]:
    with state.lock:
        counters = Counter(state.counters)
        sample_counter = state.latest_sample_counter
        time_code_ms = state.latest_time_code_ms
        host_wall_ts_s = state.latest_host_wall_ts_s
        latest_error = state.latest_error
        capture_stats = dict(state.capture_stats) if state.capture_stats is not None else None
        final_pt_path = state.final_pt_path
        checkpoint_pt_path = state.checkpoint_pt_path

    elapsed = max(1e-6, time.monotonic() - start_s)
    lines = [
        "decoded_pose={decoded} retargeted={retargeted} saved={saved} "
        "mesh={mesh} skipped={skipped} errors={errors}".format(
            decoded=counters["decoded_pose_samples"],
            retargeted=counters["retargeted_frames"],
            saved=counters["saved_frames"],
            mesh=counters["mesh_frames"],
            skipped=counters["skipped_pose_samples"],
            errors=counters["retarget_errors"],
        ),
        "rates: decoded={:.1f}/s retargeted={:.1f}/s queue={}".format(
            counters["decoded_pose_samples"] / elapsed,
            counters["retargeted_frames"] / elapsed,
            pose_queue.qsize(),
        ),
    ]
    if sample_counter is not None:
        lines.append(f"latest sample={sample_counter} time_code_ms={time_code_ms}")
    if host_wall_ts_s is not None:
        lines.append(f"latest host_wall_ts_s={host_wall_ts_s:.6f}")
    if checkpoint_pt_path is not None:
        lines.append(f"checkpoint={checkpoint_pt_path}")
    if final_pt_path is not None:
        lines.append(f"final={final_pt_path}")
    if capture_stats is not None:
        lines.append(
            "capture packets={packets} samples={samples} pose_rows={pose_rows}".format(
                packets=capture_stats.get("packets", 0),
                samples=capture_stats.get("samples_decoded", 0),
                pose_rows=capture_stats.get("pose_rows", 0),
            )
        )
    if latest_error:
        lines.append(f"last error: {latest_error}")
    return lines


def retarget_worker(
    args: argparse.Namespace,
    retargeter: RealtimeSmplhRetargeter,
    pose_queue: "queue.Queue[List[Dict[str, object]]]",
    stop_event: threading.Event,
    state: SharedRealtimeState,
) -> None:
    include_vertices = args.viewer != "none"
    pt_output = args.pt_output or (args.output_dir / "xsens_mvn_smplh_realtime.pt")
    checkpoint_output = pt_output.with_name(pt_output.stem + "_checkpoint.pt")
    accumulator = SmplhPtAccumulator(
        metadata=retargeter.metadata(args.output_dir / "mvn_pose_segments.csv"),
        num_betas=args.num_betas,
    )
    last_checkpoint_s = time.monotonic()
    last_checkpoint_frames = 0

    try:
        while not stop_event.is_set() or not pose_queue.empty():
            try:
                rows = pose_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            try:
                frame = frame_from_pose_rows(rows)
                smplh_frame = retargeter.retarget_frame(frame, include_vertices=include_vertices)
                smplh_frame.host_wall_ts_s, smplh_frame.host_perf_ts_s = host_timestamps_from_pose_rows(rows)
                accumulator.add(smplh_frame)
                with state.lock:
                    state.counters["retargeted_frames"] += 1
                    state.counters["saved_frames"] = accumulator.frame_count
                    state.latest_sample_counter = smplh_frame.sample_counter
                    state.latest_time_code_ms = smplh_frame.time_code_ms
                    state.latest_host_wall_ts_s = smplh_frame.host_wall_ts_s
                    if smplh_frame.vertices is not None:
                        state.latest_vertices = smplh_frame.vertices
                        state.counters["mesh_frames"] += 1

                now_s = time.monotonic()
                time_due = args.pt_checkpoint_every_s > 0 and now_s - last_checkpoint_s >= args.pt_checkpoint_every_s
                frame_due = (
                    args.pt_checkpoint_every_frames > 0
                    and accumulator.frame_count - last_checkpoint_frames >= args.pt_checkpoint_every_frames
                )
                if accumulator.frame_count > 0 and (time_due or frame_due):
                    accumulator.save_atomic(checkpoint_output)
                    last_checkpoint_s = now_s
                    last_checkpoint_frames = accumulator.frame_count
                    with state.lock:
                        state.checkpoint_pt_path = checkpoint_output
            except Exception as exc:
                with state.lock:
                    state.counters["retarget_errors"] += 1
                    state.latest_error = str(exc)
                    if rows:
                        state.counters["skipped_pose_samples"] += 1
            finally:
                pose_queue.task_done()
    finally:
        accumulator.save_atomic(pt_output)
        with state.lock:
            state.final_pt_path = pt_output


def capture_worker(
    args: argparse.Namespace,
    pose_queue: "queue.Queue[List[Dict[str, object]]]",
    stop_event: threading.Event,
    state: SharedRealtimeState,
) -> None:
    def enqueue_pose_rows(rows: List[Dict[str, object]]) -> None:
        with state.lock:
            state.counters["decoded_pose_samples"] += 1
        while not stop_event.is_set():
            try:
                pose_queue.put(rows, timeout=0.1)
                return
            except queue.Full:
                with state.lock:
                    state.counters["queue_full_waits"] += 1

    try:
        stats = mvn_udp.capture(
            build_capture_args(args),
            pose_callback=enqueue_pose_rows,
            stop_event=stop_event,
            install_signal_handlers=False,
        )
        with state.lock:
            state.capture_stats = dict(stats)
    except Exception as exc:
        with state.lock:
            state.latest_error = str(exc)
            state.counters["capture_errors"] += 1
    finally:
        stop_event.set()


def run_headless(
    args: argparse.Namespace,
    pose_queue: "queue.Queue[List[Dict[str, object]]]",
    stop_event: threading.Event,
    state: SharedRealtimeState,
    capture_thread: threading.Thread,
    retarget_thread: threading.Thread,
    start_s: float,
) -> None:
    last_print = 0.0
    while capture_thread.is_alive() or retarget_thread.is_alive():
        now_s = time.monotonic()
        if now_s - last_print >= args.status_every_s:
            print("[REALTIME] " + " | ".join(make_status_lines(state, pose_queue, start_s)), flush=True)
            last_print = now_s
        time.sleep(0.05)


def run_pyqtgraph_viewer(
    args: argparse.Namespace,
    retargeter: RealtimeSmplhRetargeter,
    pose_queue: "queue.Queue[List[Dict[str, object]]]",
    stop_event: threading.Event,
    state: SharedRealtimeState,
    retarget_thread: threading.Thread,
    start_s: float,
) -> None:
    import pyqtgraph as pg
    import pyqtgraph.opengl as gl
    from pyqtgraph.Qt import QtCore, QtWidgets

    pg.setConfigOptions(antialias=False, useOpenGL=True)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    app.aboutToQuit.connect(stop_event.set)

    win = QtWidgets.QWidget()
    win.setWindowTitle("Realtime Xsens MVN -> SMPL-H")
    layout = QtWidgets.QVBoxLayout(win)
    view = gl.GLViewWidget()
    view.opts["distance"] = 3.0
    view.opts["elevation"] = 15.0
    view.opts["azimuth"] = -65.0
    layout.addWidget(view, stretch=1)

    grid = gl.GLGridItem()
    grid.setSize(x=4.0, y=4.0, z=0.0)
    grid.setSpacing(x=0.5, y=0.5, z=0.5)
    view.addItem(grid)

    label = QtWidgets.QLabel("Waiting for Xsens MVN UDP pose packets...")
    label.setStyleSheet("font-family: Consolas, monospace; font-size: 12px;")
    layout.addWidget(label)

    timer = QtCore.QTimer()
    timer.setInterval(max(1, int(1000.0 / max(1.0, args.display_fps))))
    try:
        timer.setTimerType(QtCore.Qt.PreciseTimer)
    except Exception:
        pass

    mesh_item = {"item": None, "sample_counter": None}

    def tick() -> None:
        with state.lock:
            vertices = None if state.latest_vertices is None else state.latest_vertices.copy()
            sample_counter = state.latest_sample_counter
        if vertices is not None and sample_counter != mesh_item["sample_counter"]:
            if mesh_item["item"] is None:
                mesh_item["item"] = gl.GLMeshItem(
                    vertexes=vertices,
                    faces=retargeter.faces,
                    color=(0.72, 0.72, 0.72, 1.0),
                    smooth=False,
                    drawEdges=False,
                    drawFaces=True,
                    shader="shaded",
                )
                view.addItem(mesh_item["item"])
            else:
                mesh_item["item"].setMeshData(vertexes=vertices, faces=retargeter.faces)
            mesh_item["sample_counter"] = sample_counter

        label.setText("\n".join(make_status_lines(state, pose_queue, start_s)))
        if stop_event.is_set() and not retarget_thread.is_alive():
            app.quit()

    timer.timeout.connect(tick)
    timer.start()
    win.resize(1100, 820)
    win.show()
    app.exec()


def main() -> None:
    args = parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    stop_event = threading.Event()
    state = SharedRealtimeState()
    pose_queue: "queue.Queue[List[Dict[str, object]]]" = queue.Queue(maxsize=max(1, args.retarget_queue_size))
    start_s = time.monotonic()

    retargeter = RealtimeSmplhRetargeter(
        body_model_path=args.body_model,
        num_betas=args.num_betas,
        device_arg=args.device,
        config=RetargetConfig(
            arm_correction=args.arm_correction,
            arm_twist_filter=args.arm_twist_filter,
        ),
    )

    def request_stop(_signum: int, _frame: object) -> None:
        stop_event.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    capture_thread = threading.Thread(
        target=capture_worker,
        args=(args, pose_queue, stop_event, state),
        name="xsens-udp-capture",
        daemon=True,
    )
    retarget_thread = threading.Thread(
        target=retarget_worker,
        args=(args, retargeter, pose_queue, stop_event, state),
        name="xsens-smplh-retarget",
        daemon=True,
    )

    print(f"[REALTIME] Listening on {args.bind_ip}:{args.port}", flush=True)
    print(f"[REALTIME] CSV output: {args.output_dir}", flush=True)
    print(f"[REALTIME] PT output: {args.pt_output or (args.output_dir / 'xsens_mvn_smplh_realtime.pt')}", flush=True)
    capture_thread.start()
    retarget_thread.start()

    try:
        if args.viewer == "none":
            run_headless(args, pose_queue, stop_event, state, capture_thread, retarget_thread, start_s)
        else:
            run_pyqtgraph_viewer(args, retargeter, pose_queue, stop_event, state, retarget_thread, start_s)
    finally:
        stop_event.set()
        capture_thread.join(timeout=2.0)
        retarget_thread.join(timeout=10.0)
        print("[REALTIME] " + " | ".join(make_status_lines(state, pose_queue, start_s)), flush=True)


if __name__ == "__main__":
    main()
