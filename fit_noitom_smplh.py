#!/usr/bin/env python3
"""
Convert Noitom Calc global Bone-Quat data to SMPLH pose.

Official Noitom Calc documentation states:
- Calc data is reported in the global frame, not parent-relative.
- Bone-Quat is a global bone rotation quantity.

This script follows that definition:
1. Read global Bone-Quat and Joint-Posi from the Calc CSV.
2. Map Noitom global rotations to SMPL 24-joint global rotations.
3. For missing SMPL joints, inherit the parent's global rotation.
4. Convert global SMPL rotations to local SMPL rotations.
5. Use Hips Joint-Posi to produce the model translation.
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
import pandas as pd
import torch
from scipy.spatial.transform import Rotation, Slerp


SMPL24_NAMES = [
    "pelvis",
    "left_hip",
    "right_hip",
    "spine1",
    "left_knee",
    "right_knee",
    "spine2",
    "left_ankle",
    "right_ankle",
    "spine3",
    "left_foot",
    "right_foot",
    "neck",
    "left_collar",
    "right_collar",
    "head",
    "left_shoulder",
    "right_shoulder",
    "left_elbow",
    "right_elbow",
    "left_wrist",
    "right_wrist",
    "left_hand",
    "right_hand",
]

SMPL24_PARENTS = np.array(
    [
        -1,  # pelvis
        0,   # left_hip
        0,   # right_hip
        0,   # spine1
        1,   # left_knee
        2,   # right_knee
        3,   # spine2
        4,   # left_ankle
        5,   # right_ankle
        6,   # spine3
        7,   # left_foot
        8,   # right_foot
        9,   # neck
        9,   # left_collar
        9,   # right_collar
        12,  # head
        13,  # left_shoulder
        14,  # right_shoulder
        16,  # left_elbow
        17,  # right_elbow
        18,  # left_wrist
        19,  # right_wrist
        20,  # left_hand
        21,  # right_hand
    ],
    dtype=np.int64,
)

BODY22_NAMES = SMPL24_NAMES[:22]
_IGNORED_INDICES: List[int] = []
DEFAULT_BM_PATH = Path("/mnt/d/a_WORK/Projects/PhD/datasets/smpl_models/smplh/neutral/model.npz")
DEFAULT_RACKET_OBJ = Path("obj/badminton_racket/Racket.obj")
PRESSURE_COLUMNS = [f"p_{i}" for i in range(256)]
# The saved smpl24 array's right_hand entry is not a stable palm anchor in this
# pipeline; right_wrist is the reliable right-side hand attachment point.
RACKET_ANCHOR_INDEX = SMPL24_NAMES.index("right_wrist")

# Map SMPL24 joints to Noitom Calc global Bone-Quat joints.
# Values can be a string (single candidate) or a list of strings (tried in
# order; first one found in the CSV wins).  None means inherit parent.
SMPL24_TO_NOITOM_GLOBAL = {
    "pelvis": "Hips",
    "left_hip": "LeftUpLeg",
    "right_hip": "RightUpLeg",
    "spine1": "Spine",
    "left_knee": "LeftLeg",
    "right_knee": "RightLeg",
    "spine2": "Spine1",
    "left_ankle": "LeftFoot",
    "right_ankle": "RightFoot",
    "spine3": "Spine2",
    "left_foot": "LeftToeBase",
    "right_foot": "RightToeBase",
    "neck": "Neck",
    "left_collar": "LeftShoulder",
    "right_collar": "RightShoulder",
    "head": "Head",
    "left_shoulder": "LeftArm",
    "right_shoulder": "RightArm",
    "left_elbow": "LeftForeArm",
    "right_elbow": "RightForeArm",
    "left_wrist": "LeftHand",
    "right_wrist": "RightHand",
    "left_hand": ["LeftInHandMiddle", "LeftHandMiddle1"],
    "right_hand": ["RightInHandMiddle", "RightHandMiddle1"],
}


@dataclass
class ObjMesh:
    vertices: np.ndarray
    faces: np.ndarray
    texcoords: Optional[np.ndarray] = None
    texture_path: Optional[Path] = None
    vertex_colors: Optional[np.ndarray] = None


@dataclass
class SyncSequence:
    imu_host_ts: np.ndarray
    rotmats: np.ndarray
    pressure: np.ndarray


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--csv-path", type=Path, default=Path("noitom_badmin/output_noitom_csv/take006_chr02.csv"))
    parser.add_argument("--output-dir", type=Path, default=Path("output/noitom_smplh_global_bonequat"))
    parser.add_argument("--sync-path", type=Path, default="output/20260428_001800/sync.csv", help="Optional sync.csv from run_dual_sensor_sage.py.")
    parser.add_argument("--bm-path", type=Path, default=DEFAULT_BM_PATH)
    parser.add_argument("--max-frames", type=int, default=None)
    parser.add_argument("--frame-stride", type=int, default=1)
    parser.add_argument("--device", type=str, default="cuda", help="auto/cpu/cuda")
    parser.add_argument("--num-betas", type=int, default=16)
    parser.add_argument("--betas-npz", type=Path, default=None, help="Optional npz file containing a 'betas' array.")
    parser.add_argument("--batch-size", type=int, default=256)
    parser.add_argument("--fps", type=float, default=60.0, help="Noitom CSV FPS used to align sync timestamps.")
    parser.add_argument("--visualize", default=True)
    parser.add_argument("--view-fit", type=Path, default=None, help="Only visualize an existing output npz.")
    parser.add_argument(
        "--sync-start-frame",
        type=int,
        default=0,
        help="Noitom Frame-No value corresponding to the first sync frame.",
    )
    parser.add_argument("--racket-obj", type=Path, default=DEFAULT_RACKET_OBJ)
    parser.add_argument("--racket-face-budget", type=int, default=8000)
    parser.add_argument("--racket-align-roll", type=float, default=0.0)
    parser.add_argument("--racket-align-pitch", type=float, default=0.0)
    parser.add_argument("--racket-align-yaw", type=float, default=0.0)
    parser.add_argument("--contact-smooth-window", type=int, default=3)
    parser.add_argument("--contact-enter-sum", type=float, default=250.0)
    parser.add_argument("--contact-exit-sum", type=float, default=150.0)
    parser.add_argument("--contact-enter-max", type=float, default=60.0)
    parser.add_argument("--contact-exit-max", type=float, default=25.0)
    parser.add_argument("--contact-enter-frames", type=int, default=2)
    parser.add_argument("--contact-exit-frames", type=int, default=3)
    return parser.parse_args()


def read_noitom_calc(
    csv_path: Path,
) -> Tuple[np.ndarray, Dict[str, np.ndarray], Dict[str, np.ndarray]]:
    df = pd.read_csv(csv_path, header=1)
    if "Frame-No" not in df.columns:
        raise ValueError(f"Frame-No column not found in {csv_path}")

    frame_no = df["Frame-No"].to_numpy(dtype=np.int32)
    positions: Dict[str, np.ndarray] = {}
    bone_quats: Dict[str, np.ndarray] = {}

    for col in df.columns:
        if not isinstance(col, str):
            continue
        if col.endswith("-Joint-Posi-x"):
            joint = col[: -len("-Joint-Posi-x")]
            cols = [f"{joint}-Joint-Posi-{axis}" for axis in "xyz"]
            if all(name in df.columns for name in cols):
                positions[joint] = df[cols].to_numpy(dtype=np.float32)
        if col.endswith("-Bone-Quat-x"):
            joint = col[: -len("-Bone-Quat-x")]
            cols = [f"{joint}-Bone-Quat-{axis}" for axis in "xyzw"]
            if all(name in df.columns for name in cols):
                bone_quats[joint] = df[cols].to_numpy(dtype=np.float32)

    if "Hips" not in positions or "Hips" not in bone_quats:
        raise ValueError("Expected Hips Joint-Posi and Hips Bone-Quat in Calc CSV.")
    return frame_no, positions, bone_quats


def subsample_data(
    frame_no: np.ndarray,
    positions: Dict[str, np.ndarray],
    bone_quats: Dict[str, np.ndarray],
    frame_stride: int,
    max_frames: int | None,
) -> Tuple[np.ndarray, Dict[str, np.ndarray], Dict[str, np.ndarray]]:
    if frame_stride > 1:
        frame_no = frame_no[::frame_stride]
        positions = {name: value[::frame_stride] for name, value in positions.items()}
        bone_quats = {name: value[::frame_stride] for name, value in bone_quats.items()}

    if max_frames is not None:
        frame_no = frame_no[:max_frames]
        positions = {name: value[:max_frames] for name, value in positions.items()}
        bone_quats = {name: value[:max_frames] for name, value in bone_quats.items()}

    return frame_no, positions, bone_quats


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
    elif idx < 0:
        idx = n_items + idx
    else:
        raise ValueError("OBJ indices are 1-based.")
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
                    tail = tex_rel.split()[-1]
                    cand_tail = (mtl_path.parent / tail).resolve()
                    if cand_tail.exists():
                        return cand_tail
        except Exception:
            continue

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
    return ObjMesh(
        vertices=vertices[used].astype(np.float32, copy=False),
        faces=remap[faces].astype(np.int32, copy=False),
        texcoords=texcoords[used].astype(np.float32, copy=False) if texcoords is not None else None,
        texture_path=texture_path,
        vertex_colors=vertex_colors[used].astype(np.float32, copy=False) if vertex_colors is not None else None,
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

    butt_slice = z >= (z_max - 0.10 * z_span)
    if int(np.count_nonzero(butt_slice)) < 100:
        butt_slice = z >= (z_max - 0.18 * z_span)
    if int(np.count_nonzero(butt_slice)) < 20:
        butt_slice = np.ones(vertices.shape[0], dtype=bool)
    butt_pts = vertices[butt_slice]
    center_xy = np.mean(butt_pts[:, :2], axis=0).astype(np.float32)

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
    if mesh.texcoords is not None:
        faces = mesh.faces
    else:
        faces = decimate_faces_stride(mesh.faces, face_budget)
    return compact_mesh(aligned, faces, texcoords=mesh.texcoords, texture_path=mesh.texture_path)


def has_contact_change(current_frame: np.ndarray, prev_frame: np.ndarray) -> bool:
    delta_max = abs(float(np.max(current_frame)) - float(np.max(prev_frame)))
    delta_sum = abs(float(np.sum(current_frame)) - float(np.sum(prev_frame)))
    return delta_max > 3.0 or delta_sum > 30.0


def read_sync_csv(sync_path: Path) -> SyncSequence:
    df = pd.read_csv(sync_path)
    required = {"imu_host_ts", "imu_qw", "imu_qx", "imu_qy", "imu_qz", *PRESSURE_COLUMNS}
    missing = sorted(required.difference(df.columns))
    if missing:
        raise ValueError(f"Missing columns in {sync_path}: {', '.join(missing)}")

    host_ts = pd.to_numeric(df["imu_host_ts"], errors="coerce").to_numpy(dtype=np.float64)
    valid_ts = np.isfinite(host_ts)
    if not np.any(valid_ts):
        raise ValueError(f"No valid imu_host_ts found in {sync_path}")
    df = df.loc[valid_ts].reset_index(drop=True)
    host_ts = host_ts[valid_ts]

    order = np.argsort(host_ts, kind="stable")
    df = df.iloc[order].reset_index(drop=True)
    host_ts = host_ts[order]

    quats = (
        df[["imu_qw", "imu_qx", "imu_qy", "imu_qz"]]
        .apply(pd.to_numeric, errors="coerce")
        .to_numpy(dtype=np.float32)
    )
    invalid_quat = ~np.isfinite(quats).all(axis=1)
    quats[invalid_quat] = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)

    rotmats = np.empty((quats.shape[0], 3, 3), dtype=np.float32)
    for idx, quat in enumerate(quats):
        rotmats[idx] = quat_wxyz_to_rotmat((float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3])))

    pressure = (
        df[PRESSURE_COLUMNS]
        .apply(pd.to_numeric, errors="coerce")
        .ffill()
        .fillna(0.0)
        .to_numpy(dtype=np.float32)
    )
    return SyncSequence(imu_host_ts=host_ts.astype(np.float64), rotmats=rotmats, pressure=pressure)


def align_sync_to_noitom_frames(
    sync_seq: SyncSequence,
    frame_no: np.ndarray,
    fps: float,
    sync_start_frame: int,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    if fps <= 0.0:
        raise ValueError(f"fps must be > 0, got {fps}")
    if sync_seq.imu_host_ts.size == 0:
        raise ValueError("Sync sequence is empty.")

    sync_time = sync_seq.imu_host_ts - float(sync_seq.imu_host_ts[0])
    noitom_time = (frame_no.astype(np.float64) - float(sync_start_frame)) / float(fps)
    sync_indices = np.searchsorted(sync_time, noitom_time, side="right") - 1
    sync_indices = np.clip(sync_indices, 0, sync_time.shape[0] - 1).astype(np.int32, copy=False)

    if sync_time.shape[0] == 1:
        rotmats = np.repeat(sync_seq.rotmats[:1], frame_no.shape[0], axis=0)
        pressure = np.repeat(sync_seq.pressure[:1], frame_no.shape[0], axis=0)
        return rotmats, pressure, sync_indices

    # Slerp requires strictly increasing keyframe times. Collapse duplicate
    # timestamps to the last sample so lower-rate sync streams can be smoothly
    # interpolated up to the Noitom frame rate.
    unique_time, unique_indices = np.unique(sync_time, return_index=True)
    unique_rotmats = sync_seq.rotmats[unique_indices]
    unique_pressure = sync_seq.pressure[unique_indices]

    if unique_time.shape[0] == 1:
        rotmats = np.repeat(unique_rotmats[:1], frame_no.shape[0], axis=0)
        pressure = np.repeat(unique_pressure[:1], frame_no.shape[0], axis=0)
        return rotmats, pressure, sync_indices

    clipped_time = np.clip(noitom_time, unique_time[0], unique_time[-1])
    slerp = Slerp(unique_time, Rotation.from_matrix(unique_rotmats))
    rotmats = slerp(clipped_time).as_matrix().astype(np.float32)

    pressure = np.empty((frame_no.shape[0], unique_pressure.shape[1]), dtype=np.float32)
    for col_idx in range(unique_pressure.shape[1]):
        pressure[:, col_idx] = np.interp(
            clipped_time,
            unique_time,
            unique_pressure[:, col_idx],
            left=float(unique_pressure[0, col_idx]),
            right=float(unique_pressure[-1, col_idx]),
        )

    return rotmats, pressure.astype(np.float32, copy=False), sync_indices


def smooth_1d_signal(values: np.ndarray, window: int) -> np.ndarray:
    if window <= 1 or values.shape[0] <= 1:
        return values.astype(np.float32, copy=True)
    window = max(1, int(window))
    if window % 2 == 0:
        window += 1
    pad = window // 2
    padded = np.pad(values.astype(np.float32, copy=False), (pad, pad), mode="edge")
    kernel = np.full((window,), 1.0 / float(window), dtype=np.float32)
    return np.convolve(padded, kernel, mode="valid").astype(np.float32, copy=False)


def build_filtered_contact_state(
    pressure_frames: np.ndarray,
    smooth_window: int,
    enter_sum: float,
    exit_sum: float,
    enter_max: float,
    exit_max: float,
    enter_frames: int,
    exit_frames: int,
) -> np.ndarray:
    if pressure_frames.ndim != 2:
        raise ValueError(f"Expected [T, 256] pressure frames, got {tuple(pressure_frames.shape)}")

    pressure_sum = smooth_1d_signal(np.sum(pressure_frames, axis=1), smooth_window)
    pressure_max = smooth_1d_signal(np.max(pressure_frames, axis=1), smooth_window)

    contact_state = np.zeros((pressure_frames.shape[0],), dtype=bool)
    in_contact = False
    enter_count = 0
    exit_count = 0
    need_enter = max(1, int(enter_frames))
    need_exit = max(1, int(exit_frames))

    for idx in range(pressure_frames.shape[0]):
        if not in_contact:
            enter_candidate = (pressure_sum[idx] >= enter_sum) or (pressure_max[idx] >= enter_max)
            if enter_candidate:
                enter_count += 1
            else:
                enter_count = 0
            if enter_count >= need_enter:
                in_contact = True
                exit_count = 0
        else:
            exit_candidate = (pressure_sum[idx] <= exit_sum) and (pressure_max[idx] <= exit_max)
            if exit_candidate:
                exit_count += 1
            else:
                exit_count = 0
            if exit_count >= need_exit:
                in_contact = False
                enter_count = 0
        contact_state[idx] = in_contact

    return contact_state


def build_racket_vertices_sequence(
    racket_mesh: ObjMesh,
    rotmats: np.ndarray,
    hand_positions: np.ndarray,
    pressure_frames: np.ndarray,
    contact_smooth_window: int,
    contact_enter_sum: float,
    contact_exit_sum: float,
    contact_enter_max: float,
    contact_exit_max: float,
    contact_enter_frames: int,
    contact_exit_frames: int,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    if rotmats.shape[0] != hand_positions.shape[0] or rotmats.shape[0] != pressure_frames.shape[0]:
        raise ValueError("Racket inputs must share the same frame count.")

    n_frames = rotmats.shape[0]
    contact_frames = build_filtered_contact_state(
        pressure_frames=pressure_frames,
        smooth_window=contact_smooth_window,
        enter_sum=contact_enter_sum,
        exit_sum=contact_exit_sum,
        enter_max=contact_enter_max,
        exit_max=contact_exit_max,
        enter_frames=contact_enter_frames,
        exit_frames=contact_exit_frames,
    )
    anchor_positions = np.empty_like(hand_positions, dtype=np.float32)
    current_anchor = hand_positions[0].astype(np.float32, copy=True)

    for idx in range(n_frames):
        if idx == 0 or contact_frames[idx]:
            current_anchor = hand_positions[idx].astype(np.float32, copy=True)
        anchor_positions[idx] = current_anchor

    racket_vertices = np.empty((n_frames, racket_mesh.vertices.shape[0], 3), dtype=np.float32)
    for idx in range(n_frames):
        racket_vertices[idx] = rotate_points(racket_mesh.vertices, rotmats[idx]) + anchor_positions[idx][None, :]

    return racket_vertices, anchor_positions, contact_frames


def load_betas(betas_npz: Path | None, num_betas: int) -> np.ndarray:
    if betas_npz is None:
        return np.zeros((num_betas,), dtype=np.float32)

    data = np.load(betas_npz, allow_pickle=True)
    if "betas" not in data.files:
        raise ValueError(f"'betas' array not found in {betas_npz}")

    betas = np.asarray(data["betas"], dtype=np.float32).reshape(-1)
    out = np.zeros((num_betas,), dtype=np.float32)
    out[: min(num_betas, betas.shape[0])] = betas[: min(num_betas, betas.shape[0])]
    return out


def _global_to_local_rotmat(global_rotmats: torch.Tensor, parents: np.ndarray) -> torch.Tensor:
    if global_rotmats.dim() != 4 or global_rotmats.shape[-2:] != (3, 3):
        raise ValueError(f"Expected [T, J, 3, 3], got {tuple(global_rotmats.shape)}")

    seq_len, num_joints = global_rotmats.shape[:2]
    local_rotmats = torch.zeros_like(global_rotmats)
    local_rotmats[:, 0] = global_rotmats[:, 0]

    for i in range(1, num_joints):
        parent_idx = int(parents[i]) if i < len(parents) else 0
        parent_idx = max(0, min(parent_idx, num_joints - 1))
        parent_rot = global_rotmats[:, parent_idx]
        local_rotmats[:, i] = torch.matmul(parent_rot.transpose(-1, -2), global_rotmats[:, i])

    if _IGNORED_INDICES:
        valid_ignored = [idx for idx in _IGNORED_INDICES if idx < num_joints]
        if valid_ignored:
            eye = torch.eye(3, device=global_rotmats.device, dtype=global_rotmats.dtype).view(1, 1, 3, 3)
            local_rotmats[:, valid_ignored] = eye.expand(seq_len, len(valid_ignored), 3, 3)

    return local_rotmats


def _resolve_noitom_name(candidates, bone_quats: Dict[str, np.ndarray]) -> Optional[str]:
    """Resolve a mapping entry (str, list of str, or None) to the first available Noitom joint name."""
    if candidates is None:
        return None
    if isinstance(candidates, str):
        return candidates if candidates in bone_quats else None
    for name in candidates:
        if name in bone_quats:
            return name
    return None


def build_smpl24_global_rotmats(bone_quats: Dict[str, np.ndarray]) -> np.ndarray:
    n_frames = bone_quats["Hips"].shape[0]
    global_rotmats = np.zeros((n_frames, len(SMPL24_NAMES), 3, 3), dtype=np.float32)

    for joint_idx, joint_name in enumerate(SMPL24_NAMES):
        candidates = SMPL24_TO_NOITOM_GLOBAL[joint_name]
        noitom_name = _resolve_noitom_name(candidates, bone_quats)

        if noitom_name is None:
            parent_idx = int(SMPL24_PARENTS[joint_idx])
            if parent_idx < 0:
                global_rotmats[:, joint_idx] = np.eye(3, dtype=np.float32)
            else:
                global_rotmats[:, joint_idx] = global_rotmats[:, parent_idx]
            continue

        global_rotmats[:, joint_idx] = Rotation.from_quat(bone_quats[noitom_name]).as_matrix().astype(np.float32)

    return global_rotmats


def smpl_pose_from_global_rotmats(global_rotmats_np: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    global_rotmats = torch.tensor(global_rotmats_np, dtype=torch.float32)
    local_rotmats = _global_to_local_rotmat(global_rotmats, SMPL24_PARENTS)

    root_orient = Rotation.from_matrix(global_rotmats_np[:, 0]).as_rotvec().astype(np.float32)
    local_aa = Rotation.from_matrix(local_rotmats[:, :22].reshape(-1, 3, 3).cpu().numpy()).as_rotvec().astype(np.float32)
    local_aa = local_aa.reshape(global_rotmats_np.shape[0], 22, 3)
    pose_body = local_aa[:, 1:22].reshape(global_rotmats_np.shape[0], 63)
    return root_orient, pose_body, local_aa


def export_smpl24_csv(frame_no: np.ndarray, smpl24: np.ndarray, out_csv: Path) -> None:
    records = {"Frame-No": frame_no}
    for joint_idx, joint_name in enumerate(SMPL24_NAMES):
        for axis_idx, axis_name in enumerate("xyz"):
            records[f"{joint_name}-Joint-Posi-{axis_name}"] = smpl24[:, joint_idx, axis_idx]
    pd.DataFrame(records).to_csv(out_csv, index=False)


def select_device(device_arg: str):
    if device_arg == "auto":
        return torch.device("cuda" if torch.cuda.is_available() else "cpu")
    return torch.device(device_arg)


def create_body_model(bm_path: Path, num_betas: int, device):
    from human_body_prior.body_model.body_model import BodyModel

    bm = BodyModel(bm_fname=str(bm_path), num_betas=num_betas)
    bm = bm.to(device)
    bm.eval()
    return bm


def compute_trans_from_hips(hips_pos: np.ndarray, root_orient: np.ndarray, rest_root: np.ndarray) -> np.ndarray:
    root_rot = Rotation.from_rotvec(root_orient)
    rotated_rest_root = root_rot.apply(rest_root)
    return (hips_pos - rotated_rest_root).astype(np.float32)


def forward_body_model(
    bm,
    pose_body: np.ndarray,
    root_orient: np.ndarray,
    trans: np.ndarray,
    betas: np.ndarray,
    device,
    batch_size: int,
) -> Tuple[np.ndarray, np.ndarray]:
    n_frames = pose_body.shape[0]
    betas_batch = torch.tensor(betas[None], dtype=torch.float32, device=device)
    verts_batches: List[np.ndarray] = []
    joints_batches: List[np.ndarray] = []

    with torch.no_grad():
        for start in range(0, n_frames, batch_size):
            end = min(start + batch_size, n_frames)
            count = end - start
            out = bm(
                pose_body=torch.tensor(pose_body[start:end], dtype=torch.float32, device=device),
                pose_hand=torch.zeros(count, 90, dtype=torch.float32, device=device),
                betas=betas_batch.expand(count, -1),
                root_orient=torch.tensor(root_orient[start:end], dtype=torch.float32, device=device),
                trans=torch.tensor(trans[start:end], dtype=torch.float32, device=device),
            )
            verts_batches.append(out.v.detach().cpu().numpy().astype(np.float32))
            joints_batches.append(out.Jtr.detach().cpu().numpy().astype(np.float32))

    return np.concatenate(verts_batches, axis=0), np.concatenate(joints_batches, axis=0)


def save_pose_result(
    out_path: Path,
    frame_no: np.ndarray,
    pose_body: np.ndarray,
    root_orient: np.ndarray,
    trans: np.ndarray,
    betas: np.ndarray,
    smpl24: np.ndarray,
    global_rotmats: np.ndarray,
    local_rotmats: np.ndarray,
    bm_path: Path,
) -> None:
    np.savez_compressed(
        out_path,
        frame_no=frame_no.astype(np.int32),
        mode=np.array("bone_quat_global_to_local"),
        pose_body=pose_body.astype(np.float32),
        pose_hand=np.zeros((pose_body.shape[0], 90), dtype=np.float32),
        root_orient=root_orient.astype(np.float32),
        trans=trans.astype(np.float32),
        betas=betas.astype(np.float32),
        smpl24=smpl24.astype(np.float32),
        smpl24_names=np.array(SMPL24_NAMES),
        body22_names=np.array(BODY22_NAMES),
        smpl24_global_rotmats=global_rotmats.astype(np.float32),
        smpl24_local_rotmats=local_rotmats.astype(np.float32),
        smpl24_parents=SMPL24_PARENTS.astype(np.int64),
        bm_path=np.array(str(bm_path)),
    )


def visualize_fit(
    fit_npz: Path,
    sync_path: Path | None = None,
    fps: float = 60.0,
    sync_start_frame: int = 0,
    racket_obj: Path = DEFAULT_RACKET_OBJ,
    racket_face_budget: int = 8000,
    racket_align_roll: float = 0.0,
    racket_align_pitch: float = 0.0,
    racket_align_yaw: float = 0.0,
    contact_smooth_window: int = 3,
    contact_enter_sum: float = 250.0,
    contact_exit_sum: float = 150.0,
    contact_enter_max: float = 60.0,
    contact_exit_max: float = 25.0,
    contact_enter_frames: int = 2,
    contact_exit_frames: int = 3,
) -> None:
    from aitviewer.renderables.meshes import Meshes
    from aitviewer.viewer import Viewer

    data = np.load(fit_npz, allow_pickle=True)
    bm_path = Path(str(data["bm_path"].item()))
    device = torch.device("cpu")

    bm = create_body_model(bm_path, int(data["betas"].shape[-1]), device)
    vertices, _ = forward_body_model(
        bm=bm,
        pose_body=np.asarray(data["pose_body"], dtype=np.float32),
        root_orient=np.asarray(data["root_orient"], dtype=np.float32),
        trans=np.asarray(data["trans"], dtype=np.float32),
        betas=np.asarray(data["betas"], dtype=np.float32),
        device=device,
        batch_size=256,
    )
    faces = bm.f.detach().cpu().numpy().astype(np.int32)

    viewer = Viewer()
    viewer.scene.floor.enabled = False
    viewer.scene.add(Meshes(vertices, faces, color=(0.72, 0.72, 0.72, 1.0), name="SMPLH Mesh"))

    if sync_path is not None:
        if "smpl24" not in data or "frame_no" not in data:
            raise ValueError(f"{fit_npz} does not contain smpl24/frame_no required for racket visualization.")

        sync_seq = read_sync_csv(sync_path)
        racket_mesh = load_prepare_racket_mesh(
            obj_path=racket_obj,
            face_budget=max(100, racket_face_budget),
            align_roll=racket_align_roll,
            align_pitch=racket_align_pitch,
            align_yaw=racket_align_yaw,
        )
        frame_no = np.asarray(data["frame_no"], dtype=np.int32)
        smpl24 = np.asarray(data["smpl24"], dtype=np.float32)
        rotmats, pressure_frames, sync_indices = align_sync_to_noitom_frames(
            sync_seq=sync_seq,
            frame_no=frame_no,
            fps=fps,
            sync_start_frame=sync_start_frame,
        )
        racket_vertices, _, contact_frames = build_racket_vertices_sequence(
            racket_mesh=racket_mesh,
            rotmats=rotmats,
            hand_positions=smpl24[:, RACKET_ANCHOR_INDEX],
            pressure_frames=pressure_frames,
            contact_smooth_window=contact_smooth_window,
            contact_enter_sum=contact_enter_sum,
            contact_exit_sum=contact_exit_sum,
            contact_enter_max=contact_enter_max,
            contact_exit_max=contact_exit_max,
            contact_enter_frames=contact_enter_frames,
            contact_exit_frames=contact_exit_frames,
        )
        viewer.scene.add(
            Meshes(racket_vertices, racket_mesh.faces, color=(0.12, 0.46, 0.96, 1.0), name="Badminton Racket")
        )
        print(
            "[INFO] Racket sync aligned "
            f"{sync_seq.imu_host_ts.shape[0]} sync frames -> {frame_no.shape[0]} noitom frames, "
            f"contact snaps={int(contact_frames.sum())}, "
            f"sync frame range=[{int(sync_indices.min())}, {int(sync_indices.max())}]"
        )

    viewer.run()


def main() -> None:
    args = parse_args()

    if args.view_fit is not None:
        visualize_fit(
            fit_npz=args.view_fit,
            sync_path=args.sync_path,
            fps=args.fps,
            sync_start_frame=args.sync_start_frame,
            racket_obj=args.racket_obj,
            racket_face_budget=args.racket_face_budget,
            racket_align_roll=args.racket_align_roll,
            racket_align_pitch=args.racket_align_pitch,
            racket_align_yaw=args.racket_align_yaw,
            contact_smooth_window=args.contact_smooth_window,
            contact_enter_sum=args.contact_enter_sum,
            contact_exit_sum=args.contact_exit_sum,
            contact_enter_max=args.contact_enter_max,
            contact_exit_max=args.contact_exit_max,
            contact_enter_frames=args.contact_enter_frames,
            contact_exit_frames=args.contact_exit_frames,
        )
        return

    frame_no, positions, bone_quats = read_noitom_calc(args.csv_path)
    frame_no, positions, bone_quats = subsample_data(
        frame_no=frame_no,
        positions=positions,
        bone_quats=bone_quats,
        frame_stride=args.frame_stride,
        max_frames=args.max_frames,
    )

    global_rotmats = build_smpl24_global_rotmats(bone_quats)
    root_orient, pose_body, local_aa = smpl_pose_from_global_rotmats(global_rotmats)
    local_rotmats = Rotation.from_rotvec(local_aa.reshape(-1, 3)).as_matrix().astype(np.float32).reshape(-1, 22, 3, 3)
    betas = load_betas(args.betas_npz, args.num_betas)
    device = select_device(args.device)
    bm = create_body_model(args.bm_path, args.num_betas, device)

    with torch.no_grad():
        rest = bm(
            pose_body=torch.zeros(1, 63, dtype=torch.float32, device=device),
            pose_hand=torch.zeros(1, 90, dtype=torch.float32, device=device),
            betas=torch.tensor(betas[None], dtype=torch.float32, device=device),
            root_orient=torch.zeros(1, 3, dtype=torch.float32, device=device),
            trans=torch.zeros(1, 3, dtype=torch.float32, device=device),
        ).Jtr[0].detach().cpu().numpy()

    trans = compute_trans_from_hips(positions["Hips"], root_orient, rest[0])
    _, joints = forward_body_model(
        bm=bm,
        pose_body=pose_body,
        root_orient=root_orient,
        trans=trans,
        betas=betas,
        device=device,
        batch_size=args.batch_size,
    )
    smpl24 = joints[:, : len(SMPL24_NAMES)]

    args.output_dir.mkdir(parents=True, exist_ok=True)
    smpl24_csv = args.output_dir / f"{args.csv_path.stem}_smpl24.csv"
    pose_npz = args.output_dir / f"{args.csv_path.stem}_smplh_bonequat_global.npz"

    export_smpl24_csv(frame_no, smpl24, smpl24_csv)
    save_pose_result(
        out_path=pose_npz,
        frame_no=frame_no,
        pose_body=pose_body,
        root_orient=root_orient,
        trans=trans,
        betas=betas,
        smpl24=smpl24,
        global_rotmats=global_rotmats,
        local_rotmats=local_rotmats,
        bm_path=args.bm_path,
    )

    print(f"[SAVE] SMPL24 joints from global Bone-Quat -> {smpl24_csv}")
    print(f"[SAVE] SMPLH pose/mesh sequence -> {pose_npz}")
    print(f"[INFO] Frames: {frame_no.shape[0]}, device: {device}, betas source: {args.betas_npz or 'zeros'}")

    if args.visualize:
        visualize_fit(
            fit_npz=pose_npz,
            sync_path=args.sync_path,
            fps=args.fps,
            sync_start_frame=args.sync_start_frame,
            racket_obj=args.racket_obj,
            racket_face_budget=args.racket_face_budget,
            racket_align_roll=args.racket_align_roll,
            racket_align_pitch=args.racket_align_pitch,
            racket_align_yaw=args.racket_align_yaw,
            contact_smooth_window=args.contact_smooth_window,
            contact_enter_sum=args.contact_enter_sum,
            contact_exit_sum=args.contact_exit_sum,
            contact_enter_max=args.contact_enter_max,
            contact_exit_max=args.contact_exit_max,
            contact_enter_frames=args.contact_enter_frames,
            contact_exit_frames=args.contact_exit_frames,
        )


if __name__ == "__main__":
    main()
