#!/usr/bin/env python3
"""
Retarget Xsens MVN Network Streamer CSV output to SMPL-H parameters.

Input is the long-format ``mvn_pose_segments.csv`` produced by
``capture_xsens_mvn_udp.py``.  The default path follows public MVNX->SMPL
retargeting practice: convert the Xsens world frame into the SMPL frame, map
global segment orientations onto the SMPL/SMPL-H kinematic tree, then convert
them into local pose parameters.
"""

from __future__ import annotations

import argparse
import csv
import functools
import inspect
from collections import OrderedDict
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
import torch


DEFAULT_BODY_MODEL = Path("../../datasets/smpl_models/smplh/neutral/model.npz")

XSENS_ZUP_TO_SMPL = np.array(
    [
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
        [1.0, 0.0, 0.0],
    ],
    dtype=np.float32,
)

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

LEFT_SHOULDER_IDX = SMPL24_NAMES.index("left_shoulder")
RIGHT_SHOULDER_IDX = SMPL24_NAMES.index("right_shoulder")
LEFT_ELBOW_IDX = SMPL24_NAMES.index("left_elbow")
RIGHT_ELBOW_IDX = SMPL24_NAMES.index("right_elbow")
LEFT_WRIST_IDX = SMPL24_NAMES.index("left_wrist")
RIGHT_WRIST_IDX = SMPL24_NAMES.index("right_wrist")

SMPL24_PARENTS = np.array(
    [
        -1,
        0,
        0,
        0,
        1,
        2,
        3,
        4,
        5,
        6,
        7,
        8,
        9,
        9,
        9,
        12,
        13,
        14,
        16,
        17,
        18,
        19,
        20,
        21,
    ],
    dtype=np.int64,
)

SMPLH_52_NAMES = [
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
    "left_index1",
    "left_index2",
    "left_index3",
    "left_middle1",
    "left_middle2",
    "left_middle3",
    "left_pinky1",
    "left_pinky2",
    "left_pinky3",
    "left_ring1",
    "left_ring2",
    "left_ring3",
    "left_thumb1",
    "left_thumb2",
    "left_thumb3",
    "right_index1",
    "right_index2",
    "right_index3",
    "right_middle1",
    "right_middle2",
    "right_middle3",
    "right_pinky1",
    "right_pinky2",
    "right_pinky3",
    "right_ring1",
    "right_ring2",
    "right_ring3",
    "right_thumb1",
    "right_thumb2",
    "right_thumb3",
]

MVN_BODY_TO_SMPL24 = {
    "pelvis": "Pelvis",
    "left_hip": "Left Upper Leg",
    "right_hip": "Right Upper Leg",
    "spine1": "L5",
    "left_knee": "Left Lower Leg",
    "right_knee": "Right Lower Leg",
    "spine2": "T12",
    "left_ankle": "Left Foot",
    "right_ankle": "Right Foot",
    "spine3": "T8",
    "left_foot": "Left Toe",
    "right_foot": "Right Toe",
    "neck": "Neck",
    "left_collar": "Left Shoulder",
    "right_collar": "Right Shoulder",
    "head": "Head",
    "left_shoulder": "Left Upper Arm",
    "right_shoulder": "Right Upper Arm",
    "left_elbow": "Left Forearm",
    "right_elbow": "Right Forearm",
    "left_wrist": "Left Hand",
    "right_wrist": "Right Hand",
    "left_hand": "Left Hand",
    "right_hand": "Right Hand",
}

LEFT_ARM_SEGMENTS = {"Left Upper Arm", "Left Forearm", "Left Hand"}
RIGHT_ARM_SEGMENTS = {"Right Upper Arm", "Right Forearm", "Right Hand"}
LEFT_ARM_CORRECTION = np.array(
    [
        [1.0, 0.0, 0.0],
        [0.0, 0.0, -1.0],
        [0.0, 1.0, 0.0],
    ],
    dtype=np.float32,
)
RIGHT_ARM_CORRECTION = LEFT_ARM_CORRECTION.copy()
WRIST_TWIST_LIMIT_DEG = 60.0

LEFT_HAND_JOINT_MVN_PAIRS = {
    "left_index1": ("Left Second Metacarpal", "Left Second Proximal Phalange"),
    "left_index2": ("Left Second Proximal Phalange", "Left Second Middle Phalange"),
    "left_index3": ("Left Second Middle Phalange", "Left Second Distal Phalange"),
    "left_middle1": ("Left Third Metacarpal", "Left Third Proximal Phalange"),
    "left_middle2": ("Left Third Proximal Phalange", "Left Third Middle Phalange"),
    "left_middle3": ("Left Third Middle Phalange", "Left Third Distal Phalange"),
    "left_pinky1": ("Left Fifth Metacarpal", "Left Fifth Proximal Phalange"),
    "left_pinky2": ("Left Fifth Proximal Phalange", "Left Fifth Middle Phalange"),
    "left_pinky3": ("Left Fifth Middle Phalange", "Left Fifth Distal Phalange"),
    "left_ring1": ("Left Fourth Metacarpal", "Left Fourth Proximal Phalange"),
    "left_ring2": ("Left Fourth Proximal Phalange", "Left Fourth Middle Phalange"),
    "left_ring3": ("Left Fourth Middle Phalange", "Left Fourth Distal Phalange"),
    "left_thumb1": ("Left Carpus", "Left First Metacarpal"),
    "left_thumb2": ("Left First Metacarpal", "Left First Proximal Phalange"),
    "left_thumb3": ("Left First Proximal Phalange", "Left First Distal Phalange"),
}

RIGHT_HAND_JOINT_MVN_PAIRS = {
    key.replace("left_", "right_"): (
        parent.replace("Left ", "Right "),
        child.replace("Left ", "Right "),
    )
    for key, (parent, child) in LEFT_HAND_JOINT_MVN_PAIRS.items()
}

HAND_JOINT_MVN_PAIRS = {**LEFT_HAND_JOINT_MVN_PAIRS, **RIGHT_HAND_JOINT_MVN_PAIRS}
HAND_MVN_TO_SMPLH = {
    joint_name: child_mvn_name
    for joint_name, (_parent_mvn_name, child_mvn_name) in HAND_JOINT_MVN_PAIRS.items()
}

LEFT_THUMB_SWING_POSITION_PAIRS = {
    "left_thumb1": ("Left First Metacarpal", "Left First Proximal Phalange", "left_thumb2"),
    "left_thumb2": ("Left First Proximal Phalange", "Left First Distal Phalange", "left_thumb3"),
}

RIGHT_THUMB_SWING_POSITION_PAIRS = {
    key.replace("left_", "right_"): (
        start.replace("Left ", "Right "),
        end.replace("Left ", "Right "),
        child.replace("left_", "right_"),
    )
    for key, (start, end, child) in LEFT_THUMB_SWING_POSITION_PAIRS.items()
}

THUMB_SWING_POSITION_PAIRS = {
    **LEFT_THUMB_SWING_POSITION_PAIRS,
    **RIGHT_THUMB_SWING_POSITION_PAIRS,
}
THUMB_DISTAL_JOINTS = {"left_thumb3", "right_thumb3"}
THUMB_JOINTS = {
    "left_thumb1",
    "left_thumb2",
    "left_thumb3",
    "right_thumb1",
    "right_thumb2",
    "right_thumb3",
}

LEFT_FINGER_CHAIN_SEGMENTS = {
    "index": (
        "Left Second Proximal Phalange",
        "Left Second Middle Phalange",
        "Left Second Distal Phalange",
    ),
    "middle": (
        "Left Third Proximal Phalange",
        "Left Third Middle Phalange",
        "Left Third Distal Phalange",
    ),
    "pinky": (
        "Left Fifth Proximal Phalange",
        "Left Fifth Middle Phalange",
        "Left Fifth Distal Phalange",
    ),
    "ring": (
        "Left Fourth Proximal Phalange",
        "Left Fourth Middle Phalange",
        "Left Fourth Distal Phalange",
    ),
}

LEFT_FINGER_SWING_POSITION_PAIRS = {
    f"left_{finger}1": (proximal, middle)
    for finger, (proximal, middle, _distal) in LEFT_FINGER_CHAIN_SEGMENTS.items()
}
LEFT_FINGER_SWING_POSITION_PAIRS.update(
    {
        f"left_{finger}2": (middle, distal)
        for finger, (_proximal, middle, distal) in LEFT_FINGER_CHAIN_SEGMENTS.items()
    }
)

RIGHT_FINGER_SWING_POSITION_PAIRS = {
    key.replace("left_", "right_"): (
        start.replace("Left ", "Right "),
        end.replace("Left ", "Right "),
    )
    for key, (start, end) in LEFT_FINGER_SWING_POSITION_PAIRS.items()
}

FINGER_SWING_POSITION_PAIRS = {
    **LEFT_FINGER_SWING_POSITION_PAIRS,
    **RIGHT_FINGER_SWING_POSITION_PAIRS,
}

LEFT_FINGER_DISTAL_AXIS_SEGMENTS = {
    f"left_{finger}3": (distal, middle, distal)
    for finger, (_proximal, middle, distal) in LEFT_FINGER_CHAIN_SEGMENTS.items()
}
RIGHT_FINGER_DISTAL_AXIS_SEGMENTS = {
    key.replace("left_", "right_"): (
        distal.replace("Left ", "Right "),
        reference_start.replace("Left ", "Right "),
        reference_end.replace("Left ", "Right "),
    )
    for key, (distal, reference_start, reference_end) in LEFT_FINGER_DISTAL_AXIS_SEGMENTS.items()
}

FINGER_DISTAL_AXIS_SEGMENTS = {
    **LEFT_FINGER_DISTAL_AXIS_SEGMENTS,
    **RIGHT_FINGER_DISTAL_AXIS_SEGMENTS,
}


@dataclass
class SegmentPose:
    rotmat: np.ndarray
    pos_m: np.ndarray
    coordinate_system: str = ""


@dataclass
class FramePose:
    sample_counter: int
    time_code_ms: int
    body: Dict[str, SegmentPose] = field(default_factory=dict)
    left_finger: Dict[str, SegmentPose] = field(default_factory=dict)
    right_finger: Dict[str, SegmentPose] = field(default_factory=dict)


@dataclass
class MvnSequence:
    frames: List[FramePose]
    source_csv: Path


@dataclass(frozen=True)
class RetargetConfig:
    arm_correction: str
    arm_twist_filter: str


@dataclass
class HandRetargetCache:
    rest_axes: Dict[int, np.ndarray]
    distal_segment_axes: Dict[str, Tuple[int, float]] = field(default_factory=dict)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mvn-dir", type=Path, default=Path("output/xsens_mvn_udp/test04"))
    parser.add_argument("--output", type=Path, default=Path("output/xsens_mvn_udp/test04_smplh.pt"))
    parser.add_argument("--body-model", type=Path, default=DEFAULT_BODY_MODEL)
    parser.add_argument("--device", choices=["auto", "cpu", "cuda"], default="cuda")
    parser.add_argument("--max-frames", type=int, default=None)
    parser.add_argument("--frame-stride", type=int, default=1)
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
    return parser.parse_args()


def select_device(device_arg: str) -> torch.device:
    if device_arg == "auto":
        return torch.device("cuda" if torch.cuda.is_available() else "cpu")
    if device_arg == "cuda" and not torch.cuda.is_available():
        raise RuntimeError("CUDA was requested but is not available.")
    return torch.device(device_arg)


def matmul3(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    return np.sum(np.expand_dims(a, axis=-1) * np.expand_dims(b, axis=-3), axis=-2)


def matvec3(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    return np.sum(a * np.expand_dims(b, axis=-2), axis=-1)


def det3(rotmats: np.ndarray) -> np.ndarray:
    mats = np.asarray(rotmats)
    return (
        mats[..., 0, 0] * (mats[..., 1, 1] * mats[..., 2, 2] - mats[..., 1, 2] * mats[..., 2, 1])
        - mats[..., 0, 1] * (mats[..., 1, 0] * mats[..., 2, 2] - mats[..., 1, 2] * mats[..., 2, 0])
        + mats[..., 0, 2] * (mats[..., 1, 0] * mats[..., 2, 1] - mats[..., 1, 1] * mats[..., 2, 0])
    )


def quat_xyzw_to_rotmat(quats: np.ndarray) -> np.ndarray:
    quats = np.asarray(quats, dtype=np.float64)
    if quats.shape[-1] != 4:
        raise ValueError(f"Expected quaternion shape [...,4], got {quats.shape}")
    out_shape = quats.shape[:-1] + (3, 3)
    flat = quats.reshape(-1, 4).copy()
    norms = np.linalg.norm(flat, axis=1, keepdims=True)
    if np.any(~np.isfinite(norms)) or np.any(norms < 1e-8):
        raise ValueError("Invalid zero or non-finite quaternion")
    flat /= norms

    x = flat[:, 0]
    y = flat[:, 1]
    z = flat[:, 2]
    w = flat[:, 3]
    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z

    rotmats = np.empty((flat.shape[0], 3, 3), dtype=np.float64)
    rotmats[:, 0, 0] = 1.0 - 2.0 * (yy + zz)
    rotmats[:, 0, 1] = 2.0 * (xy - wz)
    rotmats[:, 0, 2] = 2.0 * (xz + wy)
    rotmats[:, 1, 0] = 2.0 * (xy + wz)
    rotmats[:, 1, 1] = 1.0 - 2.0 * (xx + zz)
    rotmats[:, 1, 2] = 2.0 * (yz - wx)
    rotmats[:, 2, 0] = 2.0 * (xz - wy)
    rotmats[:, 2, 1] = 2.0 * (yz + wx)
    rotmats[:, 2, 2] = 1.0 - 2.0 * (xx + yy)
    return rotmats.astype(np.float32).reshape(out_shape)


def rotmat_to_quat_xyzw(rotmats: np.ndarray) -> np.ndarray:
    rotmats = np.asarray(rotmats, dtype=np.float64)
    if rotmats.shape[-2:] != (3, 3):
        raise ValueError(f"Expected rotation matrix shape [...,3,3], got {rotmats.shape}")
    out_shape = rotmats.shape[:-2] + (4,)
    mats = rotmats.reshape(-1, 3, 3)
    quats = np.empty((mats.shape[0], 4), dtype=np.float64)

    m00 = mats[:, 0, 0]
    m01 = mats[:, 0, 1]
    m02 = mats[:, 0, 2]
    m10 = mats[:, 1, 0]
    m11 = mats[:, 1, 1]
    m12 = mats[:, 1, 2]
    m20 = mats[:, 2, 0]
    m21 = mats[:, 2, 1]
    m22 = mats[:, 2, 2]
    trace = m00 + m11 + m22

    mask = trace > 0.0
    s = np.sqrt(np.maximum(trace[mask] + 1.0, 0.0)) * 2.0
    quats[mask, 3] = 0.25 * s
    quats[mask, 0] = (m21[mask] - m12[mask]) / s
    quats[mask, 1] = (m02[mask] - m20[mask]) / s
    quats[mask, 2] = (m10[mask] - m01[mask]) / s

    mask_x = ~mask & (m00 > m11) & (m00 > m22)
    s = np.sqrt(np.maximum(1.0 + m00[mask_x] - m11[mask_x] - m22[mask_x], 0.0)) * 2.0
    quats[mask_x, 3] = (m21[mask_x] - m12[mask_x]) / s
    quats[mask_x, 0] = 0.25 * s
    quats[mask_x, 1] = (m01[mask_x] + m10[mask_x]) / s
    quats[mask_x, 2] = (m02[mask_x] + m20[mask_x]) / s

    mask_y = ~mask & ~mask_x & (m11 > m22)
    s = np.sqrt(np.maximum(1.0 + m11[mask_y] - m00[mask_y] - m22[mask_y], 0.0)) * 2.0
    quats[mask_y, 3] = (m02[mask_y] - m20[mask_y]) / s
    quats[mask_y, 0] = (m01[mask_y] + m10[mask_y]) / s
    quats[mask_y, 1] = 0.25 * s
    quats[mask_y, 2] = (m12[mask_y] + m21[mask_y]) / s

    mask_z = ~mask & ~mask_x & ~mask_y
    s = np.sqrt(np.maximum(1.0 + m22[mask_z] - m00[mask_z] - m11[mask_z], 0.0)) * 2.0
    quats[mask_z, 3] = (m10[mask_z] - m01[mask_z]) / s
    quats[mask_z, 0] = (m02[mask_z] + m20[mask_z]) / s
    quats[mask_z, 1] = (m12[mask_z] + m21[mask_z]) / s
    quats[mask_z, 2] = 0.25 * s

    norms = np.linalg.norm(quats, axis=1, keepdims=True)
    if np.any(~np.isfinite(norms)) or np.any(norms < 1e-8):
        raise ValueError("Could not convert rotation matrix to a valid quaternion")
    quats /= norms
    return quats.astype(np.float32).reshape(out_shape)


def rotvec_to_rotmat(rotvecs: np.ndarray) -> np.ndarray:
    rotvecs = np.asarray(rotvecs, dtype=np.float64)
    if rotvecs.shape[-1] != 3:
        raise ValueError(f"Expected rotvec shape [...,3], got {rotvecs.shape}")
    out_shape = rotvecs.shape[:-1] + (3, 3)
    flat = rotvecs.reshape(-1, 3)
    angles = np.linalg.norm(flat, axis=1)
    rotmats = np.tile(np.eye(3, dtype=np.float64), (flat.shape[0], 1, 1))
    valid = angles > 1e-8
    if np.any(valid):
        axes = flat[valid] / angles[valid, None]
        x = axes[:, 0]
        y = axes[:, 1]
        z = axes[:, 2]
        zeros = np.zeros_like(x)
        k = np.stack(
            [
                zeros,
                -z,
                y,
                z,
                zeros,
                -x,
                -y,
                x,
                zeros,
            ],
            axis=1,
        ).reshape(-1, 3, 3)
        sin_angle = np.sin(angles[valid])[:, None, None]
        cos_angle = np.cos(angles[valid])[:, None, None]
        eye = np.eye(3, dtype=np.float64)[None, :, :]
        rotmats[valid] = eye + sin_angle * k + (1.0 - cos_angle) * matmul3(k, k)
    return rotmats.astype(np.float32).reshape(out_shape)


def quat_wxyz_to_rotmat(row: Dict[str, str]) -> np.ndarray:
    quat_wxyz = np.array(
        [float(row["q_w"]), float(row["q_x"]), float(row["q_y"]), float(row["q_z"])],
        dtype=np.float64,
    )
    norm = np.linalg.norm(quat_wxyz)
    if not np.isfinite(norm) or norm < 1e-8:
        raise ValueError(f"Invalid quaternion at sample {row.get('sample_counter')}: {quat_wxyz}")
    quat_wxyz /= norm
    quat_xyzw = np.array([quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0]], dtype=np.float64)
    return quat_xyzw_to_rotmat(quat_xyzw)


def position_m_from_row(row: Dict[str, str]) -> np.ndarray:
    if row.get("x_m") and row.get("y_m") and row.get("z_m"):
        return np.array([float(row["x_m"]), float(row["y_m"]), float(row["z_m"])], dtype=np.float32)
    if row.get("x_cm") and row.get("y_cm") and row.get("z_cm"):
        return np.array([float(row["x_cm"]), float(row["y_cm"]), float(row["z_cm"])], dtype=np.float32) / 100.0
    raise ValueError(f"Missing position columns at sample {row.get('sample_counter')}")




def validate_coordinate_system(coordinate_system: str) -> None:
    if not coordinate_system:
        return
    if coordinate_system != "Z-Up right-handed":
        raise ValueError(
            f"Expected Xsens Z-Up right-handed rows, got {coordinate_system!r}"
        )


def convert_rotmat_coord(rotmat: np.ndarray, coordinate_system: str) -> np.ndarray:
    validate_coordinate_system(coordinate_system)
    rotmat = rotmat.astype(np.float32, copy=False)
    return matmul3(matmul3(XSENS_ZUP_TO_SMPL, rotmat), XSENS_ZUP_TO_SMPL.T).astype(np.float32)


def convert_position_coord(position: np.ndarray, coordinate_system: str) -> np.ndarray:
    validate_coordinate_system(coordinate_system)
    position = position.astype(np.float32, copy=False)
    return matvec3(XSENS_ZUP_TO_SMPL, position).astype(np.float32)


def apply_arm_axis_correction(mvn_name: str, rotmat: np.ndarray, arm_correction: str) -> np.ndarray:
    if arm_correction == "none":
        return rotmat
    if arm_correction != "xsens_ros2_plus90":
        raise ValueError(f"Unsupported arm correction mode: {arm_correction!r}")
    if mvn_name in LEFT_ARM_SEGMENTS:
        return matmul3(rotmat, LEFT_ARM_CORRECTION).astype(np.float32)
    if mvn_name in RIGHT_ARM_SEGMENTS:
        return matmul3(rotmat, RIGHT_ARM_CORRECTION).astype(np.float32)
    return rotmat


def segment_rotmat_for_smpl(pose: SegmentPose, mvn_name: str, arm_correction: str) -> np.ndarray:
    rotmat = convert_rotmat_coord(pose.rotmat, pose.coordinate_system)
    return apply_arm_axis_correction(mvn_name, rotmat, arm_correction)


def finger_segment_rotmat_for_smpl(pose: SegmentPose) -> np.ndarray:
    return convert_rotmat_coord(pose.rotmat, pose.coordinate_system)


def load_mvn_pose_sequence(
    mvn_dir: Path,
    frame_stride: int,
    max_frames: Optional[int],
) -> MvnSequence:
    csv_path = mvn_dir / "mvn_pose_segments.csv"
    if not csv_path.exists():
        raise FileNotFoundError(f"Missing MVN pose CSV: {csv_path}")
    if frame_stride < 1:
        raise ValueError("--frame-stride must be >= 1")

    frames_by_sample: "OrderedDict[int, FramePose]" = OrderedDict()
    with csv_path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        required = {
            "sample_counter",
            "time_code_ms",
            "item_kind",
            "item_name",
            "q_w",
            "q_x",
            "q_y",
            "q_z",
        }
        missing = required.difference(reader.fieldnames or [])
        if missing:
            raise ValueError(f"{csv_path} missing required columns: {sorted(missing)}")

        for row in reader:
            sample_counter = int(row["sample_counter"])
            frame = frames_by_sample.get(sample_counter)
            if frame is None:
                frame = FramePose(
                    sample_counter=sample_counter,
                    time_code_ms=int(float(row["time_code_ms"])),
                )
                frames_by_sample[sample_counter] = frame

            item_kind = row["item_kind"]
            item_name = row["item_name"]
            pose = SegmentPose(
                rotmat=quat_wxyz_to_rotmat(row),
                pos_m=position_m_from_row(row),
                coordinate_system=row.get("coordinate_system", ""),
            )
            if item_kind == "body":
                frame.body[item_name] = pose
            elif item_kind == "left_finger":
                frame.left_finger[item_name] = pose
            elif item_kind == "right_finger":
                frame.right_finger[item_name] = pose

    frames = list(frames_by_sample.values())
    frames = frames[::frame_stride]
    if max_frames is not None:
        frames = frames[:max_frames]
    if not frames:
        raise ValueError(f"No frames loaded from {csv_path}")

    validate_mvn_frames(frames)
    return MvnSequence(frames=frames, source_csv=csv_path)


def validate_mvn_frames(frames: Sequence[FramePose]) -> None:
    required_left_fingers = {
        name
        for joint_name, pair in HAND_JOINT_MVN_PAIRS.items()
        if joint_name.startswith("left_")
        for name in pair
    }
    required_right_fingers = {
        name
        for joint_name, pair in HAND_JOINT_MVN_PAIRS.items()
        if joint_name.startswith("right_")
        for name in pair
    }
    for frame in frames:
        missing_body = sorted(set(MVN_BODY_TO_SMPL24.values()).difference(frame.body))
        missing_left_fingers = sorted(required_left_fingers.difference(frame.left_finger))
        missing_right_fingers = sorted(required_right_fingers.difference(frame.right_finger))
        if len(frame.body) != 23:
            raise ValueError(
                f"Sample {frame.sample_counter} has {len(frame.body)} body segments, expected 23."
            )
        if missing_body:
            raise ValueError(
                f"Sample {frame.sample_counter} missing required MVN body segments: {missing_body}"
            )
        if missing_left_fingers:
            raise ValueError(
                f"Sample {frame.sample_counter} missing required left finger segments: {missing_left_fingers}"
            )
        if missing_right_fingers:
            raise ValueError(
                f"Sample {frame.sample_counter} missing required right finger segments: {missing_right_fingers}"
            )


def global_to_local_rotmats(global_rotmats: np.ndarray, parents: np.ndarray) -> np.ndarray:
    if global_rotmats.ndim != 4 or global_rotmats.shape[-2:] != (3, 3):
        raise ValueError(f"Expected [T,J,3,3], got {global_rotmats.shape}")
    local = np.zeros_like(global_rotmats, dtype=np.float32)
    local[:, 0] = global_rotmats[:, 0]
    for joint_idx in range(1, global_rotmats.shape[1]):
        parent_idx = int(parents[joint_idx])
        if parent_idx < 0:
            local[:, joint_idx] = global_rotmats[:, joint_idx]
        else:
            local[:, joint_idx] = matmul3(
                np.swapaxes(global_rotmats[:, parent_idx], -1, -2),
                global_rotmats[:, joint_idx],
            )
    return local


def local_to_global_rotmats(local_rotmats: np.ndarray, parents: np.ndarray) -> np.ndarray:
    if local_rotmats.ndim != 4 or local_rotmats.shape[-2:] != (3, 3):
        raise ValueError(f"Expected [T,J,3,3], got {local_rotmats.shape}")
    global_rotmats = np.zeros_like(local_rotmats, dtype=np.float32)
    global_rotmats[:, 0] = local_rotmats[:, 0]
    for joint_idx in range(1, local_rotmats.shape[1]):
        parent_idx = int(parents[joint_idx])
        if parent_idx < 0:
            global_rotmats[:, joint_idx] = local_rotmats[:, joint_idx]
        else:
            global_rotmats[:, joint_idx] = matmul3(
                global_rotmats[:, parent_idx],
                local_rotmats[:, joint_idx],
            )
    return global_rotmats


def rotation_angles_deg(rotmats: np.ndarray) -> np.ndarray:
    trace = np.trace(rotmats.reshape(-1, 3, 3), axis1=1, axis2=2)
    cos_angle = np.clip((trace - 1.0) * 0.5, -1.0, 1.0)
    return np.rad2deg(np.arccos(cos_angle)).reshape(rotmats.shape[:-2]).astype(np.float32)


def relative_rotation_error_deg(reference: np.ndarray, estimate: np.ndarray) -> np.ndarray:
    relative = matmul3(np.swapaxes(reference, -1, -2), estimate)
    return rotation_angles_deg(relative)


def summarize_angle_array(angles: np.ndarray) -> Dict[str, float]:
    if angles.size == 0:
        return {"median": 0.0, "p95": 0.0, "max": 0.0}
    return {
        "median": float(np.median(angles)),
        "p95": float(np.percentile(angles, 95.0)),
        "max": float(np.max(angles)),
    }


def summarize_joint_orientation_errors(
    reference_global: np.ndarray,
    estimate_global: np.ndarray,
    joint_indices: Sequence[int],
) -> Dict[str, Dict[str, float]]:
    summary: Dict[str, Dict[str, float]] = {}
    for joint_idx in joint_indices:
        joint_name = SMPLH_52_NAMES[joint_idx]
        errors = relative_rotation_error_deg(
            reference_global[:, joint_idx],
            estimate_global[:, joint_idx],
        )
        summary[joint_name] = summarize_angle_array(errors)
    return summary


ARM_DIAGNOSTIC_JOINTS = (
    LEFT_SHOULDER_IDX,
    LEFT_ELBOW_IDX,
    LEFT_WRIST_IDX,
    RIGHT_SHOULDER_IDX,
    RIGHT_ELBOW_IDX,
    RIGHT_WRIST_IDX,
)



def remove_twist_around_axis(rotmats: np.ndarray, axis: np.ndarray) -> np.ndarray:
    axis = axis.astype(np.float32, copy=False)
    axis_norm = float(np.linalg.norm(axis))
    if axis_norm < 1e-8:
        raise ValueError("Cannot remove twist around a zero-length axis")
    axis = axis / axis_norm

    flat_shape = rotmats.shape[:-2]
    flat_rotmats = rotmats.reshape(-1, 3, 3)
    quats = rotmat_to_quat_xyzw(flat_rotmats).reshape(-1, 4).astype(np.float64)
    quat_vec = quats[:, :3]
    quat_w = quats[:, 3:4]
    twist_vec = np.sum(quat_vec * axis[None, :], axis=1, keepdims=True) * axis[None, :]
    twist_quats = np.concatenate([twist_vec, quat_w], axis=1)
    twist_norms = np.linalg.norm(twist_quats, axis=1, keepdims=True)
    valid = twist_norms[:, 0] > 1e-8
    twist_quats[valid] /= twist_norms[valid]
    twist_quats[~valid] = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)

    twist_rotmats = quat_xyzw_to_rotmat(twist_quats).reshape(-1, 3, 3)
    swing = matmul3(flat_rotmats, np.swapaxes(twist_rotmats, -1, -2))
    return swing.astype(np.float32).reshape(flat_shape + (3, 3))


def clamp_twist_around_axis(rotmats: np.ndarray, axis: np.ndarray, max_angle_deg: float) -> np.ndarray:
    axis = axis.astype(np.float32, copy=False)
    axis_norm = float(np.linalg.norm(axis))
    if axis_norm < 1e-8:
        raise ValueError("Cannot clamp twist around a zero-length axis")
    axis = axis / axis_norm

    flat_shape = rotmats.shape[:-2]
    flat_rotmats = rotmats.reshape(-1, 3, 3)
    quats = rotmat_to_quat_xyzw(flat_rotmats).reshape(-1, 4).astype(np.float64)
    flip = quats[:, 3] < 0.0
    quats[flip] *= -1.0
    quat_vec = quats[:, :3]
    quat_w = quats[:, 3:4]
    signed_axis_part = np.sum(quat_vec * axis[None, :], axis=1)
    twist_vec = signed_axis_part[:, None] * axis[None, :]
    twist_quats = np.concatenate([twist_vec, quat_w], axis=1)
    twist_norms = np.linalg.norm(twist_quats, axis=1, keepdims=True)
    valid = twist_norms[:, 0] > 1e-8
    twist_quats[valid] /= twist_norms[valid]
    twist_quats[~valid] = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)

    signed_angles = 2.0 * np.arctan2(
        np.sum(twist_quats[:, :3] * axis[None, :], axis=1),
        twist_quats[:, 3],
    )
    signed_angles = (signed_angles + np.pi) % (2.0 * np.pi) - np.pi
    limit = np.deg2rad(max_angle_deg)
    clipped_angles = np.clip(signed_angles, -limit, limit)
    twist_rotmats = quat_xyzw_to_rotmat(twist_quats).reshape(-1, 3, 3)
    swing = matmul3(flat_rotmats, np.swapaxes(twist_rotmats, -1, -2))
    clipped_twist = rotvec_to_rotmat(clipped_angles[:, None] * axis[None, :]).reshape(-1, 3, 3)
    return matmul3(swing, clipped_twist).astype(np.float32).reshape(flat_shape + (3, 3))


def rest_bone_axis(rest_joints: np.ndarray, start_idx: int, end_idx: int) -> np.ndarray:
    if rest_joints.shape[0] <= max(start_idx, end_idx):
        raise ValueError(
            f"Expected rest joints to include indices {start_idx} and {end_idx}, got {rest_joints.shape[0]}"
        )
    axis = rest_joints[end_idx] - rest_joints[start_idx]
    axis_norm = float(np.linalg.norm(axis))
    if axis_norm < 1e-8:
        raise ValueError(f"Zero-length rest bone between joints {start_idx} and {end_idx}")
    return (axis / axis_norm).astype(np.float32)


def smplh_children(parents: np.ndarray) -> Dict[int, List[int]]:
    children: Dict[int, List[int]] = {}
    for idx, parent_idx in enumerate(parents):
        if int(parent_idx) >= 0:
            children.setdefault(int(parent_idx), []).append(idx)
    return children


def smplh_rest_axis_for_joint(
    joint_idx: int,
    parents: np.ndarray,
    children: Dict[int, List[int]],
    rest_joints: np.ndarray,
) -> np.ndarray:
    joint_children = children.get(joint_idx, [])
    if joint_children:
        return rest_bone_axis(rest_joints, joint_idx, joint_children[0])
    parent_idx = int(parents[joint_idx])
    if parent_idx < 0:
        raise ValueError(f"Joint {joint_idx} has no parent or child for rest bone axis")
    return rest_bone_axis(rest_joints, parent_idx, joint_idx)


def build_hand_retarget_cache(smplh_parents: np.ndarray, rest_joints: np.ndarray) -> HandRetargetCache:
    children = smplh_children(smplh_parents)
    rest_axes = {
        idx: smplh_rest_axis_for_joint(idx, smplh_parents, children, rest_joints)
        for idx in range(22, len(SMPLH_52_NAMES))
    }
    return HandRetargetCache(
        rest_axes=rest_axes,
    )


def normalize_vector(vector: np.ndarray, name: str) -> np.ndarray:
    vector = vector.astype(np.float32, copy=False)
    norm = float(np.linalg.norm(vector))
    if norm < 1e-8:
        raise ValueError(f"Cannot normalize zero-length vector for {name}")
    return (vector / norm).astype(np.float32)


def axis_angle_rotmat_rad(axis: np.ndarray, angle_rad: float) -> np.ndarray:
    axis = normalize_vector(axis, "axis-angle axis")
    x, y, z = axis.astype(np.float64)
    k = np.array(
        [
            [0.0, -z, y],
            [z, 0.0, -x],
            [-y, x, 0.0],
        ],
        dtype=np.float64,
    )
    sin_angle = np.sin(angle_rad)
    cos_angle = np.cos(angle_rad)
    return (np.eye(3, dtype=np.float64) + sin_angle * k + (1.0 - cos_angle) * matmul3(k, k)).astype(np.float32)


def rotation_between_vectors(source: np.ndarray, target: np.ndarray) -> np.ndarray:
    source = normalize_vector(source, "rotation source")
    target = normalize_vector(target, "rotation target")
    cross = np.cross(source, target).astype(np.float64)
    cross_norm = float(np.linalg.norm(cross))
    dot = float(np.clip(np.sum(source * target), -1.0, 1.0))
    if cross_norm < 1e-8:
        if dot > 0.0:
            return np.eye(3, dtype=np.float32)
        fallback = np.array([1.0, 0.0, 0.0], dtype=np.float32)
        if abs(float(np.sum(source * fallback))) > 0.9:
            fallback = np.array([0.0, 1.0, 0.0], dtype=np.float32)
        axis = normalize_vector(np.cross(source, fallback), "antiparallel rotation axis")
        return axis_angle_rotmat_rad(axis, np.pi)
    x, y, z = cross
    k = np.array(
        [
            [0.0, -z, y],
            [z, 0.0, -x],
            [-y, x, 0.0],
        ],
        dtype=np.float64,
    )
    return (
        np.eye(3, dtype=np.float64)
        + k
        + matmul3(k, k) * ((1.0 - dot) / (cross_norm * cross_norm))
    ).astype(np.float32)


def remove_arm_twist_keep_measured_wrists(
    global_rotmats: np.ndarray,
    local_rotmats: np.ndarray,
    parents: np.ndarray,
    rest_joints: np.ndarray,
) -> np.ndarray:
    adjusted = local_rotmats.copy()
    for shoulder_idx, elbow_idx, wrist_idx in (
        (LEFT_SHOULDER_IDX, LEFT_ELBOW_IDX, LEFT_WRIST_IDX),
        (RIGHT_SHOULDER_IDX, RIGHT_ELBOW_IDX, RIGHT_WRIST_IDX),
    ):
        parent_idx = int(parents[shoulder_idx])
        if parent_idx < 0:
            continue
        upper_arm_axis = rest_bone_axis(rest_joints, shoulder_idx, elbow_idx)
        shoulder_swing = remove_twist_around_axis(
            adjusted[:, shoulder_idx],
            upper_arm_axis,
        )
        adjusted[:, shoulder_idx] = shoulder_swing

        new_shoulder_global = matmul3(global_rotmats[:, parent_idx], shoulder_swing)
        elbow_target_local = matmul3(
            np.swapaxes(new_shoulder_global, -1, -2),
            global_rotmats[:, elbow_idx],
        )
        lower_arm_axis = rest_bone_axis(rest_joints, elbow_idx, wrist_idx)
        elbow_swing = remove_twist_around_axis(elbow_target_local, lower_arm_axis)
        adjusted[:, elbow_idx] = elbow_swing

        adjusted[:, wrist_idx] = clamp_twist_around_axis(
            local_rotmats[:, wrist_idx],
            lower_arm_axis,
            WRIST_TWIST_LIMIT_DEG,
        )
    return adjusted.astype(np.float32)


def remove_arm_twist_preserve_measured_hand_global(
    global_rotmats: np.ndarray,
    local_rotmats: np.ndarray,
    parents: np.ndarray,
    rest_joints: np.ndarray,
) -> np.ndarray:
    adjusted = local_rotmats.copy()
    for shoulder_idx, elbow_idx, wrist_idx in (
        (LEFT_SHOULDER_IDX, LEFT_ELBOW_IDX, LEFT_WRIST_IDX),
        (RIGHT_SHOULDER_IDX, RIGHT_ELBOW_IDX, RIGHT_WRIST_IDX),
    ):
        parent_idx = int(parents[shoulder_idx])
        if parent_idx < 0:
            continue
        upper_arm_axis = rest_bone_axis(rest_joints, shoulder_idx, elbow_idx)
        shoulder_swing = remove_twist_around_axis(
            adjusted[:, shoulder_idx],
            upper_arm_axis,
        )
        adjusted[:, shoulder_idx] = shoulder_swing

        new_shoulder_global = matmul3(global_rotmats[:, parent_idx], shoulder_swing)
        elbow_target_local = matmul3(
            np.swapaxes(new_shoulder_global, -1, -2),
            global_rotmats[:, elbow_idx],
        )
        lower_arm_axis = rest_bone_axis(rest_joints, elbow_idx, wrist_idx)
        elbow_swing = remove_twist_around_axis(elbow_target_local, lower_arm_axis)
        adjusted[:, elbow_idx] = elbow_swing

        new_elbow_global = matmul3(new_shoulder_global, elbow_swing)
        adjusted[:, wrist_idx] = matmul3(
            np.swapaxes(new_elbow_global, -1, -2),
            global_rotmats[:, wrist_idx],
        )
    return adjusted.astype(np.float32)


def matrix_to_axis_angle(rotmats: np.ndarray) -> np.ndarray:
    if rotmats.shape[-2:] != (3, 3):
        raise ValueError(f"Expected rotation matrix shape [...,3,3], got {rotmats.shape}")
    quats = rotmat_to_quat_xyzw(rotmats).reshape(-1, 4).astype(np.float64)
    flip = quats[:, 3] < 0.0
    quats[flip] *= -1.0
    quat_vec = quats[:, :3]
    quat_w = np.clip(quats[:, 3], -1.0, 1.0)
    sin_half = np.linalg.norm(quat_vec, axis=1)
    angles = 2.0 * np.arctan2(sin_half, quat_w)
    rotvecs = np.zeros_like(quat_vec)
    valid = sin_half > 1e-8
    rotvecs[valid] = quat_vec[valid] * (angles[valid] / sin_half[valid])[:, None]
    rotvecs[~valid] = 2.0 * quat_vec[~valid]
    return rotvecs.astype(np.float32).reshape(rotmats.shape[:-2] + (3,))


def read_smplh_parents(body_model_path: Path) -> np.ndarray:
    data = np.load(body_model_path, allow_pickle=True)
    if "kintree_table" not in data.files:
        raise ValueError(f"{body_model_path} does not contain kintree_table")
    kintree = np.asarray(data["kintree_table"], dtype=np.int64)
    if kintree.ndim != 2 or kintree.shape[0] != 2:
        raise ValueError(f"Expected kintree_table [2,J], got {kintree.shape}")
    parent_ids = kintree[0]
    child_ids = kintree[1]
    id_to_index = {int(joint_id): idx for idx, joint_id in enumerate(child_ids)}
    parents = np.full(child_ids.shape[0], -1, dtype=np.int64)
    for idx, parent_id in enumerate(parent_ids):
        if idx == 0:
            continue
        parent_idx = id_to_index.get(int(parent_id))
        if parent_idx is None and 0 <= int(parent_id) < child_ids.shape[0]:
            parent_idx = int(parent_id)
        if parent_idx is None:
            raise ValueError(f"Could not resolve SMPL-H parent id {int(parent_id)} for joint index {idx}")
        parents[idx] = parent_idx
    if parents.shape[0] < 52:
        raise ValueError(f"Expected SMPL-H kintree with at least 52 joints, got {parents.shape[0]}")
    return parents[:52]


def build_smpl24_global_rotmats(frames: Sequence[FramePose], arm_correction: str) -> np.ndarray:
    rotmats = np.zeros((len(frames), len(SMPL24_NAMES), 3, 3), dtype=np.float32)
    for frame_idx, frame in enumerate(frames):
        for joint_idx, joint_name in enumerate(SMPL24_NAMES):
            mvn_name = MVN_BODY_TO_SMPL24.get(joint_name)
            if mvn_name is None:
                parent_idx = int(SMPL24_PARENTS[joint_idx])
                rotmats[frame_idx, joint_idx] = (
                    np.eye(3, dtype=np.float32) if parent_idx < 0 else rotmats[frame_idx, parent_idx]
                )
                continue
            rotmats[frame_idx, joint_idx] = segment_rotmat_for_smpl(
                pose=frame.body[mvn_name],
                mvn_name=mvn_name,
                arm_correction=arm_correction,
            )
    return rotmats


def finger_source_for_joint(frame: FramePose, joint_name: str) -> Dict[str, SegmentPose]:
    if joint_name.startswith("left_"):
        return frame.left_finger
    if joint_name.startswith("right_"):
        return frame.right_finger
    raise ValueError(f"Expected hand joint name with side prefix, got {joint_name!r}")


def finger_local_rotmat_for_smpl(frame: FramePose, joint_name: str) -> np.ndarray:
    parent_mvn_name, child_mvn_name = HAND_JOINT_MVN_PAIRS[joint_name]
    source = finger_source_for_joint(frame, joint_name)
    parent_global = finger_segment_rotmat_for_smpl(source[parent_mvn_name])
    child_global = finger_segment_rotmat_for_smpl(source[child_mvn_name])
    return matmul3(parent_global.T, child_global).astype(np.float32)


def finger_positions_for_smpl(source: Dict[str, SegmentPose]) -> Dict[str, np.ndarray]:
    return {
        mvn_name: convert_position_coord(pose.pos_m, pose.coordinate_system)
        for mvn_name, pose in source.items()
    }


def thumb_swing_local_rotmat_for_smpl(
    joint_name: str,
    parent_global: np.ndarray,
    source_positions: Dict[str, np.ndarray],
    rest_direction: np.ndarray,
) -> np.ndarray:
    start_mvn_name, end_mvn_name, _child_joint_name = THUMB_SWING_POSITION_PAIRS[joint_name]
    target_global = normalize_vector(
        source_positions[end_mvn_name] - source_positions[start_mvn_name],
        f"{joint_name} MVN thumb position direction",
    )
    target_parent_local = matvec3(parent_global.T, target_global)
    return rotation_between_vectors(rest_direction, target_parent_local)


def finger_position_swing_local_rotmat_for_smpl(
    joint_name: str,
    parent_global: np.ndarray,
    source_positions: Dict[str, np.ndarray],
    rest_direction: np.ndarray,
) -> np.ndarray:
    start_mvn_name, end_mvn_name = FINGER_SWING_POSITION_PAIRS[joint_name]
    target_global = normalize_vector(
        source_positions[end_mvn_name] - source_positions[start_mvn_name],
        f"{joint_name} MVN position direction",
    )
    target_parent_local = matvec3(parent_global.T, target_global)
    return rotation_between_vectors(rest_direction, target_parent_local)


def stable_segment_long_axis_for_smpl(
    pose: SegmentPose,
    reference_direction: np.ndarray,
    axis_cache: Dict[str, Tuple[int, float]],
    cache_key: str,
) -> np.ndarray:
    rotmat = finger_segment_rotmat_for_smpl(pose)
    if cache_key not in axis_cache:
        reference = normalize_vector(reference_direction, "segment long-axis reference")
        best_axis_idx = 0
        best_sign = 1.0
        best_dot = -np.inf
        for axis_idx in range(3):
            axis = rotmat[:, axis_idx]
            for sign in (1.0, -1.0):
                dot = float(np.sum(sign * axis * reference))
                if dot > best_dot:
                    best_dot = dot
                    best_axis_idx = axis_idx
                    best_sign = sign
        axis_cache[cache_key] = (best_axis_idx, best_sign)

    axis_idx, sign = axis_cache[cache_key]
    return normalize_vector(sign * rotmat[:, axis_idx], "stable segment long axis")


def finger_distal_axis_local_rotmat_for_smpl(
    frame: FramePose,
    joint_name: str,
    parent_global: np.ndarray,
    source_positions: Dict[str, np.ndarray],
    rest_direction: np.ndarray,
    hand_cache: HandRetargetCache,
) -> np.ndarray:
    distal_mvn_name, reference_start_name, reference_end_name = FINGER_DISTAL_AXIS_SEGMENTS[joint_name]
    source = finger_source_for_joint(frame, joint_name)
    reference_direction = source_positions[reference_end_name] - source_positions[reference_start_name]
    target_global = stable_segment_long_axis_for_smpl(
        pose=source[distal_mvn_name],
        reference_direction=reference_direction,
        axis_cache=hand_cache.distal_segment_axes,
        cache_key=joint_name,
    )
    target_parent_local = matvec3(parent_global.T, target_global)
    return rotation_between_vectors(rest_direction, target_parent_local)


def thumb_distal_local_rotmat_for_smpl(
    frame: FramePose,
    joint_name: str,
    parent_global: np.ndarray,
    source_positions: Dict[str, np.ndarray],
    rest_direction: np.ndarray,
    hand_cache: HandRetargetCache,
) -> np.ndarray:
    start_mvn_name, end_mvn_name = HAND_JOINT_MVN_PAIRS[joint_name]
    source = finger_source_for_joint(frame, joint_name)
    incoming_direction = source_positions[end_mvn_name] - source_positions[start_mvn_name]
    target_global = stable_segment_long_axis_for_smpl(
        pose=source[end_mvn_name],
        reference_direction=incoming_direction,
        axis_cache=hand_cache.distal_segment_axes,
        cache_key=joint_name,
    )
    target_parent_local = matvec3(parent_global.T, target_global)
    return rotation_between_vectors(rest_direction, target_parent_local)


def hand_local_rotmat_for_smpl(
    frame: FramePose,
    joint_name: str,
    parent_global: np.ndarray,
    source_positions: Dict[str, np.ndarray],
    rest_direction: np.ndarray,
    hand_cache: HandRetargetCache,
) -> np.ndarray:
    if joint_name in THUMB_SWING_POSITION_PAIRS:
        return thumb_swing_local_rotmat_for_smpl(
            joint_name=joint_name,
            parent_global=parent_global,
            source_positions=source_positions,
            rest_direction=rest_direction,
        )
    if joint_name in THUMB_DISTAL_JOINTS:
        return thumb_distal_local_rotmat_for_smpl(
            frame=frame,
            joint_name=joint_name,
            parent_global=parent_global,
            source_positions=source_positions,
            rest_direction=rest_direction,
            hand_cache=hand_cache,
        )
    if joint_name in THUMB_JOINTS:
        return finger_local_rotmat_for_smpl(frame, joint_name)
    if joint_name in FINGER_DISTAL_AXIS_SEGMENTS:
        return finger_distal_axis_local_rotmat_for_smpl(
            frame=frame,
            joint_name=joint_name,
            parent_global=parent_global,
            source_positions=source_positions,
            rest_direction=rest_direction,
            hand_cache=hand_cache,
        )
    return finger_position_swing_local_rotmat_for_smpl(
        joint_name=joint_name,
        parent_global=parent_global,
        source_positions=source_positions,
        rest_direction=rest_direction,
    )


def build_smplh_global_rotmats(
    frames: Sequence[FramePose],
    smpl24_global: np.ndarray,
    smplh_parents: np.ndarray,
    rest_joints: np.ndarray,
    hand_cache: Optional[HandRetargetCache] = None,
) -> np.ndarray:
    full = np.tile(np.eye(3, dtype=np.float32), (len(frames), 52, 1, 1))
    full[:, :22] = smpl24_global[:, :22]
    if hand_cache is None:
        hand_cache = build_hand_retarget_cache(smplh_parents, rest_joints)
    for frame_idx, frame in enumerate(frames):
        finger_positions = {
            "left": finger_positions_for_smpl(frame.left_finger),
            "right": finger_positions_for_smpl(frame.right_finger),
        }
        for joint_idx in range(22, full.shape[1]):
            joint_name = SMPLH_52_NAMES[joint_idx]
            parent_idx = int(smplh_parents[joint_idx])
            if joint_name not in HAND_JOINT_MVN_PAIRS or parent_idx < 0:
                full[frame_idx, joint_idx] = (
                    np.eye(3, dtype=np.float32) if parent_idx < 0 else full[frame_idx, parent_idx]
                )
                continue
            side = "left" if joint_name.startswith("left_") else "right"
            local = hand_local_rotmat_for_smpl(
                frame=frame,
                joint_name=joint_name,
                parent_global=full[frame_idx, parent_idx],
                source_positions=finger_positions[side],
                rest_direction=hand_cache.rest_axes[joint_idx],
                hand_cache=hand_cache,
            )
            full[frame_idx, joint_idx] = matmul3(full[frame_idx, parent_idx], local).astype(np.float32)
    return full


def compute_trans_from_pelvis(
    frames: Sequence[FramePose],
    root_orient: np.ndarray,
    rest_root: np.ndarray,
) -> np.ndarray:
    pelvis_pos = np.stack(
        [
            convert_position_coord(
                frame.body["Pelvis"].pos_m,
                frame.body["Pelvis"].coordinate_system,
            )
            for frame in frames
        ],
        axis=0,
    ).astype(np.float32)
    root_rotmats = rotvec_to_rotmat(root_orient)
    rotated_rest_root = matvec3(root_rotmats, rest_root.astype(np.float32)).astype(np.float32)
    return (pelvis_pos - rotated_rest_root).astype(np.float32)


def instantiate_body_model(BodyModel, body_model_path: Path, num_betas: int):
    patch_human_body_prior_lbs_dtype_compat()
    model_path = str(body_model_path)
    model_type = "smplh"
    attempts = [
        lambda: BodyModel(bm_fname=model_path, num_betas=num_betas),
        lambda: BodyModel(model_path=model_path, num_betas=num_betas),
        lambda: BodyModel(model_fname=model_path, num_betas=num_betas),
        lambda: BodyModel(model_path, num_betas=num_betas),
        lambda: BodyModel(model_path, model_type=model_type, num_betas=num_betas),
        lambda: BodyModel(model_path, model_type, num_betas=num_betas),
        lambda: BodyModel(bm_fname=model_path),
        lambda: BodyModel(model_path=model_path),
        lambda: BodyModel(model_fname=model_path),
        lambda: BodyModel(model_path, model_type=model_type),
        lambda: BodyModel(model_path, model_type),
        lambda: BodyModel(model_path),
    ]
    errors: List[str] = []
    for build in attempts:
        try:
            return build()
        except TypeError as exc:
            errors.append(str(exc))
    raise TypeError(
        "Could not construct human_body_prior BodyModel with this installation. "
        "Tried bm_fname/model_path/model_fname/positional+model_type variants. Last errors: "
        + " | ".join(errors[-3:])
    )


def patch_human_body_prior_lbs_dtype_compat() -> None:
    """Allow human_body_prior 0.8.x to run with smplx.lbs variants lacking dtype."""
    try:
        import human_body_prior.body_model.body_model as body_model_module
    except ImportError:
        return

    lbs_fn = getattr(body_model_module, "lbs", None)
    if lbs_fn is None:
        return
    try:
        signature = inspect.signature(lbs_fn)
    except (TypeError, ValueError):
        return
    if "dtype" in signature.parameters or getattr(lbs_fn, "_accepts_ignored_dtype", False):
        return

    @functools.wraps(lbs_fn)
    def lbs_accepting_ignored_dtype(*args, dtype=None, **kwargs):
        return lbs_fn(*args, **kwargs)

    lbs_accepting_ignored_dtype._accepts_ignored_dtype = True
    body_model_module.lbs = lbs_accepting_ignored_dtype


def create_body_model(body_model_path: Path, num_betas: int, device: torch.device):
    from human_body_prior.body_model.body_model import BodyModel

    if not body_model_path.exists():
        raise FileNotFoundError(f"SMPL-H body model not found: {body_model_path}")
    model = instantiate_body_model(BodyModel, body_model_path, num_betas)
    model = model.to(device)
    model.eval()
    return model


def call_body_model(
    body_model,
    *,
    pose_body: torch.Tensor,
    pose_hand: torch.Tensor,
    betas: torch.Tensor,
    root_orient: torch.Tensor,
    trans: torch.Tensor,
):
    attempts = [
        lambda: body_model(
            pose_body=pose_body,
            pose_hand=pose_hand,
            betas=betas,
            root_orient=root_orient,
            trans=trans,
        ),
        lambda: body_model(
            body_pose=pose_body,
            left_hand_pose=pose_hand[:, :45],
            right_hand_pose=pose_hand[:, 45:90],
            betas=betas,
            global_orient=root_orient,
            transl=trans,
            return_verts=True,
        ),
    ]
    errors: List[str] = []
    for build in attempts:
        try:
            return build()
        except TypeError as exc:
            errors.append(str(exc))
    raise TypeError(
        "Could not call BodyModel forward with human_body_prior or SMPLX-style arguments. "
        "Last errors: "
        + " | ".join(errors[-2:])
    )


def body_model_output_tensor(output, *names: str) -> torch.Tensor:
    for name in names:
        if hasattr(output, name):
            value = getattr(output, name)
            if value is not None:
                return value
        if isinstance(output, dict) and output.get(name) is not None:
            return output[name]
    raise AttributeError(f"BodyModel output is missing expected tensor field(s): {', '.join(names)}")


def body_model_rest_root(body_model, num_betas: int, device: torch.device) -> np.ndarray:
    return body_model_rest_joints(body_model, num_betas, device)[0]


def body_model_rest_joints(body_model, num_betas: int, device: torch.device) -> np.ndarray:
    with torch.no_grad():
        out = call_body_model(
            body_model,
            pose_body=torch.zeros(1, 63, dtype=torch.float32, device=device),
            pose_hand=torch.zeros(1, 90, dtype=torch.float32, device=device),
            betas=torch.zeros(1, num_betas, dtype=torch.float32, device=device),
            root_orient=torch.zeros(1, 3, dtype=torch.float32, device=device),
            trans=torch.zeros(1, 3, dtype=torch.float32, device=device),
        )
    return body_model_output_tensor(out, "Jtr", "joints")[0].detach().cpu().numpy().astype(np.float32)


def estimate_fps(time_code_ms: np.ndarray) -> float:
    if len(time_code_ms) < 2:
        return 0.0
    diffs = np.diff(time_code_ms.astype(np.float64)) / 1000.0
    diffs = diffs[(diffs > 0) & (diffs < 1.0)]
    if diffs.size == 0:
        return 0.0
    return float(1.0 / np.median(diffs))


def check_rotation_orthogonality(name: str, rotmats: np.ndarray, atol: float = 1e-3) -> None:
    eye = np.eye(3, dtype=np.float32)
    err = np.max(np.abs(matmul3(np.swapaxes(rotmats, -1, -2), rotmats) - eye))
    det_values = det3(rotmats.reshape(-1, 3, 3))
    det_min = float(np.min(det_values))
    det_max = float(np.max(det_values))
    if err > atol or det_min < 1.0 - atol or det_max > 1.0 + atol:
        raise ValueError(
            f"{name} rotation check failed: orthogonality max error={err:.6g}, "
            f"det range=({det_min:.6g}, {det_max:.6g})"
        )




def retarget_sequence(
    sequence: MvnSequence,
    body_model_path: Path,
    num_betas: int,
    device: torch.device,
    config: RetargetConfig,
) -> Dict[str, object]:
    frames = sequence.frames
    body_model = create_body_model(body_model_path, num_betas, device)
    rest_joints = body_model_rest_joints(body_model, num_betas, device)
    smplh_parents = read_smplh_parents(body_model_path)
    hand_cache = build_hand_retarget_cache(smplh_parents, rest_joints)
    smpl24_global = build_smpl24_global_rotmats(frames, arm_correction=config.arm_correction)
    smpl24_local = global_to_local_rotmats(smpl24_global, SMPL24_PARENTS)
    smplh_global = build_smplh_global_rotmats(
        frames=frames,
        smpl24_global=smpl24_global,
        smplh_parents=smplh_parents,
        rest_joints=rest_joints,
        hand_cache=hand_cache,
    )
    smplh_local_raw = global_to_local_rotmats(smplh_global, smplh_parents)
    if config.arm_twist_filter == "none":
        smplh_local_after_arm_filter = smplh_local_raw.copy()
    elif config.arm_twist_filter == "preserve_hand_global":
        smplh_local_after_arm_filter = remove_arm_twist_preserve_measured_hand_global(
            global_rotmats=smplh_global,
            local_rotmats=smplh_local_raw,
            parents=smplh_parents,
            rest_joints=rest_joints,
        )
    elif config.arm_twist_filter == "keep_measured_wrist":
        smplh_local_after_arm_filter = remove_arm_twist_keep_measured_wrists(
            global_rotmats=smplh_global,
            local_rotmats=smplh_local_raw,
            parents=smplh_parents,
            rest_joints=rest_joints,
        )
    else:
        raise ValueError(f"Unsupported arm twist filter mode: {config.arm_twist_filter!r}")
    arm_twist_filter = config.arm_twist_filter
    smplh_local = smplh_local_after_arm_filter
    smplh_global_reconstructed = local_to_global_rotmats(smplh_local, smplh_parents)
    global_orientation_error_deg = summarize_joint_orientation_errors(
        reference_global=smplh_global,
        estimate_global=smplh_global_reconstructed,
        joint_indices=ARM_DIAGNOSTIC_JOINTS,
    )

    check_rotation_orthogonality("smpl24_global", smpl24_global)
    check_rotation_orthogonality("smplh_global", smplh_global)
    check_rotation_orthogonality("smplh_local", smplh_local)
    check_rotation_orthogonality("smplh_global_reconstructed", smplh_global_reconstructed)

    root_orient = matrix_to_axis_angle(smplh_global_reconstructed[:, 0])
    local_axis = matrix_to_axis_angle(smplh_local)
    pose_body = local_axis[:, 1:22].reshape(len(frames), 63).astype(np.float32)
    pose_hand = local_axis[:, 22:52].reshape(len(frames), 90).astype(np.float32)
    rest_root = rest_joints[0]
    trans = compute_trans_from_pelvis(frames, root_orient, rest_root)
    pose = np.concatenate([root_orient, pose_body, pose_hand], axis=1).astype(np.float32)

    sample_counter = np.array([frame.sample_counter for frame in frames], dtype=np.int64)
    time_code_ms = np.array([frame.time_code_ms for frame in frames], dtype=np.int64)
    betas = np.zeros((len(frames), num_betas), dtype=np.float32)
    fps = estimate_fps(time_code_ms)

    return {
        "pose": torch.from_numpy(pose),
        "trans": torch.from_numpy(trans),
        "root_orient": torch.from_numpy(root_orient.astype(np.float32)),
        "pose_body": torch.from_numpy(pose_body),
        "pose_hand": torch.from_numpy(pose_hand),
        "betas": torch.from_numpy(betas),
        "fps": fps,
        "sample_counter": torch.from_numpy(sample_counter),
        "time_code_ms": torch.from_numpy(time_code_ms),
        "source_mvn_dir": str(sequence.source_csv.parent),
        "source_pose_csv": str(sequence.source_csv),
        "retarget_mode": "xsens_global_segment_to_smplh_v5",
        "coord_mode": "xsens_zup_to_smpl",
        "body_mapping_profile": "mvnx_to_smpl",
        "hand_mode": "xsens_shifted_position_swing_fingers_and_thumb_stable_distal_axis",
        "thumb_retarget_mode": "xsens_thumb_position_swing_stable_distal_axis",
        "arm_correction": config.arm_correction,
        "arm_twist_filter": arm_twist_filter,
        "wrist_twist_limit_deg": float(WRIST_TWIST_LIMIT_DEG),
        "global_orientation_error_deg": global_orientation_error_deg,
        "shoulder_twist_filter": "superseded_by_arm_twist_filter",
        "body_model_path": str(body_model_path),
        "smpl24_global_rotmats": torch.from_numpy(smpl24_global),
        "smpl24_local_rotmats": torch.from_numpy(smpl24_local),
        "smplh_global_rotmats": torch.from_numpy(smplh_global),
        "smplh_global_rotmats_reconstructed": torch.from_numpy(smplh_global_reconstructed),
        "smplh_local_rotmats_before_arm_twist_filter": torch.from_numpy(smplh_local_raw),
        "smplh_local_rotmats_before_shoulder_twist_filter": torch.from_numpy(smplh_local_raw),
        "smplh_local_rotmats_after_arm_filter": torch.from_numpy(smplh_local_after_arm_filter),
        "smplh_local_rotmats": torch.from_numpy(smplh_local),
        "smpl24_names": SMPL24_NAMES,
        "smplh_52_names": SMPLH_52_NAMES,
        "smplh_parents": torch.from_numpy(smplh_parents.astype(np.int64)),
        "mvn_segment_names": {
            "body": sorted(frames[0].body.keys()),
            "left_finger": sorted(frames[0].left_finger.keys()),
            "right_finger": sorted(frames[0].right_finger.keys()),
        },
        "body_mapping": dict(MVN_BODY_TO_SMPL24),
        "hand_mapping": dict(HAND_MVN_TO_SMPLH),
        "hand_joint_mvn_pairs": dict(HAND_JOINT_MVN_PAIRS),
        "finger_position_swing_pairs": dict(FINGER_SWING_POSITION_PAIRS),
        "finger_distal_axis_segments": dict(FINGER_DISTAL_AXIS_SEGMENTS),
        "finger_distal_axis_mode": "first_frame_locked_segment_local_axis",
        "thumb_swing_position_pairs": dict(THUMB_SWING_POSITION_PAIRS),
        "thumb_distal_axis_mode": "first_frame_locked_segment_local_axis",
        "notes": (
            "Xsens global segment orientations are converted into the SMPL frame before "
            "global-to-local retargeting. SMPL-H proximal and middle finger pose is "
            "driven by shifted MVN position-chain swing directions, while distal "
            "finger joints use first-frame locked distal segment local axes because "
            "MVN has no fingertip position target. Thumb CMC/MCP use position-chain swing and the distal "
            "thumb joint uses the first-frame locked distal segment local axis. "
            "By default, arm/hand body segments keep raw Xsens segment frames with no +90 degree "
            "post-rotation and no arm twist filtering."
        ),
    }


def save_pt(data: Dict[str, object], output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    torch.save(data, output)


def print_summary(data: Dict[str, object], output: Path) -> None:
    pose = data["pose"]
    trans = data["trans"]
    pose_hand = data["pose_hand"]
    sample_counter = data["sample_counter"]
    print(f"[SAVE] {output}")
    print(f"[DATA] frames={pose.shape[0]} fps~={data['fps']:.3f}")
    print(f"[DATA] pose={tuple(pose.shape)} trans={tuple(trans.shape)} pose_hand={tuple(pose_hand.shape)}")
    print(f"[DATA] sample_counter={int(sample_counter[0])}..{int(sample_counter[-1])}")
    print(
        "[MODE] retarget={mode} coord={coord} mapping={mapping} hand={hand} arm={arm} twist={twist}".format(
            mode=data.get("retarget_mode", ""),
            coord=data.get("coord_mode", ""),
            mapping=data.get("body_mapping_profile", ""),
            hand=data.get("hand_mode", ""),
            arm=data.get("arm_correction", ""),
            twist=data.get("arm_twist_filter", ""),
        )
    )
    print(
        "[NOTE] Fixed path: Xsens Z-up -> SMPL, MVNX body mapping, MVN position-chain "
        "shifted finger swing, stable thumb/finger distal axes, and raw arm/hand body segment frames by default."
    )


def main() -> None:
    args = parse_args()
    device = select_device(args.device)
    config = RetargetConfig(
        arm_correction=args.arm_correction,
        arm_twist_filter=args.arm_twist_filter,
    )
    sequence = load_mvn_pose_sequence(
        mvn_dir=args.mvn_dir,
        frame_stride=args.frame_stride,
        max_frames=args.max_frames,
    )
    data = retarget_sequence(
        sequence=sequence,
        body_model_path=args.body_model,
        num_betas=args.num_betas,
        device=device,
        config=config,
    )
    save_pt(data, args.output)
    print_summary(data, args.output)


if __name__ == "__main__":
    main()
