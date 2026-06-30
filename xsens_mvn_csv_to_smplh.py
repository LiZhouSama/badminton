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
from collections import OrderedDict
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
import torch
from scipy.spatial.transform import Rotation


DEFAULT_BODY_MODEL = Path("/mnt/d/a_WORK/Projects/PhD/datasets/smpl_models/smplh/neutral/model.npz")

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
LEFT_ARM_CORRECTION = Rotation.from_euler("x", 90.0, degrees=True).as_matrix().astype(np.float32)
RIGHT_ARM_CORRECTION = Rotation.from_euler("x", 90.0, degrees=True).as_matrix().astype(np.float32)
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
    return Rotation.from_quat(quat_xyzw).as_matrix().astype(np.float32)


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
    return (XSENS_ZUP_TO_SMPL @ rotmat @ XSENS_ZUP_TO_SMPL.T).astype(np.float32)


def convert_position_coord(position: np.ndarray, coordinate_system: str) -> np.ndarray:
    validate_coordinate_system(coordinate_system)
    position = position.astype(np.float32, copy=False)
    return (XSENS_ZUP_TO_SMPL @ position).astype(np.float32)


def apply_arm_axis_correction(mvn_name: str, rotmat: np.ndarray, arm_correction: str) -> np.ndarray:
    if arm_correction == "none":
        return rotmat
    if arm_correction != "xsens_ros2_plus90":
        raise ValueError(f"Unsupported arm correction mode: {arm_correction!r}")
    if mvn_name in LEFT_ARM_SEGMENTS:
        return (rotmat @ LEFT_ARM_CORRECTION).astype(np.float32)
    if mvn_name in RIGHT_ARM_SEGMENTS:
        return (rotmat @ RIGHT_ARM_CORRECTION).astype(np.float32)
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
            local[:, joint_idx] = np.matmul(
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
            global_rotmats[:, joint_idx] = np.matmul(
                global_rotmats[:, parent_idx],
                local_rotmats[:, joint_idx],
            )
    return global_rotmats


def rotation_angles_deg(rotmats: np.ndarray) -> np.ndarray:
    trace = np.trace(rotmats.reshape(-1, 3, 3), axis1=1, axis2=2)
    cos_angle = np.clip((trace - 1.0) * 0.5, -1.0, 1.0)
    return np.rad2deg(np.arccos(cos_angle)).reshape(rotmats.shape[:-2]).astype(np.float32)


def relative_rotation_error_deg(reference: np.ndarray, estimate: np.ndarray) -> np.ndarray:
    relative = np.matmul(np.swapaxes(reference, -1, -2), estimate)
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
    quats = Rotation.from_matrix(rotmats.reshape(-1, 3, 3)).as_quat()
    quat_vec = quats[:, :3]
    quat_w = quats[:, 3:4]
    twist_vec = np.sum(quat_vec * axis[None, :], axis=1, keepdims=True) * axis[None, :]
    twist_quats = np.concatenate([twist_vec, quat_w], axis=1)
    twist_norms = np.linalg.norm(twist_quats, axis=1, keepdims=True)
    valid = twist_norms[:, 0] > 1e-8
    twist_quats[valid] /= twist_norms[valid]
    twist_quats[~valid] = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)

    swing = Rotation.from_quat(quats) * Rotation.from_quat(twist_quats).inv()
    return swing.as_matrix().astype(np.float32).reshape(flat_shape + (3, 3))


def clamp_twist_around_axis(rotmats: np.ndarray, axis: np.ndarray, max_angle_deg: float) -> np.ndarray:
    axis = axis.astype(np.float32, copy=False)
    axis_norm = float(np.linalg.norm(axis))
    if axis_norm < 1e-8:
        raise ValueError("Cannot clamp twist around a zero-length axis")
    axis = axis / axis_norm

    flat_shape = rotmats.shape[:-2]
    quats = Rotation.from_matrix(rotmats.reshape(-1, 3, 3)).as_quat()
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

    swing = Rotation.from_quat(quats) * Rotation.from_quat(twist_quats).inv()
    signed_angles = 2.0 * np.arctan2(
        np.sum(twist_quats[:, :3] * axis[None, :], axis=1),
        twist_quats[:, 3],
    )
    signed_angles = (signed_angles + np.pi) % (2.0 * np.pi) - np.pi
    limit = np.deg2rad(max_angle_deg)
    clipped_angles = np.clip(signed_angles, -limit, limit)
    clipped_twist = Rotation.from_rotvec(clipped_angles[:, None] * axis[None, :])
    return (swing * clipped_twist).as_matrix().astype(np.float32).reshape(flat_shape + (3, 3))


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


def normalize_vector(vector: np.ndarray, name: str) -> np.ndarray:
    vector = vector.astype(np.float32, copy=False)
    norm = float(np.linalg.norm(vector))
    if norm < 1e-8:
        raise ValueError(f"Cannot normalize zero-length vector for {name}")
    return (vector / norm).astype(np.float32)


def rotation_between_vectors(source: np.ndarray, target: np.ndarray) -> np.ndarray:
    source = normalize_vector(source, "rotation source")
    target = normalize_vector(target, "rotation target")
    axis = np.cross(source, target)
    axis_norm = float(np.linalg.norm(axis))
    dot = float(np.clip(np.dot(source, target), -1.0, 1.0))
    if axis_norm < 1e-8:
        if dot > 0.0:
            return np.eye(3, dtype=np.float32)
        fallback = np.array([1.0, 0.0, 0.0], dtype=np.float32)
        if abs(float(np.dot(source, fallback))) > 0.9:
            fallback = np.array([0.0, 1.0, 0.0], dtype=np.float32)
        axis = normalize_vector(np.cross(source, fallback), "antiparallel rotation axis")
        return Rotation.from_rotvec(axis * np.pi).as_matrix().astype(np.float32)
    axis = axis / axis_norm
    angle = np.arccos(dot)
    return Rotation.from_rotvec(axis * angle).as_matrix().astype(np.float32)


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

        new_shoulder_global = np.matmul(global_rotmats[:, parent_idx], shoulder_swing)
        elbow_target_local = np.matmul(
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

        new_shoulder_global = np.matmul(global_rotmats[:, parent_idx], shoulder_swing)
        elbow_target_local = np.matmul(
            np.swapaxes(new_shoulder_global, -1, -2),
            global_rotmats[:, elbow_idx],
        )
        lower_arm_axis = rest_bone_axis(rest_joints, elbow_idx, wrist_idx)
        elbow_swing = remove_twist_around_axis(elbow_target_local, lower_arm_axis)
        adjusted[:, elbow_idx] = elbow_swing

        new_elbow_global = np.matmul(new_shoulder_global, elbow_swing)
        adjusted[:, wrist_idx] = np.matmul(
            np.swapaxes(new_elbow_global, -1, -2),
            global_rotmats[:, wrist_idx],
        )
    return adjusted.astype(np.float32)


def matrix_to_axis_angle(rotmats: np.ndarray) -> np.ndarray:
    return Rotation.from_matrix(rotmats.reshape(-1, 3, 3)).as_rotvec().astype(np.float32).reshape(
        rotmats.shape[:-2] + (3,)
    )


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
    return (parent_global.T @ child_global).astype(np.float32)


def finger_position_for_smpl(source: Dict[str, SegmentPose], mvn_name: str) -> np.ndarray:
    pose = source[mvn_name]
    return convert_position_coord(pose.pos_m, pose.coordinate_system)


def thumb_swing_local_rotmat_for_smpl(
    frame: FramePose,
    joint_name: str,
    parent_global: np.ndarray,
    rest_joints: np.ndarray,
) -> np.ndarray:
    start_mvn_name, end_mvn_name, child_joint_name = THUMB_SWING_POSITION_PAIRS[joint_name]
    source = finger_source_for_joint(frame, joint_name)
    target_global = normalize_vector(
        finger_position_for_smpl(source, end_mvn_name) - finger_position_for_smpl(source, start_mvn_name),
        f"{joint_name} MANUS position direction",
    )
    joint_idx = SMPLH_52_NAMES.index(joint_name)
    child_idx = SMPLH_52_NAMES.index(child_joint_name)
    rest_direction = rest_bone_axis(rest_joints, joint_idx, child_idx)
    target_parent_local = parent_global.T @ target_global
    return rotation_between_vectors(rest_direction, target_parent_local)


def thumb_distal_local_rotmat_for_smpl(
    frame: FramePose,
    joint_name: str,
    rest_joints: np.ndarray,
) -> np.ndarray:
    local = finger_local_rotmat_for_smpl(frame, joint_name)
    joint_idx = SMPLH_52_NAMES.index(joint_name)
    parent_joint_name = "left_thumb2" if joint_name.startswith("left_") else "right_thumb2"
    parent_idx = SMPLH_52_NAMES.index(parent_joint_name)
    incoming_axis = rest_bone_axis(rest_joints, parent_idx, joint_idx)
    return remove_twist_around_axis(local[None, :, :], incoming_axis)[0]


def build_smplh_global_rotmats(
    frames: Sequence[FramePose],
    smpl24_global: np.ndarray,
    smplh_parents: np.ndarray,
    rest_joints: np.ndarray,
) -> np.ndarray:
    full = np.tile(np.eye(3, dtype=np.float32), (len(frames), 52, 1, 1))
    full[:, :22] = smpl24_global[:, :22]
    for frame_idx, frame in enumerate(frames):
        for joint_idx in range(22, full.shape[1]):
            joint_name = SMPLH_52_NAMES[joint_idx]
            parent_idx = int(smplh_parents[joint_idx])
            if joint_name not in HAND_JOINT_MVN_PAIRS or parent_idx < 0:
                full[frame_idx, joint_idx] = (
                    np.eye(3, dtype=np.float32) if parent_idx < 0 else full[frame_idx, parent_idx]
                )
                continue
            if joint_name in THUMB_SWING_POSITION_PAIRS:
                local = thumb_swing_local_rotmat_for_smpl(
                    frame=frame,
                    joint_name=joint_name,
                    parent_global=full[frame_idx, parent_idx],
                    rest_joints=rest_joints,
                )
            elif joint_name in THUMB_DISTAL_JOINTS:
                local = thumb_distal_local_rotmat_for_smpl(
                    frame=frame,
                    joint_name=joint_name,
                    rest_joints=rest_joints,
                )
            else:
                local = finger_local_rotmat_for_smpl(frame, joint_name)
            full[frame_idx, joint_idx] = (full[frame_idx, parent_idx] @ local).astype(np.float32)
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
    rotated_rest_root = Rotation.from_rotvec(root_orient).apply(rest_root).astype(np.float32)
    return (pelvis_pos - rotated_rest_root).astype(np.float32)


def create_body_model(body_model_path: Path, num_betas: int, device: torch.device):
    from human_body_prior.body_model.body_model import BodyModel

    if not body_model_path.exists():
        raise FileNotFoundError(f"SMPL-H body model not found: {body_model_path}")
    model = BodyModel(bm_fname=str(body_model_path), num_betas=num_betas)
    model = model.to(device)
    model.eval()
    return model


def body_model_rest_root(body_model, num_betas: int, device: torch.device) -> np.ndarray:
    return body_model_rest_joints(body_model, num_betas, device)[0]


def body_model_rest_joints(body_model, num_betas: int, device: torch.device) -> np.ndarray:
    with torch.no_grad():
        out = body_model(
            pose_body=torch.zeros(1, 63, dtype=torch.float32, device=device),
            pose_hand=torch.zeros(1, 90, dtype=torch.float32, device=device),
            betas=torch.zeros(1, num_betas, dtype=torch.float32, device=device),
            root_orient=torch.zeros(1, 3, dtype=torch.float32, device=device),
            trans=torch.zeros(1, 3, dtype=torch.float32, device=device),
        )
    return out.Jtr[0].detach().cpu().numpy().astype(np.float32)


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
    err = np.max(np.abs(np.matmul(np.swapaxes(rotmats, -1, -2), rotmats) - eye))
    det_min = float(np.min(np.linalg.det(rotmats.reshape(-1, 3, 3))))
    det_max = float(np.max(np.linalg.det(rotmats.reshape(-1, 3, 3))))
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
    smpl24_global = build_smpl24_global_rotmats(frames, arm_correction=config.arm_correction)
    smpl24_local = global_to_local_rotmats(smpl24_global, SMPL24_PARENTS)
    smplh_global = build_smplh_global_rotmats(
        frames=frames,
        smpl24_global=smpl24_global,
        smplh_parents=smplh_parents,
        rest_joints=rest_joints,
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
        "retarget_mode": "xsens_global_segment_to_smplh_v2",
        "coord_mode": "xsens_zup_to_smpl",
        "body_mapping_profile": "mvnx_to_smpl",
        "hand_mode": "xsens_relative_fingers_thumb_position_swing",
        "thumb_retarget_mode": "manus_thumb_position_swing_for_cmc_mcp_distal_swing_only",
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
        "thumb_swing_position_pairs": dict(THUMB_SWING_POSITION_PAIRS),
        "notes": (
            "Xsens global segment orientations are converted into the SMPL frame before "
            "global-to-local retargeting. SMPL-H finger pose is driven by relative "
            "Xsens finger segment rotations, except thumb CMC/MCP use MANUS position-chain "
            "swing directions and the distal thumb joint is twist-filtered. "
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
        "[NOTE] Fixed path: Xsens Z-up -> SMPL, MVNX body mapping, Xsens relative fingers, "
        "MANUS thumb position-swing, and raw arm/hand body segment frames by default."
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
