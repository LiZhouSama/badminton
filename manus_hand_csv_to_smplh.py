#!/usr/bin/env python3
"""
Retarget one MANUS hand CSV export to SMPL-H hand pose.

The input is the MANUS wide CSV export with columns such as
``Index_MCP_Rotation_X`` and ``Thumb_CMC_Position_X``.  The body, root
translation, and the missing hand are exported as zero pose.

By default this uses MANUS ergonomics joint angles (flex/spread) because the
CSV rotation columns are documented as world-space absolute joint rotations,
not SMPL-H local joint pose.  The older parent/child segment-relative rotation
path remains available with ``--retarget-source segment-rotations``.
"""

from __future__ import annotations

import argparse
import csv
from collections import OrderedDict
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np
import torch
from scipy.spatial.transform import Rotation

from xsens_mvn_csv_to_smplh import (
    DEFAULT_BODY_MODEL,
    HAND_JOINT_MVN_PAIRS,
    HAND_MVN_TO_SMPLH,
    SMPLH_52_NAMES,
    SegmentPose,
    FramePose,
    THUMB_DISTAL_JOINTS,
    THUMB_SWING_POSITION_PAIRS,
    body_model_rest_joints,
    check_rotation_orthogonality,
    create_body_model,
    finger_local_rotmat_for_smpl,
    global_to_local_rotmats,
    local_to_global_rotmats,
    matrix_to_axis_angle,
    read_smplh_parents,
    thumb_distal_local_rotmat_for_smpl,
    thumb_swing_local_rotmat_for_smpl,
)


DEFAULT_INPUT_CSV = Path("output/manusTakes/test_Z_UP_X_FORWARD_RIGHT_HANDED_MMZ_L.csv")
DEFAULT_OUTPUT = Path("output/manusTakes/test_Z_UP_X_FORWARD_RIGHT_HANDED_MMZ_L.pt")
MANUS_POSITION_SCALE_TO_METERS = 0.01
MANUS_COORDINATE_SYSTEM = "Z-Up right-handed"


MANUS_TO_MVN_BASE = {
    "Hand": "Carpus",
    "Thumb_CMC": "First Metacarpal",
    "Thumb_MCP": "First Proximal Phalange",
    "Thumb_DIP": "First Distal Phalange",
    "Index_CMC": "Second Metacarpal",
    "Index_MCP": "Second Proximal Phalange",
    "Index_PIP": "Second Middle Phalange",
    "Index_DIP": "Second Distal Phalange",
    "Middle_CMC": "Third Metacarpal",
    "Middle_MCP": "Third Proximal Phalange",
    "Middle_PIP": "Third Middle Phalange",
    "Middle_DIP": "Third Distal Phalange",
    "Pinky_CMC": "Fifth Metacarpal",
    "Pinky_MCP": "Fifth Proximal Phalange",
    "Pinky_PIP": "Fifth Middle Phalange",
    "Pinky_DIP": "Fifth Distal Phalange",
    "Ring_CMC": "Fourth Metacarpal",
    "Ring_MCP": "Fourth Proximal Phalange",
    "Ring_PIP": "Fourth Middle Phalange",
    "Ring_DIP": "Fourth Distal Phalange",
}

MANUS_REQUIRED_PREFIXES = tuple(MANUS_TO_MVN_BASE.keys())

FINGER_ANGLE_COLUMNS = {
    "index": {
        "mcp_spread": "Index_MCP_Spread",
        "mcp_flex": "Index_MCP_Flex",
        "pip_flex": "Index_PIP_Flex",
        "dip_flex": "Index_DIP_Flex",
    },
    "middle": {
        "mcp_spread": "Middle_MCP_Spread",
        "mcp_flex": "Middle_MCP_Flex",
        "pip_flex": "Middle_PIP_Flex",
        "dip_flex": "Middle_DIP_Flex",
    },
    "ring": {
        "mcp_spread": "Ring_MCP_Spread",
        "mcp_flex": "Ring_MCP_Flex",
        "pip_flex": "Ring_PIP_Flex",
        "dip_flex": "Ring_DIP_Flex",
    },
    "pinky": {
        "mcp_spread": "Pinky_MCP_Spread",
        "mcp_flex": "Pinky_MCP_Flex",
        "pip_flex": "Pinky_PIP_Flex",
        "dip_flex": "Pinky_DIP_Flex",
    },
}

THUMB_ANGLE_COLUMNS = {
    "cmc_spread": "Thumb_CMC_Spread",
    "cmc_flex": "Thumb_CMC_Flex",
    "pip_flex": "Thumb_PIP_Flex",
    "dip_flex": "Thumb_DIP_Flex",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--csv", type=Path, default=DEFAULT_INPUT_CSV, help="MANUS hand CSV export.")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT, help="Output SMPL-H .pt file.")
    parser.add_argument("--body-model", type=Path, default=DEFAULT_BODY_MODEL)
    parser.add_argument("--device", choices=["auto", "cpu", "cuda"], default="cuda")
    parser.add_argument("--num-betas", type=int, default=16)
    parser.add_argument("--side", choices=["auto", "left", "right"], default="auto")
    parser.add_argument(
        "--retarget-source",
        choices=["ergonomics", "segment-rotations"],
        default="ergonomics",
        help=(
            "Use MANUS anatomical joint angles by default. "
            "segment-rotations keeps the older world-rotation-relative path."
        ),
    )
    parser.add_argument(
        "--euler-order",
        default="xyz",
        help="Euler order for MANUS *_Rotation_X/Y/Z columns in segment-rotations mode. Default: xyz.",
    )
    parser.add_argument(
        "--finger-axis-mode",
        choices=["canonical", "rest-palm"],
        default="rest-palm",
        help="rest-palm bends fingers around axes derived from SMPL-H rest bones and the palm normal.",
    )
    parser.add_argument(
        "--thumb-source",
        choices=["segment-positions", "ergonomics"],
        default="ergonomics",
        help="ergonomics uses MANUS Thumb_CMC/Thumb_PIP/Thumb_DIP flex-spread angles.",
    )
    parser.add_argument("--finger-flex-sign", type=float, default=-1.0)
    parser.add_argument("--finger-spread-sign", type=float, default=1.0)
    parser.add_argument("--thumb-flex-sign", type=float, default=-1.0)
    parser.add_argument("--thumb-spread-sign", type=float, default=-1.0)
    parser.add_argument("--palm-normal-sign", type=float, default=1.0)
    parser.add_argument("--max-frames", type=int, default=None)
    parser.add_argument("--frame-stride", type=int, default=1)
    parser.add_argument(
        "--visualize",
        action="store_true",
        help="Open the generated sequence in aitviewer after saving.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Build SMPL-H meshes for validation but do not open the aitviewer window.",
    )
    parser.add_argument("--batch-size", type=int, default=256)
    parser.add_argument("--show-debug-points", action="store_true")
    return parser.parse_args()


def select_device(device_arg: str) -> torch.device:
    if device_arg == "auto":
        return torch.device("cuda" if torch.cuda.is_available() else "cpu")
    if device_arg == "cuda" and not torch.cuda.is_available():
        raise RuntimeError("CUDA was requested but is not available.")
    return torch.device(device_arg)


def infer_side(csv_path: Path, side_arg: str) -> str:
    if side_arg != "auto":
        return side_arg
    stem = csv_path.stem.lower()
    tokens = {token for token in stem.replace("-", "_").split("_") if token}
    if "l" in tokens or "left" in tokens:
        return "left"
    if "r" in tokens or "right" in tokens:
        return "right"
    raise ValueError(
        f"Could not infer hand side from {csv_path.name!r}; pass --side left or --side right."
    )


def side_prefix(side: str) -> str:
    if side == "left":
        return "Left "
    if side == "right":
        return "Right "
    raise ValueError(f"Unsupported side: {side!r}")


def manus_prefix_to_mvn_name(manus_prefix: str, side: str) -> str:
    base = MANUS_TO_MVN_BASE[manus_prefix]
    return f"{side_prefix(side)}{base}"


def validate_manus_columns(fieldnames: Optional[Sequence[str]]) -> None:
    if not fieldnames:
        raise ValueError("CSV has no header.")
    fields = set(fieldnames)
    missing: List[str] = []
    for name in ("Frame", "Elapsed_Time_In_Milliseconds", "Time"):
        if name not in fields:
            missing.append(name)
    for prefix in MANUS_REQUIRED_PREFIXES:
        for group in ("Position", "Rotation"):
            for axis in "XYZ":
                name = f"{prefix}_{group}_{axis}"
                if name not in fields:
                    missing.append(name)
    for columns in FINGER_ANGLE_COLUMNS.values():
        for name in columns.values():
            if name not in fields:
                missing.append(name)
    for name in THUMB_ANGLE_COLUMNS.values():
        if name not in fields:
            missing.append(name)
    if missing:
        raise ValueError(f"MANUS CSV missing required columns: {missing}")


def parse_float(row: Dict[str, str], column: str) -> float:
    value = row.get(column)
    if value is None or value == "":
        raise ValueError(f"Missing value for column {column!r} at frame {row.get('Frame')}")
    return float(value)


def parse_position_m(row: Dict[str, str], prefix: str) -> np.ndarray:
    return (
        np.array(
            [
                parse_float(row, f"{prefix}_Position_X"),
                parse_float(row, f"{prefix}_Position_Y"),
                parse_float(row, f"{prefix}_Position_Z"),
            ],
            dtype=np.float32,
        )
        * MANUS_POSITION_SCALE_TO_METERS
    )


def parse_euler_rotmat(row: Dict[str, str], prefix: str, euler_order: str) -> np.ndarray:
    angles_deg = [
        parse_float(row, f"{prefix}_Rotation_X"),
        parse_float(row, f"{prefix}_Rotation_Y"),
        parse_float(row, f"{prefix}_Rotation_Z"),
    ]
    return Rotation.from_euler(euler_order, angles_deg, degrees=True).as_matrix().astype(np.float32)


def row_to_finger_poses(row: Dict[str, str], side: str, euler_order: str) -> Dict[str, SegmentPose]:
    finger_poses: Dict[str, SegmentPose] = {}
    for manus_prefix in MANUS_REQUIRED_PREFIXES:
        mvn_name = manus_prefix_to_mvn_name(manus_prefix, side)
        finger_poses[mvn_name] = SegmentPose(
            rotmat=parse_euler_rotmat(row, manus_prefix, euler_order),
            pos_m=parse_position_m(row, manus_prefix),
            coordinate_system=MANUS_COORDINATE_SYSTEM,
        )
    return finger_poses


def row_to_ergonomic_angles(row: Dict[str, str]) -> Dict[str, float]:
    angles: Dict[str, float] = {}
    for columns in FINGER_ANGLE_COLUMNS.values():
        for name in columns.values():
            angles[name] = parse_float(row, name)
    for name in THUMB_ANGLE_COLUMNS.values():
        angles[name] = parse_float(row, name)
    return angles


def load_manus_sequence(
    csv_path: Path,
    side: str,
    euler_order: str,
    frame_stride: int,
    max_frames: Optional[int],
) -> Tuple[List[FramePose], np.ndarray, List[Dict[str, float]]]:
    if not csv_path.exists():
        raise FileNotFoundError(f"MANUS CSV not found: {csv_path}")
    if frame_stride < 1:
        raise ValueError("--frame-stride must be >= 1")
    if max_frames is not None and max_frames < 1:
        raise ValueError("--max-frames must be >= 1")

    frames: "OrderedDict[int, FramePose]" = OrderedDict()
    elapsed_time_ms: List[float] = []
    ergonomic_angles: List[Dict[str, float]] = []
    with csv_path.open(newline="", encoding="utf-8-sig") as handle:
        reader = csv.DictReader(handle)
        validate_manus_columns(reader.fieldnames)
        for row_idx, row in enumerate(reader):
            if row_idx % frame_stride != 0:
                continue
            sample_counter = int(float(row["Frame"]))
            elapsed_ms = float(row["Elapsed_Time_In_Milliseconds"])
            time_code_ms = int(round(elapsed_ms))
            finger_poses = row_to_finger_poses(row, side, euler_order)
            frame = FramePose(sample_counter=sample_counter, time_code_ms=time_code_ms)
            if side == "left":
                frame.left_finger = finger_poses
            else:
                frame.right_finger = finger_poses
            frames[sample_counter] = frame
            elapsed_time_ms.append(elapsed_ms)
            ergonomic_angles.append(row_to_ergonomic_angles(row))
            if max_frames is not None and len(frames) >= max_frames:
                break

    loaded = list(frames.values())
    if not loaded:
        raise ValueError(f"No frames loaded from {csv_path}")
    return loaded, np.asarray(elapsed_time_ms, dtype=np.float32), ergonomic_angles


def selected_hand_joint_indices(side: str) -> Iterable[int]:
    prefix = f"{side}_"
    for idx in range(22, 52):
        name = SMPLH_52_NAMES[idx]
        if name.startswith(prefix):
            yield idx


def estimate_fps_from_elapsed_ms(elapsed_time_ms: np.ndarray) -> float:
    if elapsed_time_ms.shape[0] < 2:
        return 0.0
    diffs = np.diff(elapsed_time_ms.astype(np.float64)) / 1000.0
    diffs = diffs[(diffs > 0) & (diffs < 1.0)]
    if diffs.size == 0:
        return 0.0
    return float(1.0 / np.median(diffs))


def normalize_vector(vector: np.ndarray, name: str) -> np.ndarray:
    vector = vector.astype(np.float32, copy=False)
    norm = float(np.linalg.norm(vector))
    if norm < 1e-8:
        raise ValueError(f"Cannot normalize zero-length vector for {name}")
    return (vector / norm).astype(np.float32)


def axis_angle_rotmat(axis: np.ndarray, angle_deg: float) -> np.ndarray:
    axis = normalize_vector(axis, "axis-angle axis")
    return Rotation.from_rotvec(axis * np.deg2rad(angle_deg)).as_matrix().astype(np.float32)


def canonical_finger_mcp_local_rotmat(
    spread_deg: float,
    flex_deg: float,
    finger_spread_sign: float,
    finger_flex_sign: float,
) -> np.ndarray:
    spread_rot = axis_angle_rotmat(np.array([0.0, 1.0, 0.0], dtype=np.float32), spread_deg * finger_spread_sign)
    flex_rot = axis_angle_rotmat(np.array([1.0, 0.0, 0.0], dtype=np.float32), flex_deg * finger_flex_sign)
    return (spread_rot @ flex_rot).astype(np.float32)


def canonical_finger_flex_local_rotmat(flex_deg: float, flex_sign: float) -> np.ndarray:
    return axis_angle_rotmat(np.array([1.0, 0.0, 0.0], dtype=np.float32), flex_deg * flex_sign)


def smplh_children(parents: np.ndarray) -> Dict[int, List[int]]:
    children: Dict[int, List[int]] = {}
    for idx, parent_idx in enumerate(parents):
        if int(parent_idx) >= 0:
            children.setdefault(int(parent_idx), []).append(idx)
    return children


def hand_palm_normal(side: str, rest_joints: np.ndarray, palm_normal_sign: float) -> np.ndarray:
    wrist_idx = 20 if side == "left" else 21
    index_idx = SMPLH_52_NAMES.index(f"{side}_index1")
    pinky_idx = SMPLH_52_NAMES.index(f"{side}_pinky1")
    along_hand = rest_joints[index_idx] - rest_joints[wrist_idx]
    across_hand = rest_joints[pinky_idx] - rest_joints[index_idx]
    normal = np.cross(along_hand, across_hand)
    return normalize_vector(normal * float(palm_normal_sign), f"{side} palm normal")


def joint_rest_bone_axis(
    joint_idx: int,
    parents: np.ndarray,
    children: Dict[int, List[int]],
    rest_joints: np.ndarray,
) -> np.ndarray:
    joint_children = children.get(joint_idx, [])
    if joint_children:
        child_idx = joint_children[0]
        return normalize_vector(rest_joints[child_idx] - rest_joints[joint_idx], "rest child bone")
    parent_idx = int(parents[joint_idx])
    if parent_idx < 0:
        raise ValueError(f"Joint {joint_idx} has no parent or child for rest bone axis")
    return normalize_vector(rest_joints[joint_idx] - rest_joints[parent_idx], "rest incoming bone")


def finger_flex_axis(
    joint_idx: int,
    parents: np.ndarray,
    children: Dict[int, List[int]],
    rest_joints: np.ndarray,
    palm_normal: np.ndarray,
) -> np.ndarray:
    bone_axis = joint_rest_bone_axis(joint_idx, parents, children, rest_joints)
    return normalize_vector(np.cross(bone_axis, palm_normal), "finger flex axis")


def finger_mcp_local_rotmat(
    joint_idx: int,
    spread_deg: float,
    flex_deg: float,
    parents: np.ndarray,
    children: Dict[int, List[int]],
    rest_joints: np.ndarray,
    palm_normal: np.ndarray,
    finger_spread_sign: float,
    finger_flex_sign: float,
) -> np.ndarray:
    spread_rot = axis_angle_rotmat(palm_normal, spread_deg * finger_spread_sign)
    flex_axis = finger_flex_axis(joint_idx, parents, children, rest_joints, palm_normal)
    flex_rot = axis_angle_rotmat(flex_axis, flex_deg * finger_flex_sign)
    return (spread_rot @ flex_rot).astype(np.float32)


def finger_flex_local_rotmat(
    joint_idx: int,
    flex_deg: float,
    parents: np.ndarray,
    children: Dict[int, List[int]],
    rest_joints: np.ndarray,
    palm_normal: np.ndarray,
    flex_sign: float,
) -> np.ndarray:
    flex_axis = finger_flex_axis(joint_idx, parents, children, rest_joints, palm_normal)
    return axis_angle_rotmat(flex_axis, flex_deg * flex_sign)


def thumb_cmc_local_rotmat(
    joint_idx: int,
    spread_deg: float,
    flex_deg: float,
    parents: np.ndarray,
    children: Dict[int, List[int]],
    rest_joints: np.ndarray,
    palm_normal: np.ndarray,
    thumb_spread_sign: float,
    thumb_flex_sign: float,
) -> np.ndarray:
    bone_axis = joint_rest_bone_axis(joint_idx, parents, children, rest_joints)
    spread_axis = normalize_vector(np.cross(bone_axis, palm_normal), "thumb spread axis")
    spread_rot = axis_angle_rotmat(spread_axis, spread_deg * thumb_spread_sign)
    flex_rot = axis_angle_rotmat(palm_normal, flex_deg * thumb_flex_sign)
    return (spread_rot @ flex_rot).astype(np.float32)


def thumb_segment_position_local_rotmats(
    frame: FramePose,
    side: str,
    rest_joints: np.ndarray,
) -> Dict[int, np.ndarray]:
    thumb1_name = f"{side}_thumb1"
    thumb2_name = f"{side}_thumb2"
    thumb3_name = f"{side}_thumb3"
    thumb1_idx = SMPLH_52_NAMES.index(thumb1_name)
    thumb2_idx = SMPLH_52_NAMES.index(thumb2_name)
    thumb3_idx = SMPLH_52_NAMES.index(thumb3_name)

    thumb1_local = thumb_swing_local_rotmat_for_smpl(
        frame=frame,
        joint_name=thumb1_name,
        parent_global=np.eye(3, dtype=np.float32),
        rest_joints=rest_joints,
    )
    thumb1_global = thumb1_local
    thumb2_local = thumb_swing_local_rotmat_for_smpl(
        frame=frame,
        joint_name=thumb2_name,
        parent_global=thumb1_global,
        rest_joints=rest_joints,
    )
    thumb3_local = thumb_distal_local_rotmat_for_smpl(
        frame=frame,
        joint_name=thumb3_name,
        rest_joints=rest_joints,
    )
    return {
        thumb1_idx: thumb1_local,
        thumb2_idx: thumb2_local,
        thumb3_idx: thumb3_local,
    }


def build_hand_local_rotmats_from_ergonomics(
    frames: Sequence[FramePose],
    ergonomic_angles: Sequence[Dict[str, float]],
    side: str,
    smplh_parents: np.ndarray,
    rest_joints: np.ndarray,
    finger_axis_mode: str,
    thumb_source: str,
    palm_normal_sign: float,
    finger_flex_sign: float,
    finger_spread_sign: float,
    thumb_flex_sign: float,
    thumb_spread_sign: float,
) -> np.ndarray:
    local = np.tile(np.eye(3, dtype=np.float32), (len(ergonomic_angles), 52, 1, 1))
    children = smplh_children(smplh_parents)
    palm_normal = hand_palm_normal(side, rest_joints, palm_normal_sign)

    for frame_idx, angles in enumerate(ergonomic_angles):
        for finger, columns in FINGER_ANGLE_COLUMNS.items():
            joint1 = SMPLH_52_NAMES.index(f"{side}_{finger}1")
            joint2 = SMPLH_52_NAMES.index(f"{side}_{finger}2")
            joint3 = SMPLH_52_NAMES.index(f"{side}_{finger}3")
            if finger_axis_mode == "canonical":
                local[frame_idx, joint1] = canonical_finger_mcp_local_rotmat(
                    spread_deg=angles[columns["mcp_spread"]],
                    flex_deg=angles[columns["mcp_flex"]],
                    finger_spread_sign=finger_spread_sign,
                    finger_flex_sign=finger_flex_sign,
                )
                local[frame_idx, joint2] = canonical_finger_flex_local_rotmat(
                    flex_deg=angles[columns["pip_flex"]],
                    flex_sign=finger_flex_sign,
                )
                local[frame_idx, joint3] = canonical_finger_flex_local_rotmat(
                    flex_deg=angles[columns["dip_flex"]],
                    flex_sign=finger_flex_sign,
                )
            elif finger_axis_mode == "rest-palm":
                local[frame_idx, joint1] = finger_mcp_local_rotmat(
                    joint_idx=joint1,
                    spread_deg=angles[columns["mcp_spread"]],
                    flex_deg=angles[columns["mcp_flex"]],
                    parents=smplh_parents,
                    children=children,
                    rest_joints=rest_joints,
                    palm_normal=palm_normal,
                    finger_spread_sign=finger_spread_sign,
                    finger_flex_sign=finger_flex_sign,
                )
                local[frame_idx, joint2] = finger_flex_local_rotmat(
                    joint_idx=joint2,
                    flex_deg=angles[columns["pip_flex"]],
                    parents=smplh_parents,
                    children=children,
                    rest_joints=rest_joints,
                    palm_normal=palm_normal,
                    flex_sign=finger_flex_sign,
                )
                local[frame_idx, joint3] = finger_flex_local_rotmat(
                    joint_idx=joint3,
                    flex_deg=angles[columns["dip_flex"]],
                    parents=smplh_parents,
                    children=children,
                    rest_joints=rest_joints,
                    palm_normal=palm_normal,
                    flex_sign=finger_flex_sign,
                )
            else:
                raise ValueError(f"Unsupported finger axis mode: {finger_axis_mode!r}")

        if thumb_source == "segment-positions":
            for thumb_idx, thumb_local in thumb_segment_position_local_rotmats(
                frame=frames[frame_idx],
                side=side,
                rest_joints=rest_joints,
            ).items():
                local[frame_idx, thumb_idx] = thumb_local
        elif thumb_source == "ergonomics":
            thumb1 = SMPLH_52_NAMES.index(f"{side}_thumb1")
            thumb2 = SMPLH_52_NAMES.index(f"{side}_thumb2")
            thumb3 = SMPLH_52_NAMES.index(f"{side}_thumb3")
            local[frame_idx, thumb1] = thumb_cmc_local_rotmat(
                joint_idx=thumb1,
                spread_deg=angles[THUMB_ANGLE_COLUMNS["cmc_spread"]],
                flex_deg=angles[THUMB_ANGLE_COLUMNS["cmc_flex"]],
                parents=smplh_parents,
                children=children,
                rest_joints=rest_joints,
                palm_normal=palm_normal,
                thumb_spread_sign=thumb_spread_sign,
                thumb_flex_sign=thumb_flex_sign,
            )
            local[frame_idx, thumb2] = finger_flex_local_rotmat(
                joint_idx=thumb2,
                flex_deg=angles[THUMB_ANGLE_COLUMNS["pip_flex"]],
                parents=smplh_parents,
                children=children,
                rest_joints=rest_joints,
                palm_normal=palm_normal,
                flex_sign=thumb_flex_sign,
            )
            local[frame_idx, thumb3] = finger_flex_local_rotmat(
                joint_idx=thumb3,
                flex_deg=angles[THUMB_ANGLE_COLUMNS["dip_flex"]],
                parents=smplh_parents,
                children=children,
                rest_joints=rest_joints,
                palm_normal=palm_normal,
                flex_sign=thumb_flex_sign,
            )
        else:
            raise ValueError(f"Unsupported thumb source: {thumb_source!r}")
    return local.astype(np.float32)


def build_hand_global_rotmats_from_segment_rotations(
    frames: Sequence[FramePose],
    side: str,
    smplh_parents: np.ndarray,
    rest_joints: np.ndarray,
) -> np.ndarray:
    full = np.tile(np.eye(3, dtype=np.float32), (len(frames), 52, 1, 1))
    for frame_idx, frame in enumerate(frames):
        for joint_idx in selected_hand_joint_indices(side):
            joint_name = SMPLH_52_NAMES[joint_idx]
            parent_idx = int(smplh_parents[joint_idx])
            parent_global = np.eye(3, dtype=np.float32) if parent_idx < 0 else full[frame_idx, parent_idx]
            if joint_name in THUMB_SWING_POSITION_PAIRS:
                local = thumb_swing_local_rotmat_for_smpl(
                    frame=frame,
                    joint_name=joint_name,
                    parent_global=parent_global,
                    rest_joints=rest_joints,
                )
            elif joint_name in THUMB_DISTAL_JOINTS:
                local = thumb_distal_local_rotmat_for_smpl(
                    frame=frame,
                    joint_name=joint_name,
                    rest_joints=rest_joints,
                )
            elif joint_name in HAND_JOINT_MVN_PAIRS:
                local = finger_local_rotmat_for_smpl(frame, joint_name)
            else:
                local = np.eye(3, dtype=np.float32)
            full[frame_idx, joint_idx] = (parent_global @ local).astype(np.float32)
    return full


def retarget_manus_hand(
    frames: Sequence[FramePose],
    elapsed_time_ms: np.ndarray,
    ergonomic_angles: Sequence[Dict[str, float]],
    side: str,
    csv_path: Path,
    body_model_path: Path,
    num_betas: int,
    device: torch.device,
    retarget_source: str,
    euler_order: str,
    finger_axis_mode: str,
    thumb_source: str,
    palm_normal_sign: float,
    finger_flex_sign: float,
    finger_spread_sign: float,
    thumb_flex_sign: float,
    thumb_spread_sign: float,
) -> Dict[str, object]:
    body_model = create_body_model(body_model_path, num_betas, device)
    rest_joints = body_model_rest_joints(body_model, num_betas, device)
    smplh_parents = read_smplh_parents(body_model_path)

    if retarget_source == "ergonomics":
        smplh_local = build_hand_local_rotmats_from_ergonomics(
            frames=frames,
            ergonomic_angles=ergonomic_angles,
            side=side,
            smplh_parents=smplh_parents,
            rest_joints=rest_joints,
            finger_axis_mode=finger_axis_mode,
            thumb_source=thumb_source,
            palm_normal_sign=palm_normal_sign,
            finger_flex_sign=finger_flex_sign,
            finger_spread_sign=finger_spread_sign,
            thumb_flex_sign=thumb_flex_sign,
            thumb_spread_sign=thumb_spread_sign,
        )
        smplh_global = local_to_global_rotmats(smplh_local, smplh_parents)
    elif retarget_source == "segment-rotations":
        smplh_global = build_hand_global_rotmats_from_segment_rotations(
            frames=frames,
            side=side,
            smplh_parents=smplh_parents,
            rest_joints=rest_joints,
        )
        smplh_local = global_to_local_rotmats(smplh_global, smplh_parents)
    else:
        raise ValueError(f"Unsupported retarget source: {retarget_source!r}")
    smplh_global_reconstructed = local_to_global_rotmats(smplh_local, smplh_parents)
    check_rotation_orthogonality("smplh_global", smplh_global)
    check_rotation_orthogonality("smplh_local", smplh_local)
    check_rotation_orthogonality("smplh_global_reconstructed", smplh_global_reconstructed)

    local_axis = matrix_to_axis_angle(smplh_local)
    frame_count = len(frames)
    root_orient = np.zeros((frame_count, 3), dtype=np.float32)
    pose_body = np.zeros((frame_count, 63), dtype=np.float32)
    pose_hand = local_axis[:, 22:52].reshape(frame_count, 90).astype(np.float32)
    trans = np.zeros((frame_count, 3), dtype=np.float32)
    pose = np.concatenate([root_orient, pose_body, pose_hand], axis=1).astype(np.float32)
    sample_counter = np.array([frame.sample_counter for frame in frames], dtype=np.int64)
    time_code_ms = np.rint(elapsed_time_ms).astype(np.int64)
    betas = np.zeros((frame_count, num_betas), dtype=np.float32)

    return {
        "pose": torch.from_numpy(pose),
        "trans": torch.from_numpy(trans),
        "root_orient": torch.from_numpy(root_orient),
        "pose_body": torch.from_numpy(pose_body),
        "pose_hand": torch.from_numpy(pose_hand),
        "betas": torch.from_numpy(betas),
        "fps": estimate_fps_from_elapsed_ms(elapsed_time_ms),
        "sample_counter": torch.from_numpy(sample_counter),
        "time_code_ms": torch.from_numpy(time_code_ms),
        "elapsed_time_ms": torch.from_numpy(elapsed_time_ms.astype(np.float32, copy=False)),
        "source_manus_csv": str(csv_path),
        "retarget_mode": f"manus_wide_hand_to_smplh_{retarget_source}_v2",
        "coord_mode": "xsens_zup_to_smpl",
        "hand_mode": (
            f"manus_ergonomics_angles_to_smplh_{finger_axis_mode}_finger_axes"
            if retarget_source == "ergonomics"
            else "xsens_relative_fingers_thumb_position_swing"
        ),
        "thumb_retarget_mode": (
            (
                "manus_thumb_position_swing_for_cmc_mcp_distal_swing_only"
                if thumb_source == "segment-positions"
                else "manus_ergonomics_thumb_cmc_spread_flex_pip_dip"
            )
            if retarget_source == "ergonomics"
            else "manus_thumb_position_swing_for_cmc_mcp_distal_swing_only"
        ),
        "manus_side": side,
        "manus_retarget_source": retarget_source,
        "manus_euler_order": euler_order,
        "finger_axis_mode": finger_axis_mode,
        "thumb_source": thumb_source,
        "palm_normal_sign": float(palm_normal_sign),
        "finger_flex_sign": float(finger_flex_sign),
        "finger_spread_sign": float(finger_spread_sign),
        "thumb_flex_sign": float(thumb_flex_sign),
        "thumb_spread_sign": float(thumb_spread_sign),
        "manus_position_scale_to_meters": float(MANUS_POSITION_SCALE_TO_METERS),
        "body_model_path": str(body_model_path),
        "smplh_52_names": SMPLH_52_NAMES,
        "smplh_parents": torch.from_numpy(smplh_parents.astype(np.int64)),
        "smplh_global_rotmats": torch.from_numpy(smplh_global),
        "smplh_global_rotmats_reconstructed": torch.from_numpy(smplh_global_reconstructed),
        "smplh_local_rotmats": torch.from_numpy(smplh_local),
        "hand_mapping": dict(HAND_MVN_TO_SMPLH),
        "hand_joint_mvn_pairs": dict(HAND_JOINT_MVN_PAIRS),
        "manus_to_mvn_mapping": {
            prefix: manus_prefix_to_mvn_name(prefix, side)
            for prefix in MANUS_REQUIRED_PREFIXES
        },
        "ergonomic_angle_columns": {
            "fingers": FINGER_ANGLE_COLUMNS,
            "thumb": THUMB_ANGLE_COLUMNS,
        },
        "notes": (
            "Only one MANUS hand is present. Root orientation, body pose, translation, "
            "and the missing hand are zero. The default populated hand path uses "
            "MANUS ergonomics flex/spread angles mapped to SMPL-H rest-palm finger "
            "axes, with MANUS thumb ergonomics angles for thumb CMC/PIP/DIP. "
            "Use --retarget-source segment-rotations for the older world-rotation-relative path."
        ),
    }


def save_pt(data: Dict[str, object], output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    torch.save(data, output)


def print_header_summary(csv_path: Path) -> None:
    with csv_path.open(newline="", encoding="utf-8-sig") as handle:
        reader = csv.DictReader(handle)
        fieldnames = reader.fieldnames or []
    counts = {
        "meta": 0,
        "position": 0,
        "rotation": 0,
        "velocity": 0,
        "acceleration": 0,
        "pinch": 0,
        "joint_angle": 0,
        "angular_velocity": 0,
        "angular_acceleration": 0,
        "other": 0,
    }
    for name in fieldnames:
        if name in {"Frame", "Elapsed_Time_In_Milliseconds", "Time"}:
            counts["meta"] += 1
        elif "_Position_" in name:
            counts["position"] += 1
        elif "_Rotation_" in name:
            counts["rotation"] += 1
        elif "_Velocity_" in name:
            counts["velocity"] += 1
        elif "_Acceleration_" in name:
            counts["acceleration"] += 1
        elif "_AngularVelocity" in name:
            counts["angular_velocity"] += 1
        elif "_AngularAcceleration" in name:
            counts["angular_acceleration"] += 1
        elif name.startswith("Pinch_"):
            counts["pinch"] += 1
        elif name.endswith("_Flex") or name.endswith("_Spread"):
            counts["joint_angle"] += 1
        else:
            counts["other"] += 1
    compact = " ".join(f"{key}={value}" for key, value in counts.items() if value)
    print(f"[CSV] columns={len(fieldnames)} {compact}")


def print_summary(data: Dict[str, object], output: Path) -> None:
    pose = data["pose"]
    pose_hand = data["pose_hand"]
    sample_counter = data["sample_counter"]
    time_code_ms = data["time_code_ms"]
    print(f"[SAVE] {output}")
    print(f"[DATA] frames={pose.shape[0]} fps~={data['fps']:.3f}")
    print(f"[DATA] pose={tuple(pose.shape)} pose_hand={tuple(pose_hand.shape)} trans={tuple(data['trans'].shape)}")
    print(f"[DATA] sample_counter={int(sample_counter[0])}..{int(sample_counter[-1])}")
    print(f"[DATA] time_code_ms={int(time_code_ms[0])}..{int(time_code_ms[-1])}")
    print(
        "[MODE] retarget={retarget} coord={coord} hand={hand} side={side} euler={euler}".format(
            retarget=data.get("retarget_mode", ""),
            coord=data.get("coord_mode", ""),
            hand=data.get("hand_mode", ""),
            side=data.get("manus_side", ""),
            euler=data.get("manus_euler_order", ""),
        )
    )


def maybe_visualize(args: argparse.Namespace, data: Dict[str, object], device: torch.device) -> None:
    if not args.visualize and not args.dry_run:
        return
    from visualize_xsens_smplh_pt import forward_vertices, load_body_model, visualize

    body_model = load_body_model(args.body_model, int(data["betas"].shape[-1]), device)
    vertices, joints = forward_vertices(body_model, data, device, args.batch_size)
    faces = body_model.f.detach().cpu().numpy().astype(np.int32)
    print(f"[MESH] vertices={vertices.shape} joints={joints.shape} faces={faces.shape}")
    if args.dry_run:
        return
    visualize(vertices, joints, faces, show_debug_points=args.show_debug_points)


def main() -> None:
    args = parse_args()
    device = select_device(args.device)
    side = infer_side(args.csv, args.side)
    frames, elapsed_time_ms, ergonomic_angles = load_manus_sequence(
        csv_path=args.csv,
        side=side,
        euler_order=args.euler_order,
        frame_stride=args.frame_stride,
        max_frames=args.max_frames,
    )
    data = retarget_manus_hand(
        frames=frames,
        elapsed_time_ms=elapsed_time_ms,
        ergonomic_angles=ergonomic_angles,
        side=side,
        csv_path=args.csv,
        body_model_path=args.body_model,
        num_betas=args.num_betas,
        device=device,
        retarget_source=args.retarget_source,
        euler_order=args.euler_order,
        finger_axis_mode=args.finger_axis_mode,
        thumb_source=args.thumb_source,
        palm_normal_sign=args.palm_normal_sign,
        finger_flex_sign=args.finger_flex_sign,
        finger_spread_sign=args.finger_spread_sign,
        thumb_flex_sign=args.thumb_flex_sign,
        thumb_spread_sign=args.thumb_spread_sign,
    )
    save_pt(data, args.output)
    print_header_summary(args.csv)
    print_summary(data, args.output)
    maybe_visualize(args, data, device)


if __name__ == "__main__":
    main()
