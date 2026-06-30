#!/usr/bin/env python3
"""
Visualize a SMPL-H .pt file produced by xsens_mvn_csv_to_smplh.py.
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
import torch


DEFAULT_BODY_MODEL = Path("/mnt/d/a_WORK/Projects/PhD/datasets/smpl_models/smplh/neutral/model.npz")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pt", type=Path, default=Path("output/xsens_mvn_udp/test03_smplh.pt"))
    parser.add_argument("--body-model", type=Path, default=None)
    parser.add_argument("--device", choices=["auto", "cpu", "cuda"], default="cuda")
    parser.add_argument("--batch-size", type=int, default=256)
    parser.add_argument("--max-frames", type=int, default=None)
    parser.add_argument("--frame-stride", type=int, default=1)
    parser.add_argument("--show-debug-points", action="store_true")
    parser.add_argument("--dry-run", action="store_true", help="Build meshes but do not open the viewer.")
    return parser.parse_args()


def select_device(device_arg: str) -> torch.device:
    if device_arg == "auto":
        return torch.device("cuda" if torch.cuda.is_available() else "cpu")
    if device_arg == "cuda" and not torch.cuda.is_available():
        raise RuntimeError("CUDA was requested but is not available.")
    return torch.device(device_arg)


def load_body_model(body_model_path: Path, num_betas: int, device: torch.device):
    from human_body_prior.body_model.body_model import BodyModel

    if not body_model_path.exists():
        raise FileNotFoundError(f"SMPL-H body model not found: {body_model_path}")
    model = BodyModel(bm_fname=str(body_model_path), num_betas=num_betas)
    model = model.to(device)
    model.eval()
    return model


def resolve_body_model_path(data: Dict[str, object], cli_path: Optional[Path]) -> Path:
    if cli_path is not None:
        return cli_path
    saved = data.get("body_model_path")
    if isinstance(saved, str) and saved:
        return Path(saved)
    return DEFAULT_BODY_MODEL


def as_float_tensor(data: Dict[str, object], key: str) -> torch.Tensor:
    value = data.get(key)
    if value is None:
        raise ValueError(f"Missing required key '{key}' in pt file.")
    if isinstance(value, torch.Tensor):
        return value.detach().cpu().float()
    return torch.as_tensor(value, dtype=torch.float32)


def load_smplh_pt(pt_path: Path) -> Dict[str, object]:
    if not pt_path.exists():
        raise FileNotFoundError(f"SMPL-H pt file not found: {pt_path}")
    data = torch.load(pt_path, map_location="cpu")
    if not isinstance(data, dict):
        raise ValueError(f"Expected dict pt file, got {type(data).__name__}: {pt_path}")

    if all(key in data for key in ("root_orient", "pose_body", "pose_hand")):
        root_orient = as_float_tensor(data, "root_orient")
        pose_body = as_float_tensor(data, "pose_body")
        pose_hand = as_float_tensor(data, "pose_hand")
    elif "pose" in data:
        pose = as_float_tensor(data, "pose")
        if pose.ndim != 2 or pose.shape[1] < 156:
            raise ValueError(f"Expected pose [T,156+] when split pose fields are absent, got {tuple(pose.shape)}")
        root_orient = pose[:, :3]
        pose_body = pose[:, 3:66]
        pose_hand = pose[:, 66:156]
    else:
        raise ValueError("PT file must contain either root_orient/pose_body/pose_hand or pose.")

    trans = as_float_tensor(data, "trans")
    if trans.ndim != 2 or trans.shape[1] != 3:
        raise ValueError(f"Expected trans [T,3], got {tuple(trans.shape)}")

    frame_count = root_orient.shape[0]
    if pose_body.shape != (frame_count, 63):
        raise ValueError(f"Expected pose_body [{frame_count},63], got {tuple(pose_body.shape)}")
    if pose_hand.shape != (frame_count, 90):
        raise ValueError(f"Expected pose_hand [{frame_count},90], got {tuple(pose_hand.shape)}")
    if trans.shape[0] != frame_count:
        raise ValueError(f"trans length {trans.shape[0]} does not match pose length {frame_count}")

    betas_value = data.get("betas")
    if betas_value is None:
        betas = torch.zeros(frame_count, 16, dtype=torch.float32)
    else:
        betas = torch.as_tensor(betas_value, dtype=torch.float32).detach().cpu()
        if betas.ndim == 1:
            betas = betas.unsqueeze(0).expand(frame_count, -1).clone()
        elif betas.ndim == 2 and betas.shape[0] == 1:
            betas = betas.expand(frame_count, -1).clone()
        elif betas.ndim != 2 or betas.shape[0] != frame_count:
            raise ValueError(f"Expected betas [T,B], [1,B], or [B], got {tuple(betas.shape)}")

    out = dict(data)
    out.update(
        {
            "root_orient": root_orient,
            "pose_body": pose_body,
            "pose_hand": pose_hand,
            "trans": trans,
            "betas": betas,
        }
    )
    return out


def slice_sequence(
    data: Dict[str, object],
    frame_stride: int,
    max_frames: Optional[int],
) -> Dict[str, object]:
    if frame_stride < 1:
        raise ValueError("--frame-stride must be >= 1")
    frame_count = data["root_orient"].shape[0]
    indices = torch.arange(0, frame_count, frame_stride)
    if max_frames is not None:
        indices = indices[:max_frames]
    if indices.numel() == 0:
        raise ValueError("No frames selected for visualization.")

    sliced = dict(data)
    for key in ("root_orient", "pose_body", "pose_hand", "trans", "betas"):
        sliced[key] = data[key][indices]
    for key in ("pose", "sample_counter", "time_code_ms"):
        value = data.get(key)
        if isinstance(value, torch.Tensor) and value.shape[0] == frame_count:
            sliced[key] = value[indices]
    return sliced


def forward_vertices(
    body_model,
    data: Dict[str, object],
    device: torch.device,
    batch_size: int,
) -> Tuple[np.ndarray, np.ndarray]:
    pose_body = data["pose_body"]
    pose_hand = data["pose_hand"]
    root_orient = data["root_orient"]
    trans = data["trans"]
    betas = data["betas"]

    vertices: List[np.ndarray] = []
    joints: List[np.ndarray] = []
    with torch.no_grad():
        for start in range(0, pose_body.shape[0], batch_size):
            end = min(start + batch_size, pose_body.shape[0])
            out = body_model(
                pose_body=pose_body[start:end].to(device),
                pose_hand=pose_hand[start:end].to(device),
                betas=betas[start:end].to(device),
                root_orient=root_orient[start:end].to(device),
                trans=trans[start:end].to(device),
            )
            vertices.append(out.v.detach().cpu().numpy().astype(np.float32))
            joints.append(out.Jtr.detach().cpu().numpy().astype(np.float32))
    return np.concatenate(vertices, axis=0), np.concatenate(joints, axis=0)


def visualize(
    vertices: np.ndarray,
    joints: np.ndarray,
    faces: np.ndarray,
    show_debug_points: bool,
) -> None:
    from aitviewer.renderables.meshes import Meshes
    from aitviewer.viewer import Viewer

    viewer = Viewer()
    viewer.scene.floor.enabled = False
    viewer.scene.add(Meshes(vertices, faces, color=(0.72, 0.72, 0.72, 1.0), name="Xsens MVN -> SMPL-H"))
    if show_debug_points:
        from aitviewer.renderables.point_clouds import PointClouds

        viewer.scene.add(PointClouds(joints[:, :52], name="SMPL-H joints"))
    viewer.run()


def main() -> None:
    args = parse_args()
    device = select_device(args.device)
    data = load_smplh_pt(args.pt)
    data = slice_sequence(data, frame_stride=args.frame_stride, max_frames=args.max_frames)
    body_model_path = resolve_body_model_path(data, args.body_model)
    body_model = load_body_model(body_model_path, int(data["betas"].shape[-1]), device)
    vertices, joints = forward_vertices(body_model, data, device, args.batch_size)
    faces = body_model.f.detach().cpu().numpy().astype(np.int32)

    print(f"[LOAD] {args.pt}")
    print(f"[DATA] frames={data['root_orient'].shape[0]} pose_body={tuple(data['pose_body'].shape)} pose_hand={tuple(data['pose_hand'].shape)}")
    print(f"[MESH] vertices={vertices.shape} joints={joints.shape} faces={faces.shape}")
    if args.dry_run:
        return
    visualize(vertices, joints, faces, show_debug_points=args.show_debug_points)


if __name__ == "__main__":
    main()
