#!/usr/bin/env python3
"""
viz_2.py  – Ground‑truth vs Prediction mask visualiser
======================================================

Streams **both** JRDB ground‑truth masks and LV‑DOT predicted masks into a
single Rerun recording so you can inspect frame‑by‑frame agreement.

For each frame *f* present in *both* directories it logs

* `/gt/mask_image`   – ground‑truth, grayscale (white = masked)
* `/pred/mask_image` – prediction,  grayscale (white = masked)

Usage
-----
python3 viz_2.py \
    --gt_mask_dir   /path/to/JRDB/masks/upper_velodyne/<sequence> \
    --pred_mask_dir /path/to/results/masks/upper_velodyne/<sequence>

Optional:
    --recording_id <str>   Supply a custom Rerun recording id
    --max_range <float>    Optional radial distance crop (metres) applied to PCD + masks
"""

from __future__ import annotations
import os
import glob
import math
import argparse
import numpy as np, pprint, sys
from typing import Optional
import rerun as rr  # pip install rerun-sdk
import math

import json

# Colors (uint8 RGB)
_GT_COLOR   = np.array([  0,  92, 255], np.uint8)   # blue‑ish for GT
_PRED_COLOR = np.array([255,  64,  64], np.uint8)   # red‑ish for preds

# Helper for loading boxes from JSON annotation list
def frame_boxes_json(json_list):
    """
    Convert a list of annotation dicts to an (N,7) array of boxes,
    deduplicating by 'id' if present.
    """
    boxes = []
    seen_ids = set()
    for ann in json_list:
        obj_id = ann.get("id")
        if obj_id is not None:
            if obj_id in seen_ids:
                continue
            seen_ids.add(obj_id)

        b = ann.get("box", {})
        cx, cy, cz = b.get("cx", 0.0), b.get("cy", 0.0), b.get("cz", 0.0)
        length, width, height = b.get("l", 0.0), b.get("w", 0.0), b.get("h", 0.0)
        yaw_rad = math.radians(b.get("rot_z", 0.0))
        boxes.append([cx, cy, cz, width, length, height, yaw_rad])

    return np.asarray(boxes, np.float32)

# Alias old NumPy internal modules so pickle.load can find them
import sys as _sys
_sys.modules['numpy._core'] = np.core
if hasattr(np.core, '_multiarray_umath'):
    _sys.modules['numpy._core._multiarray_umath'] = np.core._multiarray_umath



# --------------------------------------------------------------------------- #
#   Helpers                                                                   #
# --------------------------------------------------------------------------- #
def _collect_npy(dir_path: str) -> dict[str, str]:
    """Return dict{frame_id -> file_path} for every *.npy in *dir_path*."""
    files = glob.glob(os.path.join(dir_path, "*.npy"))
    return {os.path.splitext(os.path.basename(p))[0]: p for p in files}


def _mask_to_square(mask: np.ndarray) -> np.ndarray:
    """
    Pack a 1‑D boolean mask into the smallest square grayscale uint8 image.
    Background = 0, mask = 255.
    """
    mask = mask.astype(bool).flatten()
    N = mask.size
    side = math.ceil(math.sqrt(N))
    img = np.zeros((side, side), dtype=np.uint8)
    img.flat[:N] = mask * 255
    return img


def _yaw_to_quat(yaw: np.ndarray) -> np.ndarray:
    """Convert yaw angles (in radians) to quaternions [w,x,y,z]."""
    return np.column_stack([
        np.cos(yaw * 0.5),  # w
        np.zeros_like(yaw),  # x
        np.zeros_like(yaw),  # y
        np.sin(yaw * 0.5),  # z
    ])

# --------------------------------------------------------------------------- #
#   Main visualisation                                                        #
# --------------------------------------------------------------------------- #
def visualise(
    gt_mask_dir: str,
    pred_mask_dir: str,
    rec_id: Optional[str],
    gt_bbox_dir: Optional[str] = None,
    pred_bbox_dir: Optional[str] = None,
    pcd_dir: Optional[str] = None,
    gt_labels_json: Optional[str] = None,
    max_range: Optional[float] = None,
):
    gt_mask_files = _collect_npy(gt_mask_dir)
    pr_mask_files = _collect_npy(pred_mask_dir)
    gt_box_files  = _collect_npy(gt_bbox_dir)  if gt_bbox_dir  else {}
    pr_box_files  = _collect_npy(pred_bbox_dir) if pred_bbox_dir else {}
        # Optional: load a single JRDB labels_3d JSON file
    seq_labels = {}
    if gt_labels_json and os.path.isfile(gt_labels_json):
        with open(gt_labels_json) as jf:
            seq_labels = json.load(jf).get("labels", {})  # dict{"000000.pcd":[{box…}, …], …}
    
    # Optional: point cloud directory for 3‑D mask overlay
    get_pcd = (lambda fid: None)
    if pcd_dir:
        def _load_xyz(fid):
            pcd_path_npy = os.path.join(pcd_dir, f"{fid}.npy")
            if os.path.isfile(pcd_path_npy):
                return np.load(pcd_path_npy).astype(np.float32)  # (N,3)
            pcd_path_pcd = os.path.join(pcd_dir, f"{fid}.pcd")
            if os.path.isfile(pcd_path_pcd):
                # fallback to .pcd via Open3D if available
                try:
                    import open3d as o3d
                    return np.asarray(o3d.io.read_point_cloud(pcd_path_pcd).points, np.float32)
                except Exception:
                    pass
            return None
        get_pcd = _load_xyz
    
    # Initialize Rerun recording before logging or saving
    recording_id = rec_id if rec_id is not None else "mask_compare"
    rr.init(recording_id, spawn=True)

    # evaluate only frames that have *both* masks
    common = sorted(set(gt_mask_files) & set(pr_mask_files))
    if not common:
        raise RuntimeError("No overlapping frame ids between GT and predictions.")

    for frame_idx, fid in enumerate(common):
        rr.set_time_sequence("frame", frame_idx)

        gt_mask   = np.load(gt_mask_files[fid])
        pred_mask = np.load(pr_mask_files[fid])

        rr.log("/gt/mask_image",   rr.Image(_mask_to_square(gt_mask)))
        rr.log("/pred/mask_image", rr.Image(_mask_to_square(pred_mask)))

        # --- 3-D overlay of mask points --------------------------------
        xyz = get_pcd(fid)
        if xyz is not None:
            N_pts        = xyz.shape[0]
            N_mask_gt    = gt_mask.size
            N_mask_pred  = pred_mask.size
            # Always truncate all three arrays to the same minimum length
            min_len = min(N_pts, N_mask_gt, N_mask_pred)
            if min_len < N_pts or min_len < N_mask_gt or min_len < N_mask_pred:
                xyz       = xyz[:min_len]
                gt_mask   = gt_mask.flat[:min_len].reshape(-1)
                pred_mask = pred_mask.flat[:min_len].reshape(-1)
                if frame_idx == 0:
                    print(f"[WARN] Frame {fid}: truncated PCD ({N_pts}), GT mask ({N_mask_gt}), "
                          f"Pred mask ({N_mask_pred}) to {min_len}")

            # Optional radial crop (e.g. ≤5 m) so visualisation matches evaluation
            if max_range is not None and xyz is not None:
                radial = np.linalg.norm(xyz[:, :2], axis=1)
                keep   = radial <= max_range
                xyz        = xyz[keep]
                gt_mask    = gt_mask[keep]
                pred_mask  = pred_mask[keep]
                if frame_idx == 0:
                    print(f"[INFO] Applied max_range={max_range} m – kept {keep.sum()} / {keep.size} pts")

            # Now safe to index once:
            gt_pts = xyz[gt_mask.astype(bool)]
            pr_pts = xyz[pred_mask.astype(bool)]
            if gt_pts.size:
                rr.log("/gt/mask_points", rr.Points3D(positions=gt_pts, colors=_GT_COLOR, radii=0.03))
            if pr_pts.size:
                rr.log("/pred/mask_points", rr.Points3D(positions=pr_pts, colors=_PRED_COLOR, radii=0.03))

        # --- 3-D bounding boxes -------------------------------------------
        # Fetch GT and prediction paths
        gt_path = gt_box_files.get(fid, "")
        pr_path = pr_box_files.get(fid, "")

        # Load ground-truth boxes (from JSON or .npy)
        if gt_labels_json:
            gt_boxes = frame_boxes_json(seq_labels.get(f"{fid}.pcd", []))
        else:
            if gt_path.endswith('.npy'):
                data = np.load(gt_path, allow_pickle=True)
                gt_boxes = frame_boxes_json(data.tolist()) if data.dtype == object else data.astype(np.float32)
            elif gt_path.endswith('.json'):
                with open(gt_path) as f:
                    gt_boxes = frame_boxes_json(json.load(f))
            else:
                gt_boxes = np.zeros((0,7), np.float32)
        # Log GT boxes
        if gt_boxes.ndim == 2 and gt_boxes.shape[1] == 7 and gt_boxes.size:
            rr.log(
                "/gt/boxes3d",
                rr.Boxes3D(
                    centers=gt_boxes[:, :3],
                    half_sizes=gt_boxes[:, 3:6] * 0.5,
                    rotations=_yaw_to_quat(gt_boxes[:, 6]),
                    colors=_GT_COLOR,  # blue for GT
                ),
            )

        # Load predicted boxes
        if pr_path.endswith('.npy'):
            data = np.load(pr_path, allow_pickle=True)
            pr_boxes = frame_boxes_json(data.tolist()) if data.dtype == object else data.astype(np.float32)
        elif pr_path.endswith('.json'):
            with open(pr_path) as f:
                pr_json = json.load(f)
            pr_boxes = frame_boxes_json(pr_json)
        else:
            pr_boxes = np.zeros((0,7), np.float32)
        # Log predicted boxes
        if pr_boxes.ndim == 2 and pr_boxes.shape[1] == 7 and pr_boxes.size:
            rr.log(
                "/pred/boxes3d",
                rr.Boxes3D(
                    centers=pr_boxes[:, :3],
                    half_sizes=pr_boxes[:, 3:6] * 0.5,
                    rotations=_yaw_to_quat(pr_boxes[:, 6]),
                    colors=_PRED_COLOR,  # red for preds
                ),
            )

    rr.save("mask_compare.rrd")


# --------------------------------------------------------------------------- #
#   CLI                                                                       #
# --------------------------------------------------------------------------- #
if __name__ == "__main__":
    ap = argparse.ArgumentParser(
        description="Visualise ground‑truth & predicted masks side‑by‑side in Rerun"
    )
    ap.add_argument("--gt_mask_dir", required=True,
                    help="Directory of JRDB ground‑truth mask .npy files")
    ap.add_argument("--pred_mask_dir", required=True,
                    help="Directory of LV‑DOT predicted mask .npy files")
    ap.add_argument("--recording_id", default=None,
                     help="Optional custom Rerun recording id")
    ap.add_argument("--gt_bbox_dir", required=False,
                    help="Directory of GT 3-D bbox .npy files")
    ap.add_argument("--pred_bbox_dir", required=False,
                    help="Directory of predicted 3-D bbox .npy files")
    ap.add_argument("--pcd_dir", required=False,
                    help="Directory containing point‑cloud *.npy or *.pcd files for 3‑D mask overlay")
    ap.add_argument("--gt_labels_json", required=False,
                help="Path to JRDB labels_3d/<sequence>.json (single file containing all GT boxes)")
    ap.add_argument("--max_range", type=float, default=None,
                    help="Optional radial distance crop (metres) applied to PCD + masks")
    args = ap.parse_args()

    visualise(
        args.gt_mask_dir,
        args.pred_mask_dir,
        args.recording_id,
        args.gt_bbox_dir,
        args.pred_bbox_dir,
        args.pcd_dir,
        args.gt_labels_json,
        args.max_range,
    )