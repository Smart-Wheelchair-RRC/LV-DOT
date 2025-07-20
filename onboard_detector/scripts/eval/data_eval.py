#!/usr/bin/env python3
"""
data_eval.py
============
End‑to‑end evaluation script for LV‑DOT predictions on JRDB.

It compares **both**:

1. **3‑D bounding boxes** saved by *data_collector.py*  
   (shape: (N,7) → [cx, cy, cz, width, length, height, yaw])
2. **Point‑wise masks** saved by *data_collector.py*  
   (shape: (#points,) uint8)

against the ground‑truth directories that ship with JRDB.

The directory layout must mirror *evaluate_masks.py*:

    <pred_root>/bboxes/upper_velodyne/<sequence>/<frame>.npy
    <pred_root>/masks/upper_velodyne/<sequence>/<frame>.npy
    <gt_root>/bboxes/upper_velodyne/<sequence>/<frame>.npy
    <gt_root>/masks/upper_velodyne/<sequence>/<frame>.npy

Metrics reported per‑sequence and averaged across sequences:

* **Boxes**  : IoU, Precision, Recall, F1
* **Masks**  : IoU, Precision, Recall, F1

To run -
python3 data_eval.py --pred_dir /scratch/gaurav_kumar/results --gt_dir /scratch/aadith_warrier/JRDB --box_iou_thr 0.25 --mask_thr 0

    --leg_crop 0.5   # ignore first 50 cm of legs

"""
# --------------------------------------------------------------------------- #
#   Imports                                                                  #
# --------------------------------------------------------------------------- #
import os
import argparse
import numpy as np
import sys
import json
# Alias old NumPy internal modules so pickle.load can find them
sys.modules['numpy._core'] = np.core
# Some systems name the multiarray in the old path
if hasattr(np.core, '_multiarray_umath'):
    sys.modules['numpy._core._multiarray_umath'] = np.core._multiarray_umath
from scipy.optimize import linear_sum_assignment

import csv

# --- global CLI options so helpers can see them ---
args = argparse.Namespace()

# --------------------------------------------------------------------------- #
#   Mask evaluation (ported from evaluate_masks.py)                           #
# --------------------------------------------------------------------------- #

def compute_mask_metrics(pred_mask, gt_mask, thr):
    # align lengths
    if pred_mask.shape != gt_mask.shape:
        m = min(pred_mask.shape[0], gt_mask.shape[0])
        pred_mask = pred_mask[:m]
        gt_mask   = gt_mask[:m]

    gt_bin   = (gt_mask > 0).astype(np.uint8)
    pr_bin   = (pred_mask > thr).astype(np.uint8)

    tp = np.sum(pr_bin & gt_bin)
    fp = np.sum(pr_bin & ~gt_bin)
    fn = np.sum(~pr_bin & gt_bin)

    precision = tp / (tp + fp) if (tp + fp) else 0.0
    recall    = tp / (tp + fn) if (tp + fn) else 0.0
    f1        = 2 * precision * recall / (precision + recall) if (precision + recall) else 0.0
    iou       = tp / (tp + fp + fn) if (tp + fp + fn) else 0.0
    return {'iou': iou, 'precision': precision, 'recall': recall, 'f1': f1}

def frame_boxes(json_list):
    """
    Return an (N,7) float32 array extracted from one frame’s label list.
    Skips boxes whose attributes['no_eval'] is True.
    """
    out = []
    for ann in json_list:
        if ann.get('attributes', {}).get('no_eval', False):
            continue
        b = ann['box']
        out.append([b['cx'], b['cy'], b['cz'],
                    b['w'],  b['l'],  b['h'],
                    b.get('rot_z', 0.0)])
    return np.asarray(out, np.float32)


# --------------------------------------------------------------------------- #
#   3‑D box helpers                                                            #
# --------------------------------------------------------------------------- #

def bbox_corners(box):
    """Return the eight 3‑D corners of a single oriented box (7‑tuple)."""
    cx, cy, cz, w, l, h, yaw = box
    # build local corners
    x = w / 2.0
    y = l / 2.0
    z = h / 2.0
    local = np.array(
        [[-x, -y, -z], [ x, -y, -z], [ x,  y, -z], [-x,  y, -z],
         [-x, -y,  z], [ x, -y,  z], [ x,  y,  z], [-x,  y,  z]],
        dtype=np.float32)
    c, s = np.cos(yaw), np.sin(yaw)
    R = np.array([[ c, -s, 0],
                  [ s,  c, 0],
                  [ 0,  0, 1]], dtype=np.float32)
    corners = (R @ local.T).T + np.array([cx, cy, cz], dtype=np.float32)
    return corners


def bbox_iou_3d(b1, b2):
    """
    Approximate 3‑D IoU by projecting to BEV (xy plane) with oriented rectangles,
    intersecting their z‑ranges, and dividing the volumes.
    For speed we approximate the BEV overlap by axis‑aligned rectangles that
    bound the rotated rectangles.
    """
    b1 = b1.copy(); b2 = b2.copy()
    # b1[5] *= 1.2;  b2[5] *= 1.2 
    # b1[5] *= 1.5;  b2[5] *= 1.5 
    
    # volumes
    v1 = b1[3] * b1[4] * b1[5]
    v2 = b2[3] * b2[4] * b2[5]

    # axis‑aligned bounding boxes in BEV
    c1 = bbox_corners(b1)
    c2 = bbox_corners(b2)
    min1 = c1[:, :2].min(axis=0); max1 = c1[:, :2].max(axis=0)
    min2 = c2[:, :2].min(axis=0); max2 = c2[:, :2].max(axis=0)
    inter_min = np.maximum(min1, min2)
    inter_max = np.minimum(max1, max2)
    inter_xy = np.maximum(0.0, inter_max - inter_min)
    inter_area = inter_xy[0] * inter_xy[1]

    # z‑overlap
    z1_min, z1_max = b1[2] - b1[5]/2.0, b1[2] + b1[5]/2.0
    z2_min, z2_max = b2[2] - b2[5]/2.0, b2[2] + b2[5]/2.0
    # --- leg crop ----------------------------------------------------
    crop = getattr(globals().get('args', argparse.Namespace()), 'leg_crop', 0.0)
    if crop > 0.0:
        z1_min = min(z1_min + crop, z1_max)  # ensure valid range
        z2_min = min(z2_min + crop, z2_max)
    inter_z = max(0.0, min(z1_max, z2_max) - max(z1_min, z2_min))

    inter_vol = inter_area * inter_z
    union_vol = v1 + v2 - inter_vol
    return inter_vol / union_vol if union_vol > 0 else 0.0


def match_boxes(pred, gt, iou_thr):
    """
    Greedy matching using Hungarian on IoU cost.
    Returns arrays of matches (p_idx, g_idx, iou) and lists of unmatched idxs.
    """
    if len(pred) == 0 or len(gt) == 0:
        return [], list(range(len(pred))), list(range(len(gt)))

    # compute IoU matrix
    ious = np.zeros((len(pred), len(gt)), dtype=np.float32)
    for i, pb in enumerate(pred):
        for j, gb in enumerate(gt):
            ious[i, j] = bbox_iou_3d(pb, gb)

    # Hungarian on negative IoU (maximize IoU)
    row_ind, col_ind = linear_sum_assignment(-ious)
    matches = []
    for r, c in zip(row_ind, col_ind):
        if ious[r, c] >= iou_thr:
            matches.append((r, c, ious[r, c]))

    matched_pred = {m[0] for m in matches}
    matched_gt   = {m[1] for m in matches}
    unmatched_pred = [i for i in range(len(pred)) if i not in matched_pred]
    unmatched_gt   = [j for j in range(len(gt))   if j not in matched_gt]
    return matches, unmatched_pred, unmatched_gt


def compute_box_metrics(pred_boxes, gt_boxes, iou_thr):
    matches, un_pred, un_gt = match_boxes(pred_boxes, gt_boxes, iou_thr)
    tp = len(matches)
    fp = len(un_pred)
    fn = len(un_gt)

    precision = tp / (tp + fp) if (tp + fp) else 0.0
    recall    = tp / (tp + fn) if (tp + fn) else 0.0
    f1        = 2 * precision * recall / (precision + recall) if (precision + recall) else 0.0
    mean_iou  = np.mean([m[2] for m in matches]) if matches else 0.0
    return {'iou': mean_iou, 'precision': precision, 'recall': recall, 'f1': f1}


# --------------------------------------------------------------------------- #
#   Main evaluation loop                                                      #
# --------------------------------------------------------------------------- #

def evaluate(pred_root, gt_root, box_iou_thr=0.25, mask_thr=0.0, max_range=None):
    """
    pred_root : directory containing bboxes/ and masks/ trees collected by LV‑DOT
    gt_root   : directory containing JRDB ground‑truth trees
    """
    results = {}

    # iterate sequences present in prediction tree
    pred_scenarios = sorted(os.listdir(os.path.join(pred_root, 'bboxes', 'upper_velodyne')))
    for seq in pred_scenarios:
        p_bbox_dir = os.path.join(pred_root, 'bboxes', 'upper_velodyne', seq)
        # Sequence‑level JSON file containing all frame annotations
        seq_json_path = os.path.join(gt_root, 'labels', 'labels_3d', f'{seq}.json')
        # Load sequence JSON only once
        with open(seq_json_path) as jf:
            seq_labels = json.load(jf)["labels"]  # dict{"000000.pcd":[{box...}, ...], ...}
        p_mask_dir = os.path.join(pred_root, 'masks',  'upper_velodyne', seq)
        g_mask_dir = os.path.join(gt_root,  'masks',  'upper_velodyne', seq)

        if not os.path.isdir(p_bbox_dir) or not os.path.isdir(g_mask_dir):
            print(f"[WARN] Ground‑truth missing for sequence {seq}; skipping.")
            continue

        per_file_b = []
        per_file_m = []

        # Only consider frames present in predictions and (mask) GT
        common_frames = sorted(set(f for f in os.listdir(p_bbox_dir) if f.endswith('.npy')) &
                               set(f for f in os.listdir(g_mask_dir) if f.endswith('.npy')))

        for fname in common_frames:
            # Load predicted boxes (float32 array)
            pred_boxes = np.load(os.path.join(p_bbox_dir, fname))
            # -------- ground‑truth from in‑memory JSON -----------------------
            frame_key = fname.replace('.npy', '.pcd')
            gt_boxes = frame_boxes(seq_labels.get(frame_key, []))
            
            # Optional radial crop
            if max_range is not None:
                keep_p = np.sqrt(np.sum(pred_boxes[:, :2]**2, axis=1)) <= max_range
                keep_g = np.sqrt(np.sum(gt_boxes[:, :2]**2, axis=1))   <= max_range
                pred_boxes = pred_boxes[keep_p]
                gt_boxes   = gt_boxes[keep_g]
                       
            if args.max_range is not None:
                keep_p = np.sqrt(pred_boxes[:,0]**2 + pred_boxes[:,1]**2) <= args.max_range
                keep_g = np.sqrt(gt_boxes[:,0]**2   + gt_boxes[:,1]**2)   <= args.max_range
                pred_boxes = pred_boxes[keep_p]
                gt_boxes   = gt_boxes[keep_g]
                
            b_metrics  = compute_box_metrics(pred_boxes, gt_boxes, box_iou_thr)
            per_file_b.append(b_metrics)

            pred_mask = np.load(os.path.join(p_mask_dir, fname))
            gt_mask   = np.load(os.path.join(g_mask_dir, fname))
            m_metrics = compute_mask_metrics(pred_mask, gt_mask, mask_thr)
            per_file_m.append(m_metrics)

        if not per_file_b:
            continue

        # average across frames
        avg_boxes = {k: np.mean([m[k] for m in per_file_b]) for k in per_file_b[0].keys()}
        avg_masks = {k: np.mean([m[k] for m in per_file_m]) for k in per_file_m[0].keys()}
        results[seq] = {'boxes': avg_boxes, 'masks': avg_masks}

    return results


# --------------------------------------------------------------------------- #
#   CLI                                                                       #
# --------------------------------------------------------------------------- #

if __name__ == '__main__':
    ap = argparse.ArgumentParser(description="Evaluate LV‑DOT outputs vs JRDB GT")
    ap.add_argument('--pred_dir', required=False, default= '/scratch/gaurav_kumar/results',
                    help="Base dir containing prediction trees (results)")
    ap.add_argument('--gt_dir', required=False, default= '/scratch/aadith_warrier/JRDB',
                    help="Base dir containing ground‑truth trees (JRDB)")
    ap.add_argument('--box_iou_thr', type=float, default=0.05,
                    help="IoU threshold for bbox TP (default 0.25)")
    ap.add_argument('--mask_thr', type=float, default=0.0,
                    help="Threshold for mask binarisation (default 0)")
    ap.add_argument('--max_range', type=float, default=5.0,
                help="Ignore boxes & masks beyond this radial distance (m)")
    ap.add_argument('--leg_crop', type=float, default=0.0,
                    help="Ignore this height (m) from the bottom of each "
                         "box when computing IoU (default 0.0 = use full box).")
    ap.add_argument('--max_distance', type=float, default=None,
                    help="Skip GT / pred boxes whose centre radius exceeds this (m)")
    ap.add_argument('--ego_map', default=None,
                    help="Path to CSV (seq,category) listing ego‑motion class "
                         "for each sequence; categories should be "
                         "'no', 'minimal', or 'significant'.")
    args = ap.parse_args()

    globals()['args'] = args   # make CLI flags visible to helper fns

    seq_scores = evaluate(args.pred_dir, args.gt_dir,
                          box_iou_thr=args.box_iou_thr,
                          mask_thr=args.mask_thr,
                          max_range=args.max_distance)
    if not seq_scores:
        print("No sequences evaluated – check directory paths.")
        exit(1)

    # ------------------------------------------------------------------
    #   Load ego‑motion mapping if provided
    # ------------------------------------------------------------------
    ego_map = {}
    if args.ego_map:
        with open(args.ego_map) as f:
            rdr = csv.reader(f)
            for row in rdr:
                if len(row) >= 2:
                    ego_map[row[0].strip()] = row[1].strip().lower()
    cat_boxes, cat_masks = {}, {}

    # print per‑sequence and overall means
    overall_boxes = {'iou': [], 'precision': [], 'recall': [], 'f1': []}
    overall_masks = {'iou': [], 'precision': [], 'recall': [], 'f1': []}

    for seq, res in seq_scores.items():
        b, m = res['boxes'], res['masks']
        # determine ego‑motion category if mapping provided
        category = None
        if args.ego_map and seq in ego_map:
            category = ego_map[seq]
            if category not in cat_boxes:
                cat_boxes[category] = {'iou': [], 'precision': [], 'recall': [], 'f1': []}
                cat_masks[category] = {'iou': [], 'precision': [], 'recall': [], 'f1': []}
        print(f"=== {seq} ===")
        print("  Bounding boxes")
        print(f"    IoU:       {b['iou']:.3f}")
        print(f"    Precision: {b['precision']:.3f}")
        print(f"    Recall:    {b['recall']:.3f}")
        print(f"    F1:        {b['f1']:.3f}")
        print("  Masks")
        print(f"    IoU:       {m['iou']:.3f}")
        print(f"    Precision: {m['precision']:.3f}")
        print(f"    Recall:    {m['recall']:.3f}")
        print(f"    F1:        {m['f1']:.3f}")
        print("  Moving (mask‑based)")
        print(f"    IoU:       {m['iou']:.3f}")
        print(f"    Precision: {m['precision']:.3f}")
        print(f"    Recall:    {m['recall']:.3f}")
        print(f"    F1:        {m['f1']:.3f}\n")

        for k in overall_boxes:
            overall_boxes[k].append(b[k])
            overall_masks[k].append(m[k])

        if category:
            for k in cat_boxes[category]:
                cat_boxes[category][k].append(b[k])
                cat_masks[category][k].append(m[k])

    mean_boxes = {k: np.mean(v) for k, v in overall_boxes.items()}
    mean_masks = {k: np.mean(v) for k, v in overall_masks.items()}

    print("=== OVERALL ===")
    print("Bounding boxes")
    print(f"  IoU:       {mean_boxes['iou']:.3f}")
    print(f"  Precision: {mean_boxes['precision']:.3f}")
    print(f"  Recall:    {mean_boxes['recall']:.3f}")
    print(f"  F1:        {mean_boxes['f1']:.3f}\n")
    print("Masks")
    print(f"  IoU:       {mean_masks['iou']:.3f}")
    print(f"  Precision: {mean_masks['precision']:.3f}")
    print(f"  Recall:    {mean_masks['recall']:.3f}")
    print(f"  F1:        {mean_masks['f1']:.3f}")
    print("\nMoving (mask‑based)")
    print(f"  IoU:       {mean_masks['iou']:.3f}")
    print(f"  Precision: {mean_masks['precision']:.3f}")
    print(f"  Recall:    {mean_masks['recall']:.3f}")
    print(f"  F1:        {mean_masks['f1']:.3f}")

    if cat_boxes:
        print("\n=== BY EGO‑MOTION CATEGORY ===")
        for cat in sorted(cat_boxes.keys()):
            mb = {k: np.mean(v) for k, v in cat_boxes[cat].items()}
            mm = {k: np.mean(v) for k, v in cat_masks[cat].items()}
            print(f"[{cat.upper():>11}]  Boxes  IoU {mb['iou']:.3f}  P {mb['precision']:.3f}  R {mb['recall']:.3f}  F1 {mb['f1']:.3f} │ "
                  f"Masks  IoU {mm['iou']:.3f}  P {mm['precision']:.3f}  R {mm['recall']:.3f}  F1 {mm['f1']:.3f}")