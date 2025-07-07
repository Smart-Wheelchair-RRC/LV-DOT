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
python3 data_eval.py --pred_dir /scratch/gaurav_kumar/result --gt_dir  /scratch/aadith_warrier/JRDB --box_iou_thr 0.25 --mask_thr 0 --cfg_yaml /scratch/gaurav_kumar/lvdot_testing/src/LV-DOT/onboard_detector/cfg/custom_param.yaml
python3 data_eval.py --pred_dir /scratch/gaurav_kumar/results --gt_dir /scratch/aadith_warrier/JRDB --box_iou_thr 0.25 --mask_thr 0 (old)

"""
import os
import argparse
import numpy as np
import sys
import yaml
# Alias old NumPy internal modules so pickle.load can find them
sys.modules['numpy._core'] = np.core
# Some systems name the multiarray in the old path
if hasattr(np.core, '_multiarray_umath'):
    sys.modules['numpy._core._multiarray_umath'] = np.core._multiarray_umath
from scipy.optimize import linear_sum_assignment

# --------------------------------------------------------------------------- #
#   Helper: Load max evaluation range from yaml                               #
# --------------------------------------------------------------------------- #
def load_max_range(cfg_yaml):
    """
    Parse custom_param.yaml (or another yaml) to obtain the maximum
    distance (in metres) within which boxes should be evaluated.
    The function looks for, in order of preference:
        - field 'raycastMaxLength' (float)
        - field 'depth_max_value'
        - field 'max_eval_range'
    If none are found, returns None, meaning *no range filter*.
    """
    if cfg_yaml is None:
        return None
    try:
        with open(cfg_yaml, 'r') as f:
            cfg = yaml.safe_load(f)
    except (FileNotFoundError, yaml.YAMLError):
        return None

    for key in ('raycastMaxLength', 'depth_max_value', 'max_eval_range'):
        if key in cfg and isinstance(cfg[key], (int, float)):
            return float(cfg[key])
    return None


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


def bbox_iou_xy(b1, b2):
    """
    2‑D IoU in the XY plane (BEV).  We ignore height to compensate for the
    fact that LV‑DOT boxes cover only the upper body.
    """
    c1, c2 = bbox_corners(b1), bbox_corners(b2)
    min1 = c1[:, :2].min(axis=0); max1 = c1[:, :2].max(axis=0)
    min2 = c2[:, :2].min(axis=0); max2 = c2[:, :2].max(axis=0)

    inter_min = np.maximum(min1, min2)
    inter_max = np.minimum(max1, max2)
    wh = np.maximum(0.0, inter_max - inter_min)
    inter_area = wh[0] * wh[1]

    area1 = (max1[0] - min1[0]) * (max1[1] - min1[1])
    area2 = (max2[0] - min2[0]) * (max2[1] - min2[1])
    union_area = area1 + area2 - inter_area
    return inter_area / union_area if union_area > 0 else 0.0


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
            ious[i, j] = bbox_iou_xy(pb, gb)

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
        g_bbox_dir = os.path.join(gt_root,  'bboxes', 'upper_velodyne', seq)
        p_mask_dir = os.path.join(pred_root, 'masks',  'upper_velodyne', seq)
        g_mask_dir = os.path.join(gt_root,  'masks',  'upper_velodyne', seq)

        if not os.path.isdir(g_bbox_dir) or not os.path.isdir(g_mask_dir):
            print(f"[WARN] Ground‑truth missing for sequence {seq}; skipping.")
            continue

        per_file_b = []
        per_file_m = []

        common_frames = sorted(set(f for f in os.listdir(p_bbox_dir) if f.endswith('.npy')) &
                               set(f for f in os.listdir(g_bbox_dir) if f.endswith('.npy')))

        for fname in common_frames:
            # Load predicted boxes (float32 array)
            pred_boxes = np.load(os.path.join(p_bbox_dir, fname))
            # Load ground-truth boxes (may be object array), allow pickled list loading
            gt_raw = np.load(os.path.join(g_bbox_dir, fname), allow_pickle=True)
            
            # Handle different ground truth data formats
            if isinstance(gt_raw, np.ndarray) and gt_raw.dtype == object:
                # Check if it's an array of dictionaries or lists
                if len(gt_raw) > 0:
                    if isinstance(gt_raw[0], dict):
                        # Extract bbox values from dictionaries
                        # Assuming dict has keys like 'box' or direct coordinate keys
                        gt_list = []
                        for item in gt_raw:
                            if 'box' in item:
                                gt_list.append(item['box'])
                            elif all(k in item for k in ['cx', 'cy', 'cz', 'w', 'l', 'h', 'yaw']):
                                gt_list.append([item['cx'], item['cy'], item['cz'], 
                                              item['w'], item['l'], item['h'], item['yaw']])
                            elif all(k in item for k in ['x', 'y', 'z', 'width', 'length', 'height', 'yaw']):
                                gt_list.append([item['x'], item['y'], item['z'], 
                                              item['width'], item['length'], item['height'], item['yaw']])
                            else:
                                # Try to extract numeric values in order
                                values = [v for v in item.values() if isinstance(v, (int, float))]
                                if len(values) >= 7:
                                    gt_list.append(values[:7])
                        gt_boxes = np.array(gt_list, dtype=np.float32) if gt_list else np.empty((0, 7), dtype=np.float32)
                    else:
                        # Assume it's an array of lists/arrays
                        gt_boxes = np.vstack(gt_raw).astype(np.float32)
                else:
                    gt_boxes = np.empty((0, 7), dtype=np.float32)
            else:
                gt_boxes = gt_raw.astype(np.float32)
                
            # ------ Ensure arrays have shape (N,7); else make them empty ----------
            def to_Nx7(arr):
                arr = np.asarray(arr, dtype=np.float32)
                if arr.size == 0:
                    return np.empty((0, 7), np.float32)
                if arr.ndim == 1:                       # flat vector
                    if arr.size == 7:
                        return arr.reshape(1, 7)
                    if arr.size % 7 == 0:
                        return arr.reshape(-1, 7)
                    return np.empty((0, 7), np.float32)
                # if it’s 2-D but wrong width, try flatten-reshape
                if arr.ndim == 2 and arr.shape[1] != 7:
                    flat = arr.ravel()
                    if flat.size % 7 == 0:
                        return flat.reshape(-1, 7)
                    return np.empty((0, 7), np.float32)
                return arr

            pred_boxes = to_Nx7(pred_boxes)
            gt_boxes   = to_Nx7(gt_boxes)    
            # -------- Range filter (centre‑distance) --------------------
            if max_range is not None:
                keep_pred = np.sqrt(pred_boxes[:,0]**2 + pred_boxes[:,1]**2) <= max_range
                keep_gt   = np.sqrt(gt_boxes[:,0]**2   + gt_boxes[:,1]**2)   <= max_range
                pred_boxes = pred_boxes[keep_pred]
                gt_boxes   = gt_boxes[keep_gt]
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
    ap.add_argument('--pred_dir', required=True,
                    help="Base dir containing prediction trees (results)")
    ap.add_argument('--gt_dir', required=True,
                    help="Base dir containing ground‑truth trees (JRDB)")
    ap.add_argument('--box_iou_thr', type=float, default=0.25,
                    help="IoU threshold for bbox TP (default 0.25)")
    ap.add_argument('--mask_thr', type=float, default=0.0,
                    help="Threshold for mask binarisation (default 0)")
    ap.add_argument('--cfg_yaml', default=None,
                    help="custom_param.yaml path (for max eval range)")
    args = ap.parse_args()

    max_range = load_max_range(args.cfg_yaml)

    seq_scores = evaluate(args.pred_dir, args.gt_dir,
                          box_iou_thr=args.box_iou_thr,
                          mask_thr=args.mask_thr,
                          max_range=max_range)

    if not seq_scores:
        print("No sequences evaluated – check directory paths.")
        exit(1)

    # print per‑sequence and overall means
    overall_boxes = {'iou': [], 'precision': [], 'recall': [], 'f1': []}
    overall_masks = {'iou': [], 'precision': [], 'recall': [], 'f1': []}

    for seq, res in seq_scores.items():
        b, m = res['boxes'], res['masks']
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
        print(f"    F1:        {m['f1']:.3f}\n")

        for k in overall_boxes:
            overall_boxes[k].append(b[k])
            overall_masks[k].append(m[k])

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