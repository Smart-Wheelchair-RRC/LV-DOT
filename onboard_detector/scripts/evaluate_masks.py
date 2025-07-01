#!/usr/bin/env python3
import os
import numpy as np
import argparse

def compute_metrics(pred_mask, gt_mask, thr):
    # align lengths
    if pred_mask.shape != gt_mask.shape:
        min_len = min(pred_mask.shape[0], gt_mask.shape[0])
        pred_mask = pred_mask[:min_len]
        gt_mask   = gt_mask[:min_len]

    gt_binary   = (gt_mask > 0).astype(np.uint8)
    pred_binary = (pred_mask > thr).astype(np.uint8)

    tp = np.sum(pred_binary & gt_binary)
    fp = np.sum(pred_binary & ~gt_binary)
    fn = np.sum(~pred_binary & gt_binary)

    precision = tp / (tp + fp) if (tp + fp) > 0 else 0.0
    recall    = tp / (tp + fn) if (tp + fn) > 0 else 0.0
    f1        = 2 * precision * recall / (precision + recall) if (precision + recall) > 0 else 0.0
    iou       = tp / (tp + fp + fn) if (tp + fp + fn) > 0 else 0.0

    return {'iou': iou, 'precision': precision, 'recall': recall, 'f1': f1}

def evaluate(pred_root, gt_root):
    results = {}
    for scenario in sorted(os.listdir(pred_root)):
        pred_scn = os.path.join(pred_root, scenario)
        gt_scn   = os.path.join(gt_root,   scenario)
        if not os.path.isdir(pred_scn) or not os.path.isdir(gt_scn):
            continue

        per_file_metrics = []
        pred_fnames = [f for f in os.listdir(pred_scn) if f.endswith('.npy')]
        gt_fnames   = [f for f in os.listdir(gt_scn)   if f.endswith('.npy')]

        common = sorted(set(pred_fnames) & set(gt_fnames))
        missing_pred = sorted(set(gt_fnames) - set(pred_fnames))
        missing_gt   = sorted(set(pred_fnames) - set(gt_fnames))

        if missing_pred:
            print(f"[WARN] {scenario}: {len(missing_pred)} GT frames have no prediction")
        if missing_gt:
            print(f"[WARN] {scenario}: {len(missing_gt)} predicted frames missing in GT")

        for fname in common:
            pred_mask = np.load(os.path.join(pred_scn, fname))
            gt_mask   = np.load(os.path.join(gt_scn,   fname))

            metrics = compute_metrics(pred_mask, gt_mask, args.thr)
            per_file_metrics.append(metrics)

        if not per_file_metrics:
            continue

        # average over files
        avg = {k: np.mean([m[k] for m in per_file_metrics]) 
               for k in per_file_metrics[0].keys()}
        results[scenario] = avg

    return results

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description="Evaluate predicted vs. GT masks per scenario")
    parser.add_argument('--pred_dir', required=True,
                        help="e.g. /scratch/gaurav_kumar/results/masks/upper_velodyne")
    parser.add_argument('--gt_dir', required=True,
                        help="e.g. /scratch/aadith_warrier/JRDB/masks/upper_velodyne")
    parser.add_argument('--thr', type=float, default=0.0,
                        help="Threshold for binary mask (default 0)")
    args = parser.parse_args()

    scores = evaluate(args.pred_dir, args.gt_dir)
    for scenario, m in scores.items():
        print(f"=== {scenario} ===")
        print(f"IoU:       {m['iou']:.3f}")
        print(f"Precision: {m['precision']:.3f}")
        print(f"Recall:    {m['recall']:.3f}")
        print(f"F1:        {m['f1']:.3f}\n")