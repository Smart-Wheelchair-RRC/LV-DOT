#!/usr/bin/env python3
import os
import glob
import math
import argparse

import numpy as np
import rerun as rr  # pip install rerun-sdk

def visualize_mask_sequence(mask_dir: str, recording_id: str = None):
    """
    Streams a mask sequence (000000.npy, 000001.npy, …) into Rerun
    as grayscale images (white = masked, black = background).
    """
    rr.init("mask_only_vis", recording_id=recording_id)
    rr.spawn()  # launch the viewer if not already running

    mask_files = sorted(glob.glob(os.path.join(mask_dir, "*.npy")))
    if not mask_files:
        raise RuntimeError(f"No .npy files found in {mask_dir}")

    for frame_idx, mask_path in enumerate(mask_files):
        # Advance the timeline for 'mask_sequence' to this frame
        rr.set_time_sequence("mask_sequence", frame_idx)

        mask = np.load(mask_path).astype(bool)      # shape: (N,)
        N = mask.shape[0]
        # pack into the smallest square
        side = math.ceil(math.sqrt(N))
        img = np.zeros((side, side), dtype=np.uint8)
        img.flat[:N] = mask.astype(np.uint8) * 255

        # Log a single-channel grayscale image
        rr.log(
            "mask_image",    # entity path in Rerun
            rr.Image(img)     # H x W uint8 image
        )

    # save recording for later
    rr.save("mask_sequence_only.rrd")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Visualize mask-only sequence in Rerun"
    )
    parser.add_argument(
        "--mask_dir", required=True,
        help="Directory of mask .npy files (e.g. /scratch/.../<sequence>)"
    )
    args = parser.parse_args()

    visualize_mask_sequence(args.mask_dir)