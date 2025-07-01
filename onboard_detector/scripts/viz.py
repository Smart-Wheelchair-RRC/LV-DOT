#!/usr/bin/env python3
import os
import glob
import numpy as np
import rerun as rr  # install with: pip3 install rerun-sdk  [oai_citation:0‡pypi.org](https://pypi.org/project/rerun-sdk/0.4.0/?utm_source=chatgpt.com)
import argparse
try:
    import open3d as o3d
    HAS_OPEN3D = True
except ImportError:
    HAS_OPEN3D = False

def visualize_mask_sequence(
    mask_dir: str,
    pcd_dir: str,
    recording_id: str = None
) -> None:
    """
    Streams a mask sequence into Rerun as colored point clouds.

    Args:
        mask_dir: directory containing 000000.npy, 000001.npy, … masks (bool arrays).
        pcd_dir:  directory containing matching point-cloud .npy files with shape (N,3).
        recording_id: optional UUID to group your run.
    """
        
    rr.init("mask_sequence_vis", recording_id=recording_id)
    rr.connect_tcp("127.0.0.1:9876")
    rr.spawn()  # Launch viewer if not already running

    mask_files = sorted(glob.glob(os.path.join(mask_dir, "*.npy")))
    for frame_idx, mask_path in enumerate(mask_files):
        rr.set_time_sequence("frame", frame_idx)  # advance timeline before logging
        # Load mask and point cloud
        mask = np.load(mask_path).astype(bool)                # shape: (N,)
        pcd_file = os.path.join(pcd_dir, f"{frame_idx:06d}.pcd")
        
        # Load point cloud from .pcd file
        if HAS_OPEN3D:
            pcd = o3d.io.read_point_cloud(pcd_file)
            points = np.asarray(pcd.points)                   # shape: (N,3)
        else:
            # Fallback: try to read as .npy if open3d not available
            npy_file = os.path.join(pcd_dir, f"{frame_idx:06d}.npy")
            if os.path.exists(npy_file):
                points = np.load(npy_file)
            else:
                print(f"Error: Cannot read {pcd_file}. Install open3d: pip install open3d")
                continue

        # Handle size mismatch by truncating mask to match point cloud size
        if len(mask) != len(points):
            original_mask_size = len(mask)
            if len(mask) > len(points):
                # Truncate mask to match point cloud size
                mask = mask[:len(points)]
                print(f"Info: Frame {frame_idx:06d} - truncated mask from {original_mask_size} to {len(points)} to match points")
            else:
                # Point cloud is larger than mask - pad mask with False values
                mask_padded = np.zeros(len(points), dtype=bool)
                mask_padded[:len(mask)] = mask
                mask = mask_padded
                print(f"Info: Frame {frame_idx:06d} - padded mask from {original_mask_size} to {len(points)} to match points")

        # Color masked points red, others grey
        colors = np.zeros((points.shape[0], 3), dtype=np.uint8)
        colors[mask] = np.array([255, 0, 0], dtype=np.uint8)  # red for masked
        colors[~mask] = np.array([128, 128, 128], dtype=np.uint8)  # grey otherwise

        # Log this frame's points
        rr.log(
            "dynamic_objects",        # entity path in Rerun
            rr.Points3D(
                positions=points,     # Nx3 world positions
                colors=colors        # Nx3 uint8 colors
            )
        )

    # Optionally, save out a .rrd recording for later playback
    rr.save("mask_sequence.rrd")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Visualize mask+pointcloud sequence in Rerun")
    parser.add_argument('--mask_dir', required=True,
                        help="Directory of mask .npy files (e.g. /scratch/.../<sequence>)")
    parser.add_argument('--pcd_dir', required=True,
                        help="Directory of point-cloud .npy files matching mask files.")
    args = parser.parse_args()

    # Verify directories
    mask_files = sorted(glob.glob(os.path.join(args.mask_dir, "*.npy")))
    if not mask_files:
        raise RuntimeError(f"No .npy files found in mask_dir: {args.mask_dir}")

    visualize_mask_sequence(args.mask_dir, args.pcd_dir)