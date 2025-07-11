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

def bbox_to_wireframe(bbox):
    """
    Convert a 3D bbox [cx, cy, cz, w, l, h, yaw] to wireframe edges for visualization.
    
    Returns:
        vertices: (8, 3) array of corner points
        edges: (12, 2) array of edge indices connecting corners
    """
    cx, cy, cz, w, l, h, yaw = bbox
    
    # Create local corners (8 vertices of a box)
    x = w / 2.0
    y = l / 2.0  
    z = h / 2.0
    local_corners = np.array([
        [-x, -y, -z], [x, -y, -z], [x, y, -z], [-x, y, -z],  # bottom face
        [-x, -y, z],  [x, -y, z],  [x, y, z],  [-x, y, z]    # top face
    ], dtype=np.float32)
    
    # Apply rotation around z-axis
    cos_yaw, sin_yaw = np.cos(yaw), np.sin(yaw)
    rotation_matrix = np.array([
        [cos_yaw, -sin_yaw, 0],
        [sin_yaw,  cos_yaw, 0],
        [0,        0,       1]
    ], dtype=np.float32)
    
    # Rotate and translate corners
    rotated_corners = (rotation_matrix @ local_corners.T).T
    global_corners = rotated_corners + np.array([cx, cy, cz], dtype=np.float32)
    
    # Define edges connecting the corners (wireframe)
    edges = np.array([
        # Bottom face edges
        [0, 1], [1, 2], [2, 3], [3, 0],
        # Top face edges  
        [4, 5], [5, 6], [6, 7], [7, 4],
        # Vertical edges connecting bottom to top
        [0, 4], [1, 5], [2, 6], [3, 7]
    ], dtype=np.int32)
    
    return global_corners, edges

def visualize_mask_sequence(
    mask_dir: str,
    pcd_dir: str,
    bbox_dir: str = None,
    recording_id: str = None
) -> None:
    """
    Streams a mask sequence into Rerun as colored point clouds with optional 3D bounding boxes.

    Args:
        mask_dir: directory containing 000000.npy, 000001.npy, … masks (bool arrays).
        pcd_dir:  directory containing matching point-cloud .npy files with shape (N,3).
        bbox_dir: optional directory containing bbox .npy files with shape (N,7) [cx,cy,cz,w,l,h,yaw].
        recording_id: optional UUID to group your run.
    """
        
    rr.init("mask_sequence_vis", recording_id=recording_id)
    rr.connect_tcp("127.0.0.1:9876")
    # rr.connect_tcp("10.2.135.228:9090")
    rr.spawn()  # Launch viewer if not already running

    mask_files = sorted(glob.glob(os.path.join(mask_dir, "*.npy")))
    for frame_idx, mask_path in enumerate(mask_files):
        rr.set_time_sequence("frame", frame_idx)  # advance timeline before logging
        
        # Extract frame number from filename for consistent loading
        frame_name = os.path.splitext(os.path.basename(mask_path))[0]
        
        # Load mask and point cloud
        mask = np.load(mask_path).astype(bool)                # shape: (N,)
        pcd_file = os.path.join(pcd_dir, f"{frame_name}.pcd")
        
        # Load point cloud from .pcd file
        if HAS_OPEN3D:
            pcd = o3d.io.read_point_cloud(pcd_file)
            points = np.asarray(pcd.points)                   # shape: (N,3)
        else:
            # Fallback: try to read as .npy if open3d not available
            npy_file = os.path.join(pcd_dir, f"{frame_name}.npy")
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
                print(f"Info: Frame {frame_name} - truncated mask from {original_mask_size} to {len(points)} to match points")
            else:
                # Point cloud is larger than mask - pad mask with False values
                mask_padded = np.zeros(len(points), dtype=bool)
                mask_padded[:len(mask)] = mask
                mask = mask_padded
                print(f"Info: Frame {frame_name} - padded mask from {original_mask_size} to {len(points)} to match points")

        # Color masked points red, others grey
        colors = np.zeros((points.shape[0], 3), dtype=np.uint8)
        colors[mask] = np.array([255, 0, 0], dtype=np.uint8)  # red for masked
        colors[~mask] = np.array([128, 128, 128], dtype=np.uint8)  # grey otherwise

        # Log this frame's points
        rr.log(
            "dynamic_objects/points",        # entity path in Rerun
            rr.Points3D(
                positions=points,     # Nx3 world positions
                colors=colors        # Nx3 uint8 colors
            )
        )
        
        # Load and visualize bounding boxes if bbox_dir is provided
        if bbox_dir:
            bbox_file = os.path.join(bbox_dir, f"{frame_name}.npy")
            if os.path.exists(bbox_file):
                try:
                    bboxes = np.load(bbox_file, allow_pickle=True)
                    if len(bboxes) > 0:
                        # Ensure bboxes is 2D array with shape (N, 7)
                        if bboxes.ndim == 1:
                            bboxes = bboxes.reshape(1, -1)
                        
                        # Create wireframe for each bbox
                        all_vertices = []
                        all_edges = []
                        vertex_offset = 0
                        
                        for bbox in bboxes:
                            if len(bbox) >= 7:  # Ensure we have [cx, cy, cz, w, l, h, yaw]
                                vertices, edges = bbox_to_wireframe(bbox[:7])
                                all_vertices.append(vertices)
                                # Adjust edge indices for concatenated vertices
                                adjusted_edges = edges + vertex_offset
                                all_edges.append(adjusted_edges)
                                vertex_offset += len(vertices)
                        
                        if all_vertices:
                            # Concatenate all vertices and edges
                            combined_vertices = np.vstack(all_vertices)
                            combined_edges = np.vstack(all_edges)
                            
                            # Create line segments for wireframe visualization
                            line_segments = []
                            for edge in combined_edges:
                                line_segments.append(combined_vertices[edge])
                            
                            # Log wireframe bboxes
                            rr.log(
                                "dynamic_objects/bboxes",
                                rr.LineStrips3D(
                                    strips=line_segments,
                                    colors=[0, 255, 0, 255]  # Green wireframes
                                )
                            )
                except Exception as e:
                    print(f"Warning: Could not load bboxes for frame {frame_name}: {e}")

    # Optionally, save out a .rrd recording for later playback
    rr.save("mask_sequence.rrd")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Visualize mask+pointcloud sequence with optional bboxes in Rerun")
    parser.add_argument('--mask_dir', required=True,
                        help="Directory of mask .npy files (e.g. /scratch/.../<sequence>)")
    parser.add_argument('--pcd_dir', required=True,
                        help="Directory of point-cloud .npy files matching mask files.")
    parser.add_argument('--bbox_dir', 
                        help="Optional directory of bbox .npy files with shape (N,7) [cx,cy,cz,w,l,h,yaw]")
    args = parser.parse_args()

    # Verify directories
    mask_files = sorted(glob.glob(os.path.join(args.mask_dir, "*.npy")))
    if not mask_files:
        raise RuntimeError(f"No .npy files found in mask_dir: {args.mask_dir}")

    visualize_mask_sequence(args.mask_dir, args.pcd_dir, args.bbox_dir)