#!/usr/bin/env python3
"""
data_collector.py
=================
Collect **final** LV‑DOT outputs for quantitative benchmarking against JRDB.

Subscribed Topics
-----------------
/onboard_detector/dynamic_bboxes     visualization_msgs/MarkerArray
/livox/pcd                          sensor_msgs/PointCloud2

Usage
-----
rosrun onboard_detector data_collector.py 
    _results_dir:=/scratch/gaurav_kumar/results

The node writes two NumPy arrays per frame into

    <results_dir>/
      ├── bboxes/upper_velodyne/<sequence>/<frame_idx>.npy
      └── masks/upper_velodyne/<sequence>/<frame_idx>.npy

* The **bbox array** has shape (N,7) with columns
  [cx, cy, cz, width, length, height, yaw].
* The **mask array** is a uint8 vector of length `#points` whose non‑zero
  entries indicate points that lie inside at least one bounding box.
"""
import os
import threading

import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pc2

from visualization_msgs.msg import MarkerArray
from sensor_msgs.msg import PointCloud2
from tf.transformations import euler_from_quaternion


# --------------------------------------------------------------------------- #
#   Geometry helpers                                                          #
# --------------------------------------------------------------------------- #

class BBox:
    """Axis‑aligned in its own yaw‑rotated frame, defined by JRDB‑style params."""
    def __init__(self, center, size, yaw):
        self.center = np.asarray(center, dtype=np.float32)
        self.wlh    = np.asarray(size, dtype=np.float32)  # width, length, height
        self.yaw    = float(yaw)

    def corners(self,
                w_infl: float = 1.0,
                l_infl: float = 1.0,
                h_infl: float = 1.0):
        """Return 8 corner points (3×8) in world frame with optional inflation."""
        half = 0.5 * self.wlh * np.array([w_infl, l_infl, h_infl])
        signs = np.array(np.meshgrid([-1, 1], [-1, 1], [-1, 1])).T.reshape(-1, 3)
        local = signs * half
        R = np.array([[ np.cos(self.yaw), -np.sin(self.yaw), 0],
                      [ np.sin(self.yaw),  np.cos(self.yaw), 0],
                      [ 0,                 0,                1]])
        return (R @ local.T + self.center[:, None])        # (3,8)


def points_in_box(box: "BBox", pts: np.ndarray) -> np.ndarray:
    """
    Boolean mask of |pts| (3×N) indicating which points fall inside *box*.
    Method: project onto the box's edge vectors and test extents.
    """
    C = box.corners()          # (3,8)
    p0 = C[:, 0]
    i  = C[:, 4] - p0          # width edge
    j  = C[:, 2] - p0          # length edge
    k  = C[:, 1] - p0          # height edge
    rel = pts - p0[:, None]
    iv, jv, kv = np.dot(i, rel), np.dot(j, rel), np.dot(k, rel)
    return ((0 <= iv) & (iv <= np.dot(i, i)) &
            (0 <= jv) & (jv <= np.dot(j, j)) &
            (0 <= kv) & (kv <= np.dot(k, k)))


# --------------------------------------------------------------------------- #
#   Collector node                                                            #
# --------------------------------------------------------------------------- #

class FinalBBoxCollector:
    def __init__(self):
        # Parameters
        self.results_base = rospy.get_param('~results_dir',
                                            '/scratch/gaurav_kumar/results')
        self.sequence     = rospy.get_param('~sequence', None)

        # Runtime state
        self.frame_idx        = 0
        self.latest_boxes_obj = []          # list[BBox]
        self.latest_boxes_arr = np.zeros((0, 7), dtype=np.float32)
        self.lock             = threading.Lock()

        # I/O
        rospy.Subscriber('/onboard_detector/dynamic_bboxes',
                         MarkerArray, self._box_cb, queue_size=10)
        rospy.Subscriber('/livox/pcd',
                         PointCloud2, self._pcd_cb, queue_size=10)

        if self.sequence:
            self._ensure_dirs(self.sequence)
        rospy.loginfo('data_collector ready.')

    # --------------------------------------------------------------------- #
    #  Callbacks                                                            #
    # --------------------------------------------------------------------- #

    def _box_cb(self, msg: MarkerArray):
        boxes, objs = [], []
        for m in msg.markers:
            cx, cy, cz = m.pose.position.x, m.pose.position.y, m.pose.position.z
            w, l, h    = m.scale.x, m.scale.y, m.scale.z
            q          = m.pose.orientation
            _, _, yaw  = euler_from_quaternion([q.x, q.y, q.z, q.w])
            boxes.append([cx, cy, cz, w, l, h, yaw])
            objs.append(BBox([cx, cy, cz], [w, l, h], yaw))
        with self.lock:
            self.latest_boxes_arr = np.asarray(boxes, dtype=np.float32) \
                                    if boxes else np.zeros((0, 7), np.float32)
            self.latest_boxes_obj = objs

    def _pcd_cb(self, pcd_msg: PointCloud2):
        current_seq = rospy.get_param('/current_sequence', None)
        if current_seq is None:
            return
        if current_seq != self.sequence:
            self._ensure_dirs(current_seq)

        # Clone latest boxes under lock
        with self.lock:
            boxes_arr = self.latest_boxes_arr.copy()
            boxes_obj = list(self.latest_boxes_obj)

        # Extract xyz
        pts = np.array([p for p in pc2.read_points(pcd_msg,
                                                   skip_nans=True,
                                                   field_names=('x', 'y', 'z'))],
                       dtype=np.float32)
        pts_t = pts.T                                 # 3×N
        masks = [points_in_box(b, pts_t) for b in boxes_obj]
        mask_any = np.any(np.stack(masks), axis=0) if masks else \
                   np.zeros(pts.shape[0], dtype=bool)

        # Save
        fname = f"{self.frame_idx:06d}.npy"
        np.save(os.path.join(self.bbox_dir,  fname), boxes_arr)
        np.save(os.path.join(self.mask_dir,  fname), mask_any.astype(np.uint8))
        rospy.logdebug(f"[{self.sequence}] frame {self.frame_idx:06d} "
                       f"-- {boxes_arr.shape[0]} boxes, "
                       f"{mask_any.sum()} masked pts")
        self.frame_idx += 1

    # --------------------------------------------------------------------- #
    #  Helpers                                                              #
    # --------------------------------------------------------------------- #

    def _ensure_dirs(self, seq: str):
        self.frame_idx = 0
        self.sequence = seq
        self.bbox_dir  = os.path.join(self.results_base,
                                      'bboxes', 'upper_velodyne', seq)
        self.mask_dir  = os.path.join(self.results_base,
                                      'masks',  'upper_velodyne', seq)
        os.makedirs(self.bbox_dir, exist_ok=True)
        os.makedirs(self.mask_dir,  exist_ok=True)
        rospy.loginfo(f"Output directories ready:\n  {self.bbox_dir}\n  {self.mask_dir}")


# --------------------------------------------------------------------------- #
#   Main                                                                      #
# --------------------------------------------------------------------------- #

def main():
    rospy.init_node('final_bbox_collector', anonymous=False)
    FinalBBoxCollector()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass