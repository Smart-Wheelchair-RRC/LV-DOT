#!/usr/bin/env python3
"""
data_collector.py
=================
Collect **final** LV‑DOT outputs for quantitative benchmarking against JRDB.

Subscribed Topics
-----------------
/onboard_detector/tracked_bboxes     visualization_msgs/MarkerArray
/livox/pcd                          sensor_msgs/PointCloud2

+ /onboard_detector/tracked_bboxes
+ /onboard_detector/raw_dynamic_point_cloud

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
import numpy.linalg as LA
import rospy
import sensor_msgs.point_cloud2 as pc2

from scipy.spatial.transform import Rotation as R

from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TransformStamped, Transform, Vector3, Quaternion
from scipy.spatial.transform import Rotation as Rsci


# --------------------------------------------------------------------------- #
#   TF2 imports                                                              #
# --------------------------------------------------------------------------- #
import tf2_ros
import tf2_sensor_msgs.tf2_sensor_msgs as tf2_sns


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
        
                # ------------------------------------------------------------------
        #   Bounding-box post-scaling (tune predicted boxes to GT size)
        # ------------------------------------------------------------------
        # Uniform lateral scale applied to width & length (default 1.30 ≈ torso → full-body)
        self.bbox_scale_xy = rospy.get_param('~bbox_scale_xy', 1.30)
        # Vertical scale applied to height (default 1.10)
        self.bbox_scale_z  = rospy.get_param('~bbox_scale_z', 1.20)
        # Minimum dimensions [w, l, h] enforced after scaling
        self.bbox_min_dims = rospy.get_param('~bbox_min_dims', [0.50, 0.50, 1.50])

        # --- TF / frame config -------------------------------------------------
        # Whether to transform data at all
        self.use_tf = rospy.get_param('~use_tf_transform', False)
        # Frame in which JRDB ground‑truth lives (from feeder)
        self.target_frame = rospy.get_param('~target_frame', 'upper_velodyne')

        # Runtime state
        self.frame_idx        = 0
        self.latest_boxes_obj = []          # list[BBox]
        self.latest_boxes_arr = np.zeros((0, 7), dtype=np.float32)
        self.lock             = threading.Lock()

        # TF listener so we can transform LiDAR clouds into the map frame
        self.tf_buffer  = tf2_ros.Buffer(rospy.Duration(60))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Static transform from body to lidar (for JRDB bags with no TF tree)
        self.static_tf_ok = False
        try:
            body2lidar = rospy.get_param('/onboard_detector/body_to_lidar')
            if isinstance(body2lidar, list) and len(body2lidar) == 16:
                body2lidar = np.array(body2lidar, dtype=np.float32).reshape(4,4)
                # pose = identity in JRDB feeder, so map≈body. We want lidar->map
                R_bl = body2lidar[:3,:3]
                t_bl = body2lidar[:3, 3]
                # quaternion from rotation
                from scipy.spatial.transform import Rotation as Rsc
                q = Rsc.from_matrix(R_bl).as_quat()  # x,y,z,w
                self.static_rot = q
                self.static_trans = t_bl
                self.static_tf_ok = True
        except KeyError:
            pass

        # keep full 4×4 body→lidar for pose-based transform
        self.body_T_lidar = None
        body2lidar_param = rospy.get_param('/onboard_detector/body_to_lidar', None)
        if isinstance(body2lidar_param, list) and len(body2lidar_param) == 16:
            self.body_T_lidar = np.array(body2lidar_param, dtype=np.float32).reshape(4, 4)

        # I/O /onboard_detector/tracked_bboxes
        rospy.Subscriber('/onboard_detector/dynamic_bboxes',
                        MarkerArray, self._box_cb, queue_size=10)
        # rospy.Subscriber('/onboard_detector/tracked_bboxes',
        #                  MarkerArray, self._box_cb, queue_size=10)
        rospy.Subscriber('/livox/pcd',
                         PointCloud2, self._pcd_cb, queue_size=10)

        if self.sequence:
            self._ensure_dirs(self.sequence)
        rospy.loginfo('data_collector ready.')

    # --------------------------------------------------------------------- #
    #  Callbacks                                                            #
    # --------------------------------------------------------------------- #

    def _box_cb(self, msg: MarkerArray):
        # If the incoming MarkerArray is not in the desired frame, pull TF once
        if self.use_tf and msg.markers and msg.markers[0].header.frame_id != self.target_frame:
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    msg.markers[0].header.frame_id,
                    msg.markers[0].header.stamp,
                    rospy.Duration(0.1))
                rot_q = np.array([tf.transform.rotation.x,
                                  tf.transform.rotation.y,
                                  tf.transform.rotation.z,
                                  tf.transform.rotation.w], dtype=np.float32)
                trans = np.array([tf.transform.translation.x,
                                  tf.transform.translation.y,
                                  tf.transform.translation.z], dtype=np.float32)
            except (tf2_ros.LookupException,
                    tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                rot_q, trans = None, None
        else:
            rot_q, trans = None, None

        boxes, objs = [], []
        for m in msg.markers:
            # ------------------------------------------------------------------
            # 1. World-space vertex positions
            # ------------------------------------------------------------------
            q = m.pose.orientation
            rot_mat = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()  # 3×3
            verts = []
            for p in m.points:
                local = np.array([p.x, p.y, p.z], dtype=np.float32)
                verts.append(rot_mat @ local +
                             np.array([m.pose.position.x,
                                       m.pose.position.y,
                                       m.pose.position.z], dtype=np.float32))
            if not verts:
                continue
            V = np.stack(verts)          # (N, 3)
            # ------------------------------------------------------------------
            # 2. Derive [cx, cy, cz, w, l, h, yaw]
            # ------------------------------------------------------------------
            centre = V.mean(axis=0)

            # Robust yaw extraction (works even if roll/pitch ≠ 0)
            yaw = R.from_quat([q.x, q.y, q.z, q.w]).as_euler('zyx', degrees=False)[0]

            # Prefer explicit dimensions in the Marker message when present
            if m.scale.x > 0.0 and m.scale.y > 0.0 and m.scale.z > 0.0:
                # JRDB convention: [width, length, height] = (scale.x, scale.y, scale.z)
                w, l, h = m.scale.x, m.scale.y, m.scale.z
            else:
                # Fallback – deduce extents from the 8 vertices
                Rz = np.array([[ np.cos(-yaw), -np.sin(-yaw), 0],
                               [ np.sin(-yaw),  np.cos(-yaw), 0],
                               [          0.0,           0.0, 1]], dtype=np.float32)
                V_local = (Rz @ (V - centre).T).T
                w = V_local[:, 0].ptp()
                l = V_local[:, 1].ptp()
                h = V_local[:, 2].ptp()
                        # ------------------------------------------------------------------
            # 2b.  Scale and clamp so predicted boxes match JRDB GT dimensions
            # ------------------------------------------------------------------
            w *= self.bbox_scale_xy
            l *= self.bbox_scale_xy
            h *= self.bbox_scale_z
            # Enforce minimum “full-body” size
            w = max(w, self.bbox_min_dims[0])
            l = max(l, self.bbox_min_dims[1])
            h = max(h, self.bbox_min_dims[2])

            # ------------------------------------------------------------------
            # 3. Optional transform into target_frame
            # ------------------------------------------------------------------
            if rot_q is not None:
                centre = R.from_quat(rot_q).apply(centre) + trans
            elif self.static_tf_ok:
                centre = R.from_quat(self.static_rot).apply(centre) + self.static_trans

            cx, cy, cz = centre.tolist()
            boxes.append([cx, cy, cz, w, l, h, yaw])
            # Slight inflation (5 %) so thin boxes still capture upper‑body points
            boxes[-1][3] *= 1.05
            boxes[-1][4] *= 1.05
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

        # ---- Pose‑based map→lidar transform -----------------------------------
        if self.body_T_lidar is not None and hasattr(self, "pose_list") and self.pose_list and self.frame_idx < len(self.pose_list):
            world_T_body = self.pose_list[self.frame_idx]      # map==world frame
            world_T_lidar = world_T_body @ self.body_T_lidar
            lidar_T_world = np.linalg.inv(world_T_lidar)

            for bi, bb in enumerate(boxes_obj):
                hom = np.array([bb.center[0], bb.center[1], bb.center[2], 1.0], dtype=np.float32)
                centre_lidar = lidar_T_world @ hom
                boxes_arr[bi, :3] = centre_lidar[:3]
                bb.center = centre_lidar[:3]

        # ---- Bring cloud into the same frame as the boxes ----------------------
        if self.use_tf and pcd_msg.header.frame_id != self.target_frame:
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    pcd_msg.header.frame_id,
                    rospy.Time(0),
                    rospy.Duration(0.1))
                cloud_tf = tf2_sns.do_transform_cloud(pcd_msg, tf)
            except (tf2_ros.LookupException,
                    tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                rospy.logwarn_once(
                    "[data_collector] TF to '%s' unavailable; using raw cloud frame (%s). "
                    "Masks may be mis‑aligned.",
                    self.target_frame, pcd_msg.header.frame_id)
                if self.static_tf_ok:
                    cloud_tf = tf2_sns.do_transform_cloud(
                        pcd_msg,
                        TransformStamped(
                            transform=Transform(
                                translation=Vector3(
                                    x=float(self.static_trans[0]),
                                    y=float(self.static_trans[1]),
                                    z=float(self.static_trans[2])),
                                rotation=Quaternion(
                                    x=float(self.static_rot[0]),
                                    y=float(self.static_rot[1]),
                                    z=float(self.static_rot[2]),
                                    w=float(self.static_rot[3])))),
                    )
                else:
                    cloud_tf = pcd_msg
        else:
            cloud_tf = pcd_msg

        # Extract xyz in map frame
        pts = np.array([p for p in pc2.read_points(cloud_tf,
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

        # ------------------------------------------------------------------
        # JRDB pose files come in two flavors:
        #   <seq>_poses_kitti.txt  – 12 floats (3×4 row-major) per line
        #   <seq>_poses_tum.txt    – t x y z qx qy qz qw  (timestamp + pose)
        # We search for either file and build a list[4×4] world→body matrices.
        # ------------------------------------------------------------------
        
        pose_base = f'/scratch/aadith_warrier/JRDB/poses/{seq}'
        kitti_file = pose_base + '_poses_kitti.txt'
        tum_file   = pose_base + '_poses_tum.txt'
        self.pose_list = []

        if os.path.exists(kitti_file):
            with open(kitti_file) as f:
                for line in f:
                    vals = [float(v) for v in line.strip().split()]
                    if len(vals) != 12:
                        continue
                    mat = np.eye(4, dtype=np.float32)
                    mat[:3, :4] = np.array(vals, dtype=np.float32).reshape(3, 4)
                    self.pose_list.append(mat)

        elif os.path.exists(tum_file):
            with open(tum_file) as f:
                for line in f:
                    vals = [float(v) for v in line.strip().split()]
                    if len(vals) != 8:
                        continue
                    _, x, y, z, qx, qy, qz, qw = vals
                    R_wb = Rsci.from_quat([qx, qy, qz, qw]).as_matrix()
                    mat = np.eye(4, dtype=np.float32)
                    mat[:3, :3] = R_wb
                    mat[:3, 3]  = [x, y, z]
                    self.pose_list.append(mat)

        rospy.loginfo(f"[collector] Loaded {len(self.pose_list)} poses for {seq}")


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