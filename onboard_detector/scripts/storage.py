#!/usr/bin/env python3
import rospy
import numpy as np
import threading
from visualization_msgs.msg import MarkerArray
from tf.transformations import euler_from_quaternion
import os
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2

# — your existing BBox & points_in_box definitions —
class BBox:
    def __init__(self, center, size, orientation):
        self.center = np.array(center)
        self.wlh = np.array(size)
        self.orientation = orientation  # yaw in radians, CCW‑positive

    def corners(self,
                width_inflation: float = 1.25,
                length_inflation: float = 1.25,
                height_inflation: float = 1):
        wlh = self.wlh * np.array([width_inflation, length_inflation, height_inflation]) / 2
        signs = np.array(np.meshgrid(
            [-1,1], [-1,1], [-1,1]
        )).T.reshape(-1,3)
        corners = signs * wlh
        theta = self.orientation
        R = np.array([
            [np.cos(theta), -np.sin(theta), 0],
            [np.sin(theta),  np.cos(theta), 0],
            [0,              0,             1]
        ])
        rotated    = corners @ R.T
        translated = rotated + self.center
        return translated.T  # shape (3,8)

def points_in_box(box, points):
    corners = box.corners()
    p1 = corners[:,0]
    i  = corners[:,4] - p1
    j  = corners[:,2] - p1
    k  = corners[:,1] - p1
    v  = points - p1.reshape((-1,1))

    iv = np.dot(i, v)
    jv = np.dot(j, v)
    kv = np.dot(k, v)

    mask_x = np.logical_and(0 <= iv, iv <= np.dot(i,i))
    mask_y = np.logical_and(0 <= jv, jv <= np.dot(j,j))
    mask_z = np.logical_and(0 <= kv, kv <= np.dot(k,k))
    return mask_x & mask_y & mask_z

# — node to collect boxes —
class DynamicBBoxCollector:
    def __init__(self):
        # subscribers for boxes and pointcloud
        rospy.Subscriber('/onboard_detector/dynamic_bboxes', MarkerArray, self.bbox_callback, queue_size=1)
        rospy.Subscriber('/livox/pcd', PointCloud2, self.pcd_callback, queue_size=10)

        # Base output directory for all sequences
        self.results_base = rospy.get_param('~results_dir', '/scratch/gaurav_kumar/results')
        # Track current sequence and per-sequence frame index
        self.sequence = None
        self.frame_idx = 0
        # Placeholders for latest box detections
        self.latest_boxes_objs = []
        self.latest_boxes_arr = np.zeros((0,7))
        # Protect latest_boxes_* against concurrent access
        self.lock = threading.Lock()

    def bbox_callback(self, msg: MarkerArray):
        # update latest boxes
        boxes_list = []
        boxes_obj = []
        for m in msg.markers:
            cx, cy, cz = (m.pose.position.x, m.pose.position.y, m.pose.position.z)
            w, l, h = m.scale.x, m.scale.y, m.scale.z
            q = m.pose.orientation
            _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
            boxes_obj.append(BBox(center=[cx, cy, cz], size=[w, l, h], orientation=yaw))
            boxes_list.append([cx, cy, cz, w, l, h, yaw])
        with self.lock:
            self.latest_boxes_objs = boxes_obj
            if boxes_list:
                self.latest_boxes_arr = np.stack(boxes_list)
            else:
                self.latest_boxes_arr = np.zeros((0, 7))

    def pcd_callback(self, pcd_msg: PointCloud2):
        # Determine current sequence from ROS parameter
        current_seq = rospy.get_param('sequence', None)
        if not current_seq:
            return
        self._ensure_sequence_dirs(current_seq)

        # ensure we have a recent box list
        with self.lock:
            boxes_arr = self.latest_boxes_arr.copy()
            boxes_obj = list(self.latest_boxes_objs)
        # extract point cloud
        points = np.array([p for p in pc2.read_points(pcd_msg, skip_nans=True, field_names=('x','y','z'))])
        masks_per_box = [points_in_box(b, points.T) for b in boxes_obj]
        if masks_per_box:
            mask_any = np.any(np.stack(masks_per_box), axis=0)
        else:
            mask_any = np.zeros(points.shape[0], dtype=bool)
        # Use an internal counter to name output files sequentially from 0
        frame_idx = self.frame_idx
        fname     = f"{frame_idx:06d}.npy"
        self.frame_idx += 1
        np.save(os.path.join(self.bboxes_dir, fname), boxes_arr)
        np.save(os.path.join(self.masks_dir, fname), mask_any.astype(np.uint8))
        rospy.loginfo(f"Saved frame {frame_idx:06d}: {boxes_arr.shape[0]} boxes, {int(mask_any.sum())} masked points")

    def _ensure_sequence_dirs(self, seq_name):
        if seq_name != self.sequence:
            self.sequence = seq_name
            self.frame_idx = 0
            # Create per-sequence output dirs
            self.bboxes_dir = os.path.join(self.results_base, 'bboxes', 'upper_velodyne', seq_name)
            self.masks_dir  = os.path.join(self.results_base, 'masks', 'upper_velodyne', seq_name)
            os.makedirs(self.bboxes_dir, exist_ok=True)
            os.makedirs(self.masks_dir, exist_ok=True)

if __name__ == '__main__':
    rospy.init_node('dynamic_bbox_collector')
    collector = DynamicBBoxCollector()
    rospy.spin()