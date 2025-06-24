#!/usr/bin/env python3
import rospy
import numpy as np
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
        self.orientation = orientation  # heading in radians, clockwise (+)

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
        # parameters for output
        self.data_dir = rospy.get_param('~data_dir', '.')
        self.sequence = rospy.get_param('~sequence', 'default_seq')
        # prepare output directories per sequence
        self.bboxes_dir = os.path.join(self.data_dir, 'bboxes', 'upper_velodyne', self.sequence)
        self.masks_dir = os.path.join(self.data_dir, 'masks', 'upper_velodyne', self.sequence)
        os.makedirs(self.bboxes_dir, exist_ok=True)
        os.makedirs(self.masks_dir, exist_ok=True)
        # frame counter for file naming
        self.frame_idx = 0
        # storage for sequence data
        self.sequence_boxes = []  # list of per-frame box arrays
        self.sequence_masks = []  # list of per-frame masks for points
        # storage for latest box detections
        self.latest_boxes_objs = []
        self.latest_boxes_arr = np.zeros((0,7))
        # subscribers for boxes and pointcloud
        rospy.Subscriber('/onboard_detector/dynamic_bboxes', MarkerArray, self.bbox_callback, queue_size=1)
        rospy.Subscriber('/livox/pcd', PointCloud2, self.pcd_callback, queue_size=1)

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
        self.latest_boxes_objs = boxes_obj
        self.latest_boxes_arr = np.array(boxes_list)

    def pcd_callback(self, pcd_msg: PointCloud2):
        # ensure we have a recent box list
        boxes_arr = self.latest_boxes_arr
        boxes_obj = self.latest_boxes_objs
        # extract point cloud
        points = np.array([p for p in pc2.read_points(pcd_msg, skip_nans=True, field_names=('x','y','z'))])
        masks_per_box = [points_in_box(b, points.T) for b in boxes_obj]
        if masks_per_box:
            mask_any = np.any(np.stack(masks_per_box), axis=0)
        else:
            mask_any = np.zeros(points.shape[0], dtype=bool)
        # save per-frame files matching JRDB indexing
        fname = f"{self.frame_idx:06d}.npy"
        np.save(os.path.join(self.bboxes_dir, fname), boxes_arr)
        np.save(os.path.join(self.masks_dir, fname), mask_any.astype(np.uint8))
        rospy.loginfo(f"Saved frame {self.frame_idx:06d}: {boxes_arr.shape[0]} boxes, {int(mask_any.sum())} masked points")
        self.frame_idx += 1

if __name__ == '__main__':
    rospy.init_node('dynamic_bbox_collector')
    collector = DynamicBBoxCollector()
    rospy.spin()