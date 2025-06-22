#!/usr/bin/env python
import os
import cv2
import rospy
from sensor_msgs.msg import Image, PointCloud2
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Quaternion as GeoQuaternion, Twist
from cv_bridge import CvBridge
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import torch
from torch.utils.data import Dataset
import open3d as o3d

class JRDBDataset(Dataset):
    """
    PyTorch Dataset for JRDB upper Velodyne sequences.

    Expects the following directory structure under `data_dir`:

        JRDB/
        ├── images/
        │   └── image_0/
        │       └── <sequence_name>/  # e.g. bytes-cafe-2019-02-07_0
        │           └── 000000.jpg, 000001.jpg, ...
        ├── pointclouds/
        │   └── upper_velodyne/
        │       └── <sequence_name>/  # .pcd files
        └── masks/
            └── upper_velodyne/
                └── <sequence_name>/  # .npy mask files

    Each sequence folder must contain the same number of images, PCDs, and masks.
    """
    def __init__(self, data_dir, sequence, camera='image_0'):
        self.root_dir = os.path.expanduser(data_dir)
        self.sequence = sequence
        self.camera = camera

        # Paths for this sequence
        self.img_dir = os.path.join(self.root_dir, 'images', camera, sequence)
        self.pcd_dir = os.path.join(self.root_dir, 'pointclouds', 'upper_velodyne', sequence)
        self.mask_dir = os.path.join(self.root_dir, 'masks', 'upper_velodyne', sequence)

        # List and sort
        self.imgs = sorted(os.listdir(self.img_dir))
        self.pcds = sorted(os.listdir(self.pcd_dir))
        self.masks = sorted(os.listdir(self.mask_dir))

        assert len(self.imgs) == len(self.pcds) == len(self.masks), (
            f"Mismatch counts: images({len(self.imgs)}), "
            f"pcds({len(self.pcds)}), masks({len(self.masks)})"
        )

        # Uniform relative timestamps in [0, 1]
        self.timestamps = np.linspace(0, 1, len(self.imgs))

    def __len__(self):
        return len(self.imgs)

    def _load_image(self, path):
        img = cv2.imread(path)
        return cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    def _load_pcd(self, path):
        pcd = o3d.io.read_point_cloud(path)
        return np.asarray(pcd.points, dtype=np.float32)

    def _load_mask(self, path):
        mask = np.load(path)
        return (mask > 0).astype(np.float32)

    def __getitem__(self, idx):
        img_path = os.path.join(self.img_dir, self.imgs[idx])
        pcd_path = os.path.join(self.pcd_dir, self.pcds[idx])
        mask_path = os.path.join(self.mask_dir, self.masks[idx])
        ts = self.timestamps[idx]

        image = self._load_image(img_path)
        points = self._load_pcd(pcd_path)
        mask = self._load_mask(mask_path)

        assert points.shape[0] == mask.shape[0], (
            f"Point count {points.shape[0]} != mask count {mask.shape[0]}"
        )

        return {
            'image': image,                   # H x W x 3 uint8 (RGB)
            'points': torch.from_numpy(points),
            'mask': torch.from_numpy(mask),   # N floats (0 or 1)
            'timestamp': ts
        }

class JRDBFeederNode(object):
    """
    ROS1 Node to publish JRDB sequence data for LV-DOT:
      - /camera/color/image_raw  (sensor_msgs/Image)
      - /camera/depth/image_rect_raw (sensor_msgs/Image)
      - /livox/pcd (sensor_msgs/PointCloud2)
      - /localization (nav_msgs/Odometry)

    Params:
      ~data_dir : JRDB root directory
      ~sequence : sequence folder name under images/, pointclouds/, masks/
      ~camera   : camera folder under images/ (default 'image_0')
    """
    def __init__(self):
        rospy.init_node('jrdb_feeder_node')
        self.bridge = CvBridge()

        # Parameters
        data_dir = rospy.get_param('~data_dir', '/path/to/JRDB')
        sequence = rospy.get_param('~sequence', 'bytes-cafe-2019-02-07_0')
        camera = rospy.get_param('~camera', 'image_0')

        # Dataset
        self.dataset = JRDBDataset(data_dir, sequence, camera)
        self.idx = 0

        # Publishers
        self.pub_image = rospy.Publisher('/camera/color/image_raw', Image, queue_size=1)
        self.pub_depth = rospy.Publisher('/camera/depth/image_rect_raw', Image, queue_size=1)
        self.pub_pcd = rospy.Publisher('/livox/pcd', PointCloud2, queue_size=1)
        self.pub_odom = rospy.Publisher('/localization', Odometry, queue_size=1)

        self.rate = rospy.Rate(10)
        self.spin()

    def spin(self):
        while not rospy.is_shutdown():
            sample = self.dataset[self.idx]

            # RGB image
            img_msg = self.bridge.cv2_to_imgmsg(sample['image'], encoding='rgb8')
            img_msg.header.stamp = rospy.Time.now()
            img_msg.header.frame_id = 'camera_link'
            self.pub_image.publish(img_msg)

            # Dummy depth
            H, W = sample['image'].shape[:2]
            depth = np.zeros((H, W), dtype=np.uint16)
            depth_msg = self.bridge.cv2_to_imgmsg(depth, encoding='mono16')
            depth_msg.header = img_msg.header
            self.pub_depth.publish(depth_msg)

            # PointCloud2 with mask in 'intensity'
            xyz = sample['points'].numpy()
            mask = sample['mask'].numpy().astype(np.float32)
            header = Header()
            header.stamp = rospy.Time.now()
            header.frame_id = 'livox_frame'

            fields = [
                pc2.PointField('x', 0, pc2.PointField.FLOAT32, 1),
                pc2.PointField('y', 4, pc2.PointField.FLOAT32, 1),
                pc2.PointField('z', 8, pc2.PointField.FLOAT32, 1),
                pc2.PointField('intensity', 12, pc2.PointField.FLOAT32, 1),
            ]
            pts = np.zeros(xyz.shape[0], dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32), ('intensity', np.float32)])
            pts['x'], pts['y'], pts['z'] = xyz.T
            pts['intensity'] = mask
            pcd_msg = pc2.create_cloud(header, fields, pts)
            self.pub_pcd.publish(pcd_msg)

            # Static odometry
            odom = Odometry()
            odom.header = header
            odom.child_frame_id = 'base_link'
            odom.pose.pose = Pose()
            odom.pose.pose.orientation = GeoQuaternion(0.0, 0.0, 0.0, 1.0)
            odom.twist.twist = Twist()
            self.pub_odom.publish(odom)

            # Next
            self.idx = (self.idx + 1) % len(self.dataset)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        JRDBFeederNode()
    except rospy.ROSInterruptException:
        pass