#!/usr/bin/env python
"""
Publish JRDB sequences as live ROS topics for LV-DOT evaluation.

Publishes
---------
/camera/color/image_raw         sensor_msgs/Image (rgb8)
/camera/depth/image_rect_raw    sensor_msgs/Image (16UC1)
/livox/pcd                      sensor_msgs/PointCloud2
/localization                   nav_msgs/Odometry  (static identity pose)
/yolo_detector/detected_bounding_boxes vision_msgs/Detection2DArray
"""
import os, json, cv2, rospy, numpy as np, open3d as o3d
import sys, select, tty, termios         # non-blocking keyboard control
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Quaternion as GeoQuat, Twist
from sensor_msgs.msg import Image, PointCloud2
from vision_msgs.msg import Detection2D, Detection2DArray, \
                             BoundingBox2D, ObjectHypothesisWithPose
from torch.utils.data import Dataset        # torch only for length indexing

# ---------- Dataset wrapper --------------------------------------------------

class JRDBDataset(Dataset):
    def __init__(self, root, seq, cam='image_0', debug=False):
        self.debug = debug
        self.img_dir   = os.path.join(root, 'images', cam, seq)
        self.depth_dir = os.path.join(root, 'depth_pred', seq)
        self.pcd_dir   = os.path.join(root, 'pointclouds', 'upper_velodyne', seq)
        self.box_dir   = os.path.join(root, 'bboxes', seq)      # 2-D boxes

        self.images = sorted(os.listdir(self.img_dir))
        self.depths = sorted(os.listdir(self.depth_dir))
        self.pcds   = sorted(os.listdir(self.pcd_dir))
        assert len(self.images) == len(self.depths) == len(self.pcds), \
            "RGB / depth / PCD count mismatch"

    def __len__(self): return len(self.images)

    # ---- I/O helpers --------------------------------------------------------
    def _read_rgb(self, fn):   # BGR→RGB
        return cv2.cvtColor(cv2.imread(os.path.join(self.img_dir, fn)), cv2.COLOR_BGR2RGB)
    def _read_depth(self, fn):
        return cv2.imread(os.path.join(self.depth_dir, fn), cv2.IMREAD_UNCHANGED)
    def _read_pcd(self, fn):
        pts = np.asarray(o3d.io.read_point_cloud(os.path.join(self.pcd_dir, fn)).points,
                         dtype=np.float32)    #  [oai_citation:7‡open3d.org](https://www.open3d.org/docs/0.9.0/tutorial/Basic/file_io.html?utm_source=chatgpt.com)
        return pts
    def _read_boxes(self, fn):
        json_path = os.path.join(self.box_dir, fn.replace('.png','.json').replace('.jpg','.json'))
        if not os.path.exists(json_path): return []
        with open(json_path) as f: return json.load(f)   # list[[x1,y1,x2,y2], ...]
    # -------------------------------------------------------------------------

    def __getitem__(self, idx):
        img_fn = self.images[idx]
        return {
            'image':  self._read_rgb(img_fn),
            'depth':  self._read_depth(self.depths[idx]),
            'points': self._read_pcd(self.pcds[idx]),
            'boxes':  self._read_boxes(img_fn)
        }

# ---------- Conversion helpers ----------------------------------------------

def boxes_to_detection_array(box_list, stamp, frame_id):
    """Convert [[x1,y1,x2,y2], ...] → Detection2DArray."""
    arr = Detection2DArray()
    arr.header.stamp = stamp
    arr.header.frame_id = frame_id
    for bid, (x1,y1,x2,y2) in enumerate(box_list):
        det = Detection2D()
        det.bbox.center.x = (x1+x2)/2.0
        det.bbox.center.y = (y1+y2)/2.0
        det.bbox.size_x   = (x2-x1)
        det.bbox.size_y   = (y2-y1)
        det.id = bid
        # one dummy hypothesis: class_id=0 (“person”), score=1.0
        hyp = ObjectHypothesisWithPose()
        hyp.id, hyp.score = 0, 1.0
        det.results.append(hyp)
        arr.detections.append(det)
    return arr

# ---------- ROS node ---------------------------------------------------------

class JRDBDataFeeder:
    def __init__(self):
        rospy.init_node('jrdb_data_feeder')
        self.bridge  = CvBridge()
        self.rate    = rospy.Rate(rospy.get_param('~publish_rate', 10))

        # Paths & settings
        root     = rospy.get_param('~data_dir')
        cam      = rospy.get_param('~camera', 'image_0')
        seq_list = sorted(os.listdir(os.path.join(root, 'images', cam)))

        self.pub_rgb   = rospy.Publisher('/camera/color/image_raw', Image,  queue_size=5)
        self.pub_depth = rospy.Publisher('/camera/depth/image_rect_raw', Image, queue_size=5)
        self.pub_pcd   = rospy.Publisher('/livox/pcd', PointCloud2,        queue_size=5)
        self.pub_odom  = rospy.Publisher('/localization', Odometry,        queue_size=5)
        self.pub_boxes = rospy.Publisher('/yolo_detector/detected_bounding_boxes',
                                         Detection2DArray,                 queue_size=5)
        
        # ----- keyboard: cbreak mode ----------------------------------------
        self._fd        = sys.stdin.fileno()
        self._old_term  = termios.tcgetattr(self._fd)
        tty.setcbreak(self._fd)
        rospy.on_shutdown(self._restore_terminal)

        # Keep a reference so we can iterate
        self.seq_list = seq_list

        # ------------------------------------------------------------------
        # Main publish loop over sequences and frames
        # ------------------------------------------------------------------
        try:
            for seq in self.seq_list:
                if rospy.is_shutdown():
                    break
                # broadcast current sequence
                rospy.set_param('/current_sequence', seq)
                ds = JRDBDataset(root, seq, cam)
                rospy.loginfo(f'▶ Feeding sequence {seq} ({len(ds)} frames)')
                frame_idx = 0
                while frame_idx < len(ds) and not rospy.is_shutdown():
                    sample = ds[frame_idx]
                    t   = rospy.Time.now()
                    hdr = Header(stamp=t, frame_id='camera_link', seq=frame_idx)

                    # --- publish RGB ---
                    rgb_msg = self.bridge.cv2_to_imgmsg(sample["image"], encoding='rgb8')
                    rgb_msg.header = hdr
                    self.pub_rgb.publish(rgb_msg)

                    # --- publish Depth ---
                    d_msg = self.bridge.cv2_to_imgmsg(sample["depth"], encoding='16UC1')
                    d_msg.header = hdr
                    self.pub_depth.publish(d_msg)

                    # --- publish PointCloud ---
                    xyz = sample["points"]
                    pts_np = np.zeros(xyz.shape[0], dtype=[
                        ('x',np.float32),('y',np.float32),('z',np.float32),('intensity',np.float32)])
                    pts_np['x'], pts_np['y'], pts_np['z'] = xyz.T
                    cloud_msg = pc2.create_cloud(
                        Header(stamp=t, frame_id='upper_velodyne', seq=frame_idx),
                        [pc2.PointField(n,off,pc2.PointField.FLOAT32,1)
                         for n,off in zip(('x','y','z','intensity'), (0,4,8,12))],
                        pts_np)
                    self.pub_pcd.publish(cloud_msg)

                    # --- publish Odometry (identity) ---
                    odom = Odometry()
                    odom.header = Header(stamp=t, frame_id='base_link', seq=frame_idx)
                    odom.child_frame_id = 'base_link'
                    odom.pose.pose = Pose()
                    odom.pose.pose.orientation = GeoQuat(0,0,0,1)
                    self.pub_odom.publish(odom)

                    # --- publish 2‑D detections ---
                    self.pub_boxes.publish(
                        boxes_to_detection_array(sample['boxes'], t, 'camera_link'))

                    self.rate.sleep()

                    # --- keyboard control ---
                    key = self._key_pressed()
                    if key in ('q', 'Q'):
                        rospy.loginfo('Quit requested — shutting down feeder.')
                        rospy.signal_shutdown('User quit')
                        break
                    elif key in ('n', 'N'):
                        rospy.loginfo('▶ Skipping to next sequence (key n)')
                        break
                    elif key in ('r', 'R'):
                        rospy.loginfo('↺ Replaying current sequence (key r)')
                        frame_idx = 0
                        continue

                    frame_idx += 1
        except (rospy.ROSInterruptException, KeyboardInterrupt):
            rospy.loginfo('Feeder shutdown requested.')
        finally:
            rospy.loginfo('✔ All sequences done; shutting down.')
            rospy.signal_shutdown('Finished all sequences.')

    # ---------------------------------------------------------------------
    #   keyboard helpers
    # ---------------------------------------------------------------------
    def _key_pressed(self):
        """Return a single char if a key was hit, else None (non-blocking)."""
        dr, _, _ = select.select([sys.stdin], [], [], 0)
        return sys.stdin.read(1) if dr else None

    def _restore_terminal(self):
        termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old_term)

# ---------------------------------------------------------------------------

if __name__ == '__main__':
    try:
        JRDBDataFeeder()
    except rospy.ROSInterruptException:
        pass