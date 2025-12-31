import math
import os
from dataclasses import dataclass
from nav_msgs.msg import OccupancyGrid
from .goal_snapper import GoalSnapper

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge

from tf2_ros import Buffer, TransformListener, TransformException

# YOLO
from ultralytics import YOLO
import numpy as np


# === camera model (rough) ===
H_FOV = 1.047  # rad, ~60 deg (match gazebo camera)
DEFAULT_CONF_TH = 0.90

@dataclass
class Landmark:
    x: float
    y: float
    nav_x: float = float('nan')
    nav_y: float = float('nan')
    nav_ok: bool = False
    count: int = 0
    conf_avg: float = 0.0

    def add_obs(self, x: float, y: float, conf: float):
        """Update running average for position and confidence."""
        n = self.count
        self.x = (self.x * n + x) / (n + 1)
        self.y = (self.y * n + y) / (n + 1)
        self.conf_avg = (self.conf_avg * n + conf) / (n + 1)
        self.count += 1

def yaw_from_quaternion(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def lidar_range_median(scan: LaserScan, angle: float, window: int = 5):
    """Return median of finite ranges around angle, or None."""
    if angle < scan.angle_min or angle > scan.angle_max:
        return None

    n = len(scan.ranges)
    if n == 0:
        return None

    idx_center = int(round((angle - scan.angle_min) / scan.angle_increment))
    idx_center = max(0, min(n - 1, idx_center))

    window = max(0, int(window))
    vals = []
    for di in range(-window, window + 1):
        idx = idx_center + di
        if idx < 0 or idx >= n:
            continue
        r = scan.ranges[idx]
        if math.isinf(r) or math.isnan(r):
            continue
        if r < scan.range_min or r > scan.range_max:
            continue
        vals.append(r)

    if not vals:
        return None
    vals.sort()
    return vals[len(vals) // 2]



class YoloLidarLandmarker(Node):
    def __init__(self):
        super().__init__('yolo_lidar_landmarker')

        # -----------------------
        # Declare parameters
        # -----------------------
        self.declare_parameter('image_topic', '/tb3/camera/image_raw')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')

        self.declare_parameter('conf_th', DEFAULT_CONF_TH)
        self.declare_parameter('merge_radius', 0.5)
        self.declare_parameter('min_confirm', 5)
        self.declare_parameter('output_path', os.path.expanduser('~/smart_cane/landmarks.txt'))
        self.declare_parameter('yolo_model', os.path.expanduser('~/smart_cane/yolov8n.pt'))
        self.declare_parameter('classes_allowlist', [])
        self.declare_parameter('lidar_window', 7)

        # ⭐ camera/lidar alignment tweak
        self.declare_parameter('camera_yaw_offset', 0.0)

        # costmap / snapping
        self.declare_parameter('costmap_topic', '/global_costmap/costmap')
        self.declare_parameter('occ_th', 50)
        self.declare_parameter('unknown_is_obstacle', True)
        self.declare_parameter('clearance_m', 0.30)
        self.declare_parameter('search_radius_m', 2.0)

        # -----------------------
        # Read parameters
        # -----------------------
        self.image_topic = str(self.get_parameter('image_topic').value)
        self.scan_topic  = str(self.get_parameter('scan_topic').value)
        self.map_frame   = str(self.get_parameter('map_frame').value)
        self.base_frame  = str(self.get_parameter('base_frame').value)

        self.conf_th      = float(self.get_parameter('conf_th').value)
        self.merge_radius = float(self.get_parameter('merge_radius').value)
        self.min_confirm  = int(self.get_parameter('min_confirm').value)
        self.output_path  = str(self.get_parameter('output_path').value)
        self.model_path   = str(self.get_parameter('yolo_model').value)
        self.allowlist    = list(self.get_parameter('classes_allowlist').value)
        self.lidar_window = int(self.get_parameter('lidar_window').value)

        self.camera_yaw_offset = float(self.get_parameter('camera_yaw_offset').value)

        self.costmap_topic = str(self.get_parameter('costmap_topic').value)

        # -----------------------
        # Components
        # -----------------------
        self.snapper = GoalSnapper(
            occ_th=int(self.get_parameter('occ_th').value),
            unknown_is_obstacle=bool(self.get_parameter('unknown_is_obstacle').value),
            clearance_m=float(self.get_parameter('clearance_m').value),
            search_radius_m=float(self.get_parameter('search_radius_m').value),
        )

        self.create_subscription(OccupancyGrid, self.costmap_topic, self.costmap_callback, 1)
        self.get_logger().info(f"Listening costmap: {self.costmap_topic}")

        self.bridge = CvBridge()
        self.last_scan = None

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # YOLO
        self.get_logger().info(f"Loading YOLO model: {self.model_path}")
        self.model = YOLO(self.model_path)

        # data
        self.landmarks = {}

        # subs
        self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)
        self.create_subscription(Image, self.image_topic, self.image_callback, 10)

        self.get_logger().info(f"Listening image: {self.image_topic}")
        self.get_logger().info(f"Listening scan : {self.scan_topic}")
        self.get_logger().info(
            f"conf_th={self.conf_th}, merge_radius={self.merge_radius}, "
            f"min_confirm={self.min_confirm}, yaw_offset={self.camera_yaw_offset}"
        )

    def scan_callback(self, msg: LaserScan):
        self.last_scan = msg

    def costmap_callback(self, msg: OccupancyGrid):
        self.snapper.set_grid(msg)


    def save_landmarks(self):
        """
        Output:
        class_raw: (x,y), (x,y), ...
        class_nav: (x,y), (x,y), ...
        Only keep lm.count >= min_confirm
        """
        try:
            with open(self.output_path, 'w') as f:
                for cls, lst in self.landmarks.items():
                    kept = [lm for lm in lst if lm.count >= self.min_confirm]
                    if not kept:
                        continue

                    # raw
                    raw_pts = ", ".join([f"({lm.x:.3f},{lm.y:.3f})" for lm in kept])
                    f.write(f"{cls}_raw: {raw_pts}\n")

                    # nav (only nav_ok)
                    nav_kept = [lm for lm in kept if lm.nav_ok]
                    if nav_kept:
                        nav_kept.sort(key=lambda lm: lm.count, reverse=True)
                        nav_pts = ", ".join([f"({lm.nav_x:.3f},{lm.nav_y:.3f})" for lm in nav_kept])
                        f.write(f"{cls}_nav: {nav_pts}\n")
        except Exception as e:
            self.get_logger().warn(f"Failed to write landmarks: {e}")


    def merge_landmark(self, cls: str, x: float, y: float, conf: float):
        if cls not in self.landmarks:
            self.landmarks[cls] = []

        # find nearest
        best_i = -1
        best_d = None
        for i, lm in enumerate(self.landmarks[cls]):
            d = math.hypot(lm.x - x, lm.y - y)
            if best_d is None or d < best_d:
                best_d = d
                best_i = i

        if best_d is not None and best_d < self.merge_radius:
            lm = self.landmarks[cls][best_i]

            n = lm.count
            lm.x = (lm.x * n + x) / (n + 1)
            lm.y = (lm.y * n + y) / (n + 1)
            lm.conf_avg = (lm.conf_avg * n + conf) / (n + 1)
            lm.count += 1

        else:
            lm = Landmark(x=x, y=y, count=1, conf_avg=conf)
            self.landmarks[cls].append(lm)

        # snap (only if costmap ready)
        try:
            snapped = self.snapper.snap(lm.x, lm.y)
        except Exception as e:
            self.get_logger().warn(f"snap() failed: {e}")
            snapped = None
            
        if snapped is not None:
            lm.nav_x, lm.nav_y = snapped
            lm.nav_ok = True
        else:
            lm.nav_ok = False


    def image_callback(self, msg: Image):
        if self.last_scan is None:
            return  # 別噴太吵

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f"cv_bridge error: {e}")
            return

        H, W, _ = frame.shape

        # TF latest
        try:
            t = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, Time())
        except TransformException:
            return

        tx = t.transform.translation.x
        ty = t.transform.translation.y
        yaw = yaw_from_quaternion(t.transform.rotation)

        # run YOLO (let YOLO prefilter)
        results = self.model.predict(frame, verbose=False, conf=self.conf_th)
        if not results:
            return

        r0 = results[0]
        if r0.boxes is None or len(r0.boxes) == 0:
            return

        names = r0.names

        for b in r0.boxes:
            conf = float(b.conf[0])
            cls_id = int(b.cls[0])
            cls_name = names.get(cls_id, str(cls_id))

            if self.allowlist and (cls_name not in self.allowlist):
                continue

            x1, y1, x2, y2 = [float(v) for v in b.xyxy[0]]
            u = (x1 + x2) / 2.0

            u_norm = (u - W / 2.0) / (W / 2.0)
            theta = u_norm * (H_FOV / 2.0)  + self.camera_yaw_offset # (若你加了參數)

            r = lidar_range_median(self.last_scan, theta, window=self.lidar_window)
            if r is None:
                continue

            x_bl = r * math.cos(theta)
            y_bl = r * math.sin(theta)

            cos_y = math.cos(yaw)
            sin_y = math.sin(yaw)
            x_map = tx + cos_y * x_bl - sin_y * y_bl
            y_map = ty + sin_y * x_bl + cos_y * y_bl

            self.merge_landmark(cls_name, x_map, y_map, conf)

        self.save_landmarks()


def main(args=None):
    rclpy.init(args=args)
    node = YoloLidarLandmarker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
