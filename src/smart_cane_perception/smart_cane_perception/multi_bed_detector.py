import math
import os
import rclpy
from rclpy.node import Node
from rclpy.time import Time

from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge

import cv2
import numpy as np

from tf2_ros import Buffer, TransformListener, TransformException


# 水平視角（跟 camera plugin 設的一樣）
H_FOV = 1.047  # 約 60 度

# 至少有多少比例是該顏色才算「看到」
RATIO_THRESHOLD = 0.01  # 1%


# ===== 每一張床用不同顏色區分（HSV）=====
# 這些範圍只是初版，之後可以依你實際顏色慢慢調
COLOR_BEDS = {
    "bed_red": {
        "lower": np.array([0, 120, 100]),
        "upper": np.array([10, 255, 255]),
    },
    "bed_orange": {
        "lower": np.array([10, 120, 100]),
        "upper": np.array([25, 255, 255]),
    },
    "bed_yellow": {
        "lower": np.array([25, 120, 100]),
        "upper": np.array([35, 255, 255]),
    },
    "bed_green": {
        "lower": np.array([40, 120, 100]),
        "upper": np.array([80, 255, 255]),
    },
    "bed_cyan": {
        "lower": np.array([80, 120, 100]),
        "upper": np.array([95, 255, 255]),
    },
    "bed_blue": {
        "lower": np.array([95, 120, 100]),
        "upper": np.array([120, 255, 255]),
    },
    "bed_pink": {
        "lower": np.array([140, 80, 80]),
        "upper": np.array([170, 255, 255]),
    },
    "bed_brown": {
        "lower": np.array([5, 50, 30]),
        "upper": np.array([20, 200, 150]),
    },
    "bed_black": {
        "lower": np.array([0, 0, 0]),
        "upper": np.array([180, 255, 40]),
    },
}


def yaw_from_quaternion(q):
    """從 quaternion 取出 yaw 角（2D 平面用）"""
    x = q.x
    y = q.y
    z = q.z
    w = q.w
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)

def get_lidar_range_for_angle(scan: LaserScan, angle: float, window: int = 5):
    """
    給一個角度（rad），在 LaserScan 中抓附近的一段 beam，
    回傳一個「最靠近的有限距離」，若都失敗則回傳 None。
    """
    angle_min = scan.angle_min
    angle_max = scan.angle_max
    angle_inc = scan.angle_increment
    ranges = scan.ranges
    n = len(ranges)

    # 如果角度根本不在 LiDAR FoV，就直接 return None
    if angle < angle_min or angle > angle_max:
        return None

    idx_center = int(round((angle - angle_min) / angle_inc))

    best_r = None

    for di in range(-window, window + 1):
        idx = idx_center + di
        if idx < 0 or idx >= n:
            continue
        r = ranges[idx]
        if math.isinf(r) or math.isnan(r):
            continue
        if r < scan.range_min or r > scan.range_max:
            continue
        if best_r is None or r < best_r:
            best_r = r

    return best_r



class MultiBedDetector(Node):
    def __init__(self):
        super().__init__('multi_bed_detector')

        # 相機 topic
        self.image_topic = '/tb3/camera/image_raw'
        self.bridge = CvBridge()

        self.sub_image = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10
        )

        # LiDAR topic（TurtleBot3 預設是 /scan）
        self.scan_topic = '/scan'
        self.last_scan = None

        self.sub_scan = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            10
        )

        # TF buffer：拿 map -> base_link
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info(f"Listening image on {self.image_topic}")
        self.get_logger().info(f"Listening LiDAR on {self.scan_topic}")

        self.points = {name: [] for name in COLOR_BEDS.keys()}
        self.output_path = os.path.expanduser('~/smart_cane/bed_points.txt')

    def scan_callback(self, msg: LaserScan):
        # 單純把最後一筆 scan 存起來
        self.last_scan = msg

    def save_points_to_file(self):
        """
        將每個顏色的點寫成單行格式：
        bed_red: (x,y), (x,y), (x,y)
        """
        try:
            with open(self.output_path, 'w') as f:
                for color, pts in self.points.items():
                    if not pts:
                        continue

                    line = f"{color}: "

                    point_strs = [f"({x:.3f},{y:.3f})" for (x, y) in pts]

                    line += ", ".join(point_strs)

                    f.write(line + "\n")

        except Exception as e:
            self.get_logger().warn(f"Failed to write points file: {e}")


    def image_callback(self, msg: Image):
        # 沒有 LiDAR 資料就先不算
        if self.last_scan is None:
            self.get_logger().warn_throttle = getattr(
                self.get_logger(), "warn", self.get_logger().info
            )
            self.get_logger().warn("No LaserScan received yet, skip this frame.")
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f"cv_bridge error: {e}")
            return

        h, w, _ = frame.shape

        # 取下半部當 ROI（假設床大多在畫面下方）
        roi = frame[int(h * 0.4):, :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # 先拿 TF（map -> base_link），用最新的 Transform
        try:
            t = self.tf_buffer.lookup_transform(
                'map', 'base_link',
                Time()  # 0 => latest
            )
        except TransformException as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return

        tx = t.transform.translation.x
        ty = t.transform.translation.y
        yaw = yaw_from_quaternion(t.transform.rotation)

        scan = self.last_scan
        angle_min = scan.angle_min
        angle_inc = scan.angle_increment
        ranges = scan.ranges
        range_min = scan.range_min
        range_max = scan.range_max
        n_ranges = len(ranges)

        for bed_name, cfg in COLOR_BEDS.items():
            lower = cfg["lower"]
            upper = cfg["upper"]

            mask = cv2.inRange(hsv, lower, upper)
            nonzero = np.count_nonzero(mask)
            total = mask.size
            ratio = nonzero / float(total)

            if ratio < RATIO_THRESHOLD:
                # 這個顏色太少，當作沒看到
                continue

            # 找出該顏色最大區塊的中心位置
            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            if not contours:
                continue

            c = max(contours, key=cv2.contourArea)
            x_box, y_box, w_box, h_box = cv2.boundingRect(c)

            # 水平中心在 ROI 裡的像素位置
            u_center = x_box + w_box / 2.0

            # 轉成 -1 ~ +1（左 -1、中 0、右 +1）
            u_norm = (u_center - w / 2.0) / (w / 2.0)

            # 對應到 camera 視角的偏移角度（假設光學軸對準 base_link x 正向）
            theta = u_norm * (H_FOV / 2.0)

            # 用這個角度去查 LiDAR 的距離
            # 假設 /scan 的角度是 [-pi, pi] or 類似，0 rad 朝前
            angle = theta
            r = get_lidar_range_for_angle(scan, angle, window=5)

            if r is None:
                # 這一幀在這個方向上沒有有效的 LiDAR 距離，就先略過
                # 如果你怕太吵，可以只在 ratio 很大的時候才印 log
                # self.get_logger().info(f"[{bed_name}] no valid LiDAR range at angle={angle:.3f}")
                continue


            # 在 base_link 座標系中，該床大致座標
            x_bl = r * math.cos(theta)
            y_bl = r * math.sin(theta)

            # 轉到 map 座標
            cos_yaw = math.cos(yaw)
            sin_yaw = math.sin(yaw)
            x_map = tx + cos_yaw * x_bl - sin_yaw * y_bl
            y_map = ty + sin_yaw * x_bl + cos_yaw * y_bl

            self.get_logger().info(
                f"[{bed_name}] ratio={ratio:.3f} "
                f"range={r:.2f} m "
                f"approx map=({x_map:.2f}, {y_map:.2f})"
            )
            self.points[bed_name].append((x_map, y_map))
            self.save_points_to_file()


def main(args=None):
    rclpy.init(args=args)
    node = MultiBedDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
