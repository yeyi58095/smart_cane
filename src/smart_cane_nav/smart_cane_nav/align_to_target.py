#!/usr/bin/env python3
import os

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

from ultralytics import YOLO


class AlignToTarget(Node):
    """
    Visual override controller:
    - Only publishes /cmd_vel when target is visible
    - When target disappears, publishes NOTHING (hands control back)
    - Angular control: center target in image
    - Linear control: approach target using bbox area (image-based distance)
    - Once arrived, announce once
    """

    def __init__(self):
        super().__init__('align_to_target')

        # -----------------------------
        # Parameters
        # -----------------------------
        self.declare_parameter('image_topic', '/tb3/camera/image_raw')
        self.declare_parameter('cmd_topic', '/cmd_vel')

        self.declare_parameter('target_class', 'bus')
        self.declare_parameter('yolo_model', os.path.expanduser('~/smart_cane/yolov8n.pt'))
        self.declare_parameter('conf_th', 0.6)

        # angular control
        self.declare_parameter('Kp_ang', 0.8)
        self.declare_parameter('max_w', 1.2)
        self.declare_parameter('center_tol', 0.05)

        # linear control (image-based distance)
        self.declare_parameter('area_target', 25000.0)   # close enough
        self.declare_parameter('area_min', 3000.0)       # too far -> do nothing
        self.declare_parameter('Kp_lin', 0.00004)
        self.declare_parameter('max_v', 0.15)

        # -----------------------------
        # Read parameters
        # -----------------------------
        self.image_topic = self.get_parameter('image_topic').value
        self.cmd_topic = self.get_parameter('cmd_topic').value

        self.target_class = self.get_parameter('target_class').value
        self.model_path = self.get_parameter('yolo_model').value
        self.conf_th = float(self.get_parameter('conf_th').value)

        self.Kp_ang = float(self.get_parameter('Kp_ang').value)
        self.max_w = float(self.get_parameter('max_w').value)
        self.center_tol = float(self.get_parameter('center_tol').value)

        self.area_target = float(self.get_parameter('area_target').value)
        self.area_min = float(self.get_parameter('area_min').value)
        self.Kp_lin = float(self.get_parameter('Kp_lin').value)
        self.max_v = float(self.get_parameter('max_v').value)

        # -----------------------------
        # ROS interfaces
        # -----------------------------
        self.bridge = CvBridge()
        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)
        self.create_subscription(Image, self.image_topic, self.image_cb, 10)

        # -----------------------------
        # YOLO
        # -----------------------------
        self.get_logger().info(f"Loading YOLO model: {self.model_path}")
        self.model = YOLO(self.model_path)

        self.get_logger().info(
            f"[align_to_target] target='{self.target_class}', "
            f"image='{self.image_topic}', cmd='{self.cmd_topic}'"
        )

        # -----------------------------
        # State
        # -----------------------------
        self.arrived = False   # 👈 用來確保只說一次

    # ----------------------------------------------------
    # Image callback
    # ----------------------------------------------------
    def image_cb(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f"cv_bridge error: {e}")
            return

        H, W, _ = frame.shape
        cx = W / 2.0

        # -----------------------------
        # YOLO inference
        # -----------------------------
        results = self.model.predict(frame, verbose=False, conf=self.conf_th)
        if not results or results[0].boxes is None:
            return  # 👈 看不到 → 放手

        boxes = results[0].boxes
        names = results[0].names

        best = None
        best_conf = -1.0

        for b in boxes:
            conf = float(b.conf[0])
            cls_id = int(b.cls[0])
            cls_name = names.get(cls_id, str(cls_id))

            if cls_name != self.target_class:
                continue

            if conf > best_conf:
                best_conf = conf
                best = b

        if best is None:
            return  # 👈 看不到 → 放手

        # -----------------------------
        # Bounding box info
        # -----------------------------
        x1, y1, x2, y2 = [float(v) for v in best.xyxy[0]]
        u_center = (x1 + x2) / 2.0

        bbox_w = x2 - x1
        bbox_h = y2 - y1
        area = bbox_w * bbox_h

        # -----------------------------
        # Angular control (center target)
        # -----------------------------
        e = (u_center - cx) / cx  # normalized [-1, 1]
        w = -self.Kp_ang * e
        w = max(-self.max_w, min(self.max_w, w))

        # -----------------------------
        # Linear control (approach target)
        # -----------------------------
        v = 0.0
        if area > self.area_min:
            v = self.Kp_lin * (self.area_target - area)
            v = max(0.0, min(self.max_v, v))

        # -----------------------------
        # Arrival detection
        # -----------------------------
        if area >= self.area_target:
            v = 0.0
            if not self.arrived:
                self.arrived = True
                self.get_logger().info(
                    f"✅ Arrived at target: {self.target_class}"
                )

        # -----------------------------
        # Publish cmd_vel (visual override)
        # -----------------------------
        tw = Twist()
        tw.angular.z = float(w)
        tw.linear.x = float(v)

        self.cmd_pub.publish(tw)


def main(args=None):
    rclpy.init(args=args)
    node = AlignToTarget()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
