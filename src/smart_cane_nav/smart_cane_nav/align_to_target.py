#!/usr/bin/env python3
import os
import time

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
from cv_bridge import CvBridge

from ultralytics import YOLO


def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


class AlignToTarget(Node):
    """
    Visual override controller with mux-friendly behavior:

    - Publishes ONLY to cmd_topic (default: /cmd_vel_align)
    - Does NOT publish when:
        * not enabled, or
        * target not seen, or
        * target lost > lost_timeout (release)
      => twist_mux will fall back to nav automatically.
    - On ARRIVED: publish stop, print, and exit process cleanly.

    Control inputs:
      /align/enable (std_msgs/Bool)
      /align/target_class (std_msgs/String)
    """

    def __init__(self):
        super().__init__('align_to_target')

        # -----------------------------
        # Parameters
        # -----------------------------
        self.declare_parameter('image_topic', '/tb3/camera/image_raw')
        self.declare_parameter('cmd_topic', '/cmd_vel_align')

        self.declare_parameter('target_class', 'bus')  # default (will be overwritten by /align/target_class)
        self.declare_parameter('yolo_model', os.path.expanduser('~/smart_cane/yolov8n.pt'))
        self.declare_parameter('conf_th', 0.6)

        # angular control
        self.declare_parameter('Kp_ang', 0.8)
        self.declare_parameter('max_w', 1.2)
        self.declare_parameter('center_tol_norm', 0.06)  # normalized pixel error tolerance for "aligned"

        # linear control (image-based distance proxy)
        self.declare_parameter('area_target', 25000.0)
        self.declare_parameter('area_min', 3000.0)
        self.declare_parameter('Kp_lin', 0.00004)
        self.declare_parameter('max_v', 0.15)

        # takeover / release
        self.declare_parameter('lost_timeout', 3.0)  # seconds
        self.declare_parameter('arrive_hold_sec', 0.4)  # hold stop for a short time before exit

        # topics for enable / target
        self.declare_parameter('enable_topic', '/align/enable')
        self.declare_parameter('target_topic', '/align/target_class')

        # -----------------------------
        # Read parameters
        # -----------------------------
        self.image_topic = str(self.get_parameter('image_topic').value)
        self.cmd_topic = str(self.get_parameter('cmd_topic').value)

        self.target_class = str(self.get_parameter('target_class').value)
        self.model_path = str(self.get_parameter('yolo_model').value)
        self.conf_th = float(self.get_parameter('conf_th').value)

        self.Kp_ang = float(self.get_parameter('Kp_ang').value)
        self.max_w = float(self.get_parameter('max_w').value)
        self.center_tol_norm = float(self.get_parameter('center_tol_norm').value)

        self.area_target = float(self.get_parameter('area_target').value)
        self.area_min = float(self.get_parameter('area_min').value)
        self.Kp_lin = float(self.get_parameter('Kp_lin').value)
        self.max_v = float(self.get_parameter('max_v').value)

        self.lost_timeout = float(self.get_parameter('lost_timeout').value)
        self.arrive_hold_sec = float(self.get_parameter('arrive_hold_sec').value)

        self.enable_topic = str(self.get_parameter('enable_topic').value)
        self.target_topic = str(self.get_parameter('target_topic').value)

        # -----------------------------
        # ROS interfaces
        # -----------------------------
        self.bridge = CvBridge()
        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)

        self.create_subscription(Image, self.image_topic, self.image_cb, 10)
        self.create_subscription(Bool, self.enable_topic, self.enable_cb, 10)
        self.create_subscription(String, self.target_topic, self.target_cb, 10)

        # -----------------------------
        # YOLO
        # -----------------------------
        self.get_logger().info(f"Loading YOLO model: {self.model_path}")
        self.model = YOLO(self.model_path)

        # -----------------------------
        # State
        # -----------------------------
        self.enabled = False
        self.last_seen_time = None
        self.tracking = False
        self._exit_armed = False
        self._exit_at = None

        self.get_logger().info(
            f"[align_to_target] ready. cmd_topic='{self.cmd_topic}', "
            f"image='{self.image_topic}', enable_topic='{self.enable_topic}', target_topic='{self.target_topic}', "
            f"default target='{self.target_class}'"
        )

    # -----------------------------
    def enable_cb(self, msg: Bool):
        prev = self.enabled
        self.enabled = bool(msg.data)

        if self.enabled and not prev:
            self.get_logger().info("🟢 align enabled")
            # reset tracking state
            self.last_seen_time = None
            self.tracking = False
            self._exit_armed = False
            self._exit_at = None

        if (not self.enabled) and prev:
            self.get_logger().info("⚪ align disabled (release to nav)")
            self._stop_once()
            self.last_seen_time = None
            self.tracking = False
            self._exit_armed = False
            self._exit_at = None

    def target_cb(self, msg: String):
        s = (msg.data or "").strip()
        if not s:
            return
        if s != self.target_class:
            self.target_class = s
            self.get_logger().info(f"🎯 target_class set to '{self.target_class}'")
            # reset tracking state when target changes
            self.last_seen_time = None
            self.tracking = False
            self._exit_armed = False
            self._exit_at = None

    # -----------------------------
    def image_cb(self, msg: Image):
        now = time.time()

        # If we have armed exit, hold stop a bit then exit cleanly
        if self._exit_armed:
            if self._exit_at is not None and now >= self._exit_at:
                # exit process cleanly
                rclpy.shutdown()
            return

        if not self.enabled:
            return  # do nothing, nav controls

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            return

        H, W, _ = frame.shape
        cx = W / 2.0

        # YOLO inference
        results = self.model.predict(frame, verbose=False, conf=self.conf_th)
        if (not results) or (results[0].boxes is None) or (len(results[0].boxes) == 0):
            self._check_lost(now)
            return

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
            self._check_lost(now)
            return

        # Target seen -> TRACKING
        self.last_seen_time = now
        if not self.tracking:
            self.tracking = True
            self.get_logger().info("🎯 Target detected, visual takeover")

        x1, y1, x2, y2 = [float(v) for v in best.xyxy[0]]
        u_center = (x1 + x2) / 2.0
        area = (x2 - x1) * (y2 - y1)

        # normalized horizontal error [-1,1]
        e = (u_center - cx) / cx

        # angular: steer to center
        w = clamp(-self.Kp_ang * e, -self.max_w, self.max_w)

        # linear: move forward when roughly centered
        v = 0.0
        if abs(e) < self.center_tol_norm and area > self.area_min:
            v = self.Kp_lin * (self.area_target - area)
            v = clamp(v, 0.0, self.max_v)

        # Arrival check: close enough (area large) AND almost centered
        if (area >= self.area_target) and (abs(e) < self.center_tol_norm):
            self.get_logger().info(f"✅ Arrived at target: {self.target_class}")
            self._stop_once()
            # arm exit after a short hold
            self._exit_armed = True
            self._exit_at = now + self.arrive_hold_sec
            return

        # publish override
        tw = Twist()
        tw.angular.z = float(w)
        tw.linear.x = float(v)
        self.cmd_pub.publish(tw)

    # -----------------------------
    def _stop_once(self):
        tw = Twist()
        tw.linear.x = 0.0
        tw.angular.z = 0.0
        self.cmd_pub.publish(tw)

    def _check_lost(self, now):
        if (not self.tracking) or (self.last_seen_time is None):
            return
        if now - self.last_seen_time > self.lost_timeout:
            self.tracking = False
            self.last_seen_time = None
            self.get_logger().info("👋 Target lost > timeout, release control to nav")
            # Important: do NOT publish continuously; just stop once (optional)
            self._stop_once()
            # then stop publishing -> mux falls back to nav


def main(args=None):
    rclpy.init(args=args)
    node = AlignToTarget()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    # When shutdown is called from inside, rclpy.ok() becomes False
    if rclpy.ok():
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
