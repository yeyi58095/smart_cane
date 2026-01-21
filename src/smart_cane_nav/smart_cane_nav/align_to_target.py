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
    Visual override controller for twist_mux arbitration.

    Publishes ONLY to cmd_topic (default: /cmd_vel_align).
    Does NOT publish when not needed (so mux can fall back to nav).

    Topics:
      Sub:
        /align/enable (Bool)
        /align/target_class (String)
        /align/nav_done (Bool)         # nav2 reached coarse goal => allow SEARCHING
      Pub:
        /align/status (String)         # IDLE/TRACKING/LOST/SEARCHING/ARRIVED/NOT_FOUND
    """

    def __init__(self):
        super().__init__('align_to_target')

        # -----------------------------
        # Parameters
        # -----------------------------
        self.declare_parameter('image_topic', '/tb3/camera/image_raw')
        self.declare_parameter('cmd_topic', '/cmd_vel_align')

        self.declare_parameter('target_class', 'bus')
        self.declare_parameter('yolo_model', os.path.expanduser('~/smart_cane/yolov8n.pt'))

        # Two-stage threshold:
        # - detect_th: consider "seen" if conf >= detect_th
        # - control_th: allow forward/arrived only if conf >= control_th
        self.declare_parameter('detect_th', 0.4)
        self.declare_parameter('control_th', 0.6)

        # angular control
        self.declare_parameter('Kp_ang', 0.9)
        self.declare_parameter('max_w', 1.2)
        self.declare_parameter('center_tol_norm', 0.07)  # bbox center tolerance (normalized)

        # linear control (image proxy distance by bbox area)
        self.declare_parameter('area_target', 18000.0)   # ARRIVED threshold (tune)
        self.declare_parameter('area_min', 2500.0)       # ignore tiny far bbox
        self.declare_parameter('Kp_lin', 0.00005)
        self.declare_parameter('max_v', 0.16)

        # tracking release (during nav)
        self.declare_parameter('lost_timeout', 3.0)  # seconds

        # searching (only when nav_done=True)
        self.declare_parameter('search_w', 0.6)            # rad/s rotation during search
        self.declare_parameter('search_max_sec', 11.0)     # ~one circle
        self.declare_parameter('search_start_delay', 0.2)  # avoid immediate search on tiny dropouts

        # arrived exit hold
        self.declare_parameter('arrive_hold_sec', 0.4)

        # control topics
        self.declare_parameter('enable_topic', '/align/enable')
        self.declare_parameter('target_topic', '/align/target_class')
        self.declare_parameter('nav_done_topic', '/align/nav_done')
        self.declare_parameter('status_topic', '/align/status')

        # -----------------------------
        # Read parameters
        # -----------------------------
        self.image_topic = str(self.get_parameter('image_topic').value)
        self.cmd_topic = str(self.get_parameter('cmd_topic').value)

        self.target_class = str(self.get_parameter('target_class').value)
        self.model_path = str(self.get_parameter('yolo_model').value)

        self.detect_th = float(self.get_parameter('detect_th').value)
        self.control_th = float(self.get_parameter('control_th').value)

        self.Kp_ang = float(self.get_parameter('Kp_ang').value)
        self.max_w = float(self.get_parameter('max_w').value)
        self.center_tol_norm = float(self.get_parameter('center_tol_norm').value)

        self.area_target = float(self.get_parameter('area_target').value)
        self.area_min = float(self.get_parameter('area_min').value)
        self.Kp_lin = float(self.get_parameter('Kp_lin').value)
        self.max_v = float(self.get_parameter('max_v').value)

        self.lost_timeout = float(self.get_parameter('lost_timeout').value)

        self.search_w = float(self.get_parameter('search_w').value)
        self.search_max_sec = float(self.get_parameter('search_max_sec').value)
        self.search_start_delay = float(self.get_parameter('search_start_delay').value)

        self.arrive_hold_sec = float(self.get_parameter('arrive_hold_sec').value)

        self.enable_topic = str(self.get_parameter('enable_topic').value)
        self.target_topic = str(self.get_parameter('target_topic').value)
        self.nav_done_topic = str(self.get_parameter('nav_done_topic').value)
        self.status_topic = str(self.get_parameter('status_topic').value)

        # -----------------------------
        # ROS interfaces
        # -----------------------------
        self.bridge = CvBridge()
        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 10)

        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.create_subscription(Image, self.image_topic, self.image_cb, 10)
        self.create_subscription(Bool, self.enable_topic, self.enable_cb, 10)
        self.create_subscription(String, self.target_topic, self.target_cb, 10)
        self.create_subscription(Bool, self.nav_done_topic, self.nav_done_cb, 10)

        # -----------------------------
        # YOLO
        # -----------------------------
        self.get_logger().info(f"Loading YOLO model: {self.model_path}")
        self.model = YOLO(self.model_path)

        # -----------------------------
        # State
        # -----------------------------
        self.enabled = False
        self.nav_done = False

        self.last_seen_time = None
        self.tracking = False

        self.searching = False
        self.search_start_time = None
        self.last_missing_time = None

        self._exit_armed = False
        self._exit_at = None

        self._status = "IDLE"
        self._last_rotate_only_log = 0.0

        self._set_status("IDLE")

        self.get_logger().info(
            f"[align_to_target] ready. target='{self.target_class}', detect_th={self.detect_th}, control_th={self.control_th}, "
            f"cmd='{self.cmd_topic}', image='{self.image_topic}'"
        )

    # -----------------------------
    def _set_status(self, s: str):
        if s != self._status:
            self._status = s
            msg = String()
            msg.data = s
            self.status_pub.publish(msg)

    def _stop_once(self):
        tw = Twist()
        tw.linear.x = 0.0
        tw.angular.z = 0.0
        self.cmd_pub.publish(tw)

    def _reset_runtime_state(self):
        self.last_seen_time = None
        self.tracking = False
        self.searching = False
        self.search_start_time = None
        self.last_missing_time = None
        self._exit_armed = False
        self._exit_at = None
        self._last_rotate_only_log = 0.0

    # -----------------------------
    def enable_cb(self, msg: Bool):
        prev = self.enabled
        self.enabled = bool(msg.data)

        if self.enabled and not prev:
            self.get_logger().info("🟢 align enabled")
            self._reset_runtime_state()
            self._set_status("IDLE")

        if (not self.enabled) and prev:
            self.get_logger().info("⚪ align disabled (release)")
            self._stop_once()
            self._reset_runtime_state()
            self._set_status("IDLE")

    def nav_done_cb(self, msg: Bool):
        prev = self.nav_done
        self.nav_done = bool(msg.data)

        if self.nav_done and not prev:
            self.get_logger().info("🧭 nav_done=True (if target not visible => SEARCHING allowed)")

        if (not self.nav_done) and prev:
            self.get_logger().info("🧭 nav_done=False")
            self.searching = False
            self.search_start_time = None
            self.last_missing_time = None

    def target_cb(self, msg: String):
        s = (msg.data or "").strip()
        if not s:
            return
        if s != self.target_class:
            self.target_class = s
            self.get_logger().info(f"🎯 target_class set to '{self.target_class}'")
            self._reset_runtime_state()
            self._set_status("IDLE")

    # -----------------------------
    def image_cb(self, msg: Image):
        now = time.time()

        # delayed shutdown after ARRIVED
        if self._exit_armed:
            if self._exit_at is not None and now >= self._exit_at:
                rclpy.shutdown()
            return

        if not self.enabled:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            return

        H, W, _ = frame.shape
        cx = W / 2.0

        # YOLO: use detect_th (looser) so we can "see" earlier
        results = self.model.predict(frame, verbose=False, conf=self.detect_th)

        best = None
        best_conf = -1.0

        if results and results[0].boxes is not None and len(results[0].boxes) > 0:
            boxes = results[0].boxes
            names = results[0].names

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
            self._handle_not_seen(now)
            return

        # Seen
        self.last_seen_time = now
        self.last_missing_time = None

        # Enter TRACKING
        if not self.tracking:
            self.tracking = True
            self.searching = False
            self.search_start_time = None
            self.get_logger().info(f"🎯 Target seen (conf={best_conf:.2f}), switch to TRACKING (visual takeover)")
            self._set_status("TRACKING")

        x1, y1, x2, y2 = [float(v) for v in best.xyxy[0]]
        u_center = (x1 + x2) / 2.0
        area = (x2 - x1) * (y2 - y1)

        # normalized horizontal error [-1,1]
        e = (u_center - cx) / cx

        # conf gate
        allow_forward = (best_conf >= self.control_th)

        # angular: always try to center
        w = clamp(-self.Kp_ang * e, -self.max_w, self.max_w)

        # linear: only move forward when conf strong + centered + bbox not tiny
        v = 0.0
        if allow_forward and abs(e) < self.center_tol_norm and area > self.area_min:
            v = self.Kp_lin * (self.area_target - area)
            v = clamp(v, 0.0, self.max_v)

        # Low conf: rotate-only log (throttled)
        if not allow_forward:
            if now - self._last_rotate_only_log > 1.0:
                self.get_logger().info(
                    f"👀 Seen '{self.target_class}' conf={best_conf:.2f} (<{self.control_th}), rotate-only (no forward/arrive)"
                )
                self._last_rotate_only_log = now

        # Arrival: must be confident AND centered AND close enough
        if allow_forward and (area >= self.area_target) and (abs(e) < self.center_tol_norm):
            self.get_logger().info(f"✅ Arrived visually at target: {self.target_class} (area={area:.0f}, e={e:.3f})")
            self._set_status("ARRIVED")
            self._stop_once()
            self._exit_armed = True
            self._exit_at = now + self.arrive_hold_sec
            return

        # publish override
        tw = Twist()
        tw.angular.z = float(w)
        tw.linear.x = float(v)
        self.cmd_pub.publish(tw)

    # -----------------------------
    def _handle_not_seen(self, now: float):
        # initialize missing time
        if self.last_missing_time is None:
            self.last_missing_time = now

        # If we were tracking, check lost timeout
        if self.tracking and self.last_seen_time is not None:
            if now - self.last_seen_time > self.lost_timeout:
                self.tracking = False
                self.last_seen_time = None
                self.get_logger().info("👋 Target lost > lost_timeout")
                if not self.nav_done:
                    self._set_status("LOST")
                    self._stop_once()
                    return

        # If nav not done yet: do nothing (release to nav)
        if not self.nav_done:
            # status hint
            if self.tracking:
                self._set_status("LOST")
            else:
                self._set_status("IDLE")
            return

        # nav_done=True => allow SEARCHING, but wait a small delay to avoid jitter
        if (now - self.last_missing_time) < self.search_start_delay and not self.searching:
            self._set_status("IDLE")
            return

        # start searching
        if not self.searching:
            self.searching = True
            self.search_start_time = now
            self.get_logger().info("🔎 nav_done but target not visible -> SEARCHING (rotate to find)")
            self._set_status("SEARCHING")

        # timeout searching
        if self.search_start_time is not None and (now - self.search_start_time) > self.search_max_sec:
            self.get_logger().warn(f"❌ SEARCH timeout: still cannot see target '{self.target_class}'")
            self._set_status("NOT_FOUND")
            self._stop_once()
            self.searching = False
            self.search_start_time = None
            return

        # publish rotation command (takeover via mux)
        tw = Twist()
        tw.linear.x = 0.0
        tw.angular.z = float(self.search_w)
        self.cmd_pub.publish(tw)


def main(args=None):
    rclpy.init(args=args)
    node = AlignToTarget()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    if rclpy.ok():
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
