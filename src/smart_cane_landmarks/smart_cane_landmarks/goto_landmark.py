#!/usr/bin/env python3
import os
import re
import sys
import math
from typing import List, Tuple, Optional

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

from tf2_ros import Buffer, TransformListener, TransformException


POINT_RE = re.compile(r"\(\s*([+-]?\d+(?:\.\d+)?)\s*,\s*([+-]?\d+(?:\.\d+)?)\s*\)")

def parse_points(line: str) -> List[Tuple[float, float]]:
    pts = []
    for m in POINT_RE.finditer(line):
        pts.append((float(m.group(1)), float(m.group(2))))
    return pts

def load_landmarks(path: str) -> dict:
    out = {}
    if not os.path.exists(path):
        return out
    with open(path, "r") as f:
        for raw in f:
            s = raw.strip()
            if not s or ":" not in s:
                continue
            k, v = s.split(":", 1)
            k = k.strip()
            pts = parse_points(v)
            if pts:
                out[k] = pts
    return out


class GotoLandmark(Node):
    def __init__(self):
        super().__init__("goto_landmark")

        # parameters
        self.declare_parameter("landmarks_path", os.path.expanduser("~/smart_cane/landmarks.txt"))
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("prefer_nav", True)
        self.declare_parameter("yaw_to_target", False)

        # Nav2 action server name (Foxy default)
        self.declare_parameter("nav_action", "/navigate_to_pose")

        # Align control topics
        self.declare_parameter("align_enable_topic", "/align/enable")
        self.declare_parameter("align_target_topic", "/align/target_class")
        self.declare_parameter("align_nav_done_topic", "/align/nav_done")
        self.declare_parameter("align_status_topic", "/align/status")

        # Visual success requirement
        self.declare_parameter("visual_timeout", 20.0)  # seconds to wait for visual ARRIVED after nav success

        self.landmarks_path = str(self.get_parameter("landmarks_path").value)
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.prefer_nav = bool(self.get_parameter("prefer_nav").value)
        self.yaw_to_target = bool(self.get_parameter("yaw_to_target").value)
        self.nav_action = str(self.get_parameter("nav_action").value)

        self.align_enable_topic = str(self.get_parameter("align_enable_topic").value)
        self.align_target_topic = str(self.get_parameter("align_target_topic").value)
        self.align_nav_done_topic = str(self.get_parameter("align_nav_done_topic").value)
        self.align_status_topic = str(self.get_parameter("align_status_topic").value)

        self.visual_timeout = float(self.get_parameter("visual_timeout").value)

        # pubs
        self.align_enable_pub = self.create_publisher(Bool, self.align_enable_topic, 10)
        self.align_target_pub = self.create_publisher(String, self.align_target_topic, 10)
        self.align_nav_done_pub = self.create_publisher(Bool, self.align_nav_done_topic, 10)

        # status sub
        self.align_status = "IDLE"
        self.create_subscription(String, self.align_status_topic, self._status_cb, 10)

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Action client
        self.nav_client = ActionClient(self, NavigateToPose, self.nav_action)
        self._goal_handle = None

    def _status_cb(self, msg: String):
        s = (msg.data or "").strip()
        if s and s != self.align_status:
            self.align_status = s
            self.get_logger().info(f"[align_status] {self.align_status}")

    def set_align(self, enable: bool, target: str = None):
        if target is not None:
            m = String()
            m.data = target
            self.align_target_pub.publish(m)
        b = Bool()
        b.data = bool(enable)
        self.align_enable_pub.publish(b)

    def set_nav_done(self, done: bool):
        b = Bool()
        b.data = bool(done)
        self.align_nav_done_pub.publish(b)

    def get_robot_xy(self) -> Optional[Tuple[float, float]]:
        try:
            t = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, Time())
            return (float(t.transform.translation.x), float(t.transform.translation.y))
        except TransformException as e:
            self.get_logger().warn(f"TF lookup failed ({self.map_frame}<-{self.base_frame}): {e}")
            return None

    def choose_point(self, pts: List[Tuple[float, float]]) -> Tuple[float, float]:
        if not pts:
            raise RuntimeError("empty pts")

        robot = self.get_robot_xy()
        if robot is None:
            return pts[0]

        rx, ry = robot
        best = pts[0]
        best_d = None
        for (x, y) in pts:
            d = math.hypot(x - rx, y - ry)
            if best_d is None or d < best_d:
                best_d = d
                best = (x, y)
        return best

    @staticmethod
    def yaw_to_quat(yaw: float):
        z = math.sin(yaw * 0.5)
        w = math.cos(yaw * 0.5)
        return z, w

    def make_pose(self, x: float, y: float) -> PoseStamped:
        msg = PoseStamped()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = 0.0

        z, w = 0.0, 1.0
        if self.yaw_to_target:
            robot = self.get_robot_xy()
            if robot is not None:
                rx, ry = robot
                yaw = math.atan2(y - ry, x - rx)
                z, w = self.yaw_to_quat(yaw)

        msg.pose.orientation.z = float(z)
        msg.pose.orientation.w = float(w)
        return msg

    def cancel_goal_if_any(self):
        if self._goal_handle is None:
            return
        try:
            self.get_logger().warn("Canceling Nav2 goal...")
            future = self._goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        except Exception as e:
            self.get_logger().warn(f"Cancel failed: {e}")

    def run(self, name: str) -> int:
        # wait action server
        self.get_logger().info(f"Waiting for Nav2 action server: {self.nav_action}")
        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Nav2 action server not available.")
            return 2

        db = load_landmarks(self.landmarks_path)
        if not db:
            self.get_logger().error(f"Landmark file not found or empty: {self.landmarks_path}")
            return 1

        key_nav = f"{name}_nav"
        key_raw = f"{name}_raw"

        pts = None
        if self.prefer_nav and key_nav in db:
            pts = db[key_nav]
            self.get_logger().info(f"Using NAV points: {key_nav} ({len(pts)} pts)")
        elif key_raw in db:
            pts = db[key_raw]
            self.get_logger().info(f"Using RAW points: {key_raw} ({len(pts)} pts)")
        elif name in db:
            pts = db[name]
            self.get_logger().info(f"Using points: {name} ({len(pts)} pts)")

        if not pts:
            self.get_logger().error(f"No points found for '{name}'. Tried: {key_nav}, {key_raw}")
            self.get_logger().info(f"Available keys: {', '.join(sorted(db.keys()))}")
            return 1

        x, y = self.choose_point(pts)
        pose = self.make_pose(x, y)

        # ✅ Start: enable align + set target; nav_done=False initially
        self.set_align(True, name)
        self.set_nav_done(False)

        # send goal
        goal = NavigateToPose.Goal()
        goal.pose = pose

        self.get_logger().info(f"Sending Nav2 goal: ({x:.3f}, {y:.3f}) target='{name}'")
        send_future = self.nav_client.send_goal_async(goal)

        # spin until goal accepted, but also allow align status to come in
        while rclpy.ok() and not send_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
            # If align already arrived visually early, cancel nav immediately
            if self.align_status == "ARRIVED":
                self.get_logger().info("✅ Visual ARRIVED before nav accepted -> cancel nav, success")
                self.set_nav_done(True)
                self.set_align(False)
                return 0

        if not rclpy.ok():
            self.set_align(False)
            return 130

        self._goal_handle = send_future.result()

        if self._goal_handle is None or not self._goal_handle.accepted:
            self.get_logger().error("Nav2 goal rejected.")
            self.set_align(False)
            return 3

        self.get_logger().info("Nav2 goal accepted. Navigating... (Ctrl+C to cancel)")

        result_future = self._goal_handle.get_result_async()

        # Main loop: wait either visual ARRIVED or nav result
        nav_succeeded = False
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

            if self.align_status == "ARRIVED":
                self.get_logger().info("✅ Visual ARRIVED -> cancel nav goal and finish")
                self.cancel_goal_if_any()
                self.set_align(False)
                return 0

            if result_future.done():
                res = result_future.result()
                status = getattr(res, "status", None)
                # 4 = SUCCEEDED
                if status == 4:
                    nav_succeeded = True
                else:
                    self.get_logger().warn(f"Nav2 finished with status={status} (not succeeded).")
                break

        if not rclpy.ok():
            self.cancel_goal_if_any()
            self.set_align(False)
            return 130

        # If nav failed, stop here (visual may still find target but you're not moving anymore)
        if not nav_succeeded:
            self.set_align(False)
            return 4

        # ✅ nav succeeded, but success must be VISUAL
        self.get_logger().info("🧭 Nav2 reached coarse goal. Now require VISUAL ARRIVED.")
        self.set_nav_done(True)  # allow align to SEARCH (rotate) if target not visible

        t0 = self.get_clock().now()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

            if self.align_status == "ARRIVED":
                self.get_logger().info("✅ Visual ARRIVED after nav success.")
                self.set_align(False)
                return 0

            dt = (self.get_clock().now() - t0).nanoseconds * 1e-9
            if dt > self.visual_timeout:
                self.get_logger().warn("❌ Visual timeout: nav arrived but target still not visually ARRIVED.")
                self.set_align(False)
                return 5

        self.set_align(False)
        return 130
    
    def cleanup_action_client(self):
        """
        Prevent rclpy.action.client.ActionClient.__del__ from trying to destroy
        after the node handle is already destroyed.
        """
        try:
            # if a goal is active, try cancel once (best-effort)
            if self._goal_handle is not None:
                try:
                    future = self._goal_handle.cancel_goal_async()
                    rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
                except Exception:
                    pass

            # Explicitly destroy the action client before destroying the node
            if getattr(self, "nav_client", None) is not None:
                self.nav_client.destroy()
                self.nav_client = None
        except Exception as e:
            self.get_logger().warn(f"cleanup_action_client failed: {e}")



def main():
    rclpy.init()

    if len(sys.argv) < 2:
        print("Usage: ros2 run smart_cane_landmarks goto_landmark <name>")
        rclpy.shutdown()
        return 2

    name = sys.argv[1].strip()
    node = GotoLandmark()

    try:
        code = node.run(name)
    except KeyboardInterrupt:
        code = 130
        node.cancel_goal_if_any()
    finally:
        # Always disable align at end
        try:
            node.set_align(False)
            node.set_nav_done(False)
        except Exception:
            pass
                # Make shutdown pretty: destroy ActionClient before node is destroyed
        try:
            node.cleanup_action_client()
        except Exception:
            pass

        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


    return int(code)


if __name__ == "__main__":
    raise SystemExit(main())
