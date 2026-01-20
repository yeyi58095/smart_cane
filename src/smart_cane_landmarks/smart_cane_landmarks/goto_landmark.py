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

        self.landmarks_path = str(self.get_parameter("landmarks_path").value)
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.prefer_nav = bool(self.get_parameter("prefer_nav").value)
        self.yaw_to_target = bool(self.get_parameter("yaw_to_target").value)
        self.nav_action = str(self.get_parameter("nav_action").value)

        self.align_enable_topic = str(self.get_parameter("align_enable_topic").value)
        self.align_target_topic = str(self.get_parameter("align_target_topic").value)

        # pubs
        self.align_enable_pub = self.create_publisher(Bool, self.align_enable_topic, 10)
        self.align_target_pub = self.create_publisher(String, self.align_target_topic, 10)

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Action client
        self.nav_client = ActionClient(self, NavigateToPose, self.nav_action)
        self._goal_handle = None

    def set_align(self, enable: bool, target: str = None):
        if target is not None:
            m = String()
            m.data = target
            self.align_target_pub.publish(m)
        b = Bool()
        b.data = bool(enable)
        self.align_enable_pub.publish(b)

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

        # ✅ enable align + set target
        self.set_align(True, name)

        # send goal
        goal = NavigateToPose.Goal()
        goal.pose = pose

        self.get_logger().info(f"Sending Nav2 goal: ({x:.3f}, {y:.3f}) target='{name}'")
        send_future = self.nav_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        self._goal_handle = send_future.result()

        if self._goal_handle is None or not self._goal_handle.accepted:
            self.get_logger().error("Nav2 goal rejected.")
            self.set_align(False)
            return 3

        self.get_logger().info("Nav2 goal accepted. Navigating... (Ctrl+C to cancel)")

        # wait result
        result_future = self._goal_handle.get_result_async()
        try:
            rclpy.spin_until_future_complete(self, result_future)
        except KeyboardInterrupt:
            self.get_logger().warn("Interrupted by user.")
            self.cancel_goal_if_any()
            self.set_align(False)
            return 130

        res = result_future.result()
        status = getattr(res, "status", None)

        # ✅ always disable align at end
        self.set_align(False)

        if status == 4:  # STATUS_SUCCEEDED in action_msgs/GoalStatus
            self.get_logger().info(f"✅ Nav2 arrived near landmark: {name}")
            return 0

        self.get_logger().warn(f"Nav2 finished with status={status} (not succeeded).")
        return 4


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
    finally:
        # make sure align disabled even if errors
        try:
            node.set_align(False)
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return int(code)


if __name__ == "__main__":
    raise SystemExit(main())
