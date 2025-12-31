#!/usr/bin/env python3
import os
import re
import sys
import math
from typing import List, Tuple, Optional

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener, TransformException


POINT_RE = re.compile(r"\(\s*([+-]?\d+(?:\.\d+)?)\s*,\s*([+-]?\d+(?:\.\d+)?)\s*\)")

def parse_points(line: str) -> List[Tuple[float, float]]:
    """Extract all (x,y) pairs from a line."""
    pts = []
    for m in POINT_RE.finditer(line):
        x = float(m.group(1))
        y = float(m.group(2))
        pts.append((x, y))
    return pts

def load_landmarks(path: str) -> dict:
    """
    Parse file lines like:
      bus_nav: (x,y), (x,y)
      bus_raw: (x,y)
    Return dict key->list[(x,y)]
    """
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
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("prefer_nav", True)   # prefer <name>_nav over <name>_raw
        self.declare_parameter("yaw_to_target", False)  # if True, set orientation toward target

        self.landmarks_path = str(self.get_parameter("landmarks_path").value)
        self.goal_topic = str(self.get_parameter("goal_topic").value)
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.prefer_nav = bool(self.get_parameter("prefer_nav").value)
        self.yaw_to_target = bool(self.get_parameter("yaw_to_target").value)

        self.pub = self.create_publisher(PoseStamped, self.goal_topic, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def get_robot_xy(self) -> Optional[Tuple[float, float]]:
        try:
            t = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, Time())
            return (float(t.transform.translation.x), float(t.transform.translation.y))
        except TransformException as e:
            self.get_logger().warn(f"TF lookup failed ({self.map_frame}<-{self.base_frame}): {e}")
            return None

    def choose_point(self, pts: List[Tuple[float, float]]) -> Tuple[float, float]:
        """
        Choose nearest point to current robot position if TF available,
        otherwise choose first.
        """
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
        # planar yaw -> quaternion (z,w only)
        # q = [0,0,sin(y/2),cos(y/2)]
        z = math.sin(yaw * 0.5)
        w = math.cos(yaw * 0.5)
        return z, w

    def publish_goal(self, x: float, y: float):
        msg = PoseStamped()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = 0.0

        # default orientation
        z, w = 0.0, 1.0

        if self.yaw_to_target:
            robot = self.get_robot_xy()
            if robot is not None:
                rx, ry = robot
                yaw = math.atan2(y - ry, x - rx)
                z, w = self.yaw_to_quat(yaw)

        msg.pose.orientation.z = float(z)
        msg.pose.orientation.w = float(w)

        self.pub.publish(msg)
        self.get_logger().info(f"Published goal to {self.goal_topic}: ({x:.3f}, {y:.3f}) frame={self.map_frame}")

def main():
    rclpy.init()

    if len(sys.argv) < 2:
        print("Usage: ros2 run smart_cane_perception goto_landmark <name>")
        print("Example: ros2 run smart_cane_perception goto_landmark motorcycle")
        rclpy.shutdown()
        return 2

    name = sys.argv[1].strip()

    node = GotoLandmark()

    # wait a moment so TF/listeners are alive
    for _ in range(10):
        rclpy.spin_once(node, timeout_sec=0.1)

    db = load_landmarks(node.landmarks_path)
    if not db:
        node.get_logger().error(f"Landmark file not found or empty: {node.landmarks_path}")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    key_nav = f"{name}_nav"
    key_raw = f"{name}_raw"

    pts = None
    if node.prefer_nav and key_nav in db:
        pts = db[key_nav]
        node.get_logger().info(f"Using NAV points: {key_nav} ({len(pts)} pts)")
    elif key_raw in db:
        pts = db[key_raw]
        node.get_logger().info(f"Using RAW points: {key_raw} ({len(pts)} pts)")
    else:
        # fallback: exact match (in case you stored without suffix)
        if name in db:
            pts = db[name]
            node.get_logger().info(f"Using points: {name} ({len(pts)} pts)")

    if not pts:
        node.get_logger().error(f"No points found for '{name}'. Tried: {key_nav}, {key_raw}")
        node.get_logger().info(f"Available keys: {', '.join(sorted(db.keys()))}")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    x, y = node.choose_point(pts)
    node.publish_goal(x, y)

    # let it flush publish
    rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_node()
    rclpy.shutdown()
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
