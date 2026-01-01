#!/usr/bin/env python3
import os
import re
from typing import Dict, List, Tuple

import rclpy
from rclpy.node import Node

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA


POINT_RE = re.compile(r"\(\s*([+-]?\d+(?:\.\d+)?)\s*,\s*([+-]?\d+(?:\.\d+)?)\s*\)")


# -------------------------
# Parsing
# -------------------------
def parse_points(line: str) -> List[Tuple[float, float]]:
    pts = []
    for m in POINT_RE.finditer(line):
        pts.append((float(m.group(1)), float(m.group(2))))
    return pts


def parse_landmarks_file(path: str, want_nav: bool, want_raw: bool) -> List[Tuple[str, str, List[Tuple[float, float]]]]:
    """
    Return list of (cls, kind, points)
    kind is "nav" or "raw"
    """
    out = []
    if not os.path.exists(path):
        return out

    with open(path, "r") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue

            # examples:
            # bus_raw: (x,y), ...
            # bus_nav: (x,y), ...
            if ":" not in line:
                continue

            key, _ = line.split(":", 1)

            if key.endswith("_nav") and want_nav:
                cls = key[:-4]
                out.append((cls, "nav", parse_points(line)))
            elif key.endswith("_raw") and want_raw:
                cls = key[:-4]
                out.append((cls, "raw", parse_points(line)))

    return out


# -------------------------
# Marker helpers
# -------------------------
def color_for_class(cls: str, alpha: float = 1.0) -> ColorRGBA:
    table = {
        "bus":        (1.0, 0.2, 0.2),
        "motorcycle": (0.2, 0.6, 1.0),
        "bird":       (0.2, 1.0, 0.2),
        "person":     (1.0, 1.0, 0.2),
    }
    r, g, b = table.get(cls, (1.0, 0.0, 0.0))
    return ColorRGBA(r=float(r), g=float(g), b=float(b), a=float(alpha))


def make_sphere_marker(frame_id: str, stamp, ns: str, mid: int,
                       x: float, y: float, z: float,
                       scale: float, color: ColorRGBA) -> Marker:
    m = Marker()
    m.header.frame_id = frame_id
    m.header.stamp = stamp
    m.ns = ns
    m.id = mid
    m.type = Marker.SPHERE
    m.action = Marker.ADD

    m.pose.position.x = float(x)
    m.pose.position.y = float(y)
    m.pose.position.z = float(z)
    m.pose.orientation.w = 1.0

    m.scale.x = float(scale)
    m.scale.y = float(scale)
    m.scale.z = float(scale)

    m.color = color
    return m


def make_text_marker(frame_id: str, stamp, ns: str, mid: int,
                     x: float, y: float, z: float,
                     text: str, text_size: float,
                     color: ColorRGBA) -> Marker:
    t = Marker()
    t.header.frame_id = frame_id
    t.header.stamp = stamp
    t.ns = ns
    t.id = mid
    t.type = Marker.TEXT_VIEW_FACING
    t.action = Marker.ADD

    t.pose.position.x = float(x)
    t.pose.position.y = float(y)
    t.pose.position.z = float(z)
    t.pose.orientation.w = 1.0

    t.scale.z = float(text_size)
    t.color = color
    t.text = text
    return t


def make_deleteall_marker(frame_id: str, stamp) -> Marker:
    m = Marker()
    m.header.frame_id = frame_id
    m.header.stamp = stamp
    m.action = Marker.DELETEALL
    return m


# -------------------------
# Node
# -------------------------
class LandmarkVisualizer(Node):
    def __init__(self):
        super().__init__("landmark_visualizer")

        self.declare_parameter("landmarks_path", os.path.expanduser("~/smart_cane/landmarks.txt"))
        self.declare_parameter("marker_topic", "/landmark_markers")
        self.declare_parameter("frame_id", "map")

        self.declare_parameter("scale", 0.25)        # 點大小
        self.declare_parameter("alpha", 1.0)
        self.declare_parameter("text_size", 0.22)    # 字大小（z scale）
        self.declare_parameter("point_z", 0.05)
        self.declare_parameter("text_z", 0.45)

        self.declare_parameter("show_nav", True)
        self.declare_parameter("show_raw", False)

        self.path = str(self.get_parameter("landmarks_path").value)
        self.marker_topic = str(self.get_parameter("marker_topic").value)
        self.frame_id = str(self.get_parameter("frame_id").value)

        self.scale = float(self.get_parameter("scale").value)
        self.alpha = float(self.get_parameter("alpha").value)
        self.text_size = float(self.get_parameter("text_size").value)
        self.point_z = float(self.get_parameter("point_z").value)
        self.text_z = float(self.get_parameter("text_z").value)

        self.show_nav = bool(self.get_parameter("show_nav").value)
        self.show_raw = bool(self.get_parameter("show_raw").value)

        self.pub = self.create_publisher(MarkerArray, self.marker_topic, 10)
        self.timer = self.create_timer(1.0, self.publish_markers)

        self.get_logger().info(f"Visualizing landmarks from {self.path}")
        self.get_logger().info(f"Publishing MarkerArray on {self.marker_topic}")
        self.get_logger().info(f"show_nav={self.show_nav}, show_raw={self.show_raw}")

    def publish_markers(self):
        stamp = self.get_clock().now().to_msg()

        entries = parse_landmarks_file(self.path, want_nav=self.show_nav, want_raw=self.show_raw)

        ma = MarkerArray()
        # 先刪舊的，避免殘留
        ma.markers.append(make_deleteall_marker(self.frame_id, stamp))

        mid = 0
        for cls, kind, pts in entries:
            if not pts:
                continue

            # nav 點亮一點、raw 點淡一點（方便你測試）
            alpha = self.alpha if kind == "nav" else min(self.alpha, 0.35)
            c = color_for_class(cls, alpha=alpha)

            for i, (x, y) in enumerate(pts):
                # 點
                ma.markers.append(
                    make_sphere_marker(
                        frame_id=self.frame_id,
                        stamp=stamp,
                        ns=f"{cls}_{kind}_pt",
                        mid=mid,
                        x=x, y=y, z=self.point_z,
                        scale=self.scale,
                        color=c
                    )
                )
                mid += 1

                # 字（顯示 cls/kind/index + 座標）
                label = f"{cls}_{kind}#{i} ({x:.2f},{y:.2f})"
                ma.markers.append(
                    make_text_marker(
                        frame_id=self.frame_id,
                        stamp=stamp,
                        ns=f"{cls}_{kind}_txt",
                        mid=mid,
                        x=x, y=y, z=self.text_z,
                        text=label,
                        text_size=self.text_size,
                        color=ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
                    )
                )
                mid += 1

        self.pub.publish(ma)


def main():
    rclpy.init()
    node = LandmarkVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
