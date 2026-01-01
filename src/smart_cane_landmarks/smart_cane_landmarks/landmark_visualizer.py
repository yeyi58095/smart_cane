#!/usr/bin/env python3
import os
import re

import rclpy
from rclpy.node import Node

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


POINT_RE = re.compile(r"\(\s*([+-]?\d+(?:\.\d+)?)\s*,\s*([+-]?\d+(?:\.\d+)?)\s*\)")


def parse_points(line: str):
    pts = []
    for m in POINT_RE.finditer(line):
        pts.append((float(m.group(1)), float(m.group(2))))
    return pts


class LandmarkVisualizer(Node):
    def __init__(self):
        super().__init__('landmark_visualizer')

        self.declare_parameter(
            'landmarks_path',
            os.path.expanduser('~/smart_cane/landmarks.txt')
        )
        self.declare_parameter('marker_topic', '/landmark_markers')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('scale', 0.25)   # 點的大小
        self.declare_parameter('alpha', 1.0)

        self.path = self.get_parameter('landmarks_path').value
        self.marker_topic = self.get_parameter('marker_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.scale = float(self.get_parameter('scale').value)
        self.alpha = float(self.get_parameter('alpha').value)

        self.pub = self.create_publisher(Marker, self.marker_topic, 10)

        # 每秒重畫一次（檔案有更新也會反映）
        self.timer = self.create_timer(1.0, self.publish_markers)

        self.get_logger().info(f"Visualizing landmarks from {self.path}")
        self.get_logger().info(f"Publishing markers on {self.marker_topic}")

    def publish_markers(self):
        if not os.path.exists(self.path):
            return

        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'landmarks'
        marker.id = 0
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD

        marker.scale.x = self.scale
        marker.scale.y = self.scale
        marker.scale.z = self.scale

        # 🔴 紅色點點
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = self.alpha

        marker.points = []

        with open(self.path, 'r') as f:
            for line in f:
                # 只畫 *_nav（你已經 snap 過）
                if '_nav:' not in line:
                    continue

                pts = parse_points(line)
                for x, y in pts:
                    p = Point()
                    p.x = x
                    p.y = y
                    p.z = 0.05
                    marker.points.append(p)

        self.pub.publish(marker)


def main():
    rclpy.init()
    node = LandmarkVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
