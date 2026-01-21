#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


class CmdVelSafetyFilter(Node):
    """
    Sub:
      - /cmd_vel_align (Twist)    desired cmd from vision align
      - /scan (LaserScan)        obstacle distances
    Pub:
      - /cmd_vel_align_safe (Twist) filtered cmd (won't crash into obstacles)
    """

    def __init__(self):
        super().__init__('cmd_vel_safety_filter')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('in_cmd_topic', '/cmd_vel_align')
        self.declare_parameter('out_cmd_topic', '/cmd_vel_align_safe')

        # Front sector (degrees) to check for obstacles
        self.declare_parameter('front_half_angle_deg', 25.0)

        # Distance thresholds (meters)
        self.declare_parameter('stop_dist', 0.35)   # closer than this => stop forward
        self.declare_parameter('slow_dist', 0.75)   # within this => scale down forward

        # If scan not received recently, be conservative
        self.declare_parameter('scan_timeout', 0.6)

        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.in_cmd_topic = str(self.get_parameter('in_cmd_topic').value)
        self.out_cmd_topic = str(self.get_parameter('out_cmd_topic').value)

        self.front_half_angle = math.radians(float(self.get_parameter('front_half_angle_deg').value))
        self.stop_dist = float(self.get_parameter('stop_dist').value)
        self.slow_dist = float(self.get_parameter('slow_dist').value)
        self.scan_timeout = float(self.get_parameter('scan_timeout').value)

        self.cmd_pub = self.create_publisher(Twist, self.out_cmd_topic, 10)
        self.create_subscription(Twist, self.in_cmd_topic, self.cmd_cb, 10)
        self.create_subscription(LaserScan, self.scan_topic, self.scan_cb, 10)

        self._last_scan_time = None
        self._front_min = None

        self.get_logger().info(
            f"[safety_filter] scan='{self.scan_topic}', in='{self.in_cmd_topic}', out='{self.out_cmd_topic}', "
            f"front_half_angle_deg={math.degrees(self.front_half_angle):.1f}, stop={self.stop_dist}, slow={self.slow_dist}"
        )

    def scan_cb(self, msg: LaserScan):
        now = time.time()
        self._last_scan_time = now

        # Compute min distance in front sector
        a = msg.angle_min
        inc = msg.angle_increment

        # Indices for [-front_half_angle, +front_half_angle]
        i0 = int(( -self.front_half_angle - a) / inc)
        i1 = int(( +self.front_half_angle - a) / inc)

        i0 = max(0, min(i0, len(msg.ranges) - 1))
        i1 = max(0, min(i1, len(msg.ranges) - 1))
        if i1 < i0:
            i0, i1 = i1, i0

        front_ranges = []
        for r in msg.ranges[i0:i1+1]:
            if math.isfinite(r) and r > 0.0:
                front_ranges.append(r)

        self._front_min = min(front_ranges) if front_ranges else None

    def cmd_cb(self, msg: Twist):
        now = time.time()

        # Default: pass-through
        out = Twist()
        out.linear.x = float(msg.linear.x)
        out.angular.z = float(msg.angular.z)

        # If moving forward, apply safety based on scan
        if out.linear.x > 0.0:
            # if scan stale, be conservative: stop forward
            if self._last_scan_time is None or (now - self._last_scan_time) > self.scan_timeout:
                out.linear.x = 0.0
                self.cmd_pub.publish(out)
                return

            d = self._front_min
            if d is None:
                # no valid ranges => conservative
                out.linear.x = 0.0
                self.cmd_pub.publish(out)
                return

            if d < self.stop_dist:
                out.linear.x = 0.0
            elif d < self.slow_dist:
                # scale forward speed linearly between stop_dist..slow_dist
                scale = (d - self.stop_dist) / max(1e-6, (self.slow_dist - self.stop_dist))
                scale = clamp(scale, 0.0, 1.0)
                out.linear.x = out.linear.x * scale

        self.cmd_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelSafetyFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
