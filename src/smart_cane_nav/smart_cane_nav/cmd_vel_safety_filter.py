#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class CmdVelSafetyFilter(Node):
    """
    Minimal forward-collision safety filter.

    Philosophy:
      - ONLY stop when there is a REAL obstacle very close in front.
      - If scan is alive but front sector has no returns -> consider CLEAR.
      - If scan times out -> STOP (sensor failure).

    This is a "bumper-like" safety, not a planner.
    """

    def __init__(self):
        super().__init__('cmd_vel_safety_filter')

        # -------------------------
        # Parameters
        # -------------------------
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('in_cmd_topic', '/cmd_vel_align')
        self.declare_parameter('out_cmd_topic', '/cmd_vel_align_safe')

        # front check
        self.declare_parameter('front_half_angle_deg', 15.0)  # +/- degrees
        self.declare_parameter('stop_dist', 0.25)             # meters

        # sensor sanity
        self.declare_parameter('scan_timeout', 0.6)
        self.declare_parameter('min_valid_margin', 0.02)      # ignore < range_min + margin

        # logging
        self.declare_parameter('log_every_sec', 1.0)

        # -------------------------
        # Read params
        # -------------------------
        self.scan_topic = self.get_parameter('scan_topic').value
        self.in_cmd_topic = self.get_parameter('in_cmd_topic').value
        self.out_cmd_topic = self.get_parameter('out_cmd_topic').value

        self.front_half_angle = math.radians(
            float(self.get_parameter('front_half_angle_deg').value)
        )
        self.stop_dist = float(self.get_parameter('stop_dist').value)

        self.scan_timeout = float(self.get_parameter('scan_timeout').value)
        self.min_valid_margin = float(self.get_parameter('min_valid_margin').value)

        self.log_every_sec = float(self.get_parameter('log_every_sec').value)

        # -------------------------
        # ROS interfaces
        # -------------------------
        self.cmd_pub = self.create_publisher(Twist, self.out_cmd_topic, 10)
        self.create_subscription(Twist, self.in_cmd_topic, self.cmd_cb, 10)
        self.create_subscription(LaserScan, self.scan_topic, self.scan_cb, 10)

        # -------------------------
        # State
        # -------------------------
        self.last_scan_time = None
        self.front_blocked = False
        self.front_block_dist = None

        self.range_min = None
        self.range_max = None

        self.last_log_t = 0.0

        self.get_logger().info(
            f"[safety] front_half_angle_deg={math.degrees(self.front_half_angle):.1f}, "
            f"stop_dist={self.stop_dist:.2f}"
        )

    # ------------------------------------------------
    # Scan callback
    # ------------------------------------------------
    def scan_cb(self, msg: LaserScan):
        now = time.time()
        self.last_scan_time = now

        self.range_min = msg.range_min
        self.range_max = msg.range_max

        a0 = msg.angle_min
        inc = msg.angle_increment
        n = len(msg.ranges)

        if n == 0 or inc == 0.0:
            self.front_blocked = False
            self.front_block_dist = None
            return

        # indices for front sector
        i0 = int(((-self.front_half_angle) - a0) / inc)
        i1 = int(((+self.front_half_angle) - a0) / inc)

        i0 = max(0, min(i0, n - 1))
        i1 = max(0, min(i1, n - 1))
        if i1 < i0:
            i0, i1 = i1, i0

        min_valid = self.range_min + self.min_valid_margin

        blocked = False
        blocked_dist = None

        for r in msg.ranges[i0:i1 + 1]:
            if not math.isfinite(r):
                continue
            if r < min_valid or r > self.range_max:
                continue

            # ONLY care about very close obstacle
            if r < self.stop_dist:
                blocked = True
                blocked_dist = r
                break

        self.front_blocked = blocked
        self.front_block_dist = blocked_dist

        # throttled log
        if (now - self.last_log_t) >= self.log_every_sec:
            self.last_log_t = now
            if blocked:
                self.get_logger().info(
                    f"[scan] FRONT BLOCKED d={blocked_dist:.2f} < stop={self.stop_dist:.2f}"
                )
            else:
                self.get_logger().info(
                    f"[scan] FRONT CLEAR (no obstacle within {self.stop_dist:.2f} m)"
                )

    # ------------------------------------------------
    # Cmd callback
    # ------------------------------------------------
    def cmd_cb(self, msg: Twist):
        now = time.time()

        out = Twist()
        out.linear.x = msg.linear.x
        out.angular.z = msg.angular.z

        reason = "PASS"

        # only protect forward motion
        if out.linear.x > 0.0:

            # scan timeout -> stop
            if self.last_scan_time is None or (now - self.last_scan_time) > self.scan_timeout:
                out.linear.x = 0.0
                reason = "SCAN_TIMEOUT_STOP"

            # real close obstacle -> stop
            elif self.front_blocked:
                out.linear.x = 0.0
                reason = f"STOP_OBSTACLE d={self.front_block_dist:.2f}"

            # else: CLEAR -> PASS

        self.cmd_pub.publish(out)

        # log (throttled)
        if (now - self.last_log_t) >= self.log_every_sec:
            self.get_logger().info(
                f"[cmd] v_in={msg.linear.x:.2f} -> v_out={out.linear.x:.2f} "
                f"w={out.angular.z:.2f} reason={reason}"
            )


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
