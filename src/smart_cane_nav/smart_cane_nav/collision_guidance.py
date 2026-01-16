#!/usr/bin/env python3
'''import math
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


def finite_min(vals):
    best = None
    for v in vals:
        if math.isinf(v) or math.isnan(v):
            continue
        if best is None or v < best:
            best = v
    return best


class CollisionGuidance(Node):
    """
    Subscribe:
      - /scan
      - /cmd_vel (user teleop intent)
    Publish:
      - /nav_cmd_vel (suggested safe twist)
      - /guidance_text (human-readable hint)
    """

    def __init__(self):
        super().__init__('collision_guidance')

        # ---- params ----
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('nav_cmd_vel_topic', '/nav_cmd_vel')
        self.declare_parameter('text_topic', '/guidance_text')

        # sector angles (deg)
        self.declare_parameter('front_deg', 15.0)
        self.declare_parameter('side_min_deg', 15.0)
        self.declare_parameter('side_max_deg', 60.0)

        # safety distances (m)
        self.declare_parameter('stop_dist', 0.45)    # too close -> stop suggestion
        self.declare_parameter('slow_dist', 0.80)    # close -> slow down
        self.declare_parameter('turn_dist', 0.60)    # suggest turning if forward blocked

        # speed limits for suggested nav_cmd_vel
        self.declare_parameter('max_lin', 0.15)
        self.declare_parameter('max_ang', 0.60)

        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.nav_cmd_vel_topic = str(self.get_parameter('nav_cmd_vel_topic').value)
        self.text_topic = str(self.get_parameter('text_topic').value)

        self.front_deg = float(self.get_parameter('front_deg').value)
        self.side_min_deg = float(self.get_parameter('side_min_deg').value)
        self.side_max_deg = float(self.get_parameter('side_max_deg').value)

        self.stop_dist = float(self.get_parameter('stop_dist').value)
        self.slow_dist = float(self.get_parameter('slow_dist').value)
        self.turn_dist = float(self.get_parameter('turn_dist').value)

        self.max_lin = float(self.get_parameter('max_lin').value)
        self.max_ang = float(self.get_parameter('max_ang').value)

        # ---- state ----
        self.last_scan: Optional[LaserScan] = None
        self.last_cmd: Twist = Twist()

        # ---- pubs/subs ----
        self.create_subscription(LaserScan, self.scan_topic, self._on_scan, 10)
        self.create_subscription(Twist, self.cmd_vel_topic, self._on_cmd, 10)

        self.pub_nav = self.create_publisher(Twist, self.nav_cmd_vel_topic, 10)
        self.pub_txt = self.create_publisher(String, self.text_topic, 10)

        # periodic evaluate
        self.timer = self.create_timer(0.1, self._tick)  # 10 Hz

        self.get_logger().info(f"scan={self.scan_topic}, cmd_vel={self.cmd_vel_topic}")
        self.get_logger().info(f"publishing nav_cmd_vel={self.nav_cmd_vel_topic}, text={self.text_topic}")

    def _on_scan(self, msg: LaserScan):
        self.last_scan = msg

    def _on_cmd(self, msg: Twist):
        self.last_cmd = msg

    def _sector_min(self, scan: LaserScan, deg_from: float, deg_to: float) -> Optional[float]:
        """min range in [deg_from, deg_to] (degrees, in scan frame where 0 is forward)."""
        a1 = math.radians(deg_from)
        a2 = math.radians(deg_to)
        if a2 < a1:
            a1, a2 = a2, a1

        # clamp
        a1 = max(a1, scan.angle_min)
        a2 = min(a2, scan.angle_max)
        if a2 <= a1:
            return None

        i1 = int(round((a1 - scan.angle_min) / scan.angle_increment))
        i2 = int(round((a2 - scan.angle_min) / scan.angle_increment))
        i1 = max(0, min(len(scan.ranges) - 1, i1))
        i2 = max(0, min(len(scan.ranges) - 1, i2))
        if i2 < i1:
            i1, i2 = i2, i1

        return finite_min(scan.ranges[i1:i2 + 1])

    def _tick(self):
        if self.last_scan is None:
            return

        scan = self.last_scan
        cmd = self.last_cmd

        # sectors
        d_front = self._sector_min(scan, -self.front_deg, +self.front_deg)
        d_left  = self._sector_min(scan, +self.side_min_deg, +self.side_max_deg)
        d_right = self._sector_min(scan, -self.side_max_deg, -self.side_min_deg)

        # default suggestion: mirror user's intent but clipped
        sug = Twist()
        sug.linear.x = max(-self.max_lin, min(self.max_lin, cmd.linear.x))
        sug.angular.z = max(-self.max_ang, min(self.max_ang, cmd.angular.z))

        hint = "OK"

        # if moving forward, check front safety
        if cmd.linear.x > 0.01:
            if d_front is not None and d_front < self.stop_dist:
                # emergency: suggest stop + turn to freer side
                sug.linear.x = 0.0
                if (d_left is None) and (d_right is None):
                    sug.angular.z = 0.0
                    hint = "前方太近：建議停止"
                else:
                    # choose better side
                    left = d_left if d_left is not None else -1.0
                    right = d_right if d_right is not None else -1.0
                    if left >= right:
                        sug.angular.z = +self.max_ang
                        hint = f"前方危險(d={d_front:.2f})：建議停下並左轉"
                    else:
                        sug.angular.z = -self.max_ang
                        hint = f"前方危險(d={d_front:.2f})：建議停下並右轉"

            elif d_front is not None and d_front < self.slow_dist:
                # slow down
                scale = max(0.0, min(1.0, (d_front - self.stop_dist) / (self.slow_dist - self.stop_dist)))
                sug.linear.x = sug.linear.x * scale
                hint = f"前方接近(d={d_front:.2f})：建議減速"

            elif d_front is not None and d_front < self.turn_dist:
                # mild suggestion to bias turn away from closer side
                if d_left is not None and d_right is not None:
                    if d_left < d_right:
                        sug.angular.z = min(self.max_ang, sug.angular.z + 0.2)
                        hint = f"左側較近(dL={d_left:.2f})：建議偏右"
                    elif d_right < d_left:
                        sug.angular.z = max(-self.max_ang, sug.angular.z - 0.2)
                        hint = f"右側較近(dR={d_right:.2f})：建議偏左"

        # publish
        self.pub_nav.publish(sug)
        self.pub_txt.publish(String(data=hint))

        # optional: also print sometimes
        # self.get_logger().info(hint)

def main():
    rclpy.init()
    node = CollisionGuidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
'''

#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String

def wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a

class CollisionGuidance(Node):
    def __init__(self):
        super().__init__('collision_guidance')

        # ---- params ----
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('out_topic', '/nav_cmd_vel')  # 給 UI 看/或給 mux 用
        self.declare_parameter('state_topic', '/collision_state')

        self.declare_parameter('hz', 10.0)

        # 前方扇形（以車頭為 0 rad）
        self.declare_parameter('front_half_angle_deg', 20.0)   # 前方±20°
        self.declare_parameter('side_half_angle_deg', 70.0)    # 左右判斷用的±70°

        # 觸發距離
        self.declare_parameter('warn_dist', 0.55)   # 快要撞到就觸發
        self.declare_parameter('stop_dist', 0.35)   # 更近：停止並轉向

        # 輸出速度（只是導引/避障建議）
        self.declare_parameter('turn_w', 0.8)       # rad/s
        self.declare_parameter('crawl_v', 0.05)     # m/s (可設 0.0 只轉)

        # ---- state ----
        self.scan = None
        self.last_state = ""

        scan_topic = self.get_parameter('scan_topic').value
        out_topic  = self.get_parameter('out_topic').value
        st_topic   = self.get_parameter('state_topic').value

        self.sub = self.create_subscription(LaserScan, scan_topic, self.on_scan, 10)
        self.pub = self.create_publisher(Twist, out_topic, 10)
        self.pub_state = self.create_publisher(String, st_topic, 10)

        hz = float(self.get_parameter('hz').value)
        self.timer = self.create_timer(1.0 / max(1.0, hz), self.on_timer)

        self.get_logger().info(f"collision_guidance: scan={scan_topic} -> {out_topic} + {st_topic}")

    def on_scan(self, msg: LaserScan):
        self.scan = msg

    def sector_min(self, center_rad: float, half_angle_rad: float) -> float:
        """取 scan 在 [center-half, center+half] 的最小距離（忽略 inf/0/NaN）"""
        if self.scan is None:
            return float('inf')

        a_min = self.scan.angle_min
        a_inc = self.scan.angle_increment
        n = len(self.scan.ranges)

        lo = wrap_pi(center_rad - half_angle_rad)
        hi = wrap_pi(center_rad + half_angle_rad)

        # 因為 scan 的角度是連續從 angle_min 走到 angle_max，
        # 這裡簡化用逐點檢查（n 通常不大，10Hz OK）
        best = float('inf')
        for i in range(n):
            ang = a_min + i * a_inc
            # normalize to [-pi,pi]
            ang = wrap_pi(ang)
            # 判斷是否在 sector（處理跨越 pi 的情況）
            in_sector = False
            if lo <= hi:
                in_sector = (lo <= ang <= hi)
            else:
                in_sector = (ang >= lo or ang <= hi)

            if not in_sector:
                continue

            r = self.scan.ranges[i]
            if r is None or r <= 0.0 or math.isinf(r) or math.isnan(r):
                continue
            best = min(best, float(r))
        return best

    def publish_state(self, text: str):
        if text == self.last_state:
            return
        self.last_state = text
        m = String()
        m.data = text
        self.pub_state.publish(m)

    def on_timer(self):
        # 沒 scan 就不做事
        if self.scan is None:
            return

        front_half = math.radians(float(self.get_parameter('front_half_angle_deg').value))
        side_half  = math.radians(float(self.get_parameter('side_half_angle_deg').value))
        warn_dist  = float(self.get_parameter('warn_dist').value)
        stop_dist  = float(self.get_parameter('stop_dist').value)
        turn_w     = float(self.get_parameter('turn_w').value)
        crawl_v    = float(self.get_parameter('crawl_v').value)

        # 前方最小距離
        d_front = self.sector_min(center_rad=0.0, half_angle_rad=front_half)

        # 沒有障礙就不發布（或發布 0）
        if d_front > warn_dist:
            # 你希望「只有快撞到才持續導引」
            self.publish_state("")  # 清空狀態
            t = Twist()
            t.linear.x = 0.0
            t.angular.z = 0.0
            self.pub.publish(t)
            return

        # 左右空間（判斷往哪邊轉比較安全）
        d_left  = self.sector_min(center_rad=+math.pi/2, half_angle_rad=side_half)
        d_right = self.sector_min(center_rad=-math.pi/2, half_angle_rad=side_half)

        # 避障策略：往更空的一側轉
        wz = +turn_w if d_left > d_right else -turn_w

        # 距離太近：不前進，只轉
        vx = 0.0 if d_front < stop_dist else crawl_v

        t = Twist()
        t.linear.x = float(vx)
        t.angular.z = float(wz)
        self.pub.publish(t)

        self.publish_state("EMERGENCY COLLISION AVOIDANCE TRIGGERED")

def main():
    rclpy.init()
    node = CollisionGuidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
