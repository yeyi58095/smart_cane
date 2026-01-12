#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class NavCmdVelUI(Node):
    def __init__(self):
        super().__init__('nav_cmd_vel_ui')

        self.declare_parameter('topic', '/nav_cmd_vel')
        self.declare_parameter('lin_th', 0.05)     # m/s
        self.declare_parameter('ang_th', 0.10)     # rad/s
        self.declare_parameter('print_hz', 5.0)    # 限制輸出頻率

        self.topic = str(self.get_parameter('topic').value)
        self.lin_th = float(self.get_parameter('lin_th').value)
        self.ang_th = float(self.get_parameter('ang_th').value)
        self.print_hz = float(self.get_parameter('print_hz').value)

        self.last_msg = None
        self.sub = self.create_subscription(Twist, self.topic, self.cb, 10)

        period = 1.0 / max(0.5, self.print_hz)
        self.timer = self.create_timer(period, self.on_timer)

        self.get_logger().info(f"UI listening: {self.topic}")

    def cb(self, msg: Twist):
        self.last_msg = msg

    def on_timer(self):
        if self.last_msg is None:
            return

        vx = float(self.last_msg.linear.x)
        wz = float(self.last_msg.angular.z)

        # 文字指令判斷（你可自行調整規則）
        cmd = []
        if abs(vx) < self.lin_th and abs(wz) < self.ang_th:
            cmd_text = "🟥 STOP（停止）"
        else:
            if vx > self.lin_th:
                cmd.append("⬆️ 前進")
            elif vx < -self.lin_th:
                cmd.append("⬇️ 後退")

            if wz > self.ang_th:
                cmd.append("↩️ 左轉")
            elif wz < -self.ang_th:
                cmd.append("↪️ 右轉")

            cmd_text = " + ".join(cmd) if cmd else "（微調）"

        # 顯示數值方便你 debug
        print(f"[GUIDE] {cmd_text} | v={vx:+.2f} m/s  w={wz:+.2f} rad/s")


def main():
    rclpy.init()
    node = NavCmdVelUI()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
