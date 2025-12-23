#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

class InitialPosePub(Node):
    def __init__(self):
        super().__init__('initialpose_pub')

        # 讓這個 node 跟 nav2 一樣用 sim time
        # self.declare_parameter('use_sim_time', True)

        # ✅ QoS：RELIABLE + TRANSIENT_LOCAL（關鍵！）
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', qos)

        # 準備 initial pose
        self.msg = PoseWithCovarianceStamped()
        self.msg.header.frame_id = 'map'
        self.msg.pose.pose.position.x = 0.0
        self.msg.pose.pose.position.y = 0.0
        self.msg.pose.pose.position.z = 0.0
        self.msg.pose.pose.orientation.z = 0.0
        self.msg.pose.pose.orientation.w = 1.0
        cov = [
            0.25, 0, 0, 0, 0, 0,
            0, 0.25, 0, 0, 0, 0,
            0, 0, 0.0, 0, 0, 0,
            0, 0, 0, 0.068, 0, 0,
            0, 0, 0, 0, 0.068, 0,
            0, 0, 0, 0, 0, 0.068
        ]
        self.msg.pose.covariance = [float(x) for x in cov]
        # lifecycle service：/amcl/get_state
        self.cli = self.create_client(GetState, '/amcl/get_state')

        self.timer = self.create_timer(0.5, self.tick)
        self.sent = False

    def tick(self):
        if self.sent:
            return

        # service 還沒起來就繼續等
        if not self.cli.service_is_ready():
            self.get_logger().info('Waiting for /amcl/get_state ...')
            return

        req = GetState.Request()
        future = self.cli.call_async(req)
        future.add_done_callback(self.on_state)

    def on_state(self, future):
        if self.sent:
            return
        try:
            resp = future.result()
            state_label = resp.current_state.label  # e.g. "active"
        except Exception as e:
            self.get_logger().warn(f'GetState failed: {e}')
            return

        if state_label.lower() != 'active':
            self.get_logger().info(f'AMCL state = {state_label}, waiting...')
            return

        # ✅ AMCL active -> 發 initialpose
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.msg)
        self.get_logger().info('Published /initialpose (AMCL is active).')
        self.sent = True

        # 結束
        self.timer.cancel()
        self.create_timer(0.5, lambda: rclpy.shutdown())

def main():
    rclpy.init()
    node = InitialPosePub()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
