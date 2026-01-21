#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy


class InitialPosePub(Node):
    def __init__(self):
        super().__init__('initialpose_pub')

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', qos)

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

        self.cli = self.create_client(GetState, '/amcl/get_state')
        self.timer = self.create_timer(0.5, self.tick)
        self.sent = False
        self._pending = False

    def tick(self):
        if self.sent or self._pending:
            return

        if not self.cli.service_is_ready():
            self.get_logger().info('Waiting for /amcl/get_state ...')
            return

        self._pending = True
        req = GetState.Request()
        future = self.cli.call_async(req)

        def done_cb(fut):
            self._pending = False
            if self.sent:
                return
            try:
                resp = fut.result()
                state_label = resp.current_state.label
            except Exception as e:
                self.get_logger().warn(f'GetState failed: {e}')
                return

            if state_label.lower() != 'active':
                self.get_logger().info(f'AMCL state = {state_label}, waiting...')
                return

            self.msg.header.stamp = self.get_clock().now().to_msg()
            self.pub.publish(self.msg)
            self.get_logger().info('Published /initialpose (AMCL is active).')
            self.sent = True

            # ✅ 不要在 callback 裡 shutdown，改成讓 main 的 while 自己退出
            try:
                self.timer.cancel()
            except Exception:
                pass

        future.add_done_callback(done_cb)


def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePub()

    try:
        # ✅ spin until sent OR Ctrl+C
        while rclpy.ok() and not node.sent:
            rclpy.spin_once(node, timeout_sec=0.2)

    except KeyboardInterrupt:
        pass

    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
