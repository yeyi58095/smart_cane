import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class ColorBedDetector(Node):
    def __init__(self):
        super().__init__('color_bed_detector')

        # 如果你的 topic 名稱是 /tb3/camera/image_raw，就改這裡
        self.image_topic = '/tb3/camera/image_raw'

        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10
        )
        self.get_logger().info(f"Listening on {self.image_topic}")

    def image_callback(self, msg: Image):
        # ROS Image -> OpenCV BGR
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        h, w, _ = frame.shape

        # 取畫面下半部當 ROI（床大多在畫面下方）
        roi = frame[int(h * 0.4):, :]

        # BGR -> HSV，比較好做顏色閾值
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # 橘色的 HSV 範圍（可以之後調）
        lower_orange = np.array([5, 120, 100])
        upper_orange = np.array([25, 255, 255])

        mask = cv2.inRange(hsv, lower_orange, upper_orange)

        orange_pixels = np.count_nonzero(mask)
        total_pixels = mask.size
        ratio = orange_pixels / float(total_pixels)

        # 比例超過 2% 就當成有看到橘色床
        if ratio > 0.02:
            self.get_logger().info(f"🧡 Orange bed detected! ratio={ratio:.3f}")
        # else:
        #     self.get_logger().info(f"no orange, ratio={ratio:.3f}")


def main(args=None):
    rclpy.init(args=args)
    node = ColorBedDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
