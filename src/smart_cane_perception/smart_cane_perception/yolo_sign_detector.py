import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2


class YoloSignDetector(Node):
    def __init__(self):
        super().__init__('yolo_sign_detector')

        # 參數：模型路徑，可以之後在 launch 裡改
        model_path = self.declare_parameter(
            'model_path',
            '/home/daniel/smart_cane/yolov8n.pt'  # 先用官方 COCO 權重
        ).get_parameter_value().string_value

        self.get_logger().info(f'Loading YOLO model from: {model_path}')
        self.model = YOLO(model_path)

        self.bridge = CvBridge()

        # 訂閱 Gazebo RGB 相機
        self.sub = self.create_subscription(
            Image,
            '/tb3/camera/image_raw',
            self.image_callback,
            10
        )

    def image_callback(self, msg: Image):
        # ROS Image -> OpenCV BGR
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # YOLO 推論（verbose 關掉比較安靜）
        results = self.model(frame, verbose=False)[0]

        # 只印出有偵測到的物件
        for box in results.boxes:
            cls_id = int(box.cls)
            conf = float(box.conf)
            name = self.model.names[cls_id]

            # 可以自己設定門檻
            if conf < 0.5:
                continue

            self.get_logger().info(
                f'YOLO detect: {name} (id={cls_id}) '
                f'conf={conf:.2f}'
            )

            # 如果之後要畫框可以加這段（現在先不用顯示）
            # x1, y1, x2, y2 = map(int, box.xyxy[0])
            # cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            # cv2.imshow('yolo_debug', frame)
            # cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = YoloSignDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
