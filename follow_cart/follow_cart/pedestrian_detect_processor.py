import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


# 보행자 처리를 위해 카메라 값을 받아서 처리하는 클래스
class PedestrianDetectProcessor(Node):
    def __init__(self):
        super().__init__('pedestrian_detect_processor')
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, '/processed/image', 10)

        # RGB 이미지 구독
        self.subscription = self.create_subscription(
            Image,
            '/convoy/Camera/image_raw',
            self.detect_processor_callback,
            10)

    def detect_processor_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        # 이미지 처리 (예시: 회색조로 변환)
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        # ROS 이미지 메시지로 변환 후 퍼블리시
        processed_image_msg = self.bridge.cv2_to_imgmsg(gray_image, encoding="mono8")

        self.publisher.publish(processed_image_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PedestrianDetectProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()