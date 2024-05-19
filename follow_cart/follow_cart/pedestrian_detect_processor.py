# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# import cv2
#
# class PedestrianDetectProcessor(Node):
#     def __init__(self):
#         super().__init__('pedestrian_detect_processor')
#         self.bridge = CvBridge()
#
#         # RGB Image Subscriber
#         self.rgb_subscription = self.create_subscription(
#             Image,
#             '/convoy/camera/color/image_raw',
#             self.rgb_callback,
#             10)
#         self.rgb_subscription  # prevent unused variable warning
#
#         # Depth Image Subscriber
#         self.depth_subscription = self.create_subscription(
#             Image,
#             '/convoy/camera/depth/image_raw',
#             self.depth_callback,
#             10)
#         self.depth_subscription  # prevent unused variable warning
#
#     def rgb_callback(self, msg):
#         cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
#         cv2.imshow("RGB Camera Image", cv_image)
#         cv2.waitKey(1)
#
#     def depth_callback(self, msg):
#         depth_image = self.bridge.imgmsg_to_cv2(msg, '32FC1')
#         cv2.imshow("Depth Camera Image", depth_image)
#         cv2.waitKey(1)
#
# def main(args=None):
#     rclpy.init(args=args)
#     pedestrian_detect_processor = PedestrianDetectProcessor()
#     rclpy.spin(pedestrian_detect_processor)
#     pedestrian_detect_processor.destroy_node()
#     rclpy.shutdown()
#
# if __name__ == '__main__':
#     main()

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class PedestrianDetectProcessor(Node):
    def __init__(self):
        super().__init__('pedestrian_detect_processor')
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, '/processed/image', 10)

        self.subscription = self.create_subscription(
            Image,
            '/convoy/Pi_Camera/image_raw',
            self.callback,
            10)

    def callback(self, msg):
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