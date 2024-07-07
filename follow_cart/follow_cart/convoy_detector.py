import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Float64MultiArray
from .fc_rgb_image_processor import FCRGBImageProcessor
from .depth_image_processor import DepthImageProcessor

# 보행자를 인식하는 로직을 처리하는 클래스
class ConvoyDetector(Node):
    def __init__(self):
        super().__init__('convoy_detector')
        self.bridge = CvBridge()

        # RGB 카메라 이미지 구독
        self.subscription_rgb = self.create_subscription(
            Image,
            '/fc1/front_camera/image_raw',
            self.rgb_callback,
            10
        )

        # Depth 카메라 이미지 구독
        self.subscription_depth = self.create_subscription(
            Image,
            '/fc1/front_camera/depth/image_raw',
            self.depth_callback,
            10
        )

        # Depth Camera를 통한 보행자와의 거리 토픽 발행
        self.detection_publisher = self.create_publisher(
            Float64MultiArray,
            '/fc1/detection',
            10
        )

        self.rgb_image_processor = FCRGBImageProcessor()
        self.depth_image_processor = DepthImageProcessor()

    def rgb_callback(self, msg):
        cv_image_rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        self.rgb_image_processor.process(cv_image_rgb, "fc1")

        # Depth 카메라 callback

    def depth_callback(self, msg):
        if self.rgb_image_processor.x is not None and self.rgb_image_processor.y is not None:
            cv_image_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')

            self.depth_image_processor.process(cv_image_depth, self.rgb_image_processor.x, self.rgb_image_processor.y)

            # self.get_logger().info("Distance to pedestrian: {0}".format(self.depth_image_processor.distance))

            detection_msg = Float64MultiArray()
            detection_msg.data = self.depth_image_processor.detection
            self.detection_publisher.publish(detection_msg)


def main(args=None):
    rclpy.init(args=args)
    convoy_detector = ConvoyDetector()

    try:
        rclpy.spin(convoy_detector)
    except KeyboardInterrupt:
        pass

    convoy_detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
