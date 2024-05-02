import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class PedestrianDetectProcessor(Node):
    def __init__(self):
        super().__init__('pedestrian_detect_processor')
        self.subscription = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.image_callback,
            10)
        self.publisher = self.create_publisher(Image, '/processed/depth/image_raw', 10)

    def image_callback(self, msg):
        # Depth 이미지를 다른 노드로 전달
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    depth_image_subscriber = PedestrianDetectProcessor()
    rclpy.spin(depth_image_subscriber)
    depth_image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

