import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

class ConvoyCollisionDetector(Node):
    def __init__(self):
        super().__init__("convoy_collision_detector")

        self.scan_subscription = self.create_subscription(LaserScan, "/convoy/scan", self.scan_cb, 10)
        self.emergency_stop_publisher = self.create_publisher(Bool, "/emergency_stop", 10)

    def scan_cb(self, msg):
        # 15cm 이내로 물체가 감지될 시 충돌로 판단 -> 긴급정지 명령 전달
        if msg.range_min <= 0.15:
            self.get_logger().info('[충돌] ! EMERGENCY STOP !')
            emergency_stop = Bool()
            emergency_stop.data = True
            self.emergency_stop_publisher.publish(emergency_stop)




def main(args=None):
    rclpy.init(args=args)
    convoy_collision_detector = ConvoyCollisionDetector()
    try:
        rclpy.spin(convoy_collision_detector)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()