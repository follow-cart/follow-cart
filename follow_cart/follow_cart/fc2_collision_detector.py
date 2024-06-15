import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

class Fc2CollisionDetector(Node):
    def __init__(self):
        super().__init__("fc2_collision_detector")

        self.scan_subscription = self.create_subscription(LaserScan, "/fc2/scan", self.scan_cb, 10)
        self.emergency_stop_publisher = self.create_publisher(Bool, "/emergency_stop", 10)

    def scan_cb(self, msg):
        if msg.range_min <= 0.15:
            self.get_logger().info('[충돌] ! EMERGENCY STOP !')
            emergency_stop = Bool()
            emergency_stop.data = True
            self.emergency_stop_publisher.publish(emergency_stop)




def main(args=None):
    rclpy.init(args=args)
    fc2_collision_detector = Fc2CollisionDetector()
    try:
        rclpy.spin(fc2_collision_detector)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()