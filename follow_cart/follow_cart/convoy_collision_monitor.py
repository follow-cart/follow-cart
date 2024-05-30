import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

class ConvoyCollisionMonitor(Node):
    def __init__(self):
        super().__init__("convoy_collision_monitor")
        self.scan_subscription = self.create_subscription(LaserScan, "/convoy/scan", self.scan_cb, 10)
        self.emergency_stop_publisher = self.create_publisher(Bool, "/emergency_stop", 10)

    def scan_cb(self, msg):
        if msg.range_min == 0.1:
            emergency_stop = Bool()
            emergency_stop.data = True
            self.emergency_stop_publisher.publish(emergency_stop)




def main(args=None):
    rclpy.init(args=args)
    convoy_collision_monitor = ConvoyCollisionMonitor()
    try:
        rclpy.spin(convoy_collision_monitor)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()