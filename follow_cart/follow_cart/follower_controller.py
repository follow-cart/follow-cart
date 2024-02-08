import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix

class FollowerController(Node):
    def __init__(self):
        super().__init__("follower_controller")
        self.follower_1_pos_listener = (NavSatFix, "follow_cart_2/pos", 10)
        self.follower_2_pos_listener = (NavSatFix, "follow_cart_3/pos", 10)