import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix
from .formation_keeper import FormationKeeper

class LeaderController(Node):
    def __init__(self):
        super().__init__('leader_controller')
        self.my_x = None
        self.my_y = None
        self.last_my_x = None
        self.last_my_y = None

        self.twist_publisher = self.create_publisher(Twist, "follow_cart_1/cmd_vel", 10)
        self.my_pos_listener = self.create_subscription(NavSatFix, "follow_cart_1/pos", self.my_post_listener_cb, 10)
        self.formation_keeper = None
        self.convoy_pos_listener = self.create_subscription(NavSatFix, "convoy/pos", self.convoy_pos_listener_cb, 10)

    def convoy_pos_listener_cb(self, pos):
        self.formation_keeper = FormationKeeper("leader", self.my_x, self.my_y, pos.latitude, pos.longitude, self.last_my_x, self.last_my_y)

    def my_post_listener_cb(self, pos):
        self.my_x = pos.latitude
        self.my_y = pos.longitude
def main(args=None):
    rclpy.init(args=args)
    leader_controller = LeaderController()
    try:
        rclpy.spin(leader_controller)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()