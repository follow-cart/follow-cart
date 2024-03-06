import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

class FollowerController(Node):
    def __init__(self):
        super().__init__("follower_controller")
        self.pose_publisher = self.create_publisher(PoseStamped, "/follow_cart_1/goal_pose", 10)
        self.odom_subscription = self.create_subscription(Odometry, "/convoy/odometry/filtered", self.odom_cb, 10)

    def odom_cb(self, odom_msg):
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = odom_msg.header.frame_id
        pose_stamped.header.stamp = odom_msg.header.stamp
        pose_stamped.pose = odom_msg.pose.pose
        self.pose_publisher.publish(pose_stamped)
def main(args=None):
    rclpy.init(args=args)
    follower_controller = FollowerController()
    try:
        rclpy.spin(follower_controller)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()