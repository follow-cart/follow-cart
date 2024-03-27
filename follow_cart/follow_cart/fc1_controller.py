import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from .fc1_formation_keeper import FC1FormationKeeper

class FC1Controller(Node):
    def __init__(self):
        super().__init__("fc1_controller")
        self.pose_publisher = self.create_publisher(PoseStamped, "/fc1/goal_pose", 10)
        self.odom_subscription = self.create_subscription(Odometry, "/convoy/odometry/filtered", self.odom_cb, 10)
        self.fc1_formation_keeper = FC1FormationKeeper()

    # odom msg를 goal_pose로 전달한 msg 타입을 변경 후 전달
    def odom_cb(self, odom_msg):
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "map"
        pose_stamped.header.stamp = odom_msg.header.stamp

        pose_stamped.pose.orientation.w = 1.0

        convoy_x = odom_msg.pose.pose.orientation.x
        convoy_y = odom_msg.pose.pose.orientation.y
        convoy_z = odom_msg.pose.pose.orientation.z
        convoy_w = odom_msg.pose.pose.orientation.w

        # 대형 유지를 위해 convoy 기준으로 x축, y축 어디에 위치해야 하는지
        x_from_convoy, y_from_convoy = self.fc1_formation_keeper.calculate(convoy_x, convoy_y, convoy_z, convoy_w)

        # 대형 유지를 위한 새로운 위치 계산
        new_x = odom_msg.pose.pose.position.x - x_from_convoy
        new_y = odom_msg.pose.pose.position.y - y_from_convoy

        pose_stamped.pose.position.x = new_x
        pose_stamped.pose.position.y = new_y
        pose_stamped.pose.position.z = 0.0

        self.pose_publisher.publish(pose_stamped)
def main(args=None):
    rclpy.init(args=args)
    fc1_controller = FC1Controller()
    try:
        rclpy.spin(fc1_controller)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()