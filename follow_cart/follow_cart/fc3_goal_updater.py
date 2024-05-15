import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from .fc3_formation_keeper import FC3FormationKeeper

class FC3GoalUpdater(Node):
    def __init__(self):
        super().__init__("fc3_goal_updater")
        self.pose_publisher = self.create_publisher(PoseStamped, "/fc3/goal_update", 10)
        # self.pose_subscription = self.create_subscription(PoseWithCovarianceStamped, "/convoy/amcl_pose", self.update_cb, 10)
        self.odom_subscription = self.create_subscription(Odometry, "/convoy/odometry/filtered", self.update_cb,
                                                          10)
        self.fc3_formation_keeper = FC3FormationKeeper()

    def update_cb(self, pose_msg):

        convoy_x = pose_msg.pose.pose.orientation.x
        convoy_y = pose_msg.pose.pose.orientation.y
        convoy_z = pose_msg.pose.pose.orientation.z
        convoy_w = pose_msg.pose.pose.orientation.w

        # 대형 유지를 위해 convoy 기준으로 x축, y축 어디에 위치해야 하는지
        x_from_convoy, y_from_convoy = self.fc3_formation_keeper.calculate(convoy_x, convoy_y, convoy_z, convoy_w)

        # 대형 유지를 위한 새로운 위치 계산
        new_x = pose_msg.pose.pose.position.x - x_from_convoy
        new_y = pose_msg.pose.pose.position.y - y_from_convoy

        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "fc3/map"
        pose_stamped.header.stamp = self.get_clock().now().to_msg()

        pose_stamped.pose.orientation = pose_msg.pose.pose.orientation

        pose_stamped.pose.position.x = new_x
        pose_stamped.pose.position.y = new_y
        pose_stamped.pose.position.z = 0.0

        self.pose_publisher.publish(pose_stamped)

def main(args=None):
    rclpy.init(args=args)
    fc3_goal_updater = FC3GoalUpdater()
    try:
        rclpy.spin(fc3_goal_updater)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()