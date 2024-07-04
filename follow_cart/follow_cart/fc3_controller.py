import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from .fc3_formation_keeper import FC3FormationKeeper
from std_msgs.msg import Bool

class FC3Controller(Node):
    def __init__(self):
        super().__init__("fc3_controller")

        self.pose_subscription = self.create_subscription(PoseWithCovarianceStamped, "/fc1/amcl_pose", self.pose_cb, 10)

        self._action_client = ActionClient(self, NavigateToPose, '/fc3/navigate_to_pose')

        # 긴급 정지 명령을 받아옴
        self.emergency_stop_subscription = self.create_subscription(Bool, "/emergency_stop", self.emergency_stop_cb, 10)

        self.fc3_formation_keeper = FC3FormationKeeper()
        self.initial_goal = True

    # odom msg를 goal_pose로 전달한 msg 타입을 변경 후 전달
    def pose_cb(self, pose_msg):
        # convoy의 위치 정보를 수신할 수 없을 시 긴급 정지
        if pose_msg is None:
            self.get_logger().info('[위치 정보 수신 불가] ! EMERGENCY STOP !')
            rclpy.shutdown()

        if self.initial_goal:

            convoy_x = pose_msg.pose.pose.orientation.x
            convoy_y = pose_msg.pose.pose.orientation.y
            convoy_z = pose_msg.pose.pose.orientation.z
            convoy_w = pose_msg.pose.pose.orientation.w

            # 대형 유지를 위해 convoy 기준으로 x축, y축 어디에 위치해야 하는지
            x_from_convoy, y_from_convoy = self.fc3_formation_keeper.calculate(convoy_x, convoy_y, convoy_z, convoy_w)

            # 대형 유지를 위한 follow_cart의 새로운 목표 위치 계산
            new_x = pose_msg.pose.pose.position.x - x_from_convoy
            new_y = pose_msg.pose.pose.position.y - y_from_convoy

            self.get_logger().info('init goal')
            orientation = pose_msg.pose.pose.orientation
            self.send_goal(new_x, new_y, orientation)
        else:
            pass

    def send_goal(self, x, y, orientation):
        self.get_logger().info('sending goal to action server')
        goal_pose = NavigateToPose.Goal()

        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "map"
        pose_stamped.header.stamp = self.get_clock().now().to_msg()

        pose_stamped.pose.orientation.x = orientation.x
        pose_stamped.pose.orientation.y = orientation.y
        pose_stamped.pose.orientation.z = orientation.z
        pose_stamped.pose.orientation.w = orientation.w

        pose_stamped.pose.position.x = x
        pose_stamped.pose.position.y = y
        pose_stamped.pose.position.z = 0.0

        goal_pose.pose = pose_stamped
        goal_pose.behavior_tree = "/home/bluevery8/workspace/follow_cart_ws/src/follow_cart/config/follow_point_bt.xml"

        self.get_logger().info('waiting for action server')
        self._action_client.wait_for_server()
        self.get_logger().info('action server detected')

        _send_goal_future = self._action_client.send_goal_async(
            goal_pose,
            feedback_callback=self.feedback_callback)
        self.get_logger().info('goal sent')

        _send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        self.initial_goal = False
        _get_result_future = self.goal_handle.get_result_async()

        _get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}' + str(result))
        self.initial_goal = True

    def feedback_callback(self, feedback_msg):
        pass

    def emergency_stop_cb(self, msg):
        if msg.data:
            self.get_logger().info('[충돌] ! EMERGENCY STOP !')

            cancel_future = self.goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_cb)

    def cancel_cb(self, future):
        self.get_logger().info('[충돌] ! FORCE QUIT !')
        self.destroy_node()
        rclpy.shutdown()
def main(args=None):
    rclpy.init(args=args)
    fc3_controller = FC3Controller()
    try:
        rclpy.spin(fc3_controller)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()