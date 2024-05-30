import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from .fc1_formation_keeper import FC1FormationKeeper
from std_msgs.msg import Bool

# follow cart의 주행을 담당하는 노드
class FC1Controller(Node):
    def __init__(self):
        super().__init__("fc1_controller")
        # convoy의 amcl_pose를 통해 pose 정보 받아옴
        self.pose_subscription = self.create_subscription(PoseWithCovarianceStamped, "/convoy/amcl_pose", self.pose_cb, 10)
        # self.odom_subscription = self.create_subscription(Odometry, "/convoy/odometry/filtered", self.pose_cb, 10)

        # navigation action을 전달할 client 생성
        self._action_client = ActionClient(self, NavigateToPose, '/fc1/navigate_to_pose')

        # 긴급 정지 명령을 받아옴
        self.emergency_stop_subscription = self.create_subscription(Bool, "/emergency_stop", self.emergency_stop_cb, 10)

        self.fc1_formation_keeper = FC1FormationKeeper()

        # action 수행 중이 아닌지 확인
        self.initial_goal = True

    def pose_cb(self, pose_msg):

        # convoy의 위치 정보를 수신할 수 없을 시 긴급 정지
        if pose_msg is None:
            self.get_logger().info('[위치 정보 수신 불가] ! EMERGENCY STOP !')
            rclpy.shutdown()

        # action 수행 중이 아니면
        if self.initial_goal:

            convoy_x = pose_msg.pose.pose.orientation.x
            convoy_y = pose_msg.pose.pose.orientation.y
            convoy_z = pose_msg.pose.pose.orientation.z
            convoy_w = pose_msg.pose.pose.orientation.w

            # 대형 유지를 위해 convoy 기준으로 x축, y축 어디에 위치해야 하는지
            x_from_convoy, y_from_convoy = self.fc1_formation_keeper.calculate(convoy_x, convoy_y, convoy_z, convoy_w)

            # 대형 유지를 위한 follow_cart의 새로운 목표 위치 계산
            new_x = pose_msg.pose.pose.position.x - x_from_convoy
            new_y = pose_msg.pose.pose.position.y - y_from_convoy

            self.get_logger().info('init goal')
            orientation = pose_msg.pose.pose.orientation
            self.send_goal(new_x, new_y, orientation)

        # action을 수행 중이라면
        else:
            pass

    def send_goal(self, x, y, orientation):
        self.get_logger().info('sending goal to action server')
        goal_pose = NavigateToPose.Goal()

        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "map"
        pose_stamped.header.stamp = self.get_clock().now().to_msg() # 노드 시간

        # 목표 방향
        pose_stamped.pose.orientation.x = orientation.x
        pose_stamped.pose.orientation.y = orientation.y
        pose_stamped.pose.orientation.z = orientation.z
        pose_stamped.pose.orientation.w = orientation.w

        # 목표 위치
        pose_stamped.pose.position.x = x
        pose_stamped.pose.position.y = y
        pose_stamped.pose.position.z = 0.0

        goal_pose.pose = pose_stamped
        goal_pose.behavior_tree = "/home/bluevery8/workspace/follow_cart_ws/src/follow_cart/config/follow_point_bt.xml"

        # action server가 작동 확인
        self.get_logger().info('waiting for action server')
        self._action_client.wait_for_server()
        self.get_logger().info('action server detected')

        # goal 전달
        _send_goal_future = self._action_client.send_goal_async(
            goal_pose,
            feedback_callback=self.feedback_callback)
        self.get_logger().info('goal sent')

        # response callback 추가
        _send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')

        # action 진행 중으로 상태 변경
        self.initial_goal = False
        _get_result_future = goal_handle.get_result_async()

        # result callback 추가
        _get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}' + str(result))

        # action 종료로 상태 변경
        self.initial_goal = True

    def feedback_callback(self, feedback_msg):
        pass
        # feedback = feedback_msg.feedback
        # self.get_logger().info('FEEDBACK:' + str(feedback))

    def emergency_stop_cb(self, msg):
        if msg.data:
            self.get_logger().info('[충돌] ! EMERGENCY STOP !')
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    fc1_controller = FC1Controller()
    try:
        rclpy.spin(fc1_controller)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()