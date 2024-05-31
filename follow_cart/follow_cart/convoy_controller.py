import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from std_msgs.msg import Bool, String
import json
from .quaternion_about_axis import QuaternionAboutAxis

# convoy의 주행을 담당하는 노드
class ConvoyController(Node):
    def __init__(self):
        super().__init__("convoy_controller")

        # 어플과 웹소켓 통신을 통해 연결
        self.cmd_subscription = self.create_subscription(
            String, '/android_commands', self.cmd_cb, 10)

        # navigation action을 전달할 client 생성
        self._action_client = ActionClient(self, NavigateToPose, '/convoy/navigate_to_pose')

        # 긴급 정지 명령을 받아옴
        self.emergency_stop_subscription = self.create_subscription(Bool, "/emergency_stop", self.emergency_stop_cb, 10)

        # 어플에 정보 전달
        self.leader_pub = self.create_subscription(String, '/Leader/call_message', self.call_cb, 10)

        # 개발 진행 중
        self.leader_drive_publisher = self.create_publisher(String, '/Leader/drive_message', 10)
        self.rear_position_publisher = self.create_publisher(String, '/Rear/position', 10)

        # action 수행 중이 아닌지 확인
        self.initial_goal = True
        self.initial_position = (0.0, -2.0, 0.0)
        self.call_position = (0.0, -4.0, 0.0)

    # 반환 명령 수신 시 -> 반환 장소로 이동
    def cmd_cb(self, msg):
        data = json.loads(msg.data)
        command = data['command']

        if command == 'RETURN':
            x, y, theta = self.initial_position
            self.get_logger().info(f'Returning to initial position: ({x}, {y}, {theta}) to convoy')
            self.send_goal(x, y, theta)

    # 호출 명령 수신 시 -> 호출 장소로 이동
    def call_cb(self, msg):
        command = msg.data

        if command == 'CALL_LEADER_ROBOT':
            x, y, theta = self.call_position
            self.get_logger().info(f'Go to call position: ({x}, {y}, {theta}) ')
            self.send_goal(x, y, theta)

    def send_goal(self, x, y, theta):
        self.get_logger().info('sending goal to action server')
        goal_pose = NavigateToPose.Goal()

        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "map"

        # quaternion으로 변환
        quat = QuaternionAboutAxis.quaternion_about_axis(theta / 180.0 * 3.14159, (0, 0, 1))

        # 목표 방향
        pose_stamped.pose.orientation.x = quat[0]
        pose_stamped.pose.orientation.y = quat[1]
        pose_stamped.pose.orientation.z = quat[2]
        pose_stamped.pose.orientation.w = quat[3]

        # 목표 위치
        pose_stamped.pose.position.x = x
        pose_stamped.pose.position.y = y
        pose_stamped.pose.position.z = 0.0

        pose_stamped.header.stamp = self.get_clock().now().to_msg()  # 노드 시간

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
        self.goal_handle = future.result()

        # action server가 goal 거절 시 주행 불가 상황으로 판단 -> 강제 종료
        if not self.goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            self.get_logger().info('[주행 불가] ! EMERGENCY STOP !')
            rclpy.shutdown()
            return
        self.get_logger().info('Goal accepted :)')
        # action 진행 중으로 상태 변경
        self.initial_goal = False

        # leader_message = String()
        # leader_message.data = 'Leader is active and moving'
        # self.leader_drive_publisher.publish(leader_message)

        _get_result_future = self.goal_handle.get_result_async()

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

    # 긴급정지 명령 수신 시 수행 -> 진행 중인 action 취소 명령 전달
    def emergency_stop_cb(self, msg):
        if msg.data:
            self.get_logger().info('[충돌] ! EMERGENCY STOP !')

            cancel_future = self.goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_cb)

    # action 취소 후 노드 종료
    def cancel_cb(self, future):
        self.get_logger().info('[충돌] ! FORCE QUIT !')
        self.destroy_node()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    convoy_controller = ConvoyController()
    try:
        rclpy.spin(convoy_controller)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()