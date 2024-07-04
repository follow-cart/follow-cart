import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

# 보행자를 추적하는 로직 처리하는 클래스
class ConvoyFollower(Node):
    def __init__(self):
        super().__init__('convoy_follower')
        self.bridge = CvBridge()

        # Depth 카메라를 통한 covoy와의 거리 구독
        self.detection_subscription = self.create_subscription(
            Float64MultiArray,
            '/fc1/detection',
            self.detection_cb,
            10
        )

        # 로봇의 속도 명령  publisher
        self.publisher_ = self.create_publisher(Twist, '/fc1/cmd_vel', 10)

        # 로봇의 중앙 위치 (카메라 기준)
        self.image_width = 640  # 이미지 너비 (임의 설정, 실제 카메라 해상도에 맞춰야 함)
        self.image_height = 480  # 이미지 높이 (임의 설정, 실제 카메라 해상도에 맞춰야 함)
        self.robot_center_x = self.image_width / 2
        self.robot_center_y = self.image_height / 2

        # PID 제어 변수 초기화
        self.prev_error_x = 0
        self.integral_x = 0
        self.kp = 0.005  # 비례 게인 값을 증가시킴
        self.ki = 0.0001  # 적분 게인 값을 증가시킴
        self.kd = 0.001  # 미분 게인 값을 증가시킴

        # 최대 회전 속도 제한
        self.max_angular_speed = 0.5  # 회전 속도 제한을 증가시킴

        # 보행자 탐색 모드 변수
        self.searching = False
        self.search_start_time = None
        self.search_duration = 10  # 탐색 지속 시간 (초)

    # def search_for_convoy(self):
    #     # 로봇을 제자리에서 회전시켜 convoy를 찾기
    #     cmd_msg = Twist()
    #     cmd_msg.angular.z = 0.3  # 회전 속도 설정
    #     self.publisher_.publish(cmd_msg)

    def detection_cb(self, msg):
        distance = msg.data[2]
        x = msg.data[0]
        y = msg.data[1]

        cmd_msg = Twist()

        # 거리 유지
        if distance < 1.5:
            cmd_msg.linear.x = 0.0
        elif distance > 2.0:
            cmd_msg.linear.x = 0.2  # 전진 속도를 약간 증가시킴
        else:
            cmd_msg.linear.x = 0.0

        # x와 y 오차 계산 (이미지 기반)
        error_x_image = x - self.robot_center_x
        error_y_image = y - self.robot_center_y

        # PID 제어 (x 방향 회전에 적용)
        self.integral_x += error_x_image
        derivative_x = error_x_image - self.prev_error_x
        angular_z = (self.kp * error_x_image) + (self.ki * self.integral_x) + (self.kd * derivative_x)
        self.prev_error_x = error_x_image

        # 회전 속도 제한
        if angular_z > self.max_angular_speed:
            angular_z = self.max_angular_speed
        elif angular_z < -self.max_angular_speed:
            angular_z = -self.max_angular_speed

        cmd_msg.angular.z = -angular_z  # x 방향 오차에 따른 회전 속도

        self.publisher_.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    convoy_follower = ConvoyFollower()

    try:
        rclpy.spin(convoy_follower)
    except KeyboardInterrupt:
        pass

    convoy_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()