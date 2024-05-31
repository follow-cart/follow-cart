import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
from ament_index_python.packages import get_package_share_directory
import os
from ultralytics import YOLO
import torch

from std_msgs.msg import Float32

# 보행자를 추적하는 로직처리하는 클래스
class PedestrianFollower(Node):
    def __init__(self):
        super().__init__('pedestrian_follower')
        self.pedestrian_distance = float()
        self.robot_sign = bool()
        self.bridge = CvBridge()

        package_name = "follow_cart"
        pkg = get_package_share_directory(package_name)
        model_path = os.path.join(pkg, "YOLOV8_model", "yolov8n.pt")

        # 카메라 이미지를 구독할 subscriber
        self.subscription_image = self.create_subscription(
            Image,
            '/convoy/Camera/image_raw',
            self.image_callback,
            10)

        # 보행자 위치를 구독할 subscriber
        self.subscription_pedestrian = self.create_subscription(
            Odometry,
            '/pedestrian/current_position',
            self.pedestrian_callback,
            10)

        # Depth 카메라를 통한 보행자와의 거리 구독
        self.subscription_pedestrian_distance = self.create_subscription(
            Float32,
            '/convoy/Camera/depth/distance_to_pedestrian',
            self.pedestrian_distance_callback,
            10
        )

        # 로봇의 속도 명령  publisher
        self.publisher_ = self.create_publisher(Twist, '/convoy/cmd_vel', 10)

        # GPU 사용 가능 여부 체크
        device = 'cuda' if torch.cuda.is_available() else 'cpu'
        # self.get_logger().info(f"Using device: {device}")

        # YOLOv8 모델 로드
        self.model = YOLO(model_path)
        self.model.to(device)

        # 추적할 보행자의 중심 위치
        self.target_x = None
        self.target_y = None

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

    # 보행자 위치 값 처리
    def pedestrian_callback(self, msg):
        self.pedestrian_odom_x = msg.pose.pose.position.x
        self.pedestrian_odom_y = msg.pose.pose.position.y

    # 로봇과 보행자 간 거리 처리
    def pedestrian_distance_callback(self, msg):
        self.pedestrian_distance = msg.data

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model(cv_image, imgsz=640)

        # 보행자 인식 초기화
        self.target_x = None
        self.target_y = None

        # 보행자 인식 후 추적
        for result in results:
            for box in result.boxes:
                if box.cls == 0:  # 보행자 클래스 (COCO 데이터셋에서 사람 클래스 ID는 0)
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                    self.target_x = (x1 + x2) / 2  # 보행자의 중심 x 좌표
                    self.target_y = (y1 + y2) / 2  # 보행자의 중심 y 좌표

                    # 보행자 위치 표시
                    cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.circle(cv_image, (int(self.target_x), int(self.target_y)), 5, (0, 255, 0), -1)
                    self.searching = False  # 보행자를 인식하면 탐색 모드 종료
                    break  # 첫 번째 보행자만 추적

        # 보행자를 인식하지 못했을 때 탐색 모드로 진입
        if self.target_x is None and not self.searching:
            self.searching = True
            self.search_start_time = self.get_clock().now()

        # 탐색 모드에서 보행자를 찾기
        if self.searching:
            elapsed_time = (self.get_clock().now() - self.search_start_time).nanoseconds / 1e9
            if elapsed_time > self.search_duration:
                self.searching = False
            else:
                self.search_for_pedestrian()

        # 보행자를 인식하면 정상적으로 제어
        if not self.searching:
            self.control_robot()

    def search_for_pedestrian(self):
        # 로봇을 제자리에서 회전시켜 보행자를 찾기
        cmd_msg = Twist()
        cmd_msg.angular.z = 0.3  # 회전 속도 설정
        self.publisher_.publish(cmd_msg)

    def control_robot(self):
        if self.target_x is None or self.target_y is None:
            return

        cmd_msg = Twist()

        # 거리 유지
        if self.pedestrian_distance < 1.0:
            cmd_msg.linear.x = 0.0
        elif self.pedestrian_distance > 1.5:
            cmd_msg.linear.x = 0.35  # 전진 속도를 약간 증가시킴
        else:
            cmd_msg.linear.x = 0.0

        # x와 y 오차 계산 (이미지 기반)
        error_x_image = self.target_x - self.robot_center_x
        error_y_image = self.target_y - self.robot_center_y

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
    pedestrian_tracker = PedestrianFollower()
    rclpy.spin(pedestrian_tracker)
    pedestrian_tracker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()