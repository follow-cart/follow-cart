import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import torch
from ament_index_python.packages import get_package_share_directory
import os

from std_msgs.msg import Float32

# 보행자를 인식하는 로직을 처리하는 클래스
class PedestrianDetector(Node):
    def __init__(self):
        super().__init__('pedestrian_detector')
        self.bridge = CvBridge()

        package_name = "follow_cart"
        pkg = get_package_share_directory(package_name)
        model_path = os.path.join(pkg, "YOLOV8_model", "yolov8n.pt")

        # RGB 카메라 이미지 구독
        self.subscription_rgb = self.create_subscription(
            Image,
            '/processed/image',
            self.rgb_callback,
            10
        )

        # Depth 카메라 이미지 구독
        self.subscription_depth = self.create_subscription(
            Image,
            '/convoy/Camera/depth/image_raw',
            self.depth_callback,
            10
        )

        # Depth Camera를 통한 보행자와의 거리 토픽 발행
        self.publisher_distance = self.create_publisher(
            Float32,
            '/convoy/Camera/depth/distance_to_pedestrian',
            10
        )

        # GPU 사용 가능 여부 체크
        device = 'cuda' if torch.cuda.is_available() else 'cpu'
        # print(f"Using device: {device}")

        # Load YOLOv8 model
        self.model = YOLO(model_path)
        self.model.to(device)

        # 이미지 저장을 위한 변수
        self.cv_image_rgb = None
        self.cv_image_depth = None
        self.cv_image_depth_colormap = None

        # 보행자 중심 좌표
        self.pedestrian_center_x = None
        self.pedestrian_center_y = None

    def rgb_callback(self, msg):
        self.cv_image_rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model(self.cv_image_rgb, imgsz=640)  # Use imgsz instead of size

        # 프로세스 결과
        for result in results:
            for box in result.boxes:
                # 보행자 검출 (class 0  = 사람 / COCO dataset)
                if box.cls == 0:
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                    self.pedestrian_center_x = (x1 + x2) // 2
                    self.pedestrian_center_y = (y1 + y2) // 2
                    cv2.rectangle(self.cv_image_rgb, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.circle(self.cv_image_rgb, (self.pedestrian_center_x, self.pedestrian_center_y), 5, (0, 255, 0),
                               -1)
                    break

        # 검출 결과 화면
        # self.display_images()

    # Depth 카메라 callback
    def depth_callback(self, msg):
        self.cv_image_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')

        # Depth 이미지 클리핑 및 정규화
        depth_min = 0.1  # 최소 거리 (단위 : m)
        depth_max = 20.0  # 최대 거리 (단위 : m)
        depth_image = np.clip(self.cv_image_depth, depth_min, depth_max)
        depth_image = (depth_image - depth_min) / (depth_max - depth_min)  # 정규화

        # 8비트 이미지로 변환
        depth_image = (depth_image * 255).astype(np.uint8)
        self.cv_image_depth_colormap = cv2.applyColorMap(depth_image, cv2.COLORMAP_JET)

        # 보행자와의 거리 계산
        if self.pedestrian_center_x is not None and self.pedestrian_center_y is not None:
            distance = self.cv_image_depth[self.pedestrian_center_y, self.pedestrian_center_x]
            # print(f"Distance to pedestrian: {distance:.2f} meters")

            distance_msg = Float32()
            distance_msg.data = float(distance)
            self.publisher_distance.publish(distance_msg)


        self.display_images()

    # 이미지 처리
    def display_images(self):
        if self.cv_image_rgb is not None and self.cv_image_depth_colormap is not None:
            # 두 이미지를 수평으로 결합
            combined_image = cv2.hconcat([self.cv_image_rgb, self.cv_image_depth_colormap])
            cv2.imshow("Processed Image", combined_image)
            cv2.waitKey(1)

    def destroy_windows(self):
        cv2.destroyAllWindows()  # 프로그램 종료 시 호출


def main(args=None):
    rclpy.init(args=args)
    node = PedestrianDetector()
    rclpy.spin(node)
    node.destroy_windows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
