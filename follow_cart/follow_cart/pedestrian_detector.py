# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# import cv2
# import numpy as np
#
# class PedestrianDetector(Node):
#     def __init__(self):
#         super().__init__('pedestrian_detector')
#         self.subscription = self.create_subscription(
#             Image,
#             '/processed/depth/image_raw',
#             self.image_callback,
#             10)
#         self.bridge = CvBridge()
#
#     def image_callback(self, msg):
#         depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
#         detections = self.detect_pedestrians(depth_image)
#         # 처리 결과를 출력하거나 사용
#         print(detections)
#
#     def detect_pedestrians(self, depth_img):
#         # Depth 이미지를 사용하여 보행자 탐지
#         # 예시: 단순 깊이 임계값 적용
#         _, thresh = cv2.threshold(depth_img, 1, 2, cv2.THRESH_BINARY_INV)
#         contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#         detected_objects = []
#         for cnt in contours:
#             x, y, w, h = cv2.boundingRect(cnt)
#             if w > 30 and h > 30:  # 최소 크기 조건
#                 detected_objects.append((x, y, w, h))
#         return detected_objects
#
# def main(args=None):
#     rclpy.init(args=args)
#     pedestrian_detector = PedestrianDetector()

# pedestrian_detect.py

# 카메라 토픽을 수신하고 보여주는 것까지 완성된 V1

# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# import cv2
#
# class PedestrianDetector(Node):
#     def __init__(self):
#         super().__init__('pedestrian_detector')
#         self.bridge = CvBridge()
#
#         self.subscription = self.create_subscription(
#             Image,
#             '/processed/image',
#             self.callback,
#             10)
#
#     def callback(self, msg):
#         cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
#         cv2.imshow("Processed Image", cv_image)
#         cv2.waitKey(1)
#
#     def destroy_windows(self):
#         cv2.destroyAllWindows()  # 프로그램 종료 시 호출
#
# def main(args=None):
#     rclpy.init(args=args)
#     node = PedestrianDetector()
#     rclpy.spin(node)
#     node.destroy_windows()
#     node.destroy_node()
#     rclpy.shutdown()
#
# if __name__ == '__main__':
#     main()

# V1에서 보행자 인식 추가
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import torch
from ultralytics import YOLO

class PedestrianDetector(Node):
    def __init__(self):
        super().__init__('pedestrian_detector')
        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/processed/image',
            self.callback,
            10)

        # Load YOLOv8 model
        self.model = YOLO("yolov8n.pt")  # Download the appropriate YOLOv8 model

    def callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model(cv_image, imgsz=640)  # Use imgsz instead of size

        # Process results
        for result in results:
            for box in result.boxes:
                # Only consider pedestrians (class 0 in COCO dataset)
                if box.cls == 0:
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                    cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)

        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)  # Convert back to BGR for OpenCV
        cv2.imshow("Processed Image", cv_image)
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