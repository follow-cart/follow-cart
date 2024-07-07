import cv2
import numpy as np
from ultralytics import YOLO
import torch
from ament_index_python.packages import get_package_share_directory
import os

class ConvoyRGBImageProcessor:
    def __init__(self):
        package_name = "follow_cart"
        pkg = get_package_share_directory(package_name)
        model_path = os.path.join(pkg, "YOLOV8_model", "yolov8n.pt")

        # GPU 사용 가능 여부 체크
        device = 'cuda' if torch.cuda.is_available() else 'cpu'
        # print(f"Using device: {device}")

        # Load YOLOv8 model
        self.model = YOLO(model_path)
        self.model.to(device)

        # 보행자 중심 좌표
        self._x = None
        self._y = None

    def process(self, img: np.ndarray):
        results = self.model(img, imgsz=640)

        # 프로세스 결과
        for result in results:
            for box in result.boxes:
                # 검출 (class 0  = 사람 / COCO dataset)
                if box.cls == 0:
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                    self._x = (x1 + x2) // 2
                    self._y = (y1 + y2) // 2
                    cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.circle(img, (self._x, self._y), 5, (0, 255, 0),
                               -1)
                    break
        cv2.imshow("Convoy Processed Image", img)
        cv2.waitKey(1)

    @property
    def x(self):
        return self._x

    @property
    def y(self):
        return self._y



