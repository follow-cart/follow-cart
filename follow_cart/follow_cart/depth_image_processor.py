import cv2
import numpy as np

class DepthImageProcessor:
    def __init__(self):

        # 이미지 저장을 위한 변수
        self.cv_image_rgb = None
        self.cv_image_depth = None
        self.cv_image_depth_colormap = None

        # 보행자와의 거리
        self._detection = None

    def process(self, img, target_x, target_y):

        # Depth 이미지 클리핑 및 정규화
        depth_min = 0.1  # 최소 거리 (단위 : m)
        depth_max = 20.0  # 최대 거리 (단위 : m)
        depth_image = np.clip(img, depth_min, depth_max)
        depth_image = (depth_image - depth_min) / (depth_max - depth_min)  # 정규화

        # 8비트 이미지로 변환
        depth_image = (depth_image * 255).astype(np.uint8)
        self.cv_image_depth_colormap = cv2.applyColorMap(depth_image, cv2.COLORMAP_JET)

        # 보행자와의 거리 계산
        self._detection = [float(target_x), float(target_y), float(img[target_y, target_x])]

    @property
    def detection(self):
        return self._detection




