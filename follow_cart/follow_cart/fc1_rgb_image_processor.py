import cv2
import numpy as np
from ultralytics import YOLO
import torch
from ament_index_python.packages import get_package_share_directory
import os

class FC1RGBImageProcessor:
    def __init__(self):

        # 보행자 중심 좌표
        self._x = None
        self._y = None

    def process(self, img: np.ndarray):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        lower_orange = np.array([10, 100, 100])
        upper_orange = np.array([25, 255, 255])

        mask = cv2.inRange(hsv, lower_orange, upper_orange)

        M = cv2.moments(mask)
        if M['m00'] > 0:
            self._x = int(M['m10'] / M['m00'])
            self._y = int(M['m01'] / M['m00'])
            cv2.circle(img, (self._x, self._y), 20, (0, 0, 0), -1)
        cv2.imshow("FC1 Processed Image", img)
        cv2.waitKey(1)

    @property
    def x(self):
        return self._x

    @property
    def y(self):
        return self._y



