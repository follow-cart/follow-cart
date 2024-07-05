import cv2
import numpy as np
from ultralytics import YOLO
import torch
from ament_index_python.packages import get_package_share_directory
import os

class FCRGBImageProcessor:
    def __init__(self):

        # 보행자 중심 좌표
        self._x = None
        self._y = None

    def process(self, img: np.ndarray, user):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = None
        winname = None
        if user == "fc1":

            lower_black = np.array([0, 0, 0])
            upper_black = np.array([10, 10, 10])

            mask = cv2.inRange(hsv, lower_black, upper_black)
            winname = "fc1"
        elif user == "fc2":
            lower_red = np.array([0, 70, 50])
            upper_red = np.array([10, 255, 255])

            mask = cv2.inRange(hsv, lower_red, upper_red)
            winname = "fc2"
        elif user == "fc3":
            lower_red = np.array([0, 70, 50])
            upper_red = np.array([10, 255, 255])

            mask = cv2.inRange(hsv, lower_red, upper_red)
            winname = "fc3"

        M = cv2.moments(mask)
        if M['m00'] > 0:
            self._x = int(M['m10'] / M['m00'])
            self._y = int(M['m01'] / M['m00'])
            cv2.circle(img, (self._x, self._y), 20, (0, 0, 0), -1)
        cv2.imshow(winname=winname, mat=img)
        cv2.waitKey(1)

    @property
    def x(self):
        return self._x

    @property
    def y(self):
        return self._y



