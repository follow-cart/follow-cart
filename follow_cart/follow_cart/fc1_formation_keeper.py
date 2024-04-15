import math
import numpy as np
class FC1FormationKeeper:
    def __init__(self):
        self.distance = 1.0

    # def calculate(self, x, y, z, w):
    #     self.x = x
    #     self.y = y
    #     self.z = z
    #     self.w = w
    #
    #     # radian으로 변경
    #     roll, pitch, yaw = self.euler_from_quaternion(self.x, self.y, self.z, self.w)
    #
    #     # radian을 각도로 변경
    #     degree = math.degrees(yaw)
    #
    #     # convoy가 움직이는 각도와 convoy와의 거리를 통해 follow cart의 새로운 주행 목표 위치 계산
    #     # convoy 기준으로 y축 어디에 위치해야 하는지
    #     y = self.distance * math.sin(degree)
    #     # convoy 기준으로 x축 어디에 위치해야 하는지
    #     x = self.distance * math.cos(degree)
    #
    #     return x, y

    def calculate(self, x, y, z, w):
        # quaternion to Euler angles
        roll, pitch, yaw = self.euler_from_quaternion(x, y, z, w)

        # yaw to degrees
        degree = math.degrees(yaw) - 30

        # 삼각함수를 이용해
        x_cartesian = self.distance * math.cos(math.radians(degree))
        y_cartesian = self.distance * math.sin(math.radians(degree))

        return x_cartesian, y_cartesian

    def euler_from_quaternion(self, x, y, z, w):
        ysqr = y * y

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + ysqr)
        X = np.degrees(np.arctan2(t0, t1))

        t2 = +2.0 * (w * y - z * x)

        t2 = np.clip(t2, a_min=-1.0, a_max=1.0)
        Y = np.degrees(np.arcsin(t2))

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (ysqr + z * z)
        Z = np.degrees(np.arctan2(t3, t4))

        return X, Y, Z

    # def euler_from_quaternion(self, x, y, z, w):
    #     # Normalize the quaternion
    #     norm = np.sqrt(x ** 2 + y ** 2 + z ** 2 + w ** 2)
    #     x /= norm
    #     y /= norm
    #     z /= norm
    #     w /= norm
    #
    #     ysqr = y * y
    #
    #     # Roll (X-axis rotation)
    #     t0 = +2.0 * (w * x + y * z)
    #     t1 = +1.0 - 2.0 * (x * x + ysqr)
    #     roll = np.degrees(np.arctan2(t0, t1))
    #
    #     # Pitch (Y-axis rotation)
    #     t2 = +2.0 * (w * y - z * x)
    #     t2 = np.clip(t2, a_min=-1.0, a_max=1.0)
    #     pitch = np.degrees(np.arcsin(t2))
    #
    #     # Yaw (Z-axis rotation)
    #     t3 = +2.0 * (w * z + x * y)
    #     t4 = +1.0 - 2.0 * (ysqr + z * z)
    #     yaw = np.degrees(np.arctan2(t3, t4))
    #
    #     return roll, pitch, yaw



