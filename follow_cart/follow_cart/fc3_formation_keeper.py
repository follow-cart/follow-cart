import math
import numpy as np
class FC3FormationKeeper:
    def __init__(self):
        self.distance = 1.15

    def calculate(self, x, y, z, w):
        # quaternion to Euler angles
        roll, pitch, yaw = self.euler_from_quaternion(x, y, z, w)

        # yaw to degrees
        degree = math.degrees(yaw) + 30

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



