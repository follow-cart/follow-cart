import math
from .euler_from_quaternion import EulerFromQuaternion
class FC2FormationKeeper:
    def __init__(self):
        # 세로 대형: 2.0 가로 대형: 1.0 삼각 대형: 1.0
        self.distance = 1.0

    def calculate(self, x, y, z, w):
        # quaternion to Euler angles
        roll, pitch, yaw = EulerFromQuaternion.euler_from_quaternion(x, y, z, w)

        # yaw 형태의 방향을 degrees 형태로
        # 세로 대형: 0 가로 대형: -45 삼각 대형: -30
        degree = math.degrees(yaw) - 30.0

        # 삼각함수와 삼각비를 이용해 convoy가 바라보는 방향을 따라 변화하는 대형의 x,y 위치 차이 계산
        x_cartesian = self.distance * math.cos(math.radians(degree))
        y_cartesian = self.distance * math.sin(math.radians(degree))

        return x_cartesian, y_cartesian




