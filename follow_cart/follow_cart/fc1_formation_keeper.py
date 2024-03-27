import math
class FC1FormationKeeper:
    def __init__(self):
        self.distance = 1.0
        self.x = None
        self.y = None
        self.z = None
        self.w = None

    def calculate(self, x, y, z, w):
        self.x = x
        self.y = y
        self.z = z
        self.w = w

        # radian으로 변경
        roll, pitch, yaw = self.euler_from_quaternion(self.x, self.y, self.z, self.w)

        # radian을 각도로 변경
        degree = math.degrees(yaw)

        # convoy가 움직이는 각도와 convoy와의 거리를 통해 follow cart의 새로운 주행 목표 위치 계산
        # convoy 기준으로 y축 어디에 위치해야 하는지
        y = self.distance * math.sin(degree)
        # convoy 기준으로 x축 어디에 위치해야 하는지
        x = self.distance * math.cos(degree)

        return x, y

    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z  # in radians



