import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class PedestrianController(Node):

    def __init__(self):
        super().__init__('pedestrian_controller')

        # 사용자의 속도 정보를 전달할 publisher
        self.publisher_ = self.create_publisher(Twist, '/pedestrian/pedestrian_cmd', 10)

        # 사용자의 위치 정보를 받을 subscription
        self.subscription_ = self.create_subscription(Odometry, '/pedestrian/pedestrian_odom', self.listener_callback, 10)

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # 움직이는 방향
        self.direction = -1
        self.msg = Twist()
        # 보행자 위치
        self.x = None
        self.y = None

        # 정지를 시작한 시간
        self.pause_start = None
        # 정지 상태 여부
        self.is_paused = False

    def listener_callback(self, odom: Odometry):
        self.y = odom.pose.pose.position.y

        # 방향 전환 조건 검사
        # if not self.is_paused and ((self.direction == 1 and self.y >= 6) or ((self.direction == -1 and self.y <= -6))):
        #     self.pause_start = time.time()
        #     self.is_paused = True
        #     self.msg.linear.y = 0
        #     self.publisher_.publish(self.msg)

        # 보행자가 왕복할 수 있도록
        if self.direction == 1 and self.y >= 7:
            self.direction *= -1
        if self.direction == -1 and self.y <= -7:
            self.direction *= -1

    def timer_callback(self):
        # if self.is_paused:
        #     if time.time() - self.pause_start >= 5:
        #         self.direction *= -1
        #         self.msg.angular.z = 100
        #         self.is_paused = False
        #
        # if self.is_paused == False:
            self.msg.linear.y = 1.1 * self.direction
            self.publisher_.publish(self.msg)

def main(args=None):
    rclpy.init(args=args)
    pedestrian_controller = PedestrianController()
    rclpy.spin(pedestrian_controller)
    pedestrian_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
