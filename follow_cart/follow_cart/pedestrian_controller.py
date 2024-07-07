import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

import threading

# 보행자 움직에 대한 처리를 하는 클래스
class PedestrianController(Node):

    def __init__(self):
        super().__init__('pedestrian_controller')

        # 사용자의 속도 정보를 전달할 publisher
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/pedestrian/pedestrian_cmd',
            10
        )

        self.cmd_subscription = self.create_subscription(String, '/pedestrian_cmd', self.cmd_cb, 10)


    def cmd_cb(self, cmd):
        msg = Twist()
        if cmd.data == 'Forward':
            msg.linear.y = -0.5
            msg.angular.z = 0.0
        elif cmd.data == 'Backward':
            msg.linear.y = 0.5
            msg.angular.z = 0.0
        elif cmd.data == 'Left Turn':
            msg.linear.y = 0.0
            msg.angular.z = 0.5
        elif cmd.data == 'Right Turn':
            msg.linear.y = 0.0
            msg.angular.z = -0.5
        elif cmd.data == 'Stop':
            msg.linear.y = -0.5
            msg.angular.z = 0.0

        self.cmd_vel_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    pedestrian_controller = PedestrianController()

    # 별도의 스레드에서 ROS2 스핀을 실행
    ros_thread = threading.Thread(target=rclpy.spin, args=(pedestrian_controller,))
    ros_thread.start()

    try:
        ros_thread.join()
    except KeyboardInterrupt:
        pass
    pedestrian_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
