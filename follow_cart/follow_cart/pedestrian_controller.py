import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, PoseWithCovariance, Quaternion, TwistWithCovariance
from pynput import keyboard
import threading

# 보행자 움직에 대한 처리를 하는 클래스
class PedestrianController(Node):

    def __init__(self):
        super().__init__('pedestrian_controller')

        # 사용자의 속도 정보를 전달할 publisher
        self.publisher_ = self.create_publisher(
            Twist,
            '/pedestrian/pedestrian_cmd',
            10
        )

        # 보행자의 위치 정보를 발행할 publisher
        self.odom_publisher_ = self.create_publisher(
            Odometry,
            '/pedestrian/current_position',
            10
        )

        # 사용자의 위치 정보를 받을 subscription
        self.subscription_ = self.create_subscription(
            Odometry,
            '/pedestrian/pedestrian_odom',
            self.listener_callback,
            10
        )

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.pedestrian_locate_callback)

        # 보행자 위치
        self.x = 0.0
        self.y = 0.0

        # Twist 메시지 초기화
        self.msg = Twist()

        # 키보드 입력 상태 체크
        self.keys_pressed = set()

        # 키보드 리스너
        self.keyboard_listener = keyboard.Listener(on_press=self.on_key_press, on_release=self.on_key_release)
        self.keyboard_listener.start()

    def listener_callback(self, odom: Odometry):
        self.x = odom.pose.pose.position.x
        self.y = odom.pose.pose.position.y

    # 보행자의 위치 처리
    def pedestrian_locate_callback(self):
        self.publisher_.publish(self.msg)

        # 현재 위치를 발행
        odom_msg = Odometry()
        odom_msg.header = Header()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"

        # 위치와 자세 설정
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = Quaternion()

        # 속도 설정
        odom_msg.twist.twist = self.msg

        self.odom_publisher_.publish(odom_msg)

    def on_key_press(self, key):
        try:
            self.keys_pressed.add(key.char)
        except AttributeError:
            pass
        self.update_twist_message()

    def on_key_release(self, key):
        try:
            self.keys_pressed.discard(key.char)
        except AttributeError:
            pass
        self.update_twist_message()

    # 키보드를 통한 보행자 움직임 처리 (WASD 사용)
    def update_twist_message(self):
        if 'w' in self.keys_pressed and 'a' in self.keys_pressed:
            self.msg.linear.y = -0.5
            self.msg.angular.z = 0.5
        elif 'w' in self.keys_pressed and 'd' in self.keys_pressed:
            self.msg.linear.y = -0.5
            self.msg.angular.z = -0.5
        elif 's' in self.keys_pressed and 'a' in self.keys_pressed:
            self.msg.linear.y = 0.5
            self.msg.angular.z = 0.5
        elif 's' in self.keys_pressed and 'd' in self.keys_pressed:
            self.msg.linear.y = 0.5
            self.msg.angular.z = 0.5
        elif 'w' in self.keys_pressed:
            self.msg.linear.y = -0.5
            self.msg.angular.z = 0.0
        elif 's' in self.keys_pressed:
            self.msg.linear.y = 0.5
            self.msg.angular.z = 0.0
        elif 'a' in self.keys_pressed:
            self.msg.linear.y = 0.0
            self.msg.angular.z = 0.5
        elif 'd' in self.keys_pressed:
            self.msg.linear.y = 0.0
            self.msg.angular.z = -0.5
        else:
            self.msg.linear.y = 0.0
            self.msg.angular.z = 0.0

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

