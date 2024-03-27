import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from .dial_gui import DialGui
from .velocity_calculator import VelocityCalculator
import math

class ConvoyController(Node):
    PUB_RATE = 10.0
    DURATION = 2.0

    def __init__(self, gui: DialGui):
        super().__init__('convoy_controller')
        self.gui = gui
        self.twist_publisher = self.create_publisher(Twist, '/convoy/cmd_vel', 10)

        timer_period = 5
        self.timer = self.create_timer(timer_period, self.pub_callback)
        self.current_linear_x = 0.0
        self.target_linear_x = 0.0
        self.current_angular_z = 0.0
        self.target_angular_z = 0.0
        self.linear_x_calculator = VelocityCalculator(1 / timer_period, 1.0, 0.0, 0.0)
        self.angular_z_calculator = VelocityCalculator(1 / timer_period, 1.0, 0.0, 0.0)

    def pub_callback(self):
        new_linear_x = self.gui.linear_velocity()
        new_angular_z = self.gui.angular_velocity()

        if new_linear_x != self.target_linear_x:
            self.target_linear_x = new_linear_x
            self.linear_x_calculator = VelocityCalculator(ConvoyController.PUB_RATE,
                                                          ConvoyController.DURATION,
                                                          self.current_linear_x,
                                                          self.target_linear_x)

        if new_angular_z != self.target_angular_z:
            self.target_angular_z = new_angular_z
            self.angular_z_calculator = VelocityCalculator(ConvoyController.PUB_RATE,
                                                           ConvoyController.DURATION,
                                                           self.current_angular_z,
                                                           self.target_angular_z)

        msg = Twist()
        self.current_linear_x = self.linear_x_calculator.next_value()
        self.current_angular_z = self.angular_z_calculator.next_value()
        msg.linear.x = float(self.current_linear_x)
        msg.angular.z = float(math.radians(self.current_angular_z))

        self.twist_publisher.publish(msg)

def main(args=None):
    gui = DialGui()
    rclpy.init(args=args)
    convoy_controller = ConvoyController(gui)
    try:
        rclpy.spin(convoy_controller)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
