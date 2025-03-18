import threading
import rclpy
from rclpy.node import Node
from custom_msg.msg import Coordinates, TwoInt
from geometry_msgs.msg import Twist
import time
import math
import numpy as np

class Teleop(Node):
    def __init__(self):
        super().__init__('teleop_node')

        self.twist_subscription = self.create_subscription(
            Twist, 'cmd_vel', self.twist_callback, 10)

        # Publisher for PWM
        self.pwm_publisher = self.create_publisher(TwoInt, 'PWM', 10)

    def twist_callback(self, msg):
        """Process keyboard teleop commands."""
        linear_speed = msg.linear.x
        angular_speed = msg.angular.z / 2 # devide it by 2 by defualt

        # Convert linear/angular velocity to PWM values
        max_pwm = 100
        self.pwmr_value = int((linear_speed - angular_speed) * max_pwm)
        self.pwml_value = int((linear_speed + angular_speed) * max_pwm)

        self.get_logger().info(
            f'MOVEMENT: Linear: {round(linear_speed, 2)} Angular {round(angular_speed, 2)} PWM_R: {round(self.pwmr_value, 2)} PWM_L {round(self.pwml_value, 2)}'
        )

        pwm_msg = TwoInt()
        pwm_msg.r = self.pwmr_value
        pwm_msg.l = self.pwml_value
        self.pwm_publisher.publish(pwm_msg)


def main(args=None):
    rclpy.init(args=args)
    teleop_node = Teleop()
    try:
        rclpy.spin(teleop_node)
    except KeyboardInterrupt:
        rclpy.shutdown()
        print("Teleop node stopped.")

if __name__ == '__main__':
    main()
