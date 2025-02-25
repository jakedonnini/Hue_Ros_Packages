import rclpy
from rclpy.node import Node
from custom_msg.msg import GpsData
from custom_msg.msg import Coordinates
from custom_msg.msg import TwoInt
import numpy as np
import math

class DeadReckoning(Node):
    def __init__(self):
        super().__init__('dead_reckoning')
        
        # Subscription to encoder data
        self.encoder_subscription = self.create_subscription(
            TwoInt, 
            'encoder', 
            self.encoder_callback, 
            10
        )
        self.get_logger().info("Subscribed to encoder topic")

        # Publishers for velocity and pose
        self.vel_publisher = self.create_publisher(Coordinates, 'deadReckoning/vel', 10)
        self.pose_publisher = self.create_publisher(GpsData, 'deadReckoning/pose', 10)

        # Robot kinematics variables
        self.encoderX = 0.0
        self.encoderY = 0.0
        self.encoderTheta = 0.0
        self.wheelR = 9.708
        self.wheelL = 64.77
        self.encoderTicks = 8192.0 / 2  # Only counts half the encoder ticks
        self.errorScaler = 1
        self.dt = 0.05
        self.dir = -1  # Set to -1 to invert the forward direction

    def encoder_callback(self, msg):
        # self.get_logger().info("Received encoder data")  # Debugging print

        vL = (6.2832 * self.wheelR * msg.l * self.errorScaler * self.dir) / (self.encoderTicks * self.dt)
        vR = (6.2832 * self.wheelR * msg.r * self.errorScaler * self.dir) / (self.encoderTicks * self.dt)
        V = 0.5 * (vR + vL)
        dV = (vR - vL) / self.wheelL

        # Compute the estimated position
        self.encoderX += self.dt * V * math.cos(self.encoderTheta)
        self.encoderY += self.dt * V * math.sin(self.encoderTheta)
        self.encoderTheta += self.dt * dV

        # Publish velocity
        coord_msg = Coordinates()
        coord_msg.x = V
        coord_msg.y = dV
        coord_msg.toggle = msg.toggle
        self.vel_publisher.publish(coord_msg)

        # Publish pose
        pose_msg = GpsData()
        pose_msg.x = self.encoderX
        pose_msg.y = self.encoderY
        pose_msg.angle = self.encoderTheta
        self.pose_publisher.publish(pose_msg)

        # self.get_logger().info(f'Pose: x={self.encoderX:.2f}, y={self.encoderY:.2f}, theta={self.encoderTheta:.2f}')
        # self.get_logger().info(f'Velocity: V={V:.2f}, dV={dV:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = DeadReckoning()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
