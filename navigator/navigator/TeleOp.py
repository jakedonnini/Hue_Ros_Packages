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

        # Kalman Filter Matrices
        self.dt = 0.1  # time step
        self.x = np.array([[0], [0], [0]])  # initial state [x, y, theta]
        self.P = np.eye(3)  # Initial state covariance
        self.F = np.eye(3)  # State transition matrix
        self.B = np.zeros((3, 2))  # Control matrix (updated in loop)
        self.Q = np.diag([0.2, 0.2, 0.1])  # Process noise covariance
        self.R = np.diag([0.15, 0.15])  # Measurement noise covariance (GPS noise)
        self.H = np.array([[1, 0, 0], [0, 1, 0]])  # Observation matrix

        # Subscribers for GPS, Encoder, and Teleop Twist Keyboard
        self.gps_subscription = self.create_subscription(
            Coordinates, 'gps', self.gps_callback, 10)
        self.encoder_subscription = self.create_subscription(
            TwoInt, 'encoder', self.encoder_callback, 10)
        self.twist_subscription = self.create_subscription(
            Twist, 'cmd_vel', self.twist_callback, 10)

        # Publisher for PWM
        self.pwm_publisher = self.create_publisher(TwoInt, 'PWM', 10)

        # Encoder Variables
        self.encoder_left = 0
        self.encoder_right = 0
        self.encoder_data_updated = False
        self.encoderX = 0
        self.encoderY = 0
        self.encoderTheta = 0

        # Constants
        self.wheelR = 10.16
        self.wheelL = 64.77
        self.encoderTicks = 8192.0 / 2

        # GPS Variables
        self.latitude = None
        self.longitude = None
        self.origin_lat = None
        self.origin_lon = None
        self.new_gps_data = False
        self.x_gps_cm = 0
        self.y_gps_cm = 0
        self.lat_to_cm = 111139.0 * 100
        self.lon_to_cm = 111139.0 * 100

        # PWM Variables
        self.pwmr_value = 0
        self.pwml_value = 0

        # Threading
        self.running = True
        self.lock = threading.Lock()

        # Start threads for processing and logging
        self.processor_thread = threading.Thread(target=self.run_processing_loop)
        self.logging_thread = threading.Thread(target=self.log_positions)
        self.processor_thread.start()
        self.logging_thread.start()

    def gps_callback(self, msg):
        with self.lock:
            self.latitude = msg.x
            self.longitude = msg.y
            self.new_gps_data = True

    def encoder_callback(self, msg):
        with self.lock:
            self.encoder_left = msg.l
            self.encoder_right = msg.r
            self.encoder_data_updated = True

    def twist_callback(self, msg):
        """Process keyboard teleop commands."""
        linear_speed = msg.linear.x
        angular_speed = msg.angular.z

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

    def run_processing_loop(self):
        while self.running:
            if self.encoder_data_updated:
                with self.lock:
                    self.get_encoder_pose()
                    self.encoder_data_updated = False
            if self.new_gps_data:
                with self.lock:
                    self.update_kalman_with_gps()
                    self.new_gps_data = False
            time.sleep(0.05)

    def get_encoder_pose(self):
        vL = (6.2832 * self.wheelR * self.encoder_left) / (self.encoderTicks * self.dt)
        vR = (6.2832 * self.wheelR * self.encoder_right) / (self.encoderTicks * self.dt)
        V = 0.5 * (vR + vL)
        dV = (vR - vL) / self.wheelL

        self.encoderX += self.dt * V * math.cos(self.encoderTheta)
        self.encoderY += self.dt * V * math.sin(self.encoderTheta)
        self.encoderTheta += self.dt * dV

    def update_kalman_with_gps(self):
        if self.origin_lat is None or self.origin_lon is None:
            self.origin_lat = self.latitude
            self.origin_lon = self.longitude
            self.lon_to_cm = 111139.0 * 100 * np.cos(np.radians(self.origin_lat))

        delta_lat = self.latitude - self.origin_lat
        delta_lon = self.longitude - self.origin_lon
        self.x_gps_cm = delta_lon * self.lon_to_cm
        self.y_gps_cm = delta_lat * self.lat_to_cm

        z = np.array([[self.x_gps_cm], [self.y_gps_cm]])
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(3) - K @ self.H) @ self.P

    def log_positions(self):
        try:
            with open("/home/hue/ros2_ws/src/position_log_teleop.txt", 'w') as file:
                file.write("Time,GPS_X,GPS_Y,Encoder_X,Encoder_Y,Kalman_X,Kalman_Y,Theta\n")
                while self.running:
                    with self.lock:
                        gps_x = self.x_gps_cm
                        gps_y = self.y_gps_cm
                        encoder_x = self.encoderX
                        encoder_y = self.encoderY
                        kalman_x = self.x[0, 0]
                        kalman_y = self.x[1, 0]
                        theta = self.encoderTheta
                    timestamp = time.time()
                    file.write(f"{timestamp},{gps_x},{gps_y},{encoder_x},{encoder_y},{kalman_x},{kalman_y},{theta}\n")
                    file.flush()
                    time.sleep(0.1)
        except Exception as e:
            self.get_logger().error(f"Failed to write log: {e}")

    def stop_threads(self):
        self.running = False
        self.processor_thread.join()
        self.logging_thread.join()
        pwm_msg = TwoInt()
        pwm_msg.r = 0
        pwm_msg.l = 0
        self.pwm_publisher.publish(pwm_msg)


def main(args=None):
    rclpy.init(args=args)
    teleop_node = Teleop()
    try:
        rclpy.spin(teleop_node)
    except KeyboardInterrupt:
        teleop_node.stop_threads()
        teleop_node.destroy_node()
        rclpy.shutdown()
        print("Teleop node stopped.")

if __name__ == '__main__':
    main()
