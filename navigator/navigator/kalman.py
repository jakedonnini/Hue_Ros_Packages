import rclpy
import threading
from rclpy.node import Node
from custom_msg.msg import Coordinates
from custom_msg.msg import TwoInt
from custom_msg.msg import GpsData
import time
import math
import numpy as np
import copy

class KalmanFilter(Node):
    def __init__(self):
        super().__init__('navigation_node')

        # Kalman Filter Matrices
        self.x = np.array([[0], [0], [0]])  # initial state [x, y, theta]
        self.lastx = copy.deepcopy(self.x)

        # Define initial state covariance matrix
        self.P = np.eye(3)

        # State transition matrix (Identity unless external forces act)
        self.F = np.eye(3)

        # Control matrix
        self.B = np.array([
            [self.dt * np.cos(self.x[2, 0]), 0],
            [self.dt * np.sin(self.x[2, 0]), 0],
            [0, self.dt]
        ])

        # Process noise covariance
        self.Q = np.diag([0.4, 0.4, 0.3])

        # Measurement noise covariance (GPS noise)
        self.R = np.diag([0.11, 0.15])

        # Observation matrix
        self.H = np.array([
            [1, 0, 0],
            [0, 1, 0]
        ])

        self.gps_subscription = self.create_subscription(
            GpsData, 
            'gps/data', 
            self.gps_callback, 
            10
        )

        self.deadReck_subscription = self.create_subscription(
            Coordinates, 
            'deadReckoning/data', 
            self.deadReck_callback, 
            10
        )

        self.V = 0
        self.dV = 0
        self.isPainting = 0
        self.gps_x = 0
        self.gps_y = 0
        self.gps_Theta = 0

        self.new_gps_data = False
        self.encoder_data_updated = False  # Reset flag
        self.x_gps_cm = 0
        self.y_gps_cm = 0

        self.running = True
        self.lock = threading.Lock()

        # Publisher for midpoint and robot angle
        self.kalman_publisher = self.create_publisher(GpsData, 'kalman/data', 10)

        self.processor_thread = threading.Thread(target=self.run_processing_loop)
        self.processor_thread.start()

    def gps_callback(self, msg):
        with self.lock:
            self.gps_x = msg.x
            self.gps_y = msg.y
            self.gps_Theta = msg.angle
            self.new_gps_data = True

    def deadReck_callback(self, msg):
        with self.lock:
            self.V = msg.x
            self.dV = msg.y
            self.isPainting = msg.toggle
            self.encoder_data_updated = True  # Flag for new data

    def run_processing_loop(self):
        """Process waypoints and update encoder position as new data is available."""
        while self.running:
            # Process encoder data if updated
            if self.encoder_data_updated:
                with self.lock:
                    self.getEncoderPose()
                    self.encoder_data_updated = False  # Reset flag

            if self.new_gps_data:
                with self.lock:
                    self.update_kalman_with_gps()
                    self.new_gps_data = False

            if not (self.lastx == self.x).all():
                self.lastx = copy.deepcopy(self.x)
                # send message
                kal_msg = GpsData()
                kal_msg.x = self.x[0, 0]
                kal_msg.y = self.x[1, 0]
                kal_msg.angle = self.x[2, 0]
                self.kalman_publisher.publish(kal_msg)
                # for Kalman filiter testing
                self.get_logger().info(
                    f'\rGPS: {round(self.gps_x, 2)}, {round(self.gps_y, 2)}, {round(self.gps_Theta, 2)}  [ENCODER] V: {round(self.V, 2)} dV: {round(self.dV, 2)} [KALMAN] X: {round(self.x[0, 0], 2)} Y: {round(self.x[1, 0], 2)} Theta: {round(self.x[2, 0], 2)}'
                )
                
    def update_kalman_with_DR(self):
        """Call every time serial data comes in."""

        # Update B Control matrix
        self.B = np.array([
            [self.dt * np.cos(self.x[2, 0]), 0],
            [self.dt * np.sin(self.x[2, 0]), 0],
            [0, self.dt]
        ])
        u = np.array([[self.V], [self.dV]])

        self.x = self.F @ self.x + self.B @ u
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update_kalman_with_gps(self):
        """Correct state estimate using GPS data."""
        # Measurement update
        z = np.array([[self.x_gps], [self.y_gps]])
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(3) - K @ self.H) @ self.P

    def stop_threads(self):
        """Stop the threads gracefully."""
        self.running = False
        self.processor_thread.join()

def main(args=None):
    rclpy.init(args=args)

    KF = KalmanFilter()

    try:
        rclpy.spin(KF)
    except KeyboardInterrupt:
        KF.stop_threads()
        KF.destroy_node()
        rclpy.shutdown()
        print("Kalman Filter node stopped.")

if __name__ == '__main__':
    main()
