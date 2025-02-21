import rclpy
import threading
from rclpy.node import Node
from custom_msg.msg import Coordinates
from custom_msg.msg import TwoInt
from custom_msg.msg import GpsData
import time
import math
import numpy as np

class KalmanFilter(Node):
  def __init__(self):
        super().__init__('navigation_node')

        # Kalman Filter Matrices
        self.dt = 0.05  # time step
        self.x = np.array([[0], [0], [0]])  # initial state [x, y, theta]
        
        # Define initial state covariance matrix
        self.P = np.eye(3)
        
        # Define state transition matrix
        # this should be I unless external forces act on the bot
        # self.F = np.array([
        #     [1, 0, -self.dt * np.sin(self.x[2, 0])],
        #     [0, 1,  self.dt * np.cos(self.x[2, 0])],
        #     [0, 0, 1]
        # ])
        self.F = np.eye(3)
        
        # Control matrix
        self.B = np.array([
            [self.dt * np.cos(self.x[2, 0]), 0],
            [self.dt * np.sin(self.x[2, 0]), 0],
            [0, self.dt]
        ])
        
        # Process noise covariance
        self.Q = np.diag([0.4, 0.4, 0.3])
        
        # Measurement noise covariance (GPS noise) #0.11, 0.15
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
            TwoInt, 
            'deadReckoning/data', 
            self.deadReck_callback, 
            10
        )

        # Publisher for midpoint and robot angle
        self.kalman_publisher = self.create_publisher(GpsData, 'kalman', 10)

        self.processor_thread = threading.Thread(target=self.run_processing_loop)

        self.processor_thread.start()

  def gps_callback(self, msg):
        with self.lock:
          self.latitude = msg.x
          self.longitude = msg.y
          self.gpsTheta = msg.angle
          self.new_gps_data = True

  def deadReck_callback(self, msg):
        with self.lock:
          self.encoder_left = msg.l
          self.encoder_right = msg.r
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
# TODO: add function, rotations, zero out cm

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
