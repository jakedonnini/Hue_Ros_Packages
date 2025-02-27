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

        self.dt = 0.05

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
        self.R = np.diag([0.11, 0.15, 0.05])

        # Observation matrix
        self.H = np.array([
        [1, 0, 0],  # Observation for x
        [0, 1, 0],  # Observation for y
        [0, 0, 1]   # Observation for theta
        ])

        self.gps_subscription = self.create_subscription(
            GpsData, 'gps/data', self.gps_callback, 10)
        self.deadReck_subscription = self.create_subscription(
            Coordinates,'deadReckoning/vel', self.deadReck_callback, 10)
        self.DR_subscription = self.create_subscription(
            GpsData, 'deadReckoning/pose', self.deadReck_callback_pose, 10)

        self.V = 0
        self.dV = 0
        self.isPainting = 0
        self.DR_x = 0
        self.DR_y = 0
        self.DR_angle = 0
        self.DR_x_rot = 0
        self.DR_y_rot = 0
        self.DR_angle_rot = 0
        self.Rot = np.eye(2)
        self.rotThata = 0
        
        self.gps_x = 0
        self.gps_y = 0
        self.gps_Theta = 0

        self.new_gps_data = False
        self.encoder_data_updated = False  # Reset flag
        self.x_gps = 0
        self.y_gps = 0
        self.gps_Theta = 0

        self.running = True
        self.lock = threading.Lock()

        # Publisher for midpoint and robot angle
        self.kalman_publisher = self.create_publisher(GpsData, 'kalman/data', 10)
        self.rotation_publisher = self.create_publisher(GpsData, 'deadReckoning/rotation', 10)

        self.processor_thread = threading.Thread(target=self.run_processing_loop)
        self.publisher_thread = threading.Thread(target=self.run_publishing_loop)
        self.processor_thread.start()
        self.publisher_thread.start()

        # is rotation matrix calculated?
        self.rotation_calculated = False

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

    def deadReck_callback_pose(self, msg):
        with self.lock:
            self.DR_x = msg.x
            self.DR_y = msg.y
            self.DR_angle = msg.angle

    def run_processing_loop(self):
        """Process waypoints and update encoder position as new data is available."""
        while self.running:
            # Process encoder data if updated
            if self.encoder_data_updated:
                with self.lock:
                    self.update_kalman_with_DR()
                    self.encoder_data_updated = False  # Reset flag

            if self.new_gps_data:
                with self.lock:
                    self.update_kalman_with_gps()
                    self.new_gps_data = False
            
            time.sleep(self.dt/2)

    def run_publishing_loop(self):
        """Publishes Kalman-filtered data at a constant rate."""
        while self.running:
            with self.lock:
                # Publish Kalman Filtered State
                # Publish Kalman Filtered State
                kal_msg = GpsData()
                kal_msg.x = float(self.x[0, 0])  # Explicitly cast to float
                kal_msg.y = float(self.x[1, 0])  # Explicitly cast to float
                kal_msg.angle = float(self.x[2, 0])  # Explicitly cast to float
                self.kalman_publisher.publish(kal_msg)

                # Publish Rotated Dead Reckoning Data
                rot_msg = GpsData()
                rot_msg.x = float(self.DR_x_rot)  # Explicitly cast to float
                rot_msg.y = float(self.DR_y_rot)  # Explicitly cast to float
                rot_msg.angle = float(self.DR_angle_rot)  # Explicitly cast to float
                self.rotation_publisher.publish(rot_msg)

                #TODO: kalman and rot are both 0 for some reason

                # Debugging Log
                self.get_logger().info(
                    f'[GPS]: {round(self.gps_x, 2)}, {round(self.gps_y, 2)}, {round(self.gps_Theta, 2)} '
                    f'[ENCODER] V: {round(self.V, 2)} dV: {round(self.dV, 2)} '
                    f'[KALMAN] X: {round(self.x[0, 0], 2)} Y: {round(self.x[1, 0], 2)} Theta: {round(self.x[2, 0], 2)}'
                )

            time.sleep(self.dt)  # Publish at `dt` interval
                
    def update_kalman_with_DR(self):
        """Call every time serial data comes in."""
        
        # emmet is confused :(((((
        if not self.rotation_calculated:
            self.rotation_calculated = True
            self.rotThata = self.gps_Theta
            self.Rot = np.array([[-np.cos(self.rotThata), -np.sin(self.rotThata)], [np.sin(self.rotThata), np.cos(self.rotThata)]]) # this includes rotation and relfection about y-axis

        # apply the R to the points
        self.DR_x_rot = (self.Rot[0, 0] * self.DR_x + self.Rot[0, 1] * self.DR_y)
        self.DR_y_rot = (self.Rot[1, 0] * self.DR_x + self.Rot[1, 1] * self.DR_y)

        # aline angle using the R matrix
        theta_rot_vec = self.Rot @ np.stack((np.cos(self.rotThata), np.sin(self.rotThata)))
        self.DR_angle_rot = np.arctan2(theta_rot_vec[1], theta_rot_vec[0])

        # Update B Control matrix
        self.B = np.array([
            [self.dt * np.cos(self.x[2, 0]), 0],
            [self.dt * np.sin(self.x[2, 0]), 0],
            [0, self.dt]
        ])
        u = np.array([[self.V], [self.dV]])

        state = self.B @ u
        self.get_logger().info(f"State before rot: {state}")
        state[:2, 0] = self.Rot @ state[:2, 0]
        state[2, 0] = self.DR_angle_rot

        self.get_logger().info(f"State before rot: {state}")

        # rotate the state
        self.x = self.F @ self.x + state
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update_kalman_with_gps(self):
        """Correct state estimate using GPS data."""
        # Measurement update
        z = np.array([[self.x_gps], [self.y_gps], [self.gps_Theta]])
        y = z - self.H @ self.x # Measurement residual
        S = self.H @ self.P @ self.H.T + self.R # Residual covariance
        K = self.P @ self.H.T @ np.linalg.inv(S) # Kalman gain
        self.x = self.x + K @ y
        self.P = (np.eye(3) - K @ self.H) @ self.P

    def stop_threads(self):
        """Stop the threads gracefully."""
        self.running = False
        self.processor_thread.join()
        self.publisher_thread.join()

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
