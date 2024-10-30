import threading
import rclpy
from rclpy.node import Node
from custom_msg.msg import Coordinates
from custom_msg.msg import TwoInt
import time
import math
import numpy as np


class GPSSubscriberPublisher(Node):
    def __init__(self):
        super().__init__('navigation_node')

        # Kalman Filter Matrices
        self.dt = 0.1  # time step
        self.x = np.array([[0], [0], [0]])  # initial state [x, y, theta]
        
        # Define initial state covariance matrix
        self.P = np.eye(3)
        
        # Define state transition matrix
        self.F = np.array([
            [1, 0, -self.dt * np.sin(self.x[2, 0])],
            [0, 1,  self.dt * np.cos(self.x[2, 0])],
            [0, 0, 1]
        ])
        
        # Control matrix
        self.B = np.array([
            [self.dt * np.cos(self.x[2, 0]), 0],
            [self.dt * np.sin(self.x[2, 0]), 0],
            [0, self.dt]
        ])
        
        # Process noise covariance
        self.Q = np.diag([0.1, 0.1, 0.01])
        
        # Measurement noise covariance (GPS noise)
        self.R = np.diag([0.11, 0.15])
        
        # Observation matrix
        self.H = np.array([
            [1, 0, 0],
            [0, 1, 0]
        ])

        # Create subscriptions to the latitude and longitude topics
        self.gps_subscription = self.create_subscription(
            Coordinates, 
            'gps', 
            self.gps_callback, 
            10
        )

        self.encoder_subscription = self.create_subscription(
            TwoInt, 
            'encoder', 
            self.encoder_callback, 
            10
        )

        self.waypoint_subscription = self.create_subscription(
            Coordinates,          # Message type
            'coordinates',        # Topic name
            self.waypoint_callback,  # Callback function
            10                    # QoS profile (Queue size)
        )

        self.waypointBuffer = []
        self.currentTWayPoint = None

        # Create publishers for the PWMR and PWML topics
        self.pwm_publisher = self.create_publisher(TwoInt, 'PWM', 10)

        # Variables to store the most recent latitude and longitude values
        self.latitude = None
        self.longitude = None

        # Initial values for PWMR and PWML
        self.pwmr_value = 0
        self.pwml_value = 0

        # encoder varibles
        self.encoder_left = 0
        self.encoder_right = 0
        self.encoder_data_updated = 0

        self.currentX = 0
        self.currentY = 0

        # constants (change if drive train changes)
        self.wheelR = 3.45
        self.wheelL = 14.05

        # save old values to onlt send when it changes
        self.pwmr_value_old = 0
        self.pwml_value_old = 0

        # GPS
        # Initial GPS origin coordinates for conversion to local coordinates
        self.origin_lat = None
        self.origin_lon = None
        self.new_gps_data = False

         # Conversion factor for GPS to meters (approximately 111,139 meters per degree latitude)
        self.lat_to_cm = 111139.0 * 100 # 100 for cm
        self.lon_to_cm = 111139.0 * 100 * np.cos(np.radians(self.origin_lat or 0))  # Will be updated once origin_lat is set

        # Threading for concurrent execution
        self.running = True
        self.lock = threading.Lock()

        # Start threads for publishing and processing
        self.publisher_thread = threading.Thread(target=self.run_publish_loop)
        self.processor_thread = threading.Thread(target=self.run_processing_loop)

        self.publisher_thread.start()
        self.processor_thread.start()

    def gps_callback(self, msg):
        with self.lock:
            self.latitude = msg.x
            self.longitude = msg.y
            self.new_gps_data = True

    def encoder_callback(self, msg):
        with self.lock:
            self.encoder_left = msg.l
            self.encoder_right = msg.r
            self.encoder_data_updated = True  # Flag for new data

    def waypoint_callback(self, msg):
        with self.lock:
            self.waypointBuffer.append((msg.x, msg.y))
            # print(self.waypointBuffer)

    def run_publish_loop(self):
        """Thread to continuously publish PWM values."""
        while self.running:
            self.adjust_pwm_values()
            time.sleep(0.1)

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

            # Check for waypoints to process
            # self.get_logger().info(f"check way point {self.currentTWayPoint is None}, {len(self.waypointBuffer) > 0}")
            if self.currentTWayPoint is None and len(self.waypointBuffer) > 0:
                with self.lock:
                    self.currentTWayPoint = self.waypointBuffer.pop(0)
            time.sleep(0.05)

    def getEncoderPose(self):
        """call everytime serial data comes in"""
        vL = (6.2832*self.wheelR*self.encoder_left)/(1440.0*self.dt) #change with the number of ticks per encoder turn
        vR = (6.2832*self.wheelR*self.encoder_right)/(1440.0*self.dt)
        V = 0.5*(vR+vL)
        dV = (vR - vL) / self.wheelL

        u = np.array([[V], [dV]])

        # Predict Step
        self.x = self.F @ self.x + self.B @ u
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update_kalman_with_gps(self):
        """Correct state estimate using GPS data."""
        if self.origin_lat is None or self.origin_lon is None:
            self.origin_lat = self.latitude
            self.origin_lon = self.longitude
            self.lon_to_cm = 111139.0 * 100 * np.cos(np.radians(self.origin_lat))

        # Convert GPS data to cm relative to origin
        delta_lat = self.latitude - self.origin_lat
        delta_lon = self.longitude - self.origin_lon
        x_gps_cm = delta_lon * self.lon_to_cm
        y_gps_cm = delta_lat * self.lat_to_cm

        # Measurement update
        z = np.array([[x_gps_cm], [y_gps_cm]])
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(3) - K @ self.H) @ self.P

    def getPosError(self):
        """Compute the distance and angular error to the next waypoint."""
        if self.currentTWayPoint is None:
            return 0, 0

        waypointX, waypointY = self.currentTWayPoint

        # Current positions based on GPS and encoder data (for now just encoder)
        self.currentX = self.x[0, 0]
        self.currentY = self.x[1, 0]
        self.currentTheta = self.x[2, 0]

        dist2Go = math.sqrt(math.pow(self.currentX - waypointX/2, 2) + math.pow(self.currentY - waypointY/2, 2))
        if dist2Go < 1:  # threshold saying we hit the point
            # self.().info(f'Hit ({waypointX}, {waypointY}) waypoint')
            self.currentTWayPoint = None

        desiredQ = math.atan2(waypointY / 2-self.currentY, waypointX / 2-self.currentX)
        thetaError = desiredQ - self.currentTheta

        if thetaError > math.pi:
            thetaError -= 2 * math.pi
        elif thetaError < -math.pi:
            thetaError += 2 * math.pi

        # self.().info(
        #     f'Theat error: {thetaError} dist2go {dist2Go} desiredQ {desiredQ} CQ {self.currentTheta}'
        # )

        return dist2Go, thetaError

    def constrain(self, val, min_val, max_val):
        return max(min_val, min(val, max_val))

    def adjust_pwm_values(self):
        """Adjust and publish PWMR and PWML values based on GPS data."""
        dist, thetaError = self.getPosError()

        KQ = 20*4  # turn speed
        pwmDel = KQ * thetaError
        pwmAvg = 75

        if abs(thetaError) > 0.15 or self.currentTWayPoint is None:
            pwmAvg = 0
            pwmDel = self.constrain(pwmDel, -75, 75)

        pwmDel = self.constrain(pwmDel, -100, 100)

        self.pwmr_value = pwmAvg + pwmDel
        self.pwml_value = pwmAvg - pwmDel

        pwm_msg = TwoInt()
        pwm_msg.r = int(self.pwmr_value)
        pwm_msg.l = int(self.pwml_value)

        # if wheel still spinning send off again
        sureOff = (self.pwml_value == 0 and self.pwmr_value == 0) and (self.encoder_left != 0 or self.encoder_right != 0)

        # Publish the PWM values
        # only send if new values
        if (self.pwmr_value_old != self.pwmr_value) or (self.pwml_value_old != self.pwml_value) or sureOff:
            self.pwm_publisher.publish(pwm_msg)
            self.pwmr_value_old = self.pwmr_value
            self.pwml_value_old = self.pwml_value

        self.().info(
            f'PWM: {int(self.pwmr_value)}, {int(self.pwml_value)}, Waypoint: {self.currentTWayPoint}, Current Pos: {self.currentX}, {self.currentY} Theta error: {thetaError} dist2go {dist}'
        )

    def stop_threads(self):
        """Stop the threads gracefully."""
        self.running = False
        self.publisher_thread.join()
        self.processor_thread.join()


def main(args=None):
    rclpy.init(args=args)

    gps_subscriber_publisher = GPSSubscriberPublisher()

    try:
        rclpy.spin(gps_subscriber_publisher)
    except KeyboardInterrupt:
        gps_subscriber_publisher.stop_threads()
        gps_subscriber_publisher.destroy_node()
        rclpy.shutdown()
        print("GPS subscriber/publisher node stopped.")


if __name__ == '__main__':
    main()

