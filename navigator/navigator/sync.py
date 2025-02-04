import threading
import rclpy
from rclpy.node import Node
from custom_msg.msg import Coordinates, TwoInt
import time
import numpy as np
import os
import subprocess
import math

class Sync(Node):
    def __init__(self):
        super().__init__('sync')

        # Subscribers
        self.gps_subscription = self.create_subscription(Coordinates, 'gps', self.gps_callback, 10)
        self.encoder_subscription = self.create_subscription(TwoInt, 'encoder', self.encoder_callback, 10)

        # Publisher for PWM
        self.pwm_publisher = self.create_publisher(TwoInt, 'PWM', 10)

        # Encoder Variables
        self.encoder_x, self.encoder_y, self.encoder_theta = 0, 0, 0
        self.encoder_left, self.encoder_right = 0, 0
        self.encoder_data_updated = False

        # GPS Variables
        self.latitude, self.longitude = None, None
        self.origin_lat, self.origin_lon = None, None
        self.x_gps_cm, self.y_gps_cm = 0, 0
        self.new_gps_data = False
        self.lon_to_cm, self.lat_to_cm = 111139.0 * 100, 111139.0 * 100

        # Constants
        self.wheelR = 10.16
        self.wheelL = 64.77
        self.encoderTicks = 8192.0 / 2
        self.dt = 0.1

        # Data Storage
        self.gps_positions = []
        self.encoder_positions = []

        self.waypointBuffer = [(100,0,0), (100, 100, 0)] # start with L path
        self.firstWayPointSent = False
        self.currentTWayPoint = None

        # Initialize PID constants
        self.Kp = 20.0   # Proportional constant (oscillates at 40)
        self.Ki = 0.2  # Integral constant
        self.Kd = 0.1  # Derivative constant

        self.pwmr_value = 0
        self.pwml_value = 0
        self.pwmr_value_old = 0
        self.pwml_value_old = 0

        # Initialize PID terms
        self.integral = 0
        self.integral_min = -10  # Prevent excessive negative accumulation
        self.integral_max = 10   # Prevent excessive positive accumulation
        self.previous_error = 0

        self.doneFirstMove = False
        self.doneSecondMove = False

        # Threading
        self.running = True
        self.lock = threading.Lock()
        self.publisher_thread = threading.Thread(target=self.run_publish_loop)
        self.processor_thread = threading.Thread(target=self.run_processing_loop)
        self.logging_thread = threading.Thread(target=self.log_positions)

        self.publisher_thread.start()
        self.processor_thread.start()
        self.logging_thread.start()

    def gps_callback(self, msg):
        with self.lock:
            self.latitude, self.longitude = msg.x, msg.y
            self.new_gps_data = True

    def encoder_callback(self, msg):
        with self.lock:
            self.encoder_left, self.encoder_right = msg.l, msg.r
            self.encoder_data_updated = True

    def run_processing_loop(self):
        """Process waypoints and update encoder position as new data is available."""
        while self.running:
            # Process encoder data if updated
            if self.encoder_data_updated:
                with self.lock:
                    self.get_encoder_pose()
                    self.encoder_data_updated = False  # Reset flag

            if self.new_gps_data:
                with self.lock:
                    self.update_gps_position()
                    self.new_gps_data = False

            # Check for waypoints to process
            # self.get_logger().info(f"check way point {self.currentTWayPoint is None}, {len(self.waypointBuffer) > 0}")
            if self.latitude is not None and self.longitude is not None and: # have to get first GPS reading before go
                if self.currentTWayPoint is None and len(self.waypointBuffer) > 0:
                    with self.lock:
                        x, y, t = self.waypointBuffer.pop(0)
                        self.currentTWayPoint = (x, y)
                        
                        if not self.firstWayPointSent:
                            # when the first waypoint is sent reset the origin so we always start at 0 0
                            self.origin_lat = self.latitude
                            self.origin_lon = self.longitude
                            self.lon_to_cm = 111139.0 * 100 * np.cos(np.radians(self.origin_lat))
                            self.firstWayPointSent = True
            time.sleep(0.05)

    def get_encoder_pose(self):
        vL = (6.2832 * self.wheelR * self.encoder_left) / (self.encoderTicks * self.dt)
        vR = (6.2832 * self.wheelR * self.encoder_right) / (self.encoderTicks * self.dt)
        V = 0.5 * (vR + vL)
        dV = (vR - vL) / self.wheelL

        self.encoder_x += self.dt * V * np.cos(self.encoder_theta)
        self.encoder_y += self.dt * V * np.sin(self.encoder_theta)
        self.encoder_theta += self.dt * dV

        if self.encoder_theta > math.pi:
            self.encoder_theta -= 2 * math.pi
        elif self.encoder_theta < -math.pi:
            self.encoder_theta += 2 * math.pi

        # Store encoder position
        self.encoder_positions.append([self.encoder_x, self.encoder_y])

    def update_gps_position(self):
        if self.origin_lat is None or self.origin_lon is None:
            self.origin_lat, self.origin_lon = self.latitude, self.longitude
            self.lon_to_cm = 111139.0 * 100 * np.cos(np.radians(self.origin_lat))

        delta_lat = self.latitude - self.origin_lat
        delta_lon = self.longitude - self.origin_lon
        self.x_gps_cm = delta_lon * self.lon_to_cm
        self.y_gps_cm = delta_lat * self.lat_to_cm

        # Store GPS position
        self.gps_positions.append([self.x_gps_cm, self.y_gps_cm])

    def getPosError(self):
        """Compute the distance and angular error to the next waypoint."""
        if self.currentTWayPoint is None:
            return 0, 0

        waypointX, waypointY = self.currentTWayPoint

        # Current positions based on GPS and encoder data (for now just encoder)
        self.currentX = self.encoder_x
        self.currentY = self.encoder_y
        self.currentTheta = self.encoder_theta

        dist2Go = math.sqrt(math.pow(self.currentX - waypointX, 2) + math.pow(self.currentY - waypointY, 2))
        if dist2Go < 5:  # threshold saying we hit the point
            # self.get_logger().info(f'Hit ({waypointX}, {waypointY}) waypoint')
            self.currentTWayPoint = None

        desiredQ = math.atan2(waypointY - self.currentY, waypointX - self.currentX)
        thetaError = desiredQ - self.currentTheta

        if thetaError > math.pi:
            thetaError -= 2 * math.pi
        elif thetaError < -math.pi:
            thetaError += 2 * math.pi

        return dist2Go, thetaError

    def constrain(self, val, min_val, max_val):
        return max(min_val, min(val, max_val))

    def adjust_pwm_values(self):
        """Adjust and publish PWMR and PWML values based on GPS data."""
        dist, thetaError = self.getPosError()

        pwmAvg = 20 # normally 60 with dead zone

        # PID calculations
        # Proportional term (P)
        P_term = self.Kp * thetaError
        
        # Integral term (I)
        self.integral += thetaError
        self.integral = max(self.integral_min, min(self.integral, self.integral_max))  # Clamping
        I_term = self.Ki * self.integral
        
        # Derivative term (D)
        D_term = self.Kd * (thetaError - self.previous_error)
        
        # PID output
        pid_output = P_term + I_term + D_term
        
        # Update the previous error
        self.previous_error = thetaError

        # Adjust PWM values based on the PID output
        pwmDel = pid_output

        # If the angle is within this threshold then move forward
        # otherwise stop an turn
        threshold = 0.25
        if abs(thetaError) > threshold:
            pwmAvg = 0
        elif self.currentTWayPoint is None:
            pwmAvg = 0
            pwmDel = 0
        else: 
            # when in thresh shouldn't move alot, half the intergrator
            I_term = I_term / 2

        self.pwmr_value = pwmAvg + pwmDel
        self.pwml_value = pwmAvg - pwmDel

        max_pwm = 50
        min_pwm = -50
        
        self.pwmr_value = self.constrain(self.pwmr_value, min_pwm, max_pwm)
        self.pwml_value = self.constrain(self.pwml_value, min_pwm, max_pwm)

        # Anti-windup: Reduce integral accumulation if PWM is saturated
        if self.pwmr_value == max_pwm or self.pwmr_value == min_pwm:
            self.integral -= 0.1 * (self.pwmr_value - (pwmAvg + pwmDel))
    
        if self.pwml_value == max_pwm or self.pwml_value == min_pwm:
            self.integral -= 0.1 * (self.pwml_value - (pwmAvg - pwmDel))

        # remove dead zone between 39 and -39 for L and R
        if self.pwmr_value > 0:
            self.pwmr_value += 39
        if self.pwmr_value < 0:
            self.pwmr_value -= 39

        if self.pwml_value > 0:
            self.pwml_value += 39
        if self.pwml_value < 0:
            self.pwml_value -= 39

        self.get_logger().info(
            f'PID: Theta error: {round(thetaError, 2)} PID: {round(pid_output, 2)} P: {round(P_term, 2)} I: {round(I_term, 2)} D: {round(D_term, 2)} PWM_del {round(pwmDel, 2)}'
        )

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

        # for Kalman filiter testing
        self.get_logger().info(
            f'\rGPS: {round(self.x_gps_cm, 2)}, {round(self.y_gps_cm, 2)},  [ENCODER] X: {round(self.encoderX, 2)} Y: {round(self.encoderY, 2)} Q: {round(self.encoderTheta, 2)}, Current Pos: {round(self.currentX, 2)}, {round(self.currentY, 2)} Theta error: {round(thetaError, 2)} dist2go {round(dist, 2)} Waypoint: {self.currentTWayPoint}, PWM R: {self.pwmr_value} L: {self.pwml_value}'
        )

    def run_publish_loop(self):
        """Thread to continuously publish PWM values."""
        while self.running:
            self.adjust_pwm_values()
            time.sleep(0.1)

    def log_positions(self):
        try:
            with open("/home/hue/ros2_ws/src/position_log_sync.txt", 'w') as file:
                file.write("Time,GPS_X,GPS_Y,Encoder_X,Encoder_Y,Theta\n")
                while self.running:
                    with self.lock:
                        gps_x = self.x_gps_cm
                        gps_y = self.y_gps_cm
                        encoder_x = self.encoder_x
                        encoder_y = self.encoder_y
                        theta = self.encoder_theta
                    timestamp = time.time()
                    file.write(f"{timestamp},{gps_x},{gps_y},{encoder_x},{encoder_y},{theta}\n")
                    file.flush()
                    time.sleep(0.1)
        except Exception as e:
            self.get_logger().error(f"Failed to write log: {e}")

    def compute_transformation(self):
        """Computes the transformation matrix to align encoder positions to GPS positions."""
        encoder_array = np.array(self.encoder_positions)
        gps_array = np.array(self.gps_positions)

        if encoder_array.shape[0] < 2 or gps_array.shape[0] < 2:
            self.get_logger().error("Not enough data points to compute transformation.")
            return

        # Compute optimal rotation using Procrustes analysis
        encoder_mean = np.mean(encoder_array, axis=0)
        gps_mean = np.mean(gps_array, axis=0)

        encoder_centered = encoder_array - encoder_mean
        gps_centered = gps_array - gps_mean

        H = encoder_centered.T @ gps_centered
        U, _, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T

        if np.linalg.det(R) < 0:
            Vt[1, :] *= -1
            R = Vt.T @ U.T

        # Save matrix and angle
        save_path = "/home/hue/ros2_ws/src/navigator/navigator/transformation_matrix.txt"
        with open(save_path, 'w') as file:
            file.write(f"Rotation Matrix:\n{R}\n")
            file.write(f"Final Encoder Angle: {self.encoder_theta}\n")
            
        self.get_logger().info(f"Rotation Matrix:\n{R}\n")
        self.get_logger().info(f"Final Encoder Angle: {self.encoder_theta}\n")
        self.get_logger().info(f"Transformation matrix saved to {save_path}")

    def shutdown_node(self):
        """Stops this node and starts the navigation node."""
        self.running = False
        self.processor_thread.join()
        self.logging_thread.join()
        self.publisher_thread.join()

        # Stop ROS2 node
        self.destroy_node()
        rclpy.shutdown()

        # Start navigation node
        self.get_logger().info("Starting navigation node...")
        subprocess.Popen(["ros2", "run", "navigation_package", "navigation_node"])

def main(args=None):
    rclpy.init(args=args)
    sync_node = Sync()
    try:
        rclpy.spin(sync_node)
    except KeyboardInterrupt:
        sync_node.shutdown_node()

if __name__ == '__main__':
    main()
