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

        # Initialize PID constants
        self.Kp = 20.0   # Proportional constant (oscillates at 40)
        self.Ki = 0.2  # Integral constant
        self.Kd = 0.1  # Derivative constant

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
        self.processor_thread = threading.Thread(target=self.run_processing_loop)
        self.logging_thread = threading.Thread(target=self.log_positions)
        self.processor_thread.start()
        self.logging_thread.start()

        # Start movement sequence
        self.execute_movement_sequence()

    def gps_callback(self, msg):
        with self.lock:
            self.latitude, self.longitude = msg.x, msg.y
            self.new_gps_data = True

    def encoder_callback(self, msg):
        with self.lock:
            self.encoder_left, self.encoder_right = msg.l, msg.r
            self.encoder_data_updated = True

    def run_processing_loop(self):
        while self.running:
            if self.encoder_data_updated:
                with self.lock:
                    self.get_encoder_pose()
                    self.encoder_data_updated = False
            if self.new_gps_data:
                with self.lock:
                    self.update_gps_position()
                    self.new_gps_data = False
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

    def execute_movement_sequence(self):
        """Moves the robot forward, then turns left, then stops."""
        self.get_logger().info("Starting movement sequence.")

        # Move forward
        self.get_logger().info(f"Move Forward")
        # drive in a stright line and return true when done
        self.doneFirstMove = self.move(100, 0)
        if self.doneFirstMove:
            # Turn left and drive straight, true when done
            self.doneSecondMove = self.move(100, 100)
            if self.doneSecondMove:
                self.get_logger().info("Movement sequence complete.")
                self.set_pwm(0, 0)
    
                # Compute transformation
                self.compute_transformation()
        
                # Stop node and start navigation
                self.shutdown_node()

    def constrain(self, val, min_val, max_val):
        return max(min_val, min(val, max_val))
    
    def move(self, x, y):
        # Current positions based on GPS and encoder data (for now just encoder)
        self.currentX = self.encoder_x
        self.currentY = self.encoder_y
        self.currentTheta = self.encoder_theta

        dist2Go = math.sqrt(math.pow(self.currentX - x, 2) + math.pow(self.currentY - y, 2))
        if dist2Go < 5:  # threshold saying we hit the point (was 1)
            self.get_logger().info(f'Hit ({x}, {x}) waypoint')
            return True

        desiredQ = math.atan2(y - self.currentY, x - self.currentX)
        thetaError = desiredQ - self.currentTheta

        pwmAvg = 20 # normally 60

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
            self.get_logger().info(f"Turning")
            pwmAvg = 0
        else: 
            # when in thresh shouldn't move alot, half the intergrator
            I_term = I_term / 2

        pwmr_value = pwmAvg + pwmDel
        pwml_value = pwmAvg - pwmDel

        max_pwm = 30
        min_pwm = -30
        
        pwmr_value = self.constrain(pwmr_value, min_pwm, max_pwm)
        pwml_value = self.constrain(pwml_value, min_pwm, max_pwm)

        self.set_pwm(int(pwml_value), int(pwmr_value))

        self.get_logger().info(
            f'PWM: {int(pwmr_value)}, {int(pwml_value)}, Waypoint: {x}, {y}, Current Pos: {round(self.currentX, 2)}, {round(self.currentY, 2)} Theta error: {round(thetaError, 2)} dist2go {round(dist, 2)}'
        )

        # false to say we are still going
        return False
    
    def set_pwm(self, left_pwm, right_pwm):
        """Send PWM commands to move the robot."""
        # remove dead zone between 39 and -39 for L and R
        if right_pwm > 0:
            right_pwm += 39
        if right_pwm < 0:
            right_pwm -= 39

        if left_pwm > 0:
            left_pwm += 39
        if left_pwm < 0:
            left_pwm -= 39
      
        pwm_msg = TwoInt()
        pwm_msg.l, pwm_msg.r = left_pwm, right_pwm
        self.pwm_publisher.publish(pwm_msg)

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
