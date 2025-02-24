import threading
import rclpy
from rclpy.node import Node
from custom_msg.msg import Coordinates, TwoInt, GpsData
import time
import math
import numpy as np

class Log(Node):
    def __init__(self):

        # Subscribers for GPS, Encoder, and Teleop Twist Keyboard
        self.gps_subscription = self.create_subscription(
            GpsData, 'gps/data', self.gps_callback, 10)
        self.encoder_subscription = self.create_subscription(
            TwoInt, 'encoder', self.encoder_callback, 10)

        
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

        # Apply rotation matrix to align encoder position with GPS
        rotated_pos = self.Rot_Matrix @ np.array([[self.encoderX], [self.encoderY]])
        print("normal: ", self.encoderX, self.encoderY)
        print("vect: ",  np.array([[self.encoderX], [self.encoderY]]))
        print("rotated_pos: ", rotated_pos)
        print("Flattened: ", rotated_pos.flatten())
        self.encoderRotX, self.encoderRotY = rotated_pos.flatten()

        # Create extended 3x3 rotation matrix (includes theta)
        Rot_Extended = np.eye(3)
        Rot_Extended[:2, :2] = self.Rot_Matrix  # Embed the 2x2 rotation into the 3x3 matrix
        print("Rot_Extended: ", Rot_Extended)

        # update B Control matrix
        self.B = np.array([
            [self.dt * np.cos(self.x[2, 0]), 0],
            [self.dt * np.sin(self.x[2, 0]), 0],
            [0, self.dt]
        ])

        u = np.array([[V], [dV]])

        stateUpdate = self.B @ u
        print("B: ", self.B)
        print("u: ", u)

        print("stateUpdate: ", stateUpdate)

        # mult the u vector and B matrix by the rotation to get into the GPS frame
        correctedStateUpdate = Rot_Extended @ stateUpdate

        print("correctedStateUpdate: ", correctedStateUpdate)

        # Predict Step
        self.x = self.F @ self.x + correctedStateUpdate
        self.P = self.F @ self.P @ self.F.T + self.Q

        if self.encoderTheta > math.pi:
            self.encoderTheta -= 2 * math.pi
        elif self.encoderTheta < -math.pi:
            self.encoderTheta += 2 * math.pi

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

    def read_transformation_matrix(self, file_path):
        """
        Reads the transformation matrix and theta from a file and returns them as variables.
        
        :param file_path: Path to the transformation matrix file
        :return: Tuple (R, theta) where:
                 - R is a 2x2 numpy array (rotation matrix)
                 - theta is a float (final encoder angle)
        """
        try:
            with open(file_path, 'r') as file:
                lines = file.readlines()
    
            # Extract rotation matrix (assuming 2x2 format)
            matrix_lines = [line.strip() for line in lines if "[" in line or "]" in line]
            R = np.array([[float(num) for num in matrix_lines[0].strip('[]').split()],
                          [float(num) for num in matrix_lines[1].strip('[]').split()]])
    
            # Extract theta (last line contains the final encoder angle)
            theta_line = [line for line in lines if "Final Encoder Angle" in line][0]
            theta = float(theta_line.split(":")[1].strip())
    
            return R, theta
    
        except Exception as e:
            print(f"Error reading transformation matrix file: {e}")
            return  np.eye(2) , 0

    def log_positions(self):
        try:
            with open("/home/hue/ros2_ws/src/position_log_teleop.txt", 'w') as file:
                file.write("Time,GPS_X,GPS_Y,Encoder_X,Encoder_Y,Kalman_X,Kalman_Y,Rot_x,Rot_yTheta\n")
                while self.running:
                    with self.lock:
                        gps_x = self.x_gps_cm
                        gps_y = self.y_gps_cm
                        encoder_x = self.encoderX
                        encoder_y = self.encoderY
                        rot_x = self.encoderRotX
                        rot_y = self.encoderRotY
                        kalman_x = self.x[0, 0]
                        kalman_y = self.x[1, 0]
                        theta = self.encoderTheta
                    timestamp = time.time()
                    file.write(f"{timestamp},{gps_x},{gps_y},{encoder_x},{encoder_y},{kalman_x},{kalman_y},{rot_x},{rot_y},{theta}\n")
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
    log = Log()
    try:
        rclpy.spin(log)
    except KeyboardInterrupt:
        log.stop_threads()
        log.destroy_node()
        rclpy.shutdown()
        print("log node stopped.")

if __name__ == '__main__':
    main()
