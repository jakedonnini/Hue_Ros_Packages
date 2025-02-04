import threading
import rclpy
from rclpy.node import Node
from custom_msg.msg import Coordinates, TwoInt
import time
import numpy as np
import os
import subprocess

class Sync(Node):
    def __init__(self):
        super().__init__('sync_node')

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

        if self.encoderTheta > math.pi:
            self.encoderTheta -= 2 * math.pi
        elif self.encoderTheta < -math.pi:
            self.encoderTheta += 2 * math.pi

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
        self.set_pwm(20, 20)
        time.sleep(3)  # Drive straight for 3 seconds

        # Turn left
        self.get_logger().info(f"Turning")
        self.set_pwm(-10, 10)
        time.sleep(2)  # Turn left for 2 seconds

        # Stop
        self.get_logger().info(f"Moving Forward")
        self.set_pwm(20, 20)
        self.get_logger().info("Movement sequence complete.")

        # Compute transformation
        self.compute_transformation()

        # Stop node and start navigation
        self.shutdown_node()

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
                        encoder_x = self.encoderX
                        encoder_y = self.encoderY
                        theta = self.encoderTheta
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
