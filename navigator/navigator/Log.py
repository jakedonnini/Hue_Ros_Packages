import threading
import rclpy
from rclpy.node import Node
from custom_msg.msg import Coordinates, TwoInt, GpsData
import time
import math
import numpy as np

class Log(Node):
    def __init__(self):
        super().__init__('navigation_node')

        # Subscribers for GPS, Encoder, and Teleop Twist Keyboard
        self.gps_subscription1 = self.create_subscription(
            Coordinates, 'gps1_cm', self.gps_callback1, 10)
        self.gps_subscription2 = self.create_subscription(
            Coordinates, 'gps2_cm', self.gps_callback2, 10)
        self.gps_subscription_combo = self.create_subscription(
            GpsData, 'gps/data', self.gps_callback_combo, 10)
        self.DR_subscription = self.create_subscription(
            GpsData, 'deadReckoning/pose', self.deadReck_callback, 10)
        self.DR_subscription_vel = self.create_subscription(
            Coordinates, 'deadReckoning/vel', self.deadReck_callback_t, 10)
        self.Kalman = self.create_subscription(
            GpsData, 'kalman/data', self.kalman, 10)
        
        
        # gps varibles
        self.gps_cm_x1 = 0
        self.gps_cm_y1 = 0
        self.gps_cm_x2 = 0
        self.gps_cm_y2 = 0
        self.gps_mid_x = 0
        self.gps_mid_y = 0
        self.gps_angle = 0

        # DR
        self.isPainting = 0
        self.DR_x = 0
        self.DR_y = 0
        self.DR_angle = 0

        # kalman
        self.kalman_x = 0
        self.kalman_y = 0
        self.kalman_angle = 0

        self.running = True
        self.logging_thread = threading.Thread(target=self.log_positions)
        self.logging_thread.start()

        
    def gps_callback1(self, msg):
        with self.lock:
            self.gps_cm_x1 = msg.x
            self.gps_cm_y1 = msg.y

    def gps_callback2(self, msg):
        with self.lock:
            self.gps_cm_x2 = msg.x
            self.gps_cm_y2 = msg.y

    def gps_callback_combo(self, msg):
        with self.lock:
            self.gps_mid_x = msg.x
            self.gps_mid_y = msg.y
            self.gps_angle = msg.angle

    def deadReck_callback(self, msg):
        with self.lock:
            self.DR_x = msg.x
            self.DR_y = msg.y
            self.DR_angle = msg.angle

    def deadReck_callback_t(self, msg):
        with self.lock:
            self.isPainting = msg.toggle

    def kalman(self, msg):
        with self.lock:
            self.kalman_x = msg.x
            self.kalman_y = msg.y
            self.kalman_angle = msg.angle


    def log_positions(self):
        try:
            with open("/home/hue/ros2_ws/src/position_log_total", 'w') as file:
                file.write("Time,GPS_X1,GPS_Y1,GPS_X2,GPS_Y2,GPS_MIDX,GPS_MIDY,GPS_MID_Theta,DR_X,DR_Y,DR_Theta,Kalman_X,Kalman_Y,Kalman_Theta,Painting\n")
                while self.running:
                    with self.lock:
                        gps_x1 = self.gps_cm_x1
                        gps_y1 = self.gps_cm_y1
                        gps_x2 = self.gps_cm_x2
                        gps_y2 = self.gps_cm_x2
                        gps_mid_x = self.gps_mid_x
                        gps_mid_y = self.gps_mid_y
                        gps_mid_theta = self.gps_angle
                        encoder_x = self.DR_x
                        encoder_y = self.DR_y
                        encoder_theta = self.DR_angle
                        kalman_x = self.kalman_x
                        kalman_y = self.kalman_y
                        kalman_theta = self.kalman_angle
                        painting = self.isPainting
                    timestamp = time.time()
                    file.write(f"{timestamp},{gps_x1},{gps_y1},{gps_x2},{gps_y2},{gps_mid_x},{gps_mid_y},{gps_mid_theta},{encoder_x},{encoder_y},{encoder_theta},{kalman_x},{kalman_y},{kalman_theta},{painting}\n")
                    file.flush()
                    time.sleep(0.1)
        except Exception as e:
            self.get_logger().error(f"Failed to write log: {e}")

    def stop_threads(self):
        self.running = False
        self.logging_thread.join()


def main(args=None):
    rclpy.init(args=args)
    log = Log()
    try:
        rclpy.spin(log)
    except KeyboardInterrupt:
        log.stop_threads()
        rclpy.shutdown()
        print("log node stopped.")

if __name__ == '__main__':
    main()
