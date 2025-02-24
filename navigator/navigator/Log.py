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
        self.gps_subscription1 = self.create_subscription(
            Coordinates, 'gps1_cm', self.gps_callback1, 10)
        self.encoder_subscription = self.create_subscription(
            TwoInt, 'encoder', self.encoder_callback, 10)
        self.gps_subscription_combo = self.create_subscription(
            GpsData, 'gps/data', self.gps_callback_combo, 10)
        
        self.gps_cm_x1 = 0
        self.gps_cm_y1 = 0
        self.gps_cm_x2 = 0
        self.gps_cm_y2 = 0

        self.logging_thread = threading.Thread(target=self.log_positions)
        self.logging_thread.start()

        
    def gps_callback1(self, msg):
        self.gps_cm_x1 = msg.x
        self.gps_cm_y1 = msg.y

    def gps_callback2(self, msg):
        self.gps_cm_x2 = msg.x
        self.gps_cm_y2 = msg.y

    def encoder_callback(self, msg):
        with self.lock:
            self.encoder_left = msg.l
            self.encoder_right = msg.r
            self.encoder_data_updated = True


    def log_positions(self):
        try:
            with open("/home/hue/ros2_ws/src/position_log_total", 'w') as file:
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
