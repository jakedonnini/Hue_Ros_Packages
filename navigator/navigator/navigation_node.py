import threading
import rclpy
from rclpy.node import Node
from custom_msg.msg import Coordinates
from custom_msg.msg import TwoInt
import time
import math


class GPSSubscriberPublisher(Node):
    def __init__(self):
        super().__init__('navigation_node')

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

        # Threading for concurrent execution
        self.running = True
        self.lock = threading.Lock()

        # Start threads for publishing and processing
        self.publisher_thread = threading.Thread(target=self.run_publish_loop)
        self.processor_thread = threading.Thread(target=self.run_processing_loop)

        self.publisher_thread.start()
        self.processor_thread.start()

    def gps_callback(self, msg):
        self.lock.acquire()
        self.latitude = msg.x
        self.longitude = msg.y
        self.lock.release()

    def encoder_callback(self, msg):
        self.lock.acquire()
        self.encoder_left = msg.l
        self.encoder_right = msg.r
        self.lock.release()

    def waypoint_callback(self, msg):
        self.lock.acquire()
        self.waypointBuffer.append((msg.x, msg.y))
        self.lock.release()

    def run_publish_loop(self):
        """Thread to continuously publish PWM values."""
        while self.running:
            self.adjust_pwm_values()
            time.sleep(0.1)

    def run_processing_loop(self):
        """Thread to handle waypoint processing and error correction."""
        while self.running:
            if self.currentTWayPoint is None and len(self.waypointBuffer) > 0:
                self.lock.acquire()
                self.currentTWayPoint = self.waypointBuffer.pop(0)
                self.lock.release()
            time.sleep(0.1)

    def getPosError(self):
        """Compute the distance and angular error to the next waypoint."""
        if self.currentTWayPoint is None:
            return 0, 0

        waypointX, waypointY = self.currentTWayPoint

        # Current positions based on GPS and encoder data (for now just encoder)
        self.currentX = self.encoder_left
        self.currentY = self.encoder_right
        self.currentTheta = 0  # placeholder, should be calculated based on heading

        dist2Go = math.sqrt(math.pow(self.currentX - waypointX, 2) + math.pow(self.currentY - waypointY, 2))
        if dist2Go < 5:  # threshold saying we hit the point
            self.get_logger().info(f'Hit ({waypointX}, {waypointY}) waypoint')
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

        KQ = 20  # turn speed
        pwmDel = KQ * thetaError
        pwmAvg = 75

        if abs(thetaError) > 0.3:
            pwmAvg = 0

        pwmDel = self.constrain(pwmDel, -200, 200)

        self.pwmr_value = pwmAvg - pwmDel
        self.pwml_value = pwmAvg + pwmDel

        pwm_msg = TwoInt()
        pwm_msg.r = int(self.pwmr_value)
        pwm_msg.l = int(self.pwml_value)

        # Publish the PWM values
        self.pwm_publisher.publish(pwm_msg)

        self.get_logger().info(
            f'Published PWMR: {self.pwmr_value}, PWML: {self.pwml_value}'
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

