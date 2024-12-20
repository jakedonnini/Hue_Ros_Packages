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
        self.pantingToggle = 0
        self.shouldBePainting = False
        self.isPainting = 0
        self.sentToggle = False

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

        self.encoderX = 0 
        self.encoderY = 0
        self.encoderTheta = 0

        self.currentX = 0
        self.currentY = 0

        # constants (change if drive train changes)
        self.wheelR = 10.16
        self.wheelL = 64.77
        self.encoderTicks = 8192.0
        self.deltaT = 0.05 # 100ms time intervals

        # save old values to onlt send when it changes
        self.pwmr_value_old = 0
        self.pwml_value_old = 0
        # if we stop moveing keep increasing until gets unstuck
        self.destickAccum = 0
        self.pwmAvgAccum = 0

        # Threading for concurrent execution
        self.running = True
        self.lock = threading.Lock()

        # Start threads for publishing and processing
        self.publisher_thread = threading.Thread(target=self.run_publish_loop)
        self.processor_thread = threading.Thread(target=self.run_processing_loop)
        self.logging_thread = threading.Thread(target=self.log_positions)

        self.publisher_thread.start()
        self.processor_thread.start()
        self.logging_thread.start()
        

    def encoder_callback(self, msg):
        with self.lock:
            self.encoder_left = msg.l
            self.encoder_right = msg.r
            self.isPainting = msg.toggle
            self.encoder_data_updated = True  # Flag for new data

    def waypoint_callback(self, msg):
        with self.lock:
            self.waypointBuffer.append((msg.x, msg.y, msg.toggle))

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

            # Check for waypoints to process
            # self.get_logger().info(f"check way point {self.currentTWayPoint is None}, {len(self.waypointBuffer) > 0}")
            if self.currentTWayPoint is None and len(self.waypointBuffer) > 0:
                with self.lock:
                    x, y, t = self.waypointBuffer.pop(0)
                    self.currentTWayPoint = (x, y)
                    # keep track of spraying state
                    if t == 1:
                        # toggle every time t is 1
                        self.shouldBePainting = not self.shouldBePainting
                    self.sentToggle = False
                    self.pantingToggle = t
            time.sleep(0.05)

    def getEncoderPose(self):
        """call everytime serial data comes in"""
        vL = (6.2832*self.wheelR*self.encoder_left)/(self.encoderTicks*self.deltaT) #change with the number of ticks per encoder turn
        vR = (6.2832*self.wheelR*self.encoder_right)/(self.encoderTicks*self.deltaT)
        V = 0.5*(vR+vL)
        dV = vR - vL
        self.encoderX += self.deltaT*V*math.cos(self.encoderTheta)
        self.encoderY += self.deltaT*V*math.sin(self.encoderTheta)
        self.encoderTheta += self.deltaT*dV/self.wheelL
        # self.get_logger().info(
        #     f'X: {self.encoderX} Y: {self.encoderY} Theta {self.encoderTheta} Encoder {self.encoder_left} {self.encoder_right}'
        # )

    def getPosError(self):
        """Compute the distance and angular error to the next waypoint."""
        if self.currentTWayPoint is None:
            return 0, 0

        waypointX, waypointY = self.currentTWayPoint

        # Current positions based on GPS and encoder data (for now just encoder)
        self.currentX = self.encoderX
        self.currentY = self.encoderY
        self.currentTheta = self.encoderTheta

        dist2Go = math.sqrt(math.pow(self.currentX - waypointX, 2) + math.pow(self.currentY - waypointY, 2))
        if dist2Go < 1:  # threshold saying we hit the point
            self.get_logger().info(f'Hit ({waypointX}, {waypointY}) waypoint')
            self.currentTWayPoint = None

        desiredQ = math.atan2(waypointY - self.currentY, waypointX - self.currentX)
        thetaError = desiredQ - self.currentTheta

        if thetaError > math.pi:
            thetaError -= 2 * math.pi
        elif thetaError < -math.pi:
            thetaError += 2 * math.pi

        # self.get_logger().info(
        #     f'Theat error: {thetaError} dist2go {dist2Go} desiredQ {desiredQ} CQ {self.currentTheta}'
        # )

        return dist2Go, thetaError

    def constrain(self, val, min_val, max_val):
        return max(min_val, min(val, max_val))

    def adjust_pwm_values(self):
        """Adjust and publish PWMR and PWML values based on GPS data."""
        dist, thetaError = self.getPosError()

        KQ = 20*2  # turn speed
        pwmDel = KQ * thetaError
        pwmAvg = 80

        # adjust threshold basd on distance EXPERIMENTAL was 0.2
        # should be within 5% of the total distance
        threshold = dist * 0.05
        threshold = self.constrain(threshold, 0.1, math.pi/4)
        if abs(thetaError) > 0.25 or self.currentTWayPoint is None:
            pwmAvg = 0
            # set the ramp Accumulator back to 0 every time we stop
            self.pwmAvgAccum = 0
            pwmDel = self.constrain(pwmDel, -60/2, 60/2)

            if abs(pwmDel) <= 39 and self.currentTWayPoint is not None:
                # make this the lowest value the PWM can go. minimum speed
                pwmDel = 39 * math.copysign(1, thetaError)

            # if the robot starts to stop moving because it can't quite make it
            if self.encoder_left <= 20 and self.encoder_right <= 20 and self.currentTWayPoint is not None:
                # if we stop moveing keep increasing until gets unstuck
                pwmDel += self.destickAccum
                # include the sign of the error to turn in the right direction
                self.destickAccum += 1 * math.copysign(1, thetaError)
            else:
                # at 300 offset: 39 is lowest with 25 avg encoder count 
                self.destickAccum = 0
        else:
            if self.pwmAvgAccum < pwmAvg:
                self.pwmAvgAccum += 10
            pwmDel = self.constrain(pwmDel, -50/2, 50/2)
                
        # pwmDel = self.constrain(pwmDel, -70, 70)

        self.pwmr_value = self.pwmAvgAccum + pwmDel
        self.pwml_value = self.pwmAvgAccum - pwmDel

        pwm_msg = TwoInt()
        pwm_msg.r = int(self.pwmr_value)
        pwm_msg.l = int(self.pwml_value)

        if int(self.shouldBePainting) != self.isPainting:
            pwm_msg.toggle = 1
        else:
            pwm_msg.toggle = 0

        # if no way points make sure the sprayer is off
        if self.isPainting and self.currentTWayPoint is None and not self.shouldBePainting:
            pwm_msg.toggle = 1
            self.pwm_publisher.publish(pwm_msg)

        # if wheel still spinning send off again
        sureOff = (self.pwml_value == 0 and self.pwmr_value == 0) and (self.encoder_left != 0 or self.encoder_right != 0)

        # Publish the PWM values
        # only send if new values
        if (self.pwmr_value_old != self.pwmr_value) or (self.pwml_value_old != self.pwml_value) or sureOff:
            self.pwm_publisher.publish(pwm_msg)
            self.pwmr_value_old = self.pwmr_value
            self.pwml_value_old = self.pwml_value

        self.get_logger().info(
            f'PWM: {int(self.pwmr_value)}, {int(self.pwml_value)}, Waypoint: {self.currentTWayPoint}, Current Pos: {self.currentX}, {self.currentY} Theta error: {thetaError} dist2go {dist}'
        )

    def log_positions(self):
        """Continuously log GPS, encoder, and Kalman filter positions to a file."""
        try:
            # self.get_logger().info(f"Attempting to write log to: {os.path.abspath(self.log_file)}")
            with open("/home/hue/ros2_ws/position_log_No_GPS.txt", 'w') as file:
                file.write("Time,Encoder_X,Encoder_Y\n")  # Header
                while self.running:
                    with self.lock:
                        encoder_x = self.encoderX
                        encoder_y = self.encoderY
                    timestamp = time.time()
                    file.write(f"{timestamp},{encoder_x},{encoder_y}\n")
                    file.flush()  # Ensure data is written to the file
                    time.sleep(0.1)  # Adjust logging frequency as needed
        except Exception as e:
            self.get_logger().error(f"Failed to write log: {e}")
    
    def stop_threads(self):
        """Stop the threads gracefully."""
        self.running = False
        self.publisher_thread.join()
        self.processor_thread.join()
        self.logging_thread.join() 


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

