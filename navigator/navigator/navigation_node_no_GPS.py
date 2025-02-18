import threading
import rclpy
from rclpy.node import Node
from custom_msg.msg import Coordinates
from custom_msg.msg import TwoInt
import time
import math
import sys


class GPSSubscriberPublisher(Node):
    def __init__(self, KP, KI, KD):
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
        self.prevWaypoint = 0, 0
        self.prevWaypointHolder = 0, 0 # used to avoid timing isuses when prev = current
        self.largeTurn = False # use small threshold for large turns

        # Create publishers for the PWMR and PWML topics
        self.pwm_publisher = self.create_publisher(TwoInt, 'PWM', 10)

        # Variables to store the most recent latitude and longitude values
        self.latitude = None
        self.longitude = None

        # Initial values for PWMR and PWML
        self.pwmr_value = 0
        self.pwml_value = 0
        self.dir = -1 # set to -1 to invert the forward direction
        
        # Initialize PID constants
        self.Kp = KP   # Proportional constant (oscillates at 40)
        self.Ki = KI  # Integral constant
        self.Kd = KD  # Derivative constant

        self.get_logger().info(f"Kp: {self.Kp} Ki: {self.Ki} Kd: {self.Kd}")

        # Initialize PID terms
        self.integral = 0
        self.integral_min = -100  # Prevent excessive negative accumulation
        self.integral_max = 100   # Prevent excessive positive accumulation
        self.previous_error = 0

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
        self.wheelR = 9.708 # 10.16  
        self.wheelL = 64.77
        self.encoderTicks = 8192.0 / 2
        self.deltaT = 0.025 # 100ms time intervals New at 1/4 the time
        self.errorScaler = 1 # 0.9233 at bruces # 0.963 at our

        # save old values to onlt send when it changes
        self.pwmr_value_old = 0
        self.pwml_value_old = 0
        # if we stop moveing keep increasing until gets unstuck
        self.destickAccum = 0

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
            time.sleep(self.deltaT/2)

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
                    # set the pervious waypint when it is reached
                    self.prevWaypoint = self.prevWaypointHolder
                    x, y, t = self.waypointBuffer.pop(0)
                    self.currentTWayPoint = (x, y)
                    # keep track of spraying state
                    if t == 1:
                        # toggle every time t is 1
                        self.shouldBePainting = not self.shouldBePainting
                    self.sentToggle = False
                    self.pantingToggle = t
            time.sleep(self.deltaT/2)

    def getEncoderPose(self):
        """call everytime serial data comes in"""
        vL = (6.2832*self.wheelR*self.encoder_left*self.errorScaler*self.dir)/(self.encoderTicks*self.deltaT) #change with the number of ticks per encoder turn
        vR = (6.2832*self.wheelR*self.encoder_right*self.errorScaler*self.dir)/(self.encoderTicks*self.deltaT)
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
            return 0, 0, 0

        waypointX, waypointY = self.currentTWayPoint

        # Current positions based on GPS and encoder data (for now just encoder)
        self.currentX = self.encoderX
        self.currentY = self.encoderY
        self.currentTheta = self.encoderTheta

        dist2Go = math.sqrt(math.pow(self.currentX - waypointX, 2) + math.pow(self.currentY - waypointY, 2))
        if dist2Go < 5:  # threshold saying we hit the point (was 1)
            self.get_logger().info(f'Hit ({waypointX}, {waypointY}) waypoint')
            self.prevWaypointHolder = waypointX, waypointY
            self.largeTurn = False
            self.currentTWayPoint = None

        desiredQ = math.atan2(waypointY - self.currentY, waypointX - self.currentX)
        thetaError = desiredQ - self.currentTheta

        if thetaError > math.pi:
            thetaError -= 2 * math.pi
        elif thetaError < -math.pi:
            thetaError += 2 * math.pi

        # find the distance to the nearest point on the line between waypoints
        distPoints = math.sqrt(math.pow(waypointX - self.prevWaypoint[0], 2) + math.pow(waypointY - self.prevWaypoint[1], 2))
        distToLine = ((waypointX-self.prevWaypoint[0])*(self.prevWaypoint[1]-self.currentY)-(self.prevWaypoint[0]-self.currentX)*(waypointY-self.prevWaypoint[1]))/distPoints

        # self.get_logger().info(
        #     f'Theat error: {thetaError} dist2go {dist2Go} desiredQ {desiredQ} CQ {self.currentTheta}'
        # )

        return dist2Go, thetaError, distToLine

    def constrain(self, val, min_val, max_val):
        return max(min_val, min(val, max_val))

    def sign(self, x):
        if x != 0:
            return x/abs(x)
        return 1

    def adjust_pwm_values(self):
        """Adjust and publish PWMR and PWML values based on GPS data."""
        dist, thetaError, distToLine = self.getPosError()

        # KQ = 20*2  # turn speed
        # pwmDel = KQ * thetaError
        pwmAvg = 20 # normally 60

        # PID calculations
        # Proportional term (P)
        P_term = self.Kp * distToLine
        
        # Integral term (I)
        self.integral += distToLine
        self.integral = max(self.integral_min, min(self.integral, self.integral_max))  # Clamping
        I_term = self.Ki * self.integral
        
        # Derivative term (D)
        # D_term = self.Kd * (distToLine - self.previous_error)
        
        # PID output
        pid_output = P_term + I_term  # + D_term
        
        # Update the previous error
        self.previous_error = distToLine

        # Adjust PWM values based on the PID output
        pwmDel = pid_output
        pwmDelTheta = self.Kd * thetaError

        # If the angle is within this threshold then move forward
        # otherwise stop an turn
        threshold = 0.20

        if abs(thetaError) > 0.2618: # greater than 30 deg
            self.largeTurn = True # we have found a big turn

        if self.largeTurn and abs(thetaError) > 0.02: # get 3 deg 
            pwmAvg = 0 # 0 point turn
        else:
            self.largeTurn = False
            
        
        if self.currentTWayPoint is None: # don't move if arnt any waypoints
            pwmAvg = 0
            pwmDel = 0
            pwmDelTheta = 0

        self.pwmr_value = pwmAvg + pwmDel + pwmDelTheta
        self.pwml_value = pwmAvg - pwmDel - pwmDelTheta

        max_pwm = 128
        min_pwm = -128
        
        self.pwmr_value = self.constrain(self.pwmr_value, min_pwm, max_pwm)
        self.pwml_value = self.constrain(self.pwml_value, min_pwm, max_pwm)

        # Anti-windup: Reduce integral accumulation if PWM is saturated
        if self.pwmr_value == max_pwm or self.pwmr_value == min_pwm:
            self.integral -= 0.1 * (self.pwmr_value - (pwmAvg + pwmDel))
    
        if self.pwml_value == max_pwm or self.pwml_value == min_pwm:
            self.integral -= 0.1 * (self.pwml_value - (pwmAvg - pwmDel))

        # remove dead zone between 39 and -39
        if self.pwmr_value > 0:
            self.pwmr_value += 39
        if self.pwmr_value < 0:
            self.pwmr_value -= 39

        if self.pwml_value > 0:
            self.pwml_value += 39
        if self.pwml_value < 0:
            self.pwml_value -= 39
            

        # self.get_logger().info(
        #     f'PID: Theta error: {round(thetaError, 2)} PID: {round(pid_output, 2)} P: {round(P_term, 2)} I: {round(I_term, 2)} D: {round(D_term, 2)} PWM_del {round(pwmDel, 2)}'
        # )

        pwm_msg = TwoInt()
        pwm_msg.r = int(self.pwmr_value)*self.dir # change direction 
        pwm_msg.l = int(self.pwml_value)*self.dir

        # if speed is below a threshold then we should stop painting to avoid pooling
        avgSpeed = (self.pwmr_value + self.pwml_value) / 2

        # if less than 3/4 of nominal speed then stop painting
        # when speed is reached the next block of code should turn sprayer back on
        notUpToSpeed = avgSpeed <= ((pwmAvg+39) * 0.9) and self.shouldBePainting
        
        # only send the toggle comands once
        paintingIncorrect = int(self.shouldBePainting) != self.isPainting

        if paintingIncorrect or notUpToSpeed:
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
        if (self.pwmr_value_old != self.pwmr_value) or (self.pwml_value_old != self.pwml_value) or sureOff or paintingIncorrect:
            self.pwm_publisher.publish(pwm_msg)
            self.pwmr_value_old = self.pwmr_value
            self.pwml_value_old = self.pwml_value

        self.get_logger().info(
            f'PWM: {int(self.pwmr_value)}, {int(self.pwml_value)}, {int(avgSpeed)}, Waypoint: {self.currentTWayPoint}, Current Pos: {round(self.currentX, 2)}, {round(self.currentY, 2)} TE: {round(thetaError, 2)} D {round(dist, 2)}, DL {round(distToLine, 2)}'
        )

    def log_positions(self):
        """Continuously log GPS, encoder, and Kalman filter positions to a file."""
        try:
            # self.get_logger().info(f"Attempting to write log to: {os.path.abspath(self.log_file)}")
            with open("/home/hue/ros2_ws/src/position_log_No_GPS.txt", 'w') as file:
                file.write("Time,Encoder_X,Encoder_Y,Theta,Paint\n")  # Header
                while self.running:
                    with self.lock:
                        encoder_x = self.encoderX
                        encoder_y = self.encoderY
                        theta = self.encoderTheta
                        paint = self.isPainting
                    timestamp = time.time()
                    file.write(f"{timestamp},{encoder_x},{encoder_y},{theta},{paint}\n")
                    file.flush()  # Ensure data is written to the file
                    time.sleep(self.deltaT)  # Adjust logging frequency as needed
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

    # default
    Kp = 0.2
    Ki = 0.0
    Kd = 35.0

    # Get filename and scaler from command-line arguments
    if len(sys.argv) < 4:
        print("Usage: ros2 run <package_name> <node_name> <Kp> <Ki> <Kd>")
        print("Using default values")
    else:
        Kp = float(sys.argv[1])
        Ki = float(sys.argv[2])
        Kd = float(sys.argv[3])

    gps_subscriber_publisher = GPSSubscriberPublisher(Kp, Ki, Kd)

    try:
        rclpy.spin(gps_subscriber_publisher)
    except KeyboardInterrupt:
        gps_subscriber_publisher.stop_threads()
        gps_subscriber_publisher.destroy_node()
        rclpy.shutdown()
        print("GPS subscriber/publisher node stopped.")


if __name__ == '__main__':
    main()

