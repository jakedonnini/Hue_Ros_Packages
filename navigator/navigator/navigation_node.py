import threading
import rclpy
from rclpy.node import Node
from custom_msg.msg import Coordinates
from custom_msg.msg import TwoInt
from custom_msg.msg import GpsData
import time
import math
import numpy as np


class GPSSubscriberPublisher(Node):
    def __init__(self):
        super().__init__('navigation_node')

        # Kalman Filter Matrices
        self.dt = 0.05  # time step
        self.x = np.array([[0], [0], [0]])  # initial state [x, y, theta]
        
        # Define initial state covariance matrix
        self.P = np.eye(3)
        
        # Define state transition matrix
        # this should be I unless external forces act on the bot
        # self.F = np.array([
        #     [1, 0, -self.dt * np.sin(self.x[2, 0])],
        #     [0, 1,  self.dt * np.cos(self.x[2, 0])],
        #     [0, 0, 1]
        # ])
        self.F = np.eye(3)
        
        # Control matrix
        self.B = np.array([
            [self.dt * np.cos(self.x[2, 0]), 0],
            [self.dt * np.sin(self.x[2, 0]), 0],
            [0, self.dt]
        ])
        
        # Process noise covariance
        self.Q = np.diag([0.4, 0.4, 0.3])
        
        # Measurement noise covariance (GPS noise) #0.11, 0.15
        self.R = np.diag([0.11, 0.15])
        
        # Observation matrix
        self.H = np.array([
            [1, 0, 0],
            [0, 1, 0]
        ])

        # read from the rotation matrix
        save_path = "/home/hue/ros2_ws/src/navigator/navigator/transformation_matrix.txt"
        self.Rot_Matrix, self.startingAngle = self.read_transformation_matrix(save_path)

        self.gps_subscription = self.create_subscription(
            GpsData, 
            'gps/data', 
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
        self.pantingToggle = 0
        self.shouldBePainting = False
        self.isPainting = 0
        self.sentToggle = False
        self.prevWaypoint = 0, 0
        self.prevWaypointHolder = 0, 0 # used to avoid timing isuses when prev = current
        self.largeTurn = False # use small threshold for large turns

        # Create publishers for the PWMR and PWML topics
        self.pwm_publisher = self.create_publisher(TwoInt, 'PWM', 10)

        # Variables to store the most recent GPS values (latitude and longitude from both GPS, center point and theta calculated)
        self.latitude = None
        self.longitude = None

        # not sure what init value
        self.gpsTheta = self.startingAngle

        # Initial values for PWMR and PWML
        self.pwmr_value = 0
        self.pwml_value = 0
        self.dir = -1 # set to -1 to invert the forward direction

        # Initialize PID constants (may need more tuning)
        self.Kp = 35.0   # Proportional constant (oscillates at 40)
        self.Ki = 0.2  # Integral constant
        self.Kd = 0.1  # Derivative constant

        # Initialize PID terms
        self.integral = 0
        self.integral_min = -100  # Prevent excessive negative accumulation
        self.integral_max = 100   # Prevent excessive positive accumulation
        self.previous_error = 0

        # encoder varibles
        self.encoder_left = 0
        self.encoder_right = 0
        self.encoder_data_updated = 0

        self.currentX = 0
        self.currentY = 0

        # for checking, not used in actual calcs
        self.encoderX = 0 
        self.encoderY = 0
        # start at the starting angle we left off at from the sync process
        # we need to do this or else the rot matrix will be meaningless as the encoders reset
        self.encoderTheta = self.startingAngle
        self.x[2, 0] = self.startingAngle

        # constants (change if drive train changes)
        self.wheelR = 9.708
        self.wheelL = 64.77
        self.encoderTicks = 8192.0 / 2 # only counts half the ticks of encoder
        self.errorScaler = 1

        # save old values to onlt send when it changes
        self.pwmr_value_old = 0
        self.pwml_value_old = 0
        # if we stop moveing keep increasing until gets unstuck
        self.destickAccum = 0
        self.pwmAvgAccum = 0

        # GPS
        # Initial GPS origin coordinates for conversion to local coordinates
        self.origin_lat = None
        self.origin_lon = None
        self.new_gps_data = False
        self.x_gps_cm = 0
        self.y_gps_cm = 0

         # Conversion factor for GPS to meters (approximately 111,139 meters per degree latitude)
        self.lat_to_cm = 111139.0 * 100 # 100 for cm
        self.lon_to_cm = 111139.0 * 100 * np.cos(np.radians(self.origin_lat or 0))  # Will be updated once origin_lat is set

        # Threading for concurrent execution
        self.running = True
        self.lock = threading.Lock()

        # Start threads for publishing and processing
        self.publisher_thread = threading.Thread(target=self.run_publish_loop)
        self.processor_thread = threading.Thread(target=self.run_processing_loop)
        self.logging_thread = threading.Thread(target=self.log_positions) # for logging the postions

        self.publisher_thread.start()
        self.processor_thread.start()
        self.logging_thread.start()

        
    def gps_callback(self, msg):
        with self.lock:
            self.latitude = msg.x
            self.longitude = msg.y
            self.gpsTheta = msg.angle
            self.new_gps_data = True

    def encoder_callback(self, msg):
        with self.lock:
            self.encoder_left = msg.l
            self.encoder_right = msg.r
            self.isPainting = msg.toggle
            self.encoder_data_updated = True  # Flag for new data

    def waypoint_callback(self, msg):
        with self.lock:
            self.waypointBuffer.append((msg.x, msg.y, msg.toggle))
            # print(self.waypointBuffer)

    def run_publish_loop(self):
        """Thread to continuously publish PWM values."""
        while self.running:
            self.adjust_pwm_values()
            time.sleep(self.dt)

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
                    x, y, t = self.waypointBuffer.pop(0)
                    self.currentTWayPoint = (x, y)
                    # keep track of spraying state
                    if t == 1:
                        # toggle every time t is 1
                        self.shouldBePainting = not self.shouldBePainting
                    self.sentToggle = False
                    self.pantingToggle = t

                    if not self.firstWayPointSent:
                        # when the first waypoint is sent reset the origin so we always start at 0 0
                        self.origin_lat = self.latitude
                        self.origin_lon = self.longitude
                        self.lon_to_cm = 111139.0 * 100 * np.cos(np.radians(self.origin_lat))
                        self.firstWayPointSent = True
            time.sleep(self.dt/2)

    def getEncoderPose(self):
        """call everytime serial data comes in"""
        
        vL = (6.2832*self.wheelR*self.encoder_left*self.errorScaler*self.dir)/(self.encoderTicks*self.dt) #change with the number of ticks per encoder turn
        vR = (6.2832*self.wheelR*self.encoder_right*self.errorScaler*self.dir)/(self.encoderTicks*self.dt)
        V = 0.5*(vR+vL)
        dV = (vR - vL) / self.wheelL

        # compute the encoder pos without the gps for refrance
        self.encoderX += self.dt*V*math.cos(self.encoderTheta)
        self.encoderY += self.dt*V*math.sin(self.encoderTheta)
        self.encoderTheta += self.dt*dV

        # Apply rotation matrix to align encoder position with GPS
        rotated_pos = self.Rot_Matrix @ np.array([[self.encoderX], [self.encoderY]])
        self.encoderX, self.encoderY = rotated_pos.flatten()

        # Create extended 3x3 rotation matrix (includes theta)
        Rot_Extended = np.eye(3)
        Rot_Extended[:2, :2] = self.Rot_Matrix  # Embed the 2x2 rotation into the 3x3 matrix

        # update F state transition matrix
        # self.F = np.array([
        #     [1, 0, -V * self.dt * np.sin(self.x[2, 0])],
        #     [0, 1,  V * self.dt * np.cos(self.x[2, 0])],
        #     [0, 0, 1]
        # ])
        
        # update B Control matrix
        self.B = np.array([
            [self.dt * np.cos(self.x[2, 0]), 0],
            [self.dt * np.sin(self.x[2, 0]), 0],
            [0, self.dt]
        ])

        u = np.array([[V], [dV]])

        stateUpdate = self.B @ u

        # mult the u vector and B matrix by the rotation to get into the GPS frame
        correctedStateUpdate = Rot_Extended @ stateUpdate

        # Predict Step
        self.x = self.F @ self.x + correctedStateUpdate
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update_kalman_with_gps(self):
        """Correct state estimate using GPS data."""
        if self.origin_lat is None or self.origin_lon is None:
            # set the orign once we start getting data
            self.origin_lat = self.latitude
            self.origin_lon = self.longitude
            self.lon_to_cm = 111139.0 * 100 * np.cos(np.radians(self.origin_lat))

        # Convert GPS data to cm relative to origin
        delta_lat = self.latitude - self.origin_lat
        delta_lon = self.longitude - self.origin_lon
        self.x_gps_cm = delta_lon * self.lon_to_cm
        self.y_gps_cm = delta_lat * self.lat_to_cm

        # Measurement update
        z = np.array([[self.x_gps_cm], [self.y_gps_cm]])
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

        # self.get_logger().info(
        #     f'\rX:\n {self.x}'
        # )

        dist2Go = math.sqrt(math.pow(self.currentX - waypointX, 2) + math.pow(self.currentY - waypointY, 2))
        if dist2Go < 5:  # threshold saying we hit the point
            # self.get_logger().info(f'Hit ({waypointX}, {waypointY}) waypoint')
            self.prevWaypointsHolder = waypointX, waypointY
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
        #     f'[Theta error: {round(thetaError, 2)} desiredQ {round(desiredQ, 2)} CQ {round(self.currentTheta, 2)}'
        # )

        return dist2Go, thetaError, distToLine

    def constrain(self, val, min_val, max_val):
        return max(min_val, min(val, max_val))

    def adjust_pwm_values(self):
        """Adjust and publish PWMR and PWML values based on GPS data."""
        dist, thetaError = self.getPosError()

        # for painting 40 seems to be pretty good for paint spread
        pwmAvg = 40 # normally 60 with dead zone

        # PID calculations
        # Proportional term (P)
        P_term = self.Kp * distToLine
        
        # Integral term (I)
        self.integral += distToLine
        self.integral = max(self.integral_min, min(self.integral, self.integral_max))  # Clamping
        I_term = self.Ki * self.integral
        
        # Derivative term (D)
        # D_term = self.Kd * (thetaError - self.previous_error)
        
        # PID output
        pid_output = P_term + I_term # + D_term
        
        # Update the previous error
        self.previous_error = distToLine

        # Adjust PWM values based on the PID output
        pwmDel = pid_output
        pwmDelTheta = self.Kd * thetaError

        # If the angle is within this threshold then move forward
        # otherwise stop an turn
        threshold = 0.20

        if abs(thetaError) > 0.2618:
            self.largeTurn = True

        if self.largeTurn and abs(thetaError) > 0.03:
            pwmAvg = 0
        else:
            self.largeTurn = False

        if len(self.waypointBuffer) == 0 and self.currentTWayPoint is None:
            pwmAvg = 0
            pwmDel = 0
            pwmDelTheta = 0

        '''
        if abs(thetaError) > threshold:
            pwmAvg = 0
        elif self.currentTWayPoint is None:
            pwmAvg = 0
            pwmDel = 0
        else: 
            # when in thresh shouldn't move alot, half the intergrator
            I_term = I_term / 2
        '''

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
        pwm_msg.r = int(self.pwmr_value) * self.dir # dir will flip the direction of movement
        pwm_msg.l = int(self.pwml_value) * self.dir

        # if speed is below a threshold then we should stop painting to avoid pooling
        avgSpeed = (self.pwmr_value + self.pwml_value) / 2

        # if less than 3/4 of nominal speed then stop painting
        # when speed is reached the next block of code should turn sprayer back on
        notUpToSpeed = avgSpeed <= (pwmAvg * 0.75) and self.shouldBePainting
        
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

        # self.get_logger().info(
        #     f'PWM: {int(self.pwmr_value)}, {int(self.pwml_value)}, Waypoint: {self.currentTWayPoint}, Current Pos: {self.currentX}, {self.currentY} Theta error: {thetaError} dist2go {dist}'
        # )

        # for Kalman filiter testing
        self.get_logger().info(
            f'\rGPS: {round(self.x_gps_cm, 2)}, {round(self.y_gps_cm, 2)},  [ENCODER] X: {round(self.encoderX, 2)} Y: {round(self.encoderY, 2)} Q: {round(self.encoderTheta, 2)}, Current Pos: {round(self.currentX, 2)}, {round(self.currentY, 2)} Theta error: {round(thetaError, 2)} dist2go {round(dist, 2)} Waypoint: {self.currentTWayPoint}, PWM R: {int(self.pwmr_value)} L: {int(self.pwml_value)}, PWM AVG {int(avgSpeed)}'
        )

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
        """Log GPS, encoder, and Kalman filter positions to a file."""
        try:
            with open("/home/hue/ros2_ws/position_log_teleop.txt", 'w') as file:
                file.write("Time,GPS_X,GPS_Y,Encoder_X,Encoder_Y,Kalman_X,Kalman_Y,Theta\n")
                while self.running:
                    with self.lock:
                        gps_x = self.x_gps_cm
                        gps_y = self.y_gps_cm
                        encoder_x = self.encoderX
                        encoder_y = self.encoderY
                        kalman_x = self.x[0, 0]
                        kalman_y = self.x[1, 0]
                        theta = self.encoderTheta
                    timestamp = time.time()
                    file.write(f"{timestamp},{gps_x},{gps_y},{encoder_x},{encoder_y},{kalman_x},{kalman_y},{theta}\n")
                    file.flush()
                    time.sleep(self.dt)
        except Exception as e:
            self.get_logger().error(f"Failed to write log: {e}")

    def stop_threads(self):
        """Stop the threads gracefully."""
        self.running = False
        self.publisher_thread.join()
        self.processor_thread.join()
        self.logging_thread.join()

        # Ensure the motors turn off on close (doesn't work)
        pwm_msg = TwoInt()
        pwm_msg.r = 0
        pwm_msg.l = 0
        self.pwm_publisher.publish(pwm_msg)


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

