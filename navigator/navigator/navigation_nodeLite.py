import threading
import rclpy
from rclpy.node import Node
from custom_msg.msg import Coordinates, GpsData, TwoInt
import time
import math
import sys
import numpy as np


class GPSSubscriberPublisher(Node):
    def __init__(self):
        super().__init__('navigation_node')
        """This node is ment to be used with the new decentralized system"""

        self.waypoint_subscription = self.create_subscription(
            Coordinates, 'coordinates', self.waypoint_callback, 10)
        self.Kalman = self.create_subscription(
            GpsData, 'kalman/data', self.kalman, 10)
        self.DR_subscription = self.create_subscription(
            GpsData, 'deadReckoning/pose', self.deadReck_callback, 10)
        self.DR_subscription_vel = self.create_subscription(
            Coordinates, 'deadReckoning/vel', self.deadReck_callback_t, 10)

        self.waypointBuffer = []
        self.currentTWayPoint = None
        self.pantingToggle = 0
        self.shouldBePainting = False
        self.isPainting = 0
        self.sentToggle = False
        self.prevWaypoint = 0, 0
        self.prevWaypointHolder = 0, 0 # used to avoid timing isuses when prev = current
        self.largeTurn = False # use small threshold for large turns
        self.deltaT = 0.05 # 20Hz

        # Create publishers for the PWMR and PWML topics
        self.pwm_publisher = self.create_publisher(TwoInt, 'PWM', 10)

        # Initial values for PWMR and PWML
        self.pwmr_value = 0
        self.pwml_value = 0
        self.dir = 1 # set to -1 to invert the forward direction
        
        # Initialize PID constants
        self.Kp = 0.2   # Proportional constant
        self.Ki = 0.1  # Integral constant
        self.Kd = 25.0  # Derivative constant

        self.get_logger().info(f"Kp: {self.Kp} Ki: {self.Ki} Kd: {self.Kd}")

        # Initialize PID terms
        self.integral = 0
        self.integral_min = -50  # Prevent excessive negative accumulation
        self.integral_max = 50   # Prevent excessive positive accumulation
        self.previous_error = 0

        # self.usingGPS = UseGPS
        self.usingGPS = 1 # 1 for GPS, 0 for DR

        self.currentX = 0
        self.currentY = 0
        self.currentTheta = 0

        self.DR_x = 0
        self.DR_y = 0
        self.DR_angle = 0

        self.kalman_x = 0
        self.kalman_y = 0
        self.kalman_angle = 0

        # save old values to onlt send when it changes
        self.pwmr_value_old = 0
        self.pwml_value_old = 0
        # if we stop moveing keep increasing until gets unstuck
        self.destickAccum = 0
        self.pos_data_updated = False

        # Threading for concurrent execution
        self.running = True
        self.lock = threading.Lock()

        # Start threads for publishing and processing
        # self.publisher_thread = threading.Thread(target=self.run_publish_loop)
        self.processor_thread = threading.Thread(target=self.run_processing_loop)

        # self.publisher_thread.start()
        self.processor_thread.start()
        

    def waypoint_callback(self, msg):
        with self.lock:
            self.waypointBuffer.append((msg.x, msg.y, msg.toggle))

    def deadReck_callback(self, msg):
        with self.lock:
            self.DR_x = msg.x
            self.DR_y = msg.y
            self.DR_angle = msg.angle
            if self.usingGPS == 0:
                self.currentX = self.DR_x
                self.currentY = self.DR_y
                self.currentTheta = self.DR_angle
                self.pos_data_updated = True

        if self.usingGPS == 0:
            self.adjust_pwm_values()

    def kalman(self, msg):
        with self.lock:
            self.kalman_x = msg.x
            self.kalman_y = msg.y
            self.kalman_angle = msg.angle
            if self.usingGPS == 1:
                self.currentX = self.kalman_x
                self.currentY = self.kalman_y
                self.currentTheta = self.kalman_angle
                self.pos_data_updated = True

        if self.usingGPS == 1:
            self.adjust_pwm_values() # adjuest pwm values imediately

    def deadReck_callback_t(self, msg):
        with self.lock:
            self.isPainting = msg.toggle

    def run_processing_loop(self):
        """Process waypoints and update encoder position as new data is available."""
        while self.running:
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
            time.sleep(self.deltaT/4)

    def getPosError(self):
        """Compute the distance and angular error to the next waypoint."""
        if self.currentTWayPoint is None:
            return 0, 0, 0

        waypointX, waypointY = self.currentTWayPoint

        dist2Go = math.sqrt(math.pow(self.currentX - waypointX, 2) + math.pow(self.currentY - waypointY, 2))
        if dist2Go < 5:  # threshold saying we hit the point (was 1)
            self.get_logger().info(f'\n\n\n\n --------------------------------------\n Hit ({waypointX}, {waypointY}) waypoint \n --------------------------------------\n\n\n\n')
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

    def adjust_pwm_values(self):
        """Adjust and publish PWMR and PWML values based on GPS data."""
        dist, thetaError, distToLine = self.getPosError()

        if self.usingGPS == 0:
            thetaError = thetaError * -self.dir

        # KQ = 20*2  # turn speed
        # pwmDel = KQ * thetaError
        pwmAvg = 20 # normally 60

        # PID calculations
        # Proportional term (P)
        P_term = self.Kp * distToLine
        
        # Integral term (I) for theta error
        self.integral += thetaError
        self.integral = max(self.integral_min, min(self.integral, self.integral_max))  # Clamping
        I_term = self.Ki * self.integral
        
        # Derivative term (D)
        # D_term = self.Kd * (distToLine - self.previous_error)
        
        # PID output
        pid_output = P_term # + I_term  # + D_term
        
        # Update the previous error
        self.previous_error = distToLine

        # Adjust PWM values based on the PID output
        pwmDel = pid_output
        pwmDelTheta = self.Kd * thetaError + I_term

        pwmDelTheta = self.constrain(pwmDelTheta, -20, 20)

        # If the angle is within this threshold then move forward
        # otherwise stop an turn
        largeTurnThreshold = ((np.pi/180) / 2) * 45 # 45 deg converted to rad, /2 for abs value
        fineThreshold = ((np.pi/180) / 2) * 5 # 5 deg converted to rad, /2 for abs value for fine turning

        if abs(thetaError) > largeTurnThreshold: # greater than 45 deg
            self.largeTurn = True # we have found a big turn

        if self.largeTurn and abs(thetaError) > fineThreshold: # get 5 deg 
            pwmAvg = 0 # 0 point turn
            # pwmDel = 0 # prevents rounded corners
        else:
            self.largeTurn = False
            
        
        if len(self.waypointBuffer) == 0 and self.currentTWayPoint is None: # don't move if arnt any waypoints
            pwmAvg = 0
            pwmDel = 0
            pwmDelTheta = 0

        # slow down as we get closer to the point
        constrainedDist = self.constrain(dist/10, 0, 1) # at 10cm away we start to slow down
        speed = pwmAvg * constrainedDist

        self.pwmr_value = speed + pwmDel + pwmDelTheta
        self.pwml_value = speed - pwmDel - pwmDelTheta

        max_pwm = 100
        min_pwm = -100
        
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
        notUpToSpeed = avgSpeed <= ((pwmAvg+39) * 0.6) and self.shouldBePainting
        
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
        sureOff = (self.pwml_value == 0 and self.pwmr_value == 0) # TODO: FIND A WAY TO DO THIS and (self.encoder_left != 0 or self.encoder_right != 0)

        # Publish the PWM values
        # only send if new values
        if (self.pwmr_value_old != self.pwmr_value) or (self.pwml_value_old != self.pwml_value) or sureOff or paintingIncorrect:
            self.pwm_publisher.publish(pwm_msg)
            self.pwmr_value_old = self.pwmr_value
            self.pwml_value_old = self.pwml_value

        self.get_logger().info(
            f'PWM: {int(self.pwmr_value)}, {int(self.pwml_value)}, {int(avgSpeed)}, Waypoint: {self.currentTWayPoint}, Current Pos: {round(self.currentX, 2)}, {round(self.currentY, 2)} TE: {round(thetaError, 2)} D {round(dist, 2)}, DL {round(distToLine, 2)}, IP: {self.isPainting} SP: {self.shouldBePainting}'
        )

    def stop_threads(self):
        """Stop the threads gracefully."""
        self.running = False
        # self.publisher_thread.join()
        self.processor_thread.join()

def main(args=None):
    rclpy.init(args=args)

    # # Get filename and scaler from command-line arguments
    # if len(sys.argv) < 2:
    #     print("Usage: ros2 run <package_name> <node_name> <UseGPS(0 or 1)>")
    #     print("Using default values")
    # else:
    #     UseGPS = int(sys.argv[1])
    #     if UseGPS == 1:
    #         print("Using GPS")
    #     else:
    #         print("Not using GPS")

    gps_subscriber_publisher = GPSSubscriberPublisher() # UseGPS

    try:
        rclpy.spin(gps_subscriber_publisher)
    except KeyboardInterrupt:
        gps_subscriber_publisher.stop_threads()
        rclpy.shutdown()
        print("GPS subscriber/publisher node stopped.")


if __name__ == '__main__':
    main()

