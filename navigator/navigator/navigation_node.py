import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Int32
from custom_msg.msg import Coordinates
import time
import math


class GPSSubscriberPublisher(Node):
    def __init__(self):
        super().__init__('navigation_node')

        # Create subscriptions to the latitude and longitude topics
        self.latitude_subscription = self.create_subscription(
            Float64, 
            'gps/latitude', 
            self.latitude_callback, 
            10
        )

        self.longitude_subscription = self.create_subscription(
            Float64, 
            'gps/longitude', 
            self.longitude_callback, 
            10
        )

        self.encoder_left_subscription = self.create_subscription(
            Int32, 
            'encoder_left', 
            self.encoder_left_callback, 
            10
        )

        self.encoder_right_subscription = self.create_subscription(
            Int32, 
            'encoder_right', 
            self.encoder_right_callback, 
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
        self.pwmr_publisher = self.create_publisher(Int32, 'PWM_R', 10)
        self.pwml_publisher = self.create_publisher(Int32, 'PWM_L', 10)

        # Variables to store the most recent latitude and longitude values
        self.latitude = None
        self.longitude = None

        # Initial values for PWMR and PWML
        self.pwmr_value = 0
        self.pwml_value = 0

        # encoder varibles
        self.encoder_left = 0
        self.encoder_right = 0

        # encoder posistioh varibles
        self.deltaT = 0.1 # 100ms time intervals
        self.encoderX = 0
        self.encoderY = 0
        self.encoderTheta = 0

        # current postion, encoder + gps
        self.currentX = 0
        self.currentY = 0
        self.currentTheta = 0


        # constants (change if drive train changes)
        self.wheelR = 3.45
        self.wheelL = 14.05

        while True:
            # wait for next coord
            if self.waypointBuffer:
                for i in range(10, 50):
                    self.pwmr_value = i
                    self.pwml_value = i
                    self.adjust_pwm_values()
                    time.sleep(0.5)


    def latitude_callback(self, msg):
        """Callback function to handle incoming latitude data."""
        self.latitude = msg.data
        self.get_logger().info(f'Received Latitude: {self.latitude}')
        self.print_gps_data()


    def longitude_callback(self, msg):
        """Callback function to handle incoming longitude data."""
        self.longitude = msg.data
        self.get_logger().info(f'Received Longitude: {self.longitude}')
        self.print_gps_data()

    def encoder_left_callback(self, msg):
        """Callback function to handle incoming encoder data."""
        self.encoder_left = msg.data
        self.get_logger().info(self.encoder_left)
        self.getEncoderPose()

    def encoder_right_callback(self, msg):
        """Callback function to handle incoming encoder data."""
        self.encoder_right = msg.data
        # only call function once

    def waypoint_callback(self, msg):
        # This function will be called every time a new message is received
        x = msg.x
        y = msg.y
        self.get_logger().info(f'waypoint: {x}, {y}')
        self.waypointBuffer.append((x, y))


    def print_gps_data(self):
        """Print both latitude and longitude if both are received."""
        if self.latitude is not None and self.longitude is not None:
            self.get_logger().info(
                f'Current GPS Position: Latitude = {self.latitude}, Longitude = {self.longitude}'
            )

    def getEncoderPose(self):
        """call everytime serial data comes in"""
        vL = (6.2832*self.wheelR*self.encoder_left)/(1440.0*self.deltaT) #change with the number of ticks per encoder turn
        vR = (6.2832*self.wheelR*self.encoder_right)/(1440.0*self.deltaT)
        V = 0.5*(vR+vL)
        dV = vR - vL
        self.encoderX += self.deltaT*V*math.cos(self.encoderTheta)
        self.encoderY += self.deltaT*V*math.sin(self.encoderTheta)
        self.encoderTheta += self.deltaT*dV/self.wheelL

    def getPosError(self):
        """Find the error to the next point"""

        waypointX, waypointY = self.currentTWayPoint

        # TODO Remove this for when gps gets mixed in
        self.currentX = self.encoderX
        self.currentY = self.encoderY
        self.currentTheta = self.encoderTheta

        dist2Go = math.sqrt(math.pow(self.currentX-waypointX/2,2)+math.pow(self.currentY-waypointY/2,2))
        if dist2Go < 5: # threshold saying we hit the point
            self.get_logger().info(f'Hit ({waypointX}, {waypointY}) waypoint')
            self.currentTWayPoint = self.waypointBuffer.pop()
            

        desiredQ = math.atan2(waypointY/2-self.currentY, waypointX/2-self.currentX)

        thetaError = desiredQ-self.currentTheta

        # correct turning
        if thetaError > math.pi:
            thetaError -= 2*math.pi
        elif thetaError < -math.pi:
            thetaError += 2*math.pi

        return dist2Go, thetaError
    
    def constrain(self, val, min_val, max_val):
        return max(min_val, min(val, max_val))

    def adjust_pwm_values(self):
        """Adjust and publish PWMR and PWML values based on GPS data."""

        dist, thetaError = self.getPosError()

        KQ = 20*4 # turn speed
        pwmDel = KQ*thetaError
        pwmAvg = 75

        if abs(thetaError) > 0.3:
            pwmAvg = 0

        pwmDel = self.constrain(pwmDel,-200,200)

        self.pwmr_value = pwmAvg-pwmDel
        self.pwml_value = pwmAvg+pwmDel

        # Publish the PWM values
        self.pwmr_publisher.publish(Int32(data=self.pwmr_value))
        self.pwml_publisher.publish(Int32(data=self.pwml_value))

        self.get_logger().info(
            f'Published PWMR: {self.pwmr_value}, PWML: {self.pwml_value}'
        )


def main(args=None):
    rclpy.init(args=args)

    gps_subscriber_publisher = GPSSubscriberPublisher()

    try:
        rclpy.spin(gps_subscriber_publisher)
    except KeyboardInterrupt:
        gps_subscriber_publisher.destroy_node()
        rclpy.shutdown()
        print("GPS subscriber/publisher node stopped.")


if __name__ == '__main__':
    main()
