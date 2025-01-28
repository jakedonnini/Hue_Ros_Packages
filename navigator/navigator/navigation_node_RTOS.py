import asyncio
import threading
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
import rclpy
from custom_msg.msg import Coordinates, TwoInt
import math
import numpy as np
import time

# super delayed and slow. Doesn't work at all

class GPSSubscriberPublisher(Node):
    def __init__(self):
        super().__init__('navigation_node')

        # Kalman Filter Matrices
        self.dt = 0.1  # time step
        self.x = np.array([[0], [0], [0]])  # initial state [x, y, theta]

        # Define initial state covariance matrix
        self.P = np.eye(3)

        # Define state transition matrix
        self.F = np.array([
            [1, 0, -self.dt * np.sin(self.x[2, 0])],
            [0, 1,  self.dt * np.cos(self.x[2, 0])],
            [0, 0, 1]
        ])

        # Control matrix
        self.B = np.array([
            [self.dt * np.cos(self.x[2, 0]), 0],
            [self.dt * np.sin(self.x[2, 0]), 0],
            [0, self.dt]
        ])

        # Process noise covariance
        self.Q = np.diag([0.1, 0.1, 0.2])

        # Measurement noise covariance (GPS noise)
        self.R = np.diag([0.11, 0.15])

        # Observation matrix
        self.H = np.array([
            [1, 0, 0],
            [0, 1, 0]
        ])

        # Queues for incoming data
        self.gps_queue = asyncio.Queue()
        self.encoder_queue = asyncio.Queue()
        self.waypoint_queue = asyncio.Queue()

        # Subscriptions
        self.create_subscription(Coordinates, 'gps', self.gps_callback, 10)
        self.create_subscription(TwoInt, 'encoder', self.encoder_callback, 10)
        self.create_subscription(Coordinates, 'coordinates', self.waypoint_callback, 10)

        # Publisher
        self.pwm_publisher = self.create_publisher(TwoInt, 'PWM', 10)

        # Variables
        self.latitude = None
        self.longitude = None
        self.encoder_left = 0
        self.encoder_right = 0
        self.waypointBuffer = []
        self.currentTWayPoint = None
        self.new_gps_data = False
        self.encoder_data_updated = False

        # PWM Variables
        self.pwmr_value = 0
        self.pwml_value = 0
        # save old values to onlt send when it changes
        self.pwmr_value_old = 0
        self.pwml_value_old = 0

        # Constants
        self.wheelR = 3.45
        self.wheelL = 14.05
        self.lat_to_cm = 111139.0 * 100
        self.origin_lat = None
        self.origin_lon = None
        self.lon_to_cm = 111139.0 * 100

        # RTOS Debug
        self.RTOS_Debug = False

    def gps_callback(self, msg):
        """Handle incoming GPS data."""
        self.gps_queue.put_nowait(msg)

    def encoder_callback(self, msg):
        """Handle incoming encoder data."""
        self.encoder_queue.put_nowait(msg)

    def waypoint_callback(self, msg):
        """Handle incoming waypoint data."""
        self.waypoint_queue.put_nowait(msg)

    def getEncoderPose(self):
        """Update pose based on encoder readings."""
        vL = (6.2832 * self.wheelR * self.encoder_left) / (1440.0 * self.dt)
        vR = (6.2832 * self.wheelR * self.encoder_right) / (1440.0 * self.dt)
        V = 0.5 * (vR + vL)
        dV = (vR - vL) / self.wheelL

        u = np.array([[V], [dV]])

        # Predict Step
        self.x = self.F @ self.x + self.B @ u
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update_kalman_with_gps(self):
        """Correct state estimate using GPS data."""
        if self.origin_lat is None or self.origin_lon is None:
            self.origin_lat = self.latitude
            self.origin_lon = self.longitude
            self.lon_to_cm = 111139.0 * 100 * np.cos(np.radians(self.origin_lat))

        # Convert GPS data to cm relative to origin
        delta_lat = self.latitude - self.origin_lat
        delta_lon = self.longitude - self.origin_lon
        x_gps_cm = delta_lon * self.lon_to_cm
        y_gps_cm = delta_lat * self.lat_to_cm

        # Measurement update
        z = np.array([[x_gps_cm], [y_gps_cm]])
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(3) - K @ self.H) @ self.P

    def adjust_pwm_values(self):
        """Adjust PWM values based on navigation logic."""
        dist, thetaError = self.getPosError()

        KQ = 20 * 4  # Turn speed constant
        pwmDel = KQ * thetaError
        pwmAvg = 80 if self.currentTWayPoint else 0

        if abs(thetaError) > 0.15 or self.currentTWayPoint is None:
            pwmAvg = 0
            pwmDel = self.constrain(pwmDel, -200, 200)

        self.pwmr_value = pwmAvg + pwmDel
        self.pwml_value = pwmAvg - pwmDel

        pwm_msg = TwoInt()
        pwm_msg.r = int(self.pwmr_value)
        pwm_msg.l = int(self.pwml_value)

        # if wheel still spinning send off again
        sureOff = (self.pwml_value == 0 and self.pwmr_value == 0) and (self.encoder_left != 0 or self.encoder_right != 0)
        
        # is it getting enocder values
        self.get_logger().info(f"L {self.encoder_left}, R {self.encoder_right}")

        # Publish the PWM values: only send if new values
        if (self.pwmr_value_old != self.pwmr_value) or (self.pwml_value_old != self.pwml_value) or sureOff:
            self.pwm_publisher.publish(pwm_msg)
            self.pwmr_value_old = self.pwmr_value
            self.pwml_value_old = self.pwml_value

    def getPosError(self):
        """Compute the distance and angular error to the next waypoint."""
        if self.currentTWayPoint is None:
            return 0, 0

        waypointX, waypointY = self.currentTWayPoint
        currentX = self.x[0, 0]
        currentY = self.x[1, 0]
        currentTheta = self.x[2, 0]

        dist2Go = math.sqrt((currentX - waypointX)**2 + (currentY - waypointY)**2)
        if dist2Go < 1:
            self.currentTWayPoint = None

        desiredQ = math.atan2(waypointY - currentY, waypointX - currentX)
        thetaError = desiredQ - currentTheta

        if thetaError > math.pi:
            thetaError -= 2 * math.pi
        elif thetaError < -math.pi:
            thetaError += 2 * math.pi

        return dist2Go, thetaError
    
    def constrain(self, val, min_val, max_val):
        """Constrain to lower and upper bounds"""
        return max(min_val, min(val, max_val))

    async def process_gps_data(self):
        """Continuously process GPS data as it arrives."""
        while rclpy.ok():
            msg = await self.gps_queue.get()
            self.latitude = msg.x
            self.longitude = msg.y
            self.get_logger().info(f"callback GPS: X {self.latitude}, Y {self.longitude}")
            await asyncio.sleep(0.1)
            # self.new_gps_data = True

    async def process_encoder_data(self):
        """Continuously process encoder data as it arrives."""
        while rclpy.ok():
            msg = await self.encoder_queue.get()
            self.encoder_left = msg.l
            self.encoder_right = msg.r
            self.get_logger().info(f"callback Encoder: L {self.encoder_left}, R {self.encoder_right}")
            await asyncio.sleep(0.1)
            # self.encoder_data_updated = True

    async def process_waypoint_data(self):
        """Continuously process waypoint data as it arrives."""
        while rclpy.ok():
            msg = await self.waypoint_queue.get()
            self.get_logger().info(f"callback Waypoint: X {msg.x}, Y {msg.y}")
            self.waypointBuffer.append((msg.x, msg.y))
            await asyncio.sleep(0.1)
    
    async def gps_task(self):
        while rclpy.ok():

            # timer to test task length
            start_time = time.perf_counter()
            
            if self.new_gps_data:
                self.update_kalman_with_gps()
                self.new_gps_data = False

            end_time = time.perf_counter()
            elapsed_time = end_time - start_time
            if self.RTOS_Debug:
                self.get_logger().info(f"GPS Task Time: {elapsed_time:.6f} seconds")

            await asyncio.sleep(0.1)

    async def encoder_task(self):
        while rclpy.ok():

            # timer to test task length
            start_time = time.perf_counter()

            if self.encoder_data_updated:
                self.getEncoderPose()
                self.encoder_data_updated = False

            end_time = time.perf_counter()
            elapsed_time = end_time - start_time
            if self.RTOS_Debug:
                self.get_logger().info(f"Encoder Task Time: {elapsed_time:.6f} seconds")

            await asyncio.sleep(0.1)

    async def navigation_task(self):
        while rclpy.ok():

            # timer to test task length
            start_time = time.perf_counter()

            self.adjust_pwm_values()

            # check for next waypoints
            if self.currentTWayPoint is None and len(self.waypointBuffer) > 0:
                self.currentTWayPoint = self.waypointBuffer.pop(0)

            end_time = time.perf_counter()
            elapsed_time = end_time - start_time
            if self.RTOS_Debug:
                self.get_logger().info(f"Navigation Task Time: {elapsed_time:.6f} seconds")

            await asyncio.sleep(0.05)

    async def main_loop(self):
        tasks = [
            self.process_gps_data(),
            self.process_encoder_data(),
            self.navigation_task()
        ]
        await asyncio.gather(*tasks)

def ros_spin_thread(node, executor):
    """Function to spin the rclpy executor in a separate thread."""
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = GPSSubscriberPublisher()

    # Start rclpy executor in a separate thread
    executor = SingleThreadedExecutor()
    spin_thread = threading.Thread(target=ros_spin_thread, args=(node, executor), daemon=True)
    spin_thread.start()

    # Start asyncio event loop for RTOS tasks
    loop = asyncio.get_event_loop()
    try:
        loop.run_until_complete(node.main_loop())
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down navigation node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join()


if __name__ == '__main__':
    main()
