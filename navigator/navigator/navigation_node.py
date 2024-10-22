import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Int32
import time


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

        # Create publishers for the PWMR and PWML topics
        self.pwmr_publisher = self.create_publisher(Int32, 'speed_R', 10)
        self.pwml_publisher = self.create_publisher(Int32, 'speed_L', 10)

        # Variables to store the most recent latitude and longitude values
        self.latitude = None
        self.longitude = None

        # Initial values for PWMR and PWML
        self.pwmr_value = 0
        self.pwml_value = 0

        while True:
            self.adjust_pwm_values()
            time.sleep(5)


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


    def print_gps_data(self):
        """Print both latitude and longitude if both are received."""
        if self.latitude is not None and self.longitude is not None:
            self.get_logger().info(
                f'Current GPS Position: Latitude = {self.latitude}, Longitude = {self.longitude}'
            )

    def adjust_pwm_values(self):
        """Adjust and publish PWMR and PWML values based on GPS data."""
        self.pwmr_value = 1
        self.pwml_value = 2

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
