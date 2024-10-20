import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class GPSSubscriber(Node):
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

        # Variables to store the most recent latitude and longitude values
        self.latitude = None
        self.longitude = None

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


def main(args=None):
    rclpy.init(args=args)

    gps_subscriber = GPSSubscriber()

    try:
        rclpy.spin(gps_subscriber)
    except KeyboardInterrupt:
        gps_subscriber.destroy_node()
        rclpy.shutdown()
        print("GPS subscriber node stopped.")


if __name__ == '__main__':
    main()
