import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64  # Use Float64 for latitude/longitude
from custom_msg.msg import Coordinates
import serial
import time

# Configure the serial connection to the GPS module
ser = serial.Serial(
    # for linux
    port='/dev/ttyRobot3',  # Update this to your GPS module's serial port
    # for windows 
    # port='COM4',  # Change this to your GPS module's serial port
    baudrate=9600  # GPS modules commonly use 9600 or 115200 baud
)

class GPSPublisher(Node):
    def __init__(self):
        super().__init__('gps_publisher')

        # Create publishers for latitude and longitude
        self.coords_publisher = self.create_publisher(Coordinates, 'gps', 10)
        self.heading_publisher = self.create_publisher(Float64, 'gps/heading', 10)

        while True:
            self.read_gps_data()
        # Start reading data from GPS

    def read_gps_data(self):
        if ser.in_waiting > 0:

            # Read data from the GPS module
            gps_data = ser.readline().decode('utf-8').strip()

            # Only process lines starting with $G, which are NMEA sentences
            if gps_data.startswith('$G'):
                self.parse_nmea(gps_data)

    def parse_nmea(self, nmea_sentence):
        # Example: Parse a GGA sentence (for latitude, longitude, altitude)
        if nmea_sentence.startswith('$GPGGA') or nmea_sentence.startswith('$GNGGA'):
            parts = nmea_sentence.split(',')
            
            if len(parts) < 15:
                self.get_logger().warn("Incomplete GGA data")
                return
            
            # Extract relevant GPS data
            latitude = self.convert_to_decimal(parts[2], parts[3])  # Latitude and N/S direction
            longitude = self.convert_to_decimal(parts[4], parts[5])  # Longitude and E/W direction

            # Publish latitude and longitude
            if latitude is not None and longitude is not None:
                self.publish_gps_data(latitude, longitude)

        # Example: Parse a VTG sentence (for heading/direction in degrees)
        if nmea_sentence.startswith('$GPVTG'):
            parts = nmea_sentence.split(',')

            if len(parts) < 9:
                self.get_logger().warn("Incomplete VTG data")
                return
            
            heading = parts[1]  # Heading in degrees (true track made good)
            
            try:
                heading = float(heading)
                self.publish_heading(heading)
            except ValueError:
                self.get_logger().warn("Invalid heading data")

    def convert_to_decimal(self, value, direction):
        """Convert NMEA latitude/longitude to decimal degrees."""
        if not value or not direction:
            return None
        
        try:
            degrees = float(value[:2])
            minutes = float(value[2:])
            decimal = degrees + minutes / 60.0
            if direction in ['S', 'W']:
                decimal = -decimal
            return decimal
        except ValueError:
            return None

    def publish_gps_data(self, latitude, longitude):
        gps_msg = Coordinates()

        gps_msg.x = latitude
        gps_msg.y = longitude

        self.coords_publisher.publish(gps_msg)

        # self.get_logger().info(f'Published Latitude: {latitude}, Longitude: {longitude}')

    def publish_heading(self, heading):
        heading_msg = Float64()
        heading_msg.data = heading

        self.heading_publisher.publish(heading_msg)
        self.get_logger().info(f'Published Heading: {heading} degrees')



def main(args=None):
    rclpy.init(args=args)

    gps_publisher = GPSPublisher()

    try:
        rclpy.spin(gps_publisher)
    except KeyboardInterrupt:
        gps_publisher.destroy_node()
        ser.close()
        rclpy.shutdown()
        print("GPS data reading stopped.")


if __name__ == "__main__":
    main()
