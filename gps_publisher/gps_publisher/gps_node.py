import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64  # Use Float64 for latitude/longitude
import serial
import time

# Configure the serial connection to the GPS module
ser = serial.Serial(
    # for linux
    port='/dev/ttyACM1',  # Update this to your GPS module's serial port
    # for windows 
    # port='COM4',  # Change this to your GPS module's serial port
    baudrate=9600  # GPS modules commonly use 9600 or 115200 baud
)

class GPSPublisher(Node):
    def __init__(self):
        super().__init__('gps_publisher')

        # Create publishers for latitude and longitude
        self.latitude_publisher = self.create_publisher(Float64, 'gps/latitude', 10)
        self.longitude_publisher = self.create_publisher(Float64, 'gps/longitude', 10)
        while True:
            self.read_gps_data()
        # Start reading data from GPS
        # self.timer = self.create_timer(0.1, self.read_gps_data)  # Timer to read data every 1/10th of a second

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
        lat_msg = Float64()
        lon_msg = Float64()

        lat_msg.data = latitude
        lon_msg.data = longitude

        self.latitude_publisher.publish(lat_msg)
        self.longitude_publisher.publish(lon_msg)

        self.get_logger().info(f'Published Latitude: {latitude}, Longitude: {longitude}')


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
