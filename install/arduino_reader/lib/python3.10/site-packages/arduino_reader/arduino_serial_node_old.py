import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import serial
import time

class ArduinoSerialNode(Node):
    def __init__(self):
        super().__init__('arduino_serial_node')

        self.ser = serial.Serial(
            port='/dev/ttyACM1',  #  '/dev/ttyUSB0'.
            baudrate=9600,  # Make sure this matches the Arduino's baud rate.
            timeout=1
        )
        time.sleep(2)  # Allow some time for the serial connection to establish.

        # Create a publisher for the float data
        self.publisher_ = self.create_publisher(Float64, 'arduino/encoder_x', 10)
        self.publisher_ = self.create_publisher(Float64, 'arduino/encoder_y', 10)

        while True:
            self.read_serial()

    def publish_encoder_data(self, x, y):
        x_msg = Float64()
        y_msg = Float64()

        x_msg.data = x
        y_msg.data = y

        self.latitude_publisher.publish(x_msg)
        self.longitude_publisher.publish(y_msg)

        self.get_logger().info(f'Published X: {x}, Y: {y}')
        
    def read_serial(self):
        # TODO: publish the position x,y based on the encoder values
        if self.ser.in_waiting > 0:
            try:
                # Read the line from serial, decode it, and convert to float
                data = self.ser.readline().decode('utf-8').strip()

                # Convert data to float and publish
                float_data = float(data)

                #TODO: convert encoder values to coords
                
                self.publish_encoder_data(1, 1)

                self.get_logger().info(f'Published: {float_data}')

            except ValueError:
                self.get_logger().warn(f'Invalid data received: {data}')
            except Exception as e:
                self.get_logger().error(f'Error: {e}')

def main(args=None):
    rclpy.init(args=args)

    # Instantiate the node and spin it
    node = ArduinoSerialNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted, shutting down")
    finally:
        node.ser.close()  # Close the serial port on shutdown
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
