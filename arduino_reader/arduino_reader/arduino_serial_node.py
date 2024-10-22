import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import serial
import time
import threading

class ArduinoSerialNode(Node):
    def __init__(self):
        super().__init__('arduino_serial_node')

        # Initialize serial connection to Arduino
        self.ser = serial.Serial('/dev/ttyACM1', 115200, timeout=1)
        time.sleep(2)  # Wait for serial connection to initialize

        # Publishers for left and right encoder values
        self.encoder_left_pub = self.create_publisher(Int32, 'encoder_left', 10)
        self.encoder_right_pub = self.create_publisher(Int32, 'encoder_right', 10)

        # Subscribers for speed values (PWM inputs)
        self.pwmr_sub = self.create_subscription(Int32, 'speed_R', self.pwmr_callback, 10)
        self.pwml_sub = self.create_subscription(Int32, 'speed_L', self.pwml_callback, 10)

        # Variables to store the current speed values (PWM)
        self.pwmr_value = 0
        self.pwml_value = 0

        # Flag to control reading from serial
        self.reading_serial = True

        # Thread for reading encoder values from Arduino
        self.read_thread = threading.Thread(target=self.read_encoder_values)
        self.read_thread.start()

    def read_encoder_values(self):
        """Continuously reads encoder values from Arduino and publishes them as ROS 2 topics."""
        while rclpy.ok():
            if self.reading_serial and self.ser.in_waiting > 0:
                try:
                    # Read data from Arduino in "123 456" format
                    data = self.ser.readline().decode('utf-8').strip()
                    left_enc, right_enc = map(int, data.split())

                    # Publish encoder values
                    self.encoder_left_pub.publish(Int32(data=left_enc))
                    self.encoder_right_pub.publish(Int32(data=right_enc))

                    self.get_logger().info(f'Received encoders: Left={left_enc}, Right={right_enc}')
                except ValueError:
                    self.get_logger().error(f'Invalid data received: {data}')
            time.sleep(0.01)  # Small sleep to avoid overloading the CPU

    def pwmr_callback(self, msg):
        """Handles incoming speed right value and sends it to Arduino."""
        self.pwmr_value = msg.data
        self.get_logger().info(f'Received PWMR: {self.pwmr_value}')
        self.send_pwm_to_arduino()

    def pwml_callback(self, msg):
        """Handles incoming speed left value and sends it to Arduino."""
        self.pwml_value = msg.data
        self.get_logger().info(f'Received PWML: {self.pwml_value}')
        self.send_pwm_to_arduino()

    def send_pwm_to_arduino(self):
        """Sends the speed values to Arduino in the format '123 234'."""
        # Temporarily pause reading from serial
        self.reading_serial = False

        try:
            # Format and send the PWM values to Arduino
            pwm_message = f'{self.pwml_value} {self.pwmr_value}\n'
            self.ser.write(pwm_message.encode())
            self.get_logger().info(f'Sent speed values to Arduino: {pwm_message.strip()}')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to send data to Arduino: {e}')
        finally:
            # Resume reading from serial after sending
            self.reading_serial = True

    def destroy(self):
        """Clean up when the node is stopped."""
        self.reading_serial = False  # Stop the read loop
        self.ser.close()
        super().destroy()


def main(args=None):
    rclpy.init(args=args)

    arduino_serial_node = ArduinoSerialNode()

    try:
        rclpy.spin(arduino_serial_node)
    except KeyboardInterrupt:
        arduino_serial_node.get_logger().info('Node stopped cleanly')
    except BaseException as e:
        arduino_serial_node.get_logger().error(f'Error: {e}')
    finally:
        arduino_serial_node.destroy()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
