import rclpy
from rclpy.node import Node
from custom_msg.msg import TwoInt
import serial
import threading

class ArduinoSerialNode(Node):
    def __init__(self):
        super().__init__('arduino_serial_node')

        # Initialize serial connection
        self.ser = serial.Serial('/dev/ttyACM1', 460800, timeout=0.01)  # Increased baud rate
        self.ser_lock = threading.Lock()  # Ensure thread-safe serial access

        # Publishers and Subscribers
        self.encoder_pub = self.create_publisher(TwoInt, 'encoder', 10)
        self.pwm_sub = self.create_subscription(TwoInt, 'PWM', self.pwm_callback, 10)

        # Variables to store PWM values
        self.pwmr_value = 0
        self.pwml_value = 0
        self.isSpraying = 0

        # Thread for reading encoder values
        self.running = True
        self.read_thread = threading.Thread(target=self.read_encoder_values, daemon=True)
        self.read_thread.start()

    def read_encoder_values(self):
        """Continuously reads encoder values from Arduino and publishes them as ROS2 messages."""
        while self.running and rclpy.ok():
            try:
                with self.ser_lock:  # Prevent read/write collisions
                    data = self.ser.readline().decode('utf-8').strip()
                
                if data:
                    try:
                        left_enc, right_enc, toggleState = map(int, data.split())
                        enc_msg = TwoInt()
                        enc_msg.l = left_enc
                        enc_msg.r = right_enc
                        enc_msg.toggle = toggleState
                        self.encoder_pub.publish(enc_msg)
                    except ValueError:
                        self.get_logger().warn(f'Invalid data received: {data}')
            except serial.SerialException as e:
                self.get_logger().error(f'Serial error: {e}')

    def pwm_callback(self, msg):
        """Handles incoming PWM values and sends them to Arduino."""
        self.pwml_value, self.pwmr_value, self.isSpraying = msg.l, msg.r, msg.toggle
        self.send_pwm_to_arduino()

    def send_pwm_to_arduino(self):
        """Sends PWM values to Arduino in an optimized format."""
        try:
            with self.ser_lock:  # Ensure exclusive serial access
                self.ser.write(f'{self.pwml_value} {self.pwmr_value} {self.isSpraying}\n'.encode())
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to send PWM: {e}')

    def destroy(self):
        """Cleans up when the node is stopped."""
        self.running = False
        self.read_thread.join()
        self.ser.close()
        super().destroy()

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoSerialNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped cleanly')
    except BaseException as e:
        node.get_logger().error(f'Unexpected error: {e}')
    finally:
        node.destroy()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
