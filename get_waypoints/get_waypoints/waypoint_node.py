import rclpy
from rclpy.node import Node
from custom_msg.msg import Coordinates
import time
import sys

class CoordinatesPublisher(Node):
    def __init__(self, filename):
        super().__init__('coordinates_publisher')

        # Create publisher for the coordinates
        self.coord_pub = self.create_publisher(Coordinates, 'coordinates', 10)

        # Load coordinates from the given file
        self.coordinates = self.load_coordinates_from_file(filename)
        self.index = 0

        # Start publishing loop only if coordinates were loaded
        if self.coordinates:
            self.publish_coordinates()
        else:
            self.get_logger().error("No coordinates loaded. Exiting...")

    def load_coordinates_from_file(self, filename):
        coordinates = []
        try:
            with open(filename, 'r') as file:
                for line in file:
                    x, y, t = map(float, line.strip().split())  # split into x, y, and toggle
                    coordinates.append((x, y, int(t)))
            self.get_logger().info(f'Loaded {len(coordinates)} coordinates from file.')
        except Exception as e:
            self.get_logger().error(f'Failed to load coordinates from file: {e}')
        return coordinates

    def publish_coordinates(self):
        while self.index < len(self.coordinates):
            x, y, t = self.coordinates[self.index]
            coord_msg = Coordinates()
            coord_msg.x = x * 1.0  # Modify if scaling is needed
            coord_msg.y = y * 1.0
            coord_msg.toggle = t

            self.coord_pub.publish(coord_msg)
            self.get_logger().info(f'Published coordinates: X={x}, Y={y}, toggle: {t}')

            self.index += 1
            time.sleep(0.2)  # Adjust sleep time as needed

        self.get_logger().info("Finished publishing all coordinates. Node will shut down.")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    
    # Get filename from command-line arguments
    if len(sys.argv) < 2:
        print("Usage: ros2 run <package_name> <node_name> <filename>")
        return
    filename = sys.argv[1]

    coord_publisher = CoordinatesPublisher('/home/hue/ros2_ws/src/get_waypoints/get_waypoints/' + filename)
    try:
        rclpy.spin(coord_publisher)
    except KeyboardInterrupt:
        coord_publisher.get_logger().info('Node stopped cleanly')
    except BaseException as e:
        coord_publisher.get_logger().error(f'Error: {e}')
    finally:
        coord_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

