import rclpy
from rclpy.node import Node
from custom_msg.msg import Coordinates
import time

class CoordinatesPublisher(Node):
    def __init__(self):
        super().__init__('coordinates_publisher')

        # Create publisher for the coordinates
        self.coord_pub = self.create_publisher(Coordinates, 'coordinates', 10)

        # Load coordinates from a file
        self.coordinates = self.load_coordinates_from_file('/home/hue/ros2_ws/src/get_waypoints/get_waypoints/Square.txt')
        
        # Index to keep track of which coordinate is being published
        self.index = 0

        # Publish coordinates in a loop
        while True:
            self.publish_coordinates()
            time.sleep(0.2)

    def load_coordinates_from_file(self, filename):
        coordinates = []
        try:
            with open(filename, 'r') as file:
                for line in file:
                    x, y, t = map(float, line.strip().split()) # split into x y and toggle
                    coordinates.append((x, y, int(t)))
            self.get_logger().info(f'Loaded {len(coordinates)} coordinates from file.')
        except Exception as e:
            self.get_logger().error(f'Failed to load coordinates from file: {e}')
        return coordinates

    def publish_coordinates(self):
        # Get the current coordinate pair
        coord = self.coordinates[self.index]
        x, y, t = coord

        coord_msg = Coordinates()
        coord_msg.x = x*0.4 # make smaller by 1.5
        coord_msg.y = y*0.4
        coord_msg.toggle = t

        # Publish the x and y coordinates
        self.coord_pub.publish(coord_msg)

        self.get_logger().info(f'Published coordinates: X={x}, Y={y}, toggle: {t}')

        # Move to the next coordinate in the array
        self.index = (self.index + 1) % len(self.coordinates)


def main(args=None):
    rclpy.init(args=args)

    # Create the coordinate publisher node
    coord_publisher = CoordinatesPublisher()

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
