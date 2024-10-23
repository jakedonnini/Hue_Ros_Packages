import rclpy
from rclpy.node import Node
from custom_msg.msg import Coordinates
import time

class CoordinatesPublisher(Node):
    def __init__(self):
        super().__init__('coordinates_publisher')

        # Create publishers for the x and y coordinates
        self.coord_pub = self.create_publisher(Coordinates, 'coordinates', 10)

        # Large array of coordinates stored as tuples (x, y)
        self.coordinates = [
            (10.0, 20.0), (15.5, 25.2), (30.4, 35.6), (45.0, 50.1), 
            # Add more coordinate tuples as needed
        ]
        
        # Index to keep track of which coordinate is being published
        self.index = 0

        while True:
            self.publish_coordinates()
            time.sleep(10)

    def publish_coordinates(self):
        # Get the current coordinate pair
        coord = self.coordinates[self.index]
        x, y = coord

        coord_msg = Coordinates()
        coord_msg.x = x
        coord_msg.y = y

        # Publish the x and y coordinates
        self.coord_pub.publish(coord_msg)

        self.get_logger().info(f'Published coordinates: X={x}, Y={y}')

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
