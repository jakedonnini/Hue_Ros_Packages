import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from custom_msg.msg import Coordinates, GpsData
import math
import numpy as np
from message_filters import Subscriber, ApproximateTimeSynchronizer

class GPSFusionNode(Node):
    def __init__(self):
        super().__init__('gps_fusion_node')
        
        # Subscribing to both GPS topics with message_filters
        self.gps_sub1 = Subscriber(self, Coordinates, 'gps')
        self.gps_sub2 = Subscriber(self, Coordinates, 'gps2')
        
        self.ts = ApproximateTimeSynchronizer([self.gps_sub1, self.gps_sub2], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.gps_callback)
        
        # Publisher for midpoint and robot angle
        self.gps_fusion_publisher = self.create_publisher(GpsData, 'gps/data', 10)
        self.gps1_cm_publisher = self.create_publisher(Coordinates, 'gps1_cm', 10)
        self.gps2_cm_publisher = self.create_publisher(Coordinates, 'gps2_cm', 10)
        
        # Origin and conversion factors
        self.origin_lat = None
        self.origin_lon = None
        self.lat_to_cm = 111139.0 * 100  # Convert latitude degrees to cm
        self.lon_to_cm = None
        self.x_mid_zero = 0
        self.y_mid_zero = 0
    
    def gps_callback(self, msg1, msg2):
        lat1, lon1 = msg1.x, msg1.y
        lat2, lon2 = msg2.x, msg2.y
        
        if self.origin_lat is None and self.origin_lon is None:
            self.origin_lat = lat1
            self.origin_lon = lon1
            self.lon_to_cm = 111139.0 * 100 * np.cos(np.radians(self.origin_lat))
        
        x1, y1 = self.lat_to_cm * (lat1 - self.origin_lat), self.lon_to_cm * (lon1 - self.origin_lon)
        x2, y2 = self.lat_to_cm * (lat2 - self.origin_lat), self.lon_to_cm * (lon2 - self.origin_lon)

        mid_x = (x1 + x2) / 2.0
        mid_y = (y1 + y2) / 2.0

        if self.x_mid_zero == 0 and self.y_mid_zero == 0:
            self.x_mid_zero = mid_x
            self.y_mid_zero = mid_y

        mid_x -= self.x_mid_zero
        mid_y -= self.y_mid_zero

        delta_x = x2 - x1
        delta_y = y2 - y1
        angle = math.atan2(delta_x, delta_y)
        
        gps_data = GpsData()
        gps_data.x = mid_x
        gps_data.y = mid_y
        gps_data.angle = angle

        gps_coord1 = Coordinates()
        gps_coord1.x = x1
        gps_coord1.y = y1

        gps_coord2 = Coordinates()
        gps_coord2.x = x2
        gps_coord2.y = y2

        self.gps_fusion_publisher.publish(gps_data)
        self.gps1_cm_publisher.publish(gps_coord1)
        self.gps2_cm_publisher.publish(gps_coord2)
        
        self.get_logger().info(f'Midpoint: ({mid_x}, {mid_y}), Angle: {math.degrees(angle)} degrees')
        

def main(args=None):
    rclpy.init(args=args)
    gps_fusion_node = GPSFusionNode()
    
    try:
        rclpy.spin(gps_fusion_node)
    except KeyboardInterrupt:
        pass
    finally:
        gps_fusion_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
