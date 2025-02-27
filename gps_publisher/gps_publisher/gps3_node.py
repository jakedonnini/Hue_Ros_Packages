import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from custom_msg.msg import Coordinates
from custom_msg.msg import GpsData
import math
import numpy as np


class GPSFusionNode(Node):
    def __init__(self):
        super().__init__('gps_fusion_node')
        
        # Subscribing to both GPS topics
        self.subscription1 = self.create_subscription(
            Coordinates,
            'gps',  # Primary GPS topic
            self.gps_callback_1,
            10)
        
        self.subscription2 = self.create_subscription(
            Coordinates,
            'gps2',  # Secondary GPS topic
            self.gps_callback_2,
            10)
        
        # Publisher for midpoint and robot angle
        self.gps_fusion_publisher = self.create_publisher(GpsData, 'gps/data', 10)
        self.gps1_cm_publisher = self.create_publisher(Coordinates, 'gps1_cm', 10)
        self.gps2_cm_publisher = self.create_publisher(Coordinates, 'gps2_cm', 10)


        
        # Variables to store GPS data
        self.gps1 = None
        self.gps2 = None

        self.origin_lat_1 = None
        self.origin_lon_1 = None
        self.lat_to_cm_1 = 111139.0 * 100 
        self.lon_to_cm_1 = 111139.0 * 100 * np.cos(np.radians(self.origin_lat_1 or 0))
        
        self.origin_lat_2 = None
        self.origin_lon_2 = None
        self.lat_to_cm_2 = 111139.0 * 100
        self.lon_to_cm_2 = 111139.0 * 100 * np.cos(np.radians(self.origin_lat_2 or 0))
        self.x_mid_zero = 0
        self.y_mid_zero = 0
    
    def gps_callback_1(self, msg):
        self.gps1 = (msg.x, msg.y)
        self.compute_and_publish()
    
    def gps_callback_2(self, msg):
        self.gps2 = (msg.x, msg.y)
        self.compute_and_publish()
    
    def compute_and_publish(self):
        if self.gps1 is None or self.gps2 is None:
            return
        
        lat1, lon1 = self.gps1
        lat2, lon2 = self.gps2
        
        if not self.origin_lat_1 and not self.origin_lon_1:
            self.origin_lat_1 = lat1
            self.origin_lon_1 = lon1
            self.lon_to_cm_1 = 111139.0 * 100 * np.cos(np.radians(self.origin_lat_1))
                                                       
        if not self.origin_lat_2 and not self.origin_lon_2:
            self.origin_lat_2 = lat2
            self.origin_lon_2 = lon2
            self.lon_to_cm_2 = 111139.0 * 100 * np.cos(np.radians(self.origin_lat_2))

        # use the same origin to keep on same basis
        x1, y1 = self.lat_to_cm_1 * (lat1 - self.origin_lat_1), self.lon_to_cm_1 * (lon1 - self.origin_lon_1)
        x2, y2 = self.lat_to_cm_1 * (lat2 - self.origin_lat_1), self.lon_to_cm_1 * (lon2 - self.origin_lon_1)

        mid_x = (x1 + x2) / 2.0
        mid_y = (y1 + y2) / 2.0

        # if self.x_mid_zero == 0 and self.y_mid_zero == 0:
        #     self.x_mid_zero = mid_x
        #     self.y_mid_zero = mid_y

        # mid_x -= self.x_mid_zero
        # mid_y -= self.y_mid_zero

        delta_x = x2 - x1
        delta_y = y2 - y1
        angle = math.atan2(delta_y, delta_x)
        
        # Publish midpoint and angle
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
        
        # self.get_logger().info(f'Midpoint: ({mid_x}, {mid_y}), Angle: {angle} degrees')
        

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
