#for gps_navigation_and_direction(3)

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32
from serial import Serial
from pyubx2 import UBXReader

class Zedf9pNode(Node):
    def __init__(self):
        super().__init__('zedf9p_node')
        self.subscription = self.create_subscription(Float32MultiArray, 'from_openrouteservice', self.listener_callback, 10)
        self.publisher_number = self.create_publisher(Int32, 'from_zedf9p_number', 10)
        self.publisher_gps = self.create_publisher(Float32MultiArray, 'from_zedf9p_gps', 10)

        self.publish_number(1)
        
        # Timer to periodically publish GPS data and check position
        self.timer = self.create_timer(1.0, self.publish_and_check_position)

        # Initialize target location variables
        self.target_lat = None
        self.target_lon = None

    def gps_data_fetch(self):
        with Serial("/dev/ttyACM0", 9600, timeout=3) as stream:
            ubr = UBXReader(stream)
            for raw, parsed in ubr:
                if hasattr(parsed, "lat") and hasattr(parsed, "lon"):
                    return parsed.lat, parsed.lon
        return None, None

    def listener_callback(self, msg):
        self.target_lat, self.target_lon = msg.data[0], msg.data[1]
        self.get_logger().info(f'Received target location: Lat = {self.target_lat}, Lon = {self.target_lon}')

    def publish_and_check_position(self):
        current_lat, current_lon = self.gps_data_fetch()
        if current_lat is not None and current_lon is not None:
            # Publish current GPS data
            gps_msg = Float32MultiArray()
            gps_msg.data = [current_lat, current_lon]
            self.publisher_gps.publish(gps_msg)
            self.get_logger().info(f'Published current GPS data: Lat = {current_lat}, Lon = {current_lon}')
            
            # Check if current location is close enough to the target location
            if self.target_lat is not None and self.target_lon is not None:
                if abs(self.target_lat - current_lat) <= 0.0001 and abs(self.target_lon - current_lon) <= 0.0001:
                    # Publish number 1 if close enough to the target
                    self.get_logger().info('You are close enough to the target location')
                    self.publish_number(1)
                else:
                    self.get_logger().info('Current location is not close enough to the target location.')
        else:
            self.get_logger().error('Failed to fetch GPS data')

    def publish_number(self, number):
        msg = Int32()
        msg.data = number
        self.publisher_number.publish(msg)
        self.get_logger().info(f'Published number: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = Zedf9pNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
