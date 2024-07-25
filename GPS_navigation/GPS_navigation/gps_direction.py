#for gps_navigation_and_direction(2)

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import math
from std_msgs.msg import String

class DirectionPublisher(Node):
    def __init__(self):
        super().__init__('direction_publisher')
        self.publisher = self.create_publisher(String, 'from_direction', 10)
        self.subscription = self.create_subscription(Float32MultiArray, 'from_zedf9p_gps', self.callback, 10)
        self.subscription = self.create_subscription(Float32MultiArray, 'from_openrouteservice', self.listener_callback, 10)

        self.coordinates = [None, None]  # List to store coordinates
        self.target_lat = None
        self.target_lon = None

    def callback(self, msg):
        print("hiiii")
        print(msg)
        # Move the current coordinates in index 1 to index 0
        self.coordinates[0] = self.coordinates[1]

        # Update the coordinates at index 1 with the new data
        self.coordinates[1] = (msg.data[0], msg.data[1])

        # Extract coordinates
        lat1, lon1 = self.coordinates[0] if self.coordinates[0] else (None, None)
        lat2, lon2 = self.coordinates[1]

        print('lat1 =', lat1, ', lon1 =', lon1, ', lat2 =', lat2, ', lon2 =', lon2)

        if lat1 is not None and lon1 is not None and lat2 is not None and lon2 is not None:
            current_bearing = self.calculate_bearing(lat1, lon1, lat2, lon2)
            self.get_logger().info(f"Calculated current bearing: {current_bearing} degrees")

            # If target coordinates are available, calculate target bearing and compare
            if self.target_lat is not None and self.target_lon is not None:
                target_bearing = self.calculate_bearing(lat2, lon2, self.target_lat, self.target_lon)
                self.get_logger().info(f"Calculated target bearing: {target_bearing} degrees")

                # Determine the direction to turn
                turn_direction = self.determine_turn_direction(current_bearing, target_bearing)
                self.get_logger().info(turn_direction)

    def listener_callback(self, msg):
        self.target_lat, self.target_lon = msg.data[0], msg.data[1]
        self.get_logger().info(f'Received target location: Lat = {self.target_lat}, Lon = {self.target_lon}')

    def calculate_bearing(self, lat1, lon1, lat2, lon2):
        # Convert latitude and longitude to radians
        lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])

        # Difference in longitude
        dLon = lon2 - lon1

        # Calculate bearing
        x = math.sin(dLon) * math.cos(lat2)
        y = math.cos(lat1) * math.sin(lat2) - (math.sin(lat1) * math.cos(lat2) * math.cos(dLon))
        initial_bearing = math.atan2(x, y)

        # Convert from radians to degrees
        initial_bearing = math.degrees(initial_bearing)
        # Normalize to 0-360 degrees
        compass_bearing = (initial_bearing + 360) % 360

        return compass_bearing

    def determine_turn_direction(self, current_bearing, target_bearing):
        # Calculate the difference
        difference = target_bearing - current_bearing
        if difference > 180:
            difference -= 360
        elif difference < -180:
            difference += 360

        msg = String()
        if abs(difference) <= 10:
            msg.data = "Go straight"
            self.publisher.publish(msg)
            return "Go straight"
        elif difference > 0:
            msg.data = "Turn right"
            self.publisher.publish(msg)
            return "Turn right"
        else:
            msg.data = "Turn left"
            self.publisher.publish(msg)
            return "Turn left"

def main(args=None):
    rclpy.init(args=args)
    node = DirectionPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
