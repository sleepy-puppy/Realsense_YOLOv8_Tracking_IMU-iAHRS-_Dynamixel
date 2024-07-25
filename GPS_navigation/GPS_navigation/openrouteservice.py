#for gps_navigation_and_direction(1)

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32
import json

class NavigationPublisher(Node):
    def __init__(self):
        super().__init__('navigation_publisher')
        self.publisher = self.create_publisher(Float32MultiArray, 'from_openrouteservice', 10)
        self.subscription = self.create_subscription(Int32, 'from_zedf9p_number', self.number_callback, 10)
        self.coordinates = []
        self.current_index = 0
        self.load_coordinates()

    def load_coordinates(self):
        file_path = '/home/moon/ros2_ws/src/GPS_navigation/GPS_navigation/building3_to_cafeteria.json'
        try:
            with open(file_path, 'r') as file:
                data = json.load(file)
                for route in data['routes']:
                    self.coordinates.extend(route['geometry']['coordinates'])
        except FileNotFoundError:
            self.get_logger().error(f"The file {file_path} does not exist.")
        except json.JSONDecodeError:
            self.get_logger().error(f"The file {file_path} is not a valid JSON file.")

    def send_data(self):
        if self.current_index < len(self.coordinates):
            coord = self.coordinates[self.current_index]
            msg = Float32MultiArray()
            msg.data = [float(coord[1]), float(coord[0]), float(coord[2])]
            self.publisher.publish(msg)
            self.current_index += 1
            if self.current_index >= len(self.coordinates):
                self.get_logger().info("All data has been sent. Stopping the publisher.")
                self.destroy_node()
        else:
            self.get_logger().info("No more data to send.")

    def number_callback(self, msg):
        if msg.data == 1: 
            self.get_logger().info("Received number 1, sending data...")
            self.send_data()
        else:
            self.get_logger().info(f"Received number {msg.data}, but expecting 1 to send data.")

def main():
    rclpy.init()
    publisher = NavigationPublisher()
    rclpy.spin(publisher)

if __name__ == '__main__':
    main()
