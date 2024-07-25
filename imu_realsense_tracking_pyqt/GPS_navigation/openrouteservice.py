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
        self.send_data()  # 처음 시작할 때 자동으로 첫 데이터 전송

    def load_coordinates(self):
        file_path = '/home/songah/ros2_ws/src/my_package/GPS_navigation/cafeteria_to_building3.json'
        with open(file_path, 'r') as file:
            data = json.load(file)
            for route in data['routes']:
                self.coordinates.extend(route['geometry']['coordinates'])

    def send_data(self):
        # 데이터를 보내는 로직을 별도의 메소드로 분리
        if self.current_index < len(self.coordinates):
            coord = self.coordinates[self.current_index]
            msg = Float32MultiArray()
            msg.data = [float(coord[1]), float(coord[0]), float(coord[2])]
            self.publisher.publish(msg)
            self.current_index += 1
        else:
            self.get_logger().info("All data has been sent. Stopping the publisher.")
            self.destroy_node()  # 노드 종료

    def number_callback(self, msg):
        # 숫자 메시지를 받을 때마다 send_data 메소드를 호출하여 데이터 전송
        self.send_data()

def main():
    rclpy.init()
    publisher = NavigationPublisher()
    rclpy.spin(publisher)

if __name__ == '__main__':
    main()
