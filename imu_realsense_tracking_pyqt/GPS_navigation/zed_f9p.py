# from ublox_gps import UbloxGps
# import serial
# # Can also use SPI here - import splatitudeev
# # I2C is not supported

# port = serial.Serial('/dev/ttyACM0', baudrate=38400, timeout=1)
# gps = UbloxGps(port)

# def run():
  
#   try: 
#     print("Listenting for UBX Messages.")
#     while True:
#       try:
#         coords = gps.geo_coords()
#         if coords is not None:
#             print(coords.lon, coords.lat)
#         else:
#             print("No GPS data available.")
#       except (ValueError, IOError) as err:
#         print(err)
  
#   finally:
#     port.close()

# if __name__ == '__main__':
#   run()


# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float32MultiArray, String, Int32

# class Zedf9pNode(Node):
#     def __init__(self):
#         super().__init__('zedf9p_node')
#         # 구독자 생성
#         self.subscription = self.create_subscription(
#             Float32MultiArray, 'to_zedf9p', self.listener_callback, 10)
#         self.subscription  # prevent unused variable warning
        
#         # 퍼블리셔 생성 (숫자 메시지를 위한 것)
#         self.publisher_number = self.create_publisher(Int32, 'from_zedf9p_number', 10)

#     def listener_callback(self, msg):
#         if len(msg.data) >= 2:
#             latitude = msg.data[0]
#             longitude = msg.data[1]
#             self.get_logger().info(f'Received data - 위도: {latitude}, 경도: {longitude}')
            
#             # 한 줄의 데이터 처리를 완료했음을 나타내는 숫자 1 발행
#             number_msg = Int32()
#             number_msg.data = 1
#             self.publisher_number.publish(number_msg)

# def main(args=None):
#     rclpy.init(args=args)
#     zedf9p_node = Zedf9pNode()
#     rclpy.spin(zedf9p_node)
#     zedf9p_node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()


from ublox_gps import UbloxGps
import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32

class Zedf9pNode(Node):
    def __init__(self):
        super().__init__('zedf9p_node')
        self.subscription = self.create_subscription(Float32MultiArray, 'from_openrouteservice', self.listener_callback, 10)
        self.publisher_number = self.create_publisher(Int32, 'from_zedf9p_number', 10)
        self.publisher_gps = self.create_publisher(Float32MultiArray, 'from_zedf9p_gps', 10)
        
        # GPS 모듈 초기화
        port = serial.Serial('/dev/ttyACM0', baudrate=38400, timeout=1)
        self.gps = UbloxGps(port)
        
        # GPS 데이터를 주기적으로 발행하기 위한 타이머 설정
        self.timer = self.create_timer(1.0, self.publish_current_gps)

    def listener_callback(self, msg):
        try:
            coords = self.gps.geo_coords()
            if coords is not None:
                # ROS 2 메시지로부터 받은 목표 위치와 현재 GPS 위치를 비교
                target_lat, target_lon = msg.data[0], msg.data[1]
                current_lat, current_lon = coords.lat, coords.lon
                
                # 위도와 경도 차이가 0.001 이내인지 확인
                if abs(target_lat - current_lat) <= 0.001 and abs(target_lon - current_lon) <= 0.001:
                    # 목표 위치에 가까워진 경우 숫자 데이터(1) 발행
                    self.publish_number(1)
                else:
                    self.get_logger().info('Current location is not close enough to target location.')
        except (ValueError, IOError) as err:
            self.get_logger().error(f'GPS error: {err}')

    def publish_number(self, number):
        msg = Int32()
        msg.data = number
        self.publisher_number.publish(msg)
        self.get_logger().info(f'Published number: {msg.data}')
        
    def publish_current_gps(self):
        try:
            coords = self.gps.geo_coords()
            if coords is not None:
                # 현재 GPS 위치 데이터를 포함하는 메시지 생성 및 발행
                msg = Float32MultiArray()
                msg.data = [coords.lat, coords.lon]
                self.publisher_gps.publish(msg)
                self.get_logger().info(f'Published current GPS data: Latitude = {coords.lat}, Longitude = {coords.lon}')
        except (ValueError, IOError) as err:
            self.get_logger().error(f'GPS error: {err}')

def main(args=None):
    rclpy.init(args=args)
    node = Zedf9pNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

