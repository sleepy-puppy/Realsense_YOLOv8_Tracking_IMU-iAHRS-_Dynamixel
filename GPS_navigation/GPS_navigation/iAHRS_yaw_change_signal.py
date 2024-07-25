import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32
import math
import serial
# import time

class Zedf9pNode(Node):
    def __init__(self):
        super().__init__('zedf9p_node')
        # 목표 위, 경도
        self.x_goal = None
        self.y_goal = None
        # 현재 위, 경도
        self.x_present = None
        self.y_present = None
        # 현재 yaw 값
        self.a_present = None

        self.a_calculate = None
        
        # 시리얼 포트 설정
        self.serial_port = "/dev/ttyUSB0"
        self.serial_speed = 115200
        self.serial_connection = self.serial_open()
        
        self.subscription_to_zedf9p = self.create_subscription(Float32MultiArray, 'from_openrouteservice', self.listener_callback_from_openrouteservice, 10)
        self.subscription_from_zedf9p_gps = self.create_subscription(Float32MultiArray, 'from_zedf9p_gps', self.listener_callback_from_zedf9p_gps, 10)
        self.publisher_number = self.create_publisher(Int32, 'from_iAHRS', 10)
        
    def listener_callback_from_openrouteservice(self, msg):
        # 'from_openrouteservice' 토픽으로부터 위, 경도 값 받아 저장
        self.x_goal, self.y_goal = msg.data
        self.get_logger().info(f'목표 설정: x={self.x_goal}, y={self.y_goal}')
        
    def listener_callback_from_zedf9p_gps(self, msg):
        # 'from_zedf9p_gps' 토픽으로부터 현재 위, 경도 값 받아 저장
        self.x_present, self.y_present = msg.data
        self.get_logger().info(f'현재 위치: x={self.x_present}, y={self.y_present}')

        # 수학 공식 적용
        self.apply_math_formula()

        while True :

            # Yaw 값 업데이트
            self.update_yaw_value()

            if 0 < self.a_calculate :
                if -180 <= self.a_present - self.a_calculate <= -10 :
                    print("turn left")
                else :
                    print("turn right")
            elif self.a_calculate < 0 :
                if 10 <= self.a_present - self.a_calculate <= 180 :
                    print("turn right")
                else :
                    print("turn left")
        


    def apply_math_formula(self):
        # 간단한 예시로, 두 지점 사이의 각도를 계산
        if None not in (self.x_goal, self.y_goal, self.x_present, self.y_present):
            self.a_calculate = math.asin(abs(self.x_goal)/math.sqrt((self.x_present - self.x_goal)^2 + (self.y_present - self.y_goal)^2))

            if self.x_present < self.x_goal :
                x_angle = -x_angle

            self.get_logger().info('수학 공식 적용 중...')
            # 결과를 처리하거나 다른 메서드 호출
        else:
            self.get_logger().info('모든 좌표를 기다리는 중...')

    def publish_number(self, number):
        msg = Int32()
        msg.data = number
        self.publisher_number.publish(msg)
        self.get_logger().info(f'발행된 숫자: {msg.data}')
        
    def serial_open(self):
        try:
            ser = serial.Serial(self.serial_port, self.serial_speed, timeout=0.1)
            self.get_logger().info(f"{self.serial_port} 열기 성공")
            return ser
        except serial.SerialException as e:
            self.get_logger().info(f"{self.serial_port} 열기 실패: {e}")
            return None
        
    def send_recv(self, command, data_length):
        # 시리얼 포트로 명령 전송
        self.serial_connection.write(command.encode())
        # 데이터 수신 및 처리
        recv_buff = self.serial_connection.readline().decode().strip()
        # 수신된 데이터가 있을 경우 처리
        if recv_buff:
            if recv_buff[0] == '!':
                return -1
            if command[:-1] == recv_buff[:len(command)-1] and recv_buff[len(command)-1] == '=':
                data_str = recv_buff[len(command):]
                data_list = data_str.split(',')
                data_count = min(len(data_list), data_length)
                try:
                    returned_data = [int(data, 16) if data.startswith('0x') else float(data) for data in data_list[:data_count]]
                    return data_count, returned_data
                except ValueError:
                    pass
        return 0, []

    def update_yaw_value(self):
        # Yaw 값 읽기
        data_count, data = self.send_recv("e\n", 3)
        if data_count >= 3:
            self.a_present = data[2]  # Yaw 값을 a_present에 저장
            self.get_logger().info(f"현재 yaw: {self.a_present}")

def main(args=None):
    rclpy.init(args=args)
    node = Zedf9pNode()
    rclpy.spin(node)
    # 시리얼 포트 닫기
    if node.serial_connection is not None:
        node.serial_connection.close()
    node.destroy_node()
    rclpy.shutdown()

# 함수 호출은 주석 처리
if __name__ == '__main__':
    main()
