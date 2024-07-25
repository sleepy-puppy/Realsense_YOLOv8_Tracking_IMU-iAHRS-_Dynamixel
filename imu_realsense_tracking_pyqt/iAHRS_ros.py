#imu_realsense_tracking_pyqt (2)

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, String
import serial

# 시리얼 포트 설정
SERIAL_PORT = "/dev/ttyUSB0"
SERIAL_SPEED = 115200

class SerialPublisher(Node):
    def __init__(self):
        super().__init__('to_dynamixel_node')
        self.publisher = self.create_publisher(Int32MultiArray, 'to_dynamixel', 10)
        self.serial = self.serial_open()
        self.max_data = 10
        self.start_signal_received = False  # '시작' 신호 수신 상태를 추적하는 변수

        # '시작' 신호를 수신하기 위한 서브스크라이버 추가
        self.subscription = self.create_subscription(String, 'start_signal', self.start_signal_callback, 10)

    def start_signal_callback(self, msg):
        self.get_logger().info(f"'Start' signal received: {msg.data}")
        self.start_signal_received = True


    def serial_open(self):
        try:
            ser = serial.Serial(SERIAL_PORT, SERIAL_SPEED, timeout=0.1)
            self.get_logger().info(f"{SERIAL_PORT} open success")
            return ser
        except serial.SerialException as e:
            self.get_logger().error(f"Error unable to open {SERIAL_PORT}: {e}")
            return None

    def send_recv(self, command, data_length):
        if not self.serial:
            return 0, []

        # 시리얼 포트로 명령 전송
        self.serial.write(command.encode())

        # 데이터 수신 및 처리
        recv_buff = self.serial.readline().decode().strip()

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

    def publish_data(self):
        euler_msg = Int32MultiArray()

        # 오일러 각도 읽기
        data_count, data = self.send_recv("e\n", self.max_data)
        if data_count >= 3:
            euler_msg.data = [int(data[0]), int(data[1]), int(data[2])]
            roll = data[0]
            pitch = data[1]
            yaw = data[2]

            # '시작' 신호를 받기 전, 모든 데이터를 전송
            if not self.start_signal_received:
                self.publish_all(roll, pitch, yaw)
            else:
                # '시작' 신호를 받은 후, roll 데이터만 전송
                self.publish_roll(roll)

            self.get_logger().info(f"Published Euler angles: {roll}, {pitch}, {yaw}")
        else:
            self.get_logger().warn("Received incomplete Euler angles data")


    def publish_all(self, roll, pitch, yaw):
        # roll
        if 10 < roll < 180:
            self.publish_message(103, 0)  # roll ccw
        elif -180 < roll < -10:
            self.publish_message(103, 1)  # roll cw

        # pitch
        if 10 < pitch < 180:
            self.publish_message(102, 1)  # pitch cc
        elif -180 < pitch < -10:
            self.publish_message(102, 0)  # pitch ccw

        # yaw
        if 10 < yaw < 180:
            self.publish_message(101, 0)  # yaw ccw
        elif -180 < yaw < -10:
            self.publish_message(101, 1)  # yaw cw

    def publish_roll(self, roll):
        # roll만 전송
        if 10 < roll < 180:
            self.publish_message(103, 0)  # roll ccw
        elif -180 < roll < -10:
            self.publish_message(103, 1)  # roll cw

    def publish_message(self, dxl_id, direction):
        data_ = (dxl_id, direction)
        msg = Int32MultiArray(data=data_)
        self.publisher.publish(msg)       

def main():
    rclpy.init()
    serial_publisher = SerialPublisher()
    timer_period = 0.001  # seconds
    serial_publisher.create_timer(timer_period, serial_publisher.publish_data)

    rclpy.spin(serial_publisher)

    serial_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
