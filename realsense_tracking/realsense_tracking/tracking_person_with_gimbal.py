import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32
from geometry_msgs.msg import Twist, Vector3
import math

class DepthSubscriber(Node):
    def __init__(self):
        super().__init__('depth_subscriber')
        self.subscription = self.create_subscription(Float32MultiArray, 'target_person_depth', self.target_person_depth_listener_callback, 10)
        self.yaw_subscription = self.create_subscription(Int32, 'dynamixel_yaw', self.yaw_callback, 10)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.robot_v = 0.0  # 초기화
        self.wheelbase = 0.84  # 로봇의 휠베이스(L)를 가정 (단위: meter)
        self.current_yaw = 0
        
    def target_person_depth_listener_callback(self, msg):
        person_id = int(msg.data[0])
        depth = msg.data[1]

        # if depth > 0.5:
        #     print(f"ID {person_id}: 깊이 값이 0.5m보다 큼 ({depth:.2f} m)")
        
        # 깊이에 따른 속도 계산
        self.robot_v = self.calculate_robot_v(depth)

        # yaw 값을 받아서 오메가 계산
        robot_w = self.calculate_robot_w(self.current_yaw)

        print(f"ID {person_id}: robot_w = {robot_w:.2f}, robot_v = {self.robot_v:.2f}")

        # Twist 메시지 발행
        twist_msg = Twist()
        twist_msg.linear = Vector3(x=self.robot_v, y=0.0, z=0.0)
        twist_msg.angular = Vector3(x=0.0, y=0.0, z=robot_w)
        self.publisher_.publish(twist_msg)
        
    def yaw_callback(self, msg):
        self.current_yaw = msg.data
    
    def calculate_robot_w(self, yaw):
        # yaw 값을 통해 조향각 계산 (단위: degree)
        print(f"dynamixel yaw : {self.current_yaw}")
        steering_angle = 360 * (yaw - 2048) / 4096
        
        # 조향각을 -180도에서 180도로 제한
        steering_angle = max(min(steering_angle, 180), -180)
        print(f"steering angle : {steering_angle}")

        if abs(steering_angle) <= 5:
            return 0.0

        max_w = 1.5  # 각속도의 최대값
        
        if self.robot_v == 0.0:
            # 조향각이 180도에 가까우면 -1.5에, -180도에 가까우면 +1.5에 가까워지도록 설정
            robot_w = max_w * math.sin(math.radians(steering_angle))
        else:
            # 조향각이 0일 때 각속도는 0, ±180도일 때 각속도는 ±1.5 rad/s
            robot_w = max_w * (steering_angle / 180.0)
            
            # 각속도는 로봇의 속도에 비례하게 조정
            robot_w = robot_w * (self.robot_v / 0.2)
        
        return robot_w

    def calculate_robot_v(self, depth):
        # depth가 0.5m 이상 4m 이하일 때, 이를 0부터 1.0까지 선형 변환
        min_depth = 1.5
        max_depth = 4.0
        if depth < min_depth:
            return 0.0
        
        normalized_depth = min(max(depth - min_depth, 0), max_depth - min_depth)
        robot_v = (normalized_depth / (max_depth - min_depth)) * 0.2
        print(f"depth : {depth}")
        return robot_v


def main(args=None):
    rclpy.init(args=args)
    depth_subscriber = DepthSubscriber()
    rclpy.spin(depth_subscriber)

    depth_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
