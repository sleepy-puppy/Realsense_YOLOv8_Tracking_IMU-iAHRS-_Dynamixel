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
        self.robot_v = 0.0 
        self.robot_w = 0.0
        self.wheelbase = 0.84  # 로봇의 휠베이스(L)를 가정 (단위: meter)
        self.current_yaw = None
        self.current_depth = None  # depth 값을 저장하는 변수

        # 데이터가 수신되지 않았을 때 속도를 0으로 설정하기 위한 타이머
        self.timer = self.create_timer(1.0, self.check_yaw_timeout)  # 1초마다 타이머 호출
        self.last_yaw_time = self.get_clock().now()  # 마지막으로 yaw 데이터를 받은 시간
        
    def target_person_depth_listener_callback(self, msg):
        person_id = int(msg.data[0])
        depth = msg.data[1]

        # depth 값을 저장
        self.current_depth = depth
        print(f"ID {person_id}: depth 값 수신됨: {depth}")
        
    def yaw_callback(self, msg):
        # 유효한 yaw 데이터를 확인하고 None으로 설정
        if msg is not None and isinstance(msg.data, int):
            self.current_yaw = msg.data
            self.last_yaw_time = self.get_clock().now()
        else:
            self.current_yaw = None

        # yaw 데이터가 있을 때만 계산 수행
        if self.current_yaw is not None and self.current_depth is not None:
            # 깊이에 따른 속도 계산
            self.robot_v = self.calculate_robot_v(self.current_depth)
            self.robot_w = self.calculate_robot_w(self.current_yaw)
        
            print(f"yaw: {self.current_yaw}, depth: {self.current_depth}")
            print(f"self.robot_w = {self.robot_w:.2f}, robot_v = {self.robot_v:.2f}")

            # Twist 메시지 발행
            twist_msg = Twist()
            twist_msg.linear = Vector3(x=self.robot_v, y=0.0, z=0.0)
            twist_msg.angular = Vector3(x=0.0, y=0.0, z=self.robot_w)
            self.publisher_.publish(twist_msg)

    def calculate_robot_w(self, yaw):
        # yaw 값을 통해 조향각 계산 (단위: degree)
        steering_angle = 360 * (yaw - 2048) / 4096
        
        # 조향각을 -180도에서 180도로 제한
        steering_angle = max(min(steering_angle, 180), -180)
        print(f"steering angle : {steering_angle}")

        if abs(steering_angle) <= 10:
            return 0.0

        max_w = 1.5  # 각속도의 최대값
        
        if self.robot_v == 0.0:
            # 조향각이 180도에 가까우면 -1.5에, -180도에 가까우면 +1.5에 가까워지도록 설정
            self.robot_w = max_w * math.sin(math.radians(steering_angle))
        else:
            # 조향각이 0일 때 각속도는 0, ±180도일 때 각속도는 ±1.5 rad/s
            self.robot_w = max_w * (steering_angle / 180.0)
            
            # 각속도는 로봇의 속도에 비례하게 조정
            self.robot_w = self.robot_w * (self.robot_v / 0.5)
        
        return self.robot_w

    def calculate_robot_v(self, depth):
        # depth가 0.5m 이상 4m 이하일 때, 이를 0부터 1.0까지 선형 변환
        min_depth = 0.5
        max_depth = 4.0
        if depth < min_depth:
            return 0.0
        normalized_depth = min(max(depth - min_depth, 0), max_depth - min_depth)
        robot_v = (normalized_depth / (max_depth - min_depth)) * 0.3
        return robot_v

    def check_yaw_timeout(self):
        # 마지막 yaw 데이터를 수신한 시점에서 1초 이상 경과했는지 확인
        current_time = self.get_clock().now()
        time_diff = current_time - self.last_yaw_time
        if time_diff.nanoseconds > 1e9:  # 1초 이상 경과했을 때
            print("yaw 데이터 수신이 없으므로 속도와 각속도를 0으로 설정")
            twist_msg = Twist()
            twist_msg.linear = Vector3(x=0.0, y=0.0, z=0.0)
            twist_msg.angular = Vector3(x=0.0, y=0.0, z=0.0)
            self.publisher_.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    depth_subscriber = DepthSubscriber()
    rclpy.spin(depth_subscriber)

    depth_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
