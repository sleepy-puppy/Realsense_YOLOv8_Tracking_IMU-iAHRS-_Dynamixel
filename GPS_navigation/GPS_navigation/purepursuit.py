#pure pursuit and ackerman steering

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import math

class PurePursuit(Node):

    def __init__(self):
        super().__init__('pure_pursuit')
        self.subscription_gps = self.create_subscription(Float32MultiArray, 'from_zedf9p_gps', self.callback_gps, 10)
        self.subscription_routeservice = self.create_subscription(Float32MultiArray, 'from_openrouteservice', self.callback_routeservice, 10)
        self.current_lat = 0.0
        self.current_lon = 0.0
        self.target_lat = 0.0
        self.target_lon = 0.0
        self.L = 10000  # Example vehicle wheelbase
        self.axle_width = 100
        self.wheel_width = 80
        self.robot_v = 0.0 #로봇 속도 임의로 정하세요오오오오오

    def callback_gps(self, msg):
        self.current_lat = msg.data[0]
        self.current_lon = msg.data[1]
        self.compute_steering_angle()

    def callback_routeservice(self, msg):
        self.target_lat = msg.data[0]
        self.target_lon = msg.data[1]
        self.compute_steering_angle()

    def compute_steering_angle(self):
        delta_lon = self.target_lon - self.current_lon
        delta_lat = self.target_lat - self.current_lat
        Ld = math.sqrt(delta_lon**2 + delta_lat**2)

        if Ld == 0:
            return  # Avoid division by zero

        alpha = math.atan2(delta_lat, delta_lon)
        #theta = atan(2 * self.L * sin(alpha) / Ld)
        r = Ld/(2*math.sin(alpha))
        robot_w = self.robot_v/r

        print("robot_v :", self.robot_v, ", robot_w :", robot_w)


def main(args=None):
    rclpy.init(args=args)
    pure_pursuit = PurePursuit()
    rclpy.spin(pure_pursuit)
    pure_pursuit.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
