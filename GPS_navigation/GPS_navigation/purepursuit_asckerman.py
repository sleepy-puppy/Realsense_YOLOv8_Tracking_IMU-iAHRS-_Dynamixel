#pure pursuit and ackerman steering

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from math import atan, sin, atan2, sqrt

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
        self.robot_v = 0.0

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
        Ld = sqrt(delta_lon**2 + delta_lat**2)

        if Ld == 0:
            return  # Avoid division by zero

        alpha = atan2(delta_lat, delta_lon)
        theta = atan(2 * self.L * sin(alpha) / Ld)
        r = Ld/(2*sin(alpha))

        self.get_logger().info(f'Steering angle (theta): {theta}')

    def ackerman_steering(self, r):
        if r == 0:
            r_inter = float('inf')
            r_outer = float('inf')
        else:
            robot_w = self.robot_v/r

            r_inter = r - self.axle_width/2
            r_outer = r + self.axle_width/2
            theta_inter = atan(self.wheel_width/r_inter)
            theta_outer = atan(self.wheel_width/r_outer)

            v_inter = r_inter * robot_w
            v_outer = r_outer * robot_w

            print("theta inter :", theta_inter, " theta outer :", theta_outer, " v inter :", v_inter, " v outer :", v_outer)


def main(args=None):
    rclpy.init(args=args)
    pure_pursuit = PurePursuit()
    rclpy.spin(pure_pursuit)
    pure_pursuit.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
