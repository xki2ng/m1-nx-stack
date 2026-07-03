#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class VelSignCorrector(Node):
    def __init__(self):
        super().__init__('m1_vel_sign_corrector')
        self.sub = self.create_subscription(Twist, '/cmd_vel_nav', self.cb, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('Sign corrector: /cmd_vel_nav -> /cmd_vel (linear.x inverted only)')

    def cb(self, msg):
        out = Twist()
        out.linear.x = -msg.linear.x     # M1 SDK: positive = backward
        out.linear.y = -msg.linear.y
        out.linear.z = -msg.linear.z
        out.angular.x = msg.angular.x    # angular unchanged
        out.angular.y = msg.angular.y
        out.angular.z = msg.angular.z
        self.pub.publish(out)

def main():
    rclpy.init()
    rclpy.spin(VelSignCorrector())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
