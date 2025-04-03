#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class VelocityPublisher(Node):
    def __init__(self):
        super().__init__('velocity_publisher')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.count = 0
        self.get_logger().info('Velocity publisher started')

    def timer_callback(self):
        msg = Twist()
        
        # Alternate between different commands
        if self.count % 4 == 0:
            # Forward
            msg.linear.x = 0.5
            msg.angular.z = 0.0
            self.get_logger().info('Publishing: Forward')
        elif self.count % 4 == 1:
            # Turn left
            msg.linear.x = 0.0
            msg.angular.z = 0.5
            self.get_logger().info('Publishing: Turn left')
        elif self.count % 4 == 2:
            # Backward
            msg.linear.x = -0.5
            msg.angular.z = 0.0
            self.get_logger().info('Publishing: Backward')
        else:
            # Turn right
            msg.linear.x = 0.0
            msg.angular.z = -0.5
            self.get_logger().info('Publishing: Turn right')
        
        self.publisher_.publish(msg)
        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    velocity_publisher = VelocityPublisher()
    
    try:
        rclpy.spin(velocity_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        velocity_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
