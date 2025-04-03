#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select
import tty
import termios
import threading

msg = """
Control Your Tracked Robot!
---------------------------
Moving around:
   q    w    e
   a    s    d
   z    x    c

w/x : increase/decrease linear velocity
a/d : increase/decrease angular velocity
q/e : increase/decrease linear and angular velocity
s : stop
z/c : turn left/right

CTRL-C to quit
"""

moveBindings = {
    'w': (1, 0),
    'e': (1, -1),
    'a': (0, 1),
    'd': (0, -1),
    'q': (1, 1),
    'x': (-1, 0),
    'c': (-1, -1),
    'z': (-1, 1),
    's': (0, 0),
}

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz
        
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.target_linear = 0.0
        self.target_angular = 0.0
        self.linear_increment = 0.05  # m/s
        self.angular_increment = 0.1  # rad/s
        self.max_linear = 1.0  # m/s
        self.max_angular = 1.0  # rad/s
        
        self.key_thread = threading.Thread(target=self.get_key)
        self.key_thread.daemon = True
        self.key_thread.start()
        
        self.get_logger().info(msg)
    
    def get_key(self):
        # Save terminal settings
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            
            while rclpy.ok():
                # Check if key has been pressed
                if select.select([sys.stdin], [], [], 0)[0]:
                    key = sys.stdin.read(1)
                    
                    # Check for Ctrl+C
                    if key == '\x03':
                        self.get_logger().info("Keyboard interrupt received, shutting down...")
                        break
                    
                    # Process key command
                    if key in moveBindings.keys():
                        move = moveBindings[key]
                        # Update target velocities
                        if move == (0, 0):  # 's' key - stop
                            self.target_linear = 0.0
                            self.target_angular = 0.0
                        else:
                            # Adjust target velocities based on key press
                            if move[0] != 0:  # Linear component
                                new_linear = self.target_linear + move[0] * self.linear_increment
                                self.target_linear = max(-self.max_linear, min(self.max_linear, new_linear))
                                
                            if move[1] != 0:  # Angular component
                                new_angular = self.target_angular + move[1] * self.angular_increment
                                self.target_angular = max(-self.max_angular, min(self.max_angular, new_angular))
                        
                        self.get_logger().info(f"Linear: {self.target_linear:.2f}, Angular: {self.target_angular:.2f}")
                    
        finally:
            # Restore terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
    
    def timer_callback(self):
        # Smooth acceleration toward target velocities
        # This provides more natural control and prevents sudden jerky movements
        accel_rate = 0.1  # Determines how quickly we reach target velocity
        
        # Gradually adjust current speeds toward target speeds
        if abs(self.linear_speed - self.target_linear) > 0.01:
            self.linear_speed += (self.target_linear - self.linear_speed) * accel_rate
        else:
            self.linear_speed = self.target_linear
            
        if abs(self.angular_speed - self.target_angular) > 0.01:
            self.angular_speed += (self.target_angular - self.angular_speed) * accel_rate
        else:
            self.angular_speed = self.target_angular
        
        # Create and publish Twist message
        twist = Twist()
        twist.linear.x = self.linear_speed
        twist.angular.z = self.angular_speed
        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    teleop = KeyboardTeleop()
    
    try:
        rclpy.spin(teleop)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the robot
        twist = Twist()
        teleop.publisher.publish(twist)
        teleop.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
