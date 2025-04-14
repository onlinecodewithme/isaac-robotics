#!/usr/bin/env python3

"""
ROS 2 ODrive Differential Drive Controller

This script provides a ROS 2 node that interfaces with an ODrive controller
to operate a differential drive robot. It uses an open-loop control method
that doesn't rely on perfect encoder calibration, making it robust against
calibration issues.

Usage:
    python3 ros2_diff_drive.py [--left_axis 0] [--right_axis 1]
"""

import sys
import time
import argparse
import logging
import threading
import math

try:
    import odrive
    from odrive.enums import *
except ImportError:
    print("Error: ODrive Python library not found.")
    print("Please install it using: pip install odrive")
    sys.exit(1)

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    from std_msgs.msg import Float64, Bool
    HAS_ROS = True
except ImportError:
    print("ROS 2 libraries not found. Running in standalone mode.")
    HAS_ROS = False

# Configure logging
logging.basicConfig(level=logging.INFO, 
                    format='%(asctime)s [%(levelname)s] %(message)s',
                    datefmt='%H:%M:%S')
logger = logging.getLogger("odrive_diff_drive")

# Only define ROS Node if ROS is available
if HAS_ROS:
    class ODriveDiffDriveNode(Node):
        """ROS 2 Node for ODrive differential drive control"""
        
        def __init__(self, left_axis=0, right_axis=1):
            super().__init__('odrive_diff_drive')
            self.controller = ODriveController(left_axis, right_axis)
            self.connected = False
            self.wheel_radius = 0.1  # Default 10cm - adjust for your robot
            self.wheel_separation = 0.5  # Default 50cm - adjust for your robot
            
            # Get parameters from ROS
            self.declare_parameter('wheel_radius', self.wheel_radius)
            self.declare_parameter('wheel_separation', self.wheel_separation)
            self.wheel_radius = self.get_parameter('wheel_radius').value
            self.wheel_separation = self.get_parameter('wheel_separation').value
            
            # Create subscribers
            self.cmd_vel_sub = self.create_subscription(
                Twist, 'cmd_vel', self.cmd_vel_callback, 10)
            
            # Create publishers for motor status
            self.left_vel_pub = self.create_publisher(Float64, 'left_wheel_velocity', 10)
            self.right_vel_pub = self.create_publisher(Float64, 'right_wheel_velocity', 10)
            self.connection_pub = self.create_publisher(Bool, 'odrive_connected', 10)
            
            # Timer for status updates
            self.timer = self.create_timer(0.1, self.timer_callback)
            
            # Try to connect
            self.connect()
            
            self.get_logger().info("ODrive differential drive node initialized")
            
        def connect(self):
            """Connect to ODrive"""
            if self.controller.connect():
                self.connected = True
                self.get_logger().info("Connected to ODrive")
            else:
                self.connected = False
                self.get_logger().error("Failed to connect to ODrive")
                
        def cmd_vel_callback(self, msg):
            """Handle cmd_vel messages"""
            if not self.connected:
                self.get_logger().warning("Ignoring cmd_vel: Not connected to ODrive")
                return
            
            # Convert linear and angular velocity to wheel velocities
            linear_x = msg.linear.x
            angular_z = msg.angular.z
            
            # Calculate wheel velocities (differential drive kinematics)
            left_vel = (linear_x - angular_z * self.wheel_separation / 2) / self.wheel_radius
            right_vel = (linear_x + angular_z * self.wheel_separation / 2) / self.wheel_radius
            
            # Send velocities to ODrive
            if not self.controller.set_velocity(left_vel, right_vel):
                self.connected = False
                self.get_logger().error("Failed to set velocity, connection lost")
        
        def timer_callback(self):
            """Periodic status updates"""
            # Check connection and try to reconnect if needed
            if not self.connected:
                self.connect()
            
            # Publish connection status
            status_msg = Bool()
            status_msg.data = self.connected
            self.connection_pub.publish(status_msg)
            
            # Publish wheel velocities if connected
            if self.connected:
                try:
                    # Get actual velocities if available
                    left_vel = getattr(self.controller.left_axis.encoder, 'vel_estimate', 0.0)
                    right_vel = getattr(self.controller.right_axis.encoder, 'vel_estimate', 0.0)
                    
                    left_msg = Float64()
                    left_msg.data = float(left_vel)
                    self.left_vel_pub.publish(left_msg)
                    
                    right_msg = Float64()
                    right_msg.data = float(right_vel)
                    self.right_vel_pub.publish(right_msg)
                except Exception as e:
                    self.get_logger().error(f"Error publishing velocities: {e}")
                    self.connected = False

class ODriveController:
    """Manages communication with the ODrive controller"""
    
    def __init__(self, left_axis=0, right_axis=1):
        self.left_axis_num = left_axis
        self.right_axis_num = right_axis
        self.left_axis = None
        self.right_axis = None
        self.odrive = None
        self.connected = False
        self.lock = threading.RLock()  # For thread safety
        self.max_current = 10.0  # Maximum current in Amps
        self.watchdog_timeout = 0.5  # Seconds before motor stops if no command
        
    def connect(self, timeout=10):
        """Connect to ODrive with timeout"""
        logger.info("Connecting to ODrive...")
        try:
            self.odrive = odrive.find_any(timeout=timeout)
            logger.info(f"Connected to ODrive {self.odrive.serial_number}")
            
            # Get axis references
            self.left_axis = getattr(self.odrive, f"axis{self.left_axis_num}")
            self.right_axis = getattr(self.odrive, f"axis{self.right_axis_num}")
            self.connected = True
            
            # Configure both motors
            self._configure_motor(self.left_axis, "Left")
            self._configure_motor(self.right_axis, "Right")
            
            logger.info("ODrive initialization complete")
            return True
        except Exception as e:
            logger.error(f"Failed to connect: {str(e)}")
            self.connected = False
            return False
    
    def _configure_motor(self, axis, name):
        """Configure a motor axis for openloop velocity control"""
        with self.lock:
            try:
                # First set to idle state
                axis.requested_state = AXIS_STATE_IDLE
                time.sleep(0.5)
                
                # Clear errors
                if hasattr(axis, 'error'):
                    axis.error = 0
                if hasattr(axis.motor, 'error'):
                    axis.motor.error = 0
                if hasattr(axis.encoder, 'error'):
                    axis.encoder.error = 0
                if hasattr(axis.controller, 'error'):
                    axis.controller.error = 0
                
                # Set parameters for open loop control
                axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
                
                # Set current limit
                axis.motor.config.current_lim = self.max_current
                
                # Set velocity limit (rad/s)
                axis.controller.config.vel_limit = 10.0
                
                # Set input mode to pass-through
                axis.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
                
                # Set up watchdog
                if hasattr(axis.config, 'enable_watchdog'):
                    axis.config.enable_watchdog = True
                    axis.config.watchdog_timeout = self.watchdog_timeout
                
                logger.info(f"{name} motor configured successfully")
                return True
            except Exception as e:
                logger.error(f"Error configuring {name} motor: {str(e)}")
                return False
    
    def set_velocity(self, left_vel, right_vel, enter_control_mode=True):
        """Set velocity for both motors"""
        with self.lock:
            if not self.connected:
                logger.warning("Not connected to ODrive")
                return False
            
            try:
                # Enter closed loop control if needed
                if enter_control_mode:
                    if self.left_axis.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
                        self.left_axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
                    if self.right_axis.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
                        self.right_axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
                    time.sleep(0.01)  # Brief pause for state change
                
                # Set velocities
                self.left_axis.controller.input_vel = float(left_vel)
                self.right_axis.controller.input_vel = float(right_vel)
                
                # Feed watchdog
                if hasattr(self.left_axis, 'watchdog_feed'):
                    self.left_axis.watchdog_feed()
                if hasattr(self.right_axis, 'watchdog_feed'):
                    self.right_axis.watchdog_feed()
                
                return True
            except Exception as e:
                logger.error(f"Error setting velocity: {str(e)}")
                try:
                    # Safety: try to stop
                    self.left_axis.controller.input_vel = 0.0
                    self.right_axis.controller.input_vel = 0.0
                except:
                    pass
                self.connected = False
                return False
                
    def stop(self):
        """Stop motors"""
        with self.lock:
            if not self.connected:
                return
            
            try:
                # Set zero velocity
                self.left_axis.controller.input_vel = 0.0
                self.right_axis.controller.input_vel = 0.0
                
                # If possible, go back to idle mode
                if self.left_axis.current_state != AXIS_STATE_IDLE:
                    self.left_axis.requested_state = AXIS_STATE_IDLE
                if self.right_axis.current_state != AXIS_STATE_IDLE:
                    self.right_axis.requested_state = AXIS_STATE_IDLE
                
                logger.info("Motors stopped")
            except Exception as e:
                logger.error(f"Error stopping motors: {str(e)}")
                self.connected = False
    
    def reconnect(self):
        """Try to reconnect to ODrive"""
        with self.lock:
            self.connected = False
            try:
                self.odrive = None
                self.left_axis = None
                self.right_axis = None
                logger.info("Attempting to reconnect...")
                return self.connect()
            except Exception as e:
                logger.error(f"Reconnection failed: {str(e)}")
                return False

def main(args=None):
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='ODrive ROS 2 Differential Drive')
    parser.add_argument('--left_axis', type=int, default=0, help='Left motor axis (0 or 1)')
    parser.add_argument('--right_axis', type=int, default=1, help='Right motor axis (0 or 1)')
    parser.add_argument('--standalone', action='store_true', help='Run in standalone mode without ROS')
    
    args, remaining = parser.parse_known_args()
    
    if args.standalone or not HAS_ROS:
        # Standalone mode - manual velocity control
        controller = ODriveController(args.left_axis, args.right_axis)
        if not controller.connect():
            print("Failed to connect to ODrive. Exiting.")
            sys.exit(1)
        
        print("\nODrive Differential Drive - Standalone Mode")
        print("-------------------------------------------")
        print("Commands:")
        print("  wasd: Basic movement controls")
        print("  q: Quit")
        
        try:
            # Enter closed loop control
            controller.set_velocity(0, 0, True)
            
            while True:
                cmd = input("\nEnter command (wasd, q to quit): ").lower()
                
                left_vel = 0.0
                right_vel = 0.0
                
                if 'q' in cmd:
                    break
                
                # Basic tank-like control
                if 'w' in cmd:  # Forward
                    left_vel += 2.0
                    right_vel += 2.0
                if 's' in cmd:  # Backward
                    left_vel -= 2.0
                    right_vel -= 2.0
                if 'a' in cmd:  # Left turn
                    left_vel -= 1.0
                    right_vel += 1.0
                if 'd' in cmd:  # Right turn
                    left_vel += 1.0
                    right_vel -= 1.0
                
                print(f"Setting velocity: Left={left_vel:.2f}, Right={right_vel:.2f}")
                controller.set_velocity(left_vel, right_vel, False)
                
                # Wait a moment then stop if not continuous
                time.sleep(0.5)
                if not ('c' in cmd):  # 'c' for continuous
                    print("Stopping...")
                    controller.set_velocity(0, 0, False)
        
        except KeyboardInterrupt:
            print("\nProgram terminated by user")
        finally:
            controller.stop()
            print("Motors stopped")
    
    else:
        # ROS 2 mode
        rclpy.init(args=args)
        node = ODriveDiffDriveNode(args.left_axis, args.right_axis)
        
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            # Clean shutdown
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
