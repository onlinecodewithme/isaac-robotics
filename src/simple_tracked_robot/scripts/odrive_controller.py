#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Bool
from sensor_msgs.msg import JointState
import math
import numpy as np

# Attempt to import ODrive - with fallback to simulation mode
try:
    import odrive
    from odrive.enums import *
    ODRIVE_AVAILABLE = True
except ImportError:
    ODRIVE_AVAILABLE = False
    print("ODrive Python package not found. Running in simulation mode.")

class OdriveController(Node):
    def __init__(self):
        super().__init__('odrive_controller')
        
        # Declare parameters
        self.declare_parameter('wheel_radius', 0.15)
        self.declare_parameter('track_width', 0.78)
        self.declare_parameter('max_motor_current', 60.0)
        self.declare_parameter('left_motor_index', 0)
        self.declare_parameter('right_motor_index', 1)
        self.declare_parameter('left_motor_reversed', False)
        self.declare_parameter('right_motor_reversed', True)
        self.declare_parameter('simulation_mode', not ODRIVE_AVAILABLE)
        
        # Get parameters
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.track_width = self.get_parameter('track_width').value
        self.max_motor_current = self.get_parameter('max_motor_current').value
        self.left_motor_index = self.get_parameter('left_motor_index').value
        self.right_motor_index = self.get_parameter('right_motor_index').value
        self.left_motor_reversed = self.get_parameter('left_motor_reversed').value 
        self.right_motor_reversed = self.get_parameter('right_motor_reversed').value
        self.simulation_mode = self.get_parameter('simulation_mode').value
        
        # Current motor states
        self.left_motor_vel = 0.0
        self.right_motor_vel = 0.0
        
        # Initialize ODrive
        self.odrv = None
        if not self.simulation_mode:
            self.initialize_odrive()
        else:
            self.get_logger().info('Running in simulation mode - no actual ODrive control')
        
        # Create subscribers
        self.velocity_subscriber = self.create_subscription(
            Twist,
            'cmd_vel',
            self.velocity_callback,
            10)
            
        # Publishers
        self.joint_state_publisher = self.create_publisher(
            JointState,
            'joint_states',
            10)
            
        # Timer for publishing joint states
        self.timer = self.create_timer(0.05, self.publish_joint_states)
        
        self.get_logger().info('ODrive controller initialized')
    
    def initialize_odrive(self):
        """Initialize connection to ODrive"""
        try:
            self.get_logger().info("Looking for ODrive...")
            self.odrv = odrive.find_any()
            self.get_logger().info(f"Found ODrive: {self.odrv.serial_number}")
            
            # Configure motors if needed
            # This is a simplified version
            for axis_num in [self.left_motor_index, self.right_motor_index]:
                axis = getattr(self.odrv, f"axis{axis_num}")
                
                # Check if the axis is in CLOSED_LOOP_CONTROL mode
                if axis.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
                    axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
                
                # Configure limits
                axis.controller.config.vel_limit = 50.0  # rad/s
                axis.motor.config.current_lim = self.max_motor_current
            
            self.get_logger().info("ODrive initialized successfully")
            return True
        
        except Exception as e:
            self.get_logger().error(f"Failed to initialize ODrive: {e}")
            self.simulation_mode = True
            return False
    
    def velocity_callback(self, msg):
        """Handle incoming velocity commands"""
        # Extract linear and angular velocity from the message
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        # Print debug information
        self.get_logger().info(f"Received velocity command: linear_x={linear_x}, angular_z={angular_z}")
        
        # Convert to wheel velocities using differential drive kinematics
        left_wheel_vel, right_wheel_vel = self.twist_to_wheel_velocities(linear_x, angular_z)
        
        # Update motor velocities
        self.left_motor_vel = left_wheel_vel
        self.right_motor_vel = right_wheel_vel
        
        # Print debug information
        self.get_logger().info(f"Calculated wheel velocities: left={left_wheel_vel}, right={right_wheel_vel}")
        
        # Set motor velocities
        self.set_motor_velocities(left_wheel_vel, right_wheel_vel)
    
    def twist_to_wheel_velocities(self, linear_x, angular_z):
        """Convert linear and angular velocity to wheel velocities"""
        # Differential drive kinematics
        # v_l = (v - ω * L/2) / r
        # v_r = (v + ω * L/2) / r
        # where:
        # v_l, v_r are the left and right wheel velocities (rad/s)
        # v is the linear velocity (m/s)
        # ω is the angular velocity (rad/s)
        # L is the track width (m)
        # r is the wheel radius (m)
        
        left_wheel_linear = linear_x - (angular_z * self.track_width / 2.0)
        right_wheel_linear = linear_x + (angular_z * self.track_width / 2.0)
        
        left_wheel_angular = left_wheel_linear / self.wheel_radius
        right_wheel_angular = right_wheel_linear / self.wheel_radius
        
        # Apply motor direction reversals if needed
        if self.left_motor_reversed:
            left_wheel_angular = -left_wheel_angular
        
        if self.right_motor_reversed:
            right_wheel_angular = -right_wheel_angular
        
        return left_wheel_angular, right_wheel_angular
    
    def set_motor_velocities(self, left_vel, right_vel):
        """Set the motor velocities on the ODrive"""
        if self.simulation_mode:
            # Just log the velocities in simulation mode
            self.get_logger().debug(f"Simulated motor velocities: L={left_vel:.2f}, R={right_vel:.2f}")
            return
        
        # Ensure ODrive is available
        if self.odrv is None:
            self.get_logger().warn("ODrive not connected, can't set velocities")
            return
        
        try:
            # Set velocity of left motor
            left_axis = getattr(self.odrv, f"axis{self.left_motor_index}")
            left_axis.controller.input_vel = float(left_vel)
            
            # Set velocity of right motor
            right_axis = getattr(self.odrv, f"axis{self.right_motor_index}")
            right_axis.controller.input_vel = float(right_vel)
        
        except Exception as e:
            self.get_logger().error(f"Error setting motor velocities: {e}")
    
    def publish_joint_states(self):
        """Publish the current joint states"""
        now = self.get_clock().now().to_msg()
        
        msg = JointState()
        msg.header.stamp = now
        msg.name = ['left_wheel_joint', 'right_wheel_joint']
        
        # In simulation, we just use the commanded velocities
        # In real mode, we would read from the ODrive
        if self.simulation_mode:
            left_pos = 0.0  # We don't track position in simulation
            right_pos = 0.0
            left_vel = self.left_motor_vel
            right_vel = self.right_motor_vel
        else:
            try:
                # Get actual positions and velocities from ODrive
                left_axis = getattr(self.odrv, f"axis{self.left_motor_index}")
                right_axis = getattr(self.odrv, f"axis{self.right_motor_index}")
                
                left_pos = float(left_axis.encoder.pos_estimate)
                right_pos = float(right_axis.encoder.pos_estimate)
                left_vel = float(left_axis.encoder.vel_estimate)
                right_vel = float(right_axis.encoder.vel_estimate)
                
                # Apply motor direction reversals if needed
                if self.left_motor_reversed:
                    left_pos = -left_pos
                    left_vel = -left_vel
                
                if self.right_motor_reversed:
                    right_pos = -right_pos
                    right_vel = -right_vel
                    
            except Exception as e:
                self.get_logger().error(f"Error reading from ODrive: {e}")
                left_pos = 0.0
                right_pos = 0.0
                left_vel = self.left_motor_vel
                right_vel = self.right_motor_vel
        
        # Populate message
        msg.position = [left_pos, right_pos]
        msg.velocity = [left_vel, right_vel]
        msg.effort = [0.0, 0.0]  # We don't track effort
        
        # Publish message
        self.joint_state_publisher.publish(msg)
        

def main(args=None):
    rclpy.init(args=args)
    node = OdriveController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down ODrive controller...")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
