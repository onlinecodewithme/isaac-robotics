#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray, String
from sensor_msgs.msg import JointState
import math
import time
import odrive
from odrive.enums import *
import numpy as np
import fibre.libfibre

class ODriveControlNode(Node):
    def __init__(self):
        super().__init__('odrive_control_node')
        
        # Parameters
        self.declare_parameter('wheel_radius', 0.15)  # meters
        self.declare_parameter('track_width', 0.78)   # meters
        self.declare_parameter('gear_ratio', 1.0)     # Direct drive
        self.declare_parameter('encoder_cpr', 8192)   # Counts per revolution
        self.declare_parameter('max_motor_current', 60.0)  # Amps
        self.declare_parameter('control_mode', 'velocity')  # 'velocity' or 'position'
        self.declare_parameter('left_motor_index', 0)
        self.declare_parameter('right_motor_index', 1)
        self.declare_parameter('left_motor_reversed', False)
        self.declare_parameter('right_motor_reversed', True)
        
        # Get parameters
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.track_width = self.get_parameter('track_width').value
        self.gear_ratio = self.get_parameter('gear_ratio').value
        self.encoder_cpr = self.get_parameter('encoder_cpr').value
        self.max_motor_current = self.get_parameter('max_motor_current').value
        self.control_mode = self.get_parameter('control_mode').value
        self.left_motor_index = self.get_parameter('left_motor_index').value
        self.right_motor_index = self.get_parameter('right_motor_index').value
        self.left_motor_reversed = self.get_parameter('left_motor_reversed').value
        self.right_motor_reversed = self.get_parameter('right_motor_reversed').value
        
        # Sign multipliers for motor direction
        self.left_direction = -1.0 if self.left_motor_reversed else 1.0
        self.right_direction = -1.0 if self.right_motor_reversed else 1.0
        
        # Publishers
        self.joint_states_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.status_pub = self.create_publisher(String, 'odrive/status', 10)
        
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
            
        # Initialize odrive
        self.get_logger().info("Connecting to ODrive...")
        self.odrive_connect()
        
        # Create timers
        self.create_timer(0.01, self.publish_joint_states)
        self.create_timer(1.0, self.publish_status)
        
        self.get_logger().info("ODrive control node initialized!")
    
    def odrive_connect(self):
        try:
            self.odrive = odrive.find_any()
            self.get_logger().info(f"Connected to ODrive {self.odrive.serial_number}")
            
            # Configure motors
            self.configure_motors()
            return True
        except fibre.libfibre.ObjectLostError:
            self.get_logger().error("Failed to connect to ODrive. Is it powered and connected via USB?")
            time.sleep(2)
            return False
    
    def configure_motors(self):
        # Configure left motor
        self.get_logger().info("Configuring left motor...")
        self.left_motor = getattr(self.odrive, f"axis{self.left_motor_index}")
        
        # Configure right motor
        self.get_logger().info("Configuring right motor...")
        self.right_motor = getattr(self.odrive, f"axis{self.right_motor_index}")
        
        # Configure both motors (assuming they're already calibrated)
        for motor_name, motor in [("Left", self.left_motor), ("Right", self.right_motor)]:
            if motor.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
                self.get_logger().info(f"Setting {motor_name} motor to closed loop control mode...")
                motor.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
                
            # Set control mode
            if self.control_mode == 'velocity':
                motor.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
            else:
                motor.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
            
            # Set current limit
            motor.motor.config.current_lim = self.max_motor_current
        
        self.get_logger().info("Motors configured successfully!")
    
    def cmd_vel_callback(self, msg):
        # Convert Twist to wheel velocities
        left_wheel_vel, right_wheel_vel = self.twist_to_wheel_velocities(msg)
        
        try:
            # Send commands to ODrive (adjust for motor direction)
            if self.control_mode == 'velocity':
                self.left_motor.controller.input_vel = self.left_direction * left_wheel_vel
                self.right_motor.controller.input_vel = self.right_direction * right_wheel_vel
            else:
                # For position control, integrate the velocity (this is just for demonstration)
                current_left_pos = self.left_motor.encoder.pos_estimate
                current_right_pos = self.right_motor.encoder.pos_estimate
                
                # Calculate new position targets (simple integration)
                dt = 0.1  # Assume 100ms for simplicity
                left_pos_increment = self.left_direction * left_wheel_vel * dt
                right_pos_increment = self.right_direction * right_wheel_vel * dt
                
                self.left_motor.controller.input_pos = current_left_pos + left_pos_increment
                self.right_motor.controller.input_pos = current_right_pos + right_pos_increment
        
        except Exception as e:
            self.get_logger().error(f"Error sending commands to ODrive: {str(e)}")
            # Try to reconnect if we lost connection
            self.odrive_connect()
    
    def twist_to_wheel_velocities(self, twist):
        """Convert a Twist message to wheel velocities in turn/s"""
        # Extract linear and angular velocities
        linear_x = twist.linear.x
        angular_z = twist.angular.z
        
        # Differential drive kinematics
        left_linear_vel = linear_x - (angular_z * self.track_width / 2.0)
        right_linear_vel = linear_x + (angular_z * self.track_width / 2.0)
        
        # Convert to rotational velocity (rad/s)
        left_angular_vel = left_linear_vel / self.wheel_radius
        right_angular_vel = right_linear_vel / self.wheel_radius
        
        # Convert to turns/s for ODrive
        left_turns_per_sec = left_angular_vel / (2 * math.pi)
        right_turns_per_sec = right_angular_vel / (2 * math.pi)
        
        # Apply gear ratio if needed
        left_turns_per_sec = left_turns_per_sec * self.gear_ratio
        right_turns_per_sec = right_turns_per_sec * self.gear_ratio
        
        return left_turns_per_sec, right_turns_per_sec
    
    def publish_joint_states(self):
        """Publish joint states from encoder feedback"""
        try:
            # Get current encoder positions and velocities
            left_pos = self.left_motor.encoder.pos_estimate
            right_pos = self.right_motor.encoder.pos_estimate
            left_vel = self.left_motor.encoder.vel_estimate
            right_vel = self.right_motor.encoder.vel_estimate
            
            # Apply direction correction
            left_pos *= self.left_direction
            right_pos *= self.right_direction
            left_vel *= self.left_direction
            right_vel *= self.right_direction
            
            # Convert from turns to radians
            left_pos_rad = left_pos * 2 * math.pi / self.gear_ratio
            right_pos_rad = right_pos * 2 * math.pi / self.gear_ratio
            left_vel_rad = left_vel * 2 * math.pi / self.gear_ratio
            right_vel_rad = right_vel * 2 * math.pi / self.gear_ratio
            
            # Create JointState message
            joint_state_msg = JointState()
            joint_state_msg.header.stamp = self.get_clock().now().to_msg()
            joint_state_msg.name = ['left_wheel_joint', 'right_wheel_joint']
            joint_state_msg.position = [float(left_pos_rad), float(right_pos_rad)]
            joint_state_msg.velocity = [float(left_vel_rad), float(right_vel_rad)]
            joint_state_msg.effort = [float(self.left_motor.motor.current_control.Iq_measured), 
                                      float(self.right_motor.motor.current_control.Iq_measured)]
            
            # Publish message
            self.joint_states_pub.publish(joint_state_msg)
            
        except Exception as e:
            self.get_logger().warn(f"Error publishing joint states: {str(e)}")
    
    def publish_status(self):
        """Publish ODrive status information"""
        try:
            # Get temperatures, currents, voltages
            left_temp = self.left_motor.motor.get_inverter_temp()
            right_temp = self.right_motor.motor.get_inverter_temp()
            bus_voltage = self.odrive.vbus_voltage
            
            # Create status message
            status_msg = String()
            status_msg.data = (
                f"ODrive Status:\n"
                f"Bus Voltage: {bus_voltage:.2f}V\n"
                f"Left Motor Temp: {left_temp:.1f}°C\n"
                f"Right Motor Temp: {right_temp:.1f}°C\n"
                f"Left Motor Error: {self.left_motor.error}\n"
                f"Right Motor Error: {self.right_motor.error}"
            )
            
            # Publish message
            self.status_pub.publish(status_msg)
            
        except Exception as e:
            self.get_logger().warn(f"Error publishing ODrive status: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = ODriveControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Ensure safe shutdown
        node.get_logger().info("Shutting down, stopping motors...")
        try:
            node.left_motor.requested_state = AXIS_STATE_IDLE
            node.right_motor.requested_state = AXIS_STATE_IDLE
        except:
            pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
