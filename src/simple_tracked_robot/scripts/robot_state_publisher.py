#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
import tf2_ros
import math
from tf2_ros import TransformBroadcaster
import numpy as np
from rclpy.qos import QoSProfile

class RobotStatePublisher(Node):
    def __init__(self):
        super().__init__('robot_state_publisher')
        
        # Declare parameters
        self.declare_parameter('wheel_radius', 0.15)
        self.declare_parameter('track_width', 0.78)
        
        # Get parameters
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.track_width = self.get_parameter('track_width').value
        
        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.left_wheel_pos = 0.0
        self.right_wheel_pos = 0.0
        self.left_wheel_vel = 0.0
        self.right_wheel_vel = 0.0
        
        # Create subscribers
        self.cmd_vel_subscriber = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)
            
        # Publishers
        self.odom_publisher = self.create_publisher(
            Odometry,
            'odom',
            10)
            
        self.joint_state_publisher = self.create_publisher(
            JointState,
            'joint_states',
            10)
            
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timer for publishing state
        self.timer = self.create_timer(0.05, self.publish_state)
        
        self.get_logger().info('Robot state publisher initialized')
    
    def cmd_vel_callback(self, msg):
        """Handle incoming velocity commands"""
        # Extract linear and angular velocity from the message
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        # Convert to wheel velocities using differential drive kinematics
        left_wheel_vel, right_wheel_vel = self.twist_to_wheel_velocities(linear_x, angular_z)
        
        # Update wheel velocities
        self.left_wheel_vel = left_wheel_vel
        self.right_wheel_vel = right_wheel_vel
        
        self.get_logger().debug(f"Wheel velocities: left={left_wheel_vel}, right={right_wheel_vel}")
    
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
        
        return left_wheel_angular, right_wheel_angular
    
    def publish_state(self):
        """Publish the current robot state"""
        now = self.get_clock().now().to_msg()
        
        # Update wheel positions based on velocities
        dt = 0.05  # 50ms update rate
        self.left_wheel_pos += self.left_wheel_vel * dt
        self.right_wheel_pos += self.right_wheel_vel * dt
        
        # Update robot position based on wheel velocities
        # For differential drive:
        # dx = (v_r + v_l) * r * cos(θ) * dt / 2
        # dy = (v_r + v_l) * r * sin(θ) * dt / 2
        # dθ = (v_r - v_l) * r * dt / L
        
        v_left = self.left_wheel_vel * self.wheel_radius
        v_right = self.right_wheel_vel * self.wheel_radius
        
        v = (v_right + v_left) / 2.0  # Linear velocity
        omega = (v_right - v_left) / self.track_width  # Angular velocity
        
        # Update robot position
        self.theta += omega * dt
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        
        # Normalize theta to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # Publish joint states - this is critical for the robot model to move in RViz
        # We need to publish joint states for all joints in the URDF
        joint_state = JointState()
        joint_state.header.stamp = now
        joint_state.name = ['left_wheel_joint', 'right_wheel_joint']
        
        # Reverse the wheel positions if needed based on the URDF
        # In the URDF, the right wheel might be reversed compared to the left wheel
        joint_state.position = [self.left_wheel_pos, -self.right_wheel_pos]  # Note the negative sign for right wheel
        joint_state.velocity = [self.left_wheel_vel, -self.right_wheel_vel]  # Note the negative sign for right wheel
        joint_state.effort = [0.0, 0.0]
        
        self.joint_state_publisher.publish(joint_state)
        
        # Debug info - expanded to show calculations
        self.get_logger().info(f"Publishing joint states: left_pos={self.left_wheel_pos:.2f}, right_pos={-self.right_wheel_pos:.2f}")
        self.get_logger().info(f"Wheel velocities: left_vel={self.left_wheel_vel:.2f}, right_vel={-self.right_wheel_vel:.2f}")
        self.get_logger().info(f"Linear velocity (v): {v:.3f} m/s, Angular velocity (omega): {omega:.3f} rad/s")
        self.get_logger().info(f"Robot position: x={self.x:.2f}, y={self.y:.2f}, theta={self.theta:.2f}")
        self.get_logger().info(f"Publishing TF: odom -> base_footprint ({self.x:.2f}, {self.y:.2f}, {self.theta:.2f})")
        
        # Publish odometry
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        
        # Set position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Set orientation (quaternion from yaw)
        cy = math.cos(self.theta * 0.5)
        sy = math.sin(self.theta * 0.5)
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = sy
        odom.pose.pose.orientation.w = cy
        
        # Set velocity
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = omega
        
        self.odom_publisher.publish(odom)
        
        # Broadcast transform
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = sy
        t.transform.rotation.w = cy
        
        self.tf_broadcaster.sendTransform(t)
        

def main(args=None):
    rclpy.init(args=args)
    node = RobotStatePublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down robot state publisher...")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
