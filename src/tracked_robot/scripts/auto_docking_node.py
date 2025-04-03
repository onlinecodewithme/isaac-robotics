#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float64, String
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger
from tracked_robot_msgs.action import Dock  # Custom action type
import math
import time
import numpy as np
from enum import Enum

class DockingState(Enum):
    IDLE = 0
    SEARCHING = 1
    APPROACHING = 2
    ALIGNING = 3
    FINAL_APPROACH = 4
    DOCKED = 5
    ERROR = 6

class AutoDockingNode(Node):
    def __init__(self):
        super().__init__('auto_docking_node')
        
        # Parameters
        self.declare_parameter('dock_ir_topic', '/dock_ir')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('dock_tolerance', 0.05)  # 5cm
        self.declare_parameter('dock_approach_speed', 0.1)  # m/s
        self.declare_parameter('dock_rotation_speed', 0.3)  # rad/s
        self.declare_parameter('dock_timeout', 60.0)  # seconds
        self.declare_parameter('search_pattern', 'spiral')  # 'spiral' or 'rotate'
        
        # Get parameters
        self.dock_ir_topic = self.get_parameter('dock_ir_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.dock_tolerance = self.get_parameter('dock_tolerance').value
        self.dock_approach_speed = self.get_parameter('dock_approach_speed').value
        self.dock_rotation_speed = self.get_parameter('dock_rotation_speed').value
        self.dock_timeout = self.get_parameter('dock_timeout').value
        self.search_pattern = self.get_parameter('search_pattern').value
        
        # State variables
        self.docking_state = DockingState.IDLE
        self.last_ir_reading = 0.0
        self.last_ir_time = 0.0
        self.docking_start_time = 0.0
        self.dock_position = None
        self.search_start_time = 0.0
        self.search_angle = 0.0
        self.search_radius = 0.0
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.state_pub = self.create_publisher(String, 'dock/state', 10)
        
        # Subscribers
        self.ir_sub = self.create_subscription(
            Range,
            self.dock_ir_topic,
            self.ir_callback,
            10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )
        
        # Services
        self.start_docking_srv = self.create_service(
            Trigger, 
            'dock/start', 
            self.start_docking_callback
        )
        
        self.stop_docking_srv = self.create_service(
            Trigger, 
            'dock/stop', 
            self.stop_docking_callback
        )
        
        # Action server for docking
        self._action_server = ActionServer(
            self,
            Dock,
            'dock',
            self.execute_docking,
            callback_group=ReentrantCallbackGroup()
        )
        
        # Timers
        self.create_timer(0.1, self.docking_control_loop)
        self.create_timer(1.0, self.publish_state)
        
        self.get_logger().info('Auto-docking node initialized!')
    
    def ir_callback(self, msg):
        """Handle IR sensor readings"""
        self.last_ir_reading = msg.range
        self.last_ir_time = self.get_clock().now().to_msg().sec
        
        # Log when we first detect the dock
        if self.docking_state == DockingState.SEARCHING and msg.range < 1.5:
            self.get_logger().info(f'Dock detected at range: {msg.range}m')
            self.docking_state = DockingState.APPROACHING
    
    def odom_callback(self, msg):
        """Handle odometry data for position tracking"""
        # Store current position for search patterns
        self.current_pose = msg.pose.pose
    
    def start_docking_callback(self, request, response):
        """Service callback to start docking"""
        if self.docking_state == DockingState.IDLE:
            self.start_docking()
            response.success = True
            response.message = "Docking sequence initiated"
        else:
            response.success = False
            response.message = f"Cannot start docking, current state: {self.docking_state.name}"
        return response
    
    def stop_docking_callback(self, request, response):
        """Service callback to stop docking"""
        if self.docking_state != DockingState.IDLE and self.docking_state != DockingState.DOCKED:
            self.stop_robot()
            self.docking_state = DockingState.IDLE
            response.success = True
            response.message = "Docking sequence aborted"
        else:
            response.success = False
            response.message = f"Not in active docking state, current state: {self.docking_state.name}"
        return response
    
    def execute_docking(self, goal_handle):
        """Action server callback for docking"""
        self.get_logger().info('Executing docking action')
        
        feedback_msg = Dock.Feedback()
        result = Dock.Result()
        
        # Start docking process
        if self.docking_state != DockingState.IDLE:
            goal_handle.abort()
            return result
        
        self.start_docking()
        
        # Monitor docking progress
        rate = self.create_rate(10)  # 10Hz
        while self.docking_state not in [DockingState.DOCKED, DockingState.ERROR, DockingState.IDLE]:
            # Check if the goal was canceled
            if goal_handle.is_cancel_requested:
                self.stop_robot()
                self.docking_state = DockingState.IDLE
                goal_handle.canceled()
                result.success = False
                result.message = "Docking canceled"
                return result
            
            # Publish feedback
            feedback_msg.current_state = self.docking_state.name
            feedback_msg.distance_to_dock = self.last_ir_reading
            goal_handle.publish_feedback(feedback_msg)
            
            # Check for timeout
            current_time = self.get_clock().now().to_msg().sec
            if current_time - self.docking_start_time > self.dock_timeout:
                self.get_logger().error('Docking timed out')
                self.stop_robot()
                self.docking_state = DockingState.ERROR
            
            rate.sleep()
        
        # Set result based on final state
        if self.docking_state == DockingState.DOCKED:
            goal_handle.succeed()
            result.success = True
            result.message = "Successfully docked"
        else:
            goal_handle.abort()
            result.success = False
            result.message = f"Docking failed, final state: {self.docking_state.name}"
        
        return result
    
    def start_docking(self):
        """Start the docking process"""
        self.get_logger().info('Starting docking sequence')
        self.docking_state = DockingState.SEARCHING
        self.docking_start_time = self.get_clock().now().to_msg().sec
        self.search_start_time = self.docking_start_time
        self.search_angle = 0.0
        self.search_radius = 0.0
    
    def docking_control_loop(self):
        """Main control loop for docking state machine"""
        if self.docking_state == DockingState.IDLE or self.docking_state == DockingState.DOCKED:
            return
        
        # Create command velocity message
        cmd_vel = Twist()
        
        # State machine
        if self.docking_state == DockingState.SEARCHING:
            self.execute_search_pattern(cmd_vel)
        
        elif self.docking_state == DockingState.APPROACHING:
            # Move towards dock if IR sensor is detecting it
            ir_age = self.get_clock().now().to_msg().sec - self.last_ir_time
            
            if ir_age > 1.0:  # Lost signal for more than 1 second
                self.get_logger().warn('Lost dock signal, returning to search')
                self.docking_state = DockingState.SEARCHING
                self.search_start_time = self.get_clock().now().to_msg().sec
            else:
                # Approach at appropriate speed based on distance
                if self.last_ir_reading > 0.5:
                    # Far approach - faster
                    cmd_vel.linear.x = self.dock_approach_speed
                else:
                    # Close approach - slower
                    cmd_vel.linear.x = self.dock_approach_speed * 0.5
                
                # If very close, switch to alignment
                if self.last_ir_reading < 0.2:
                    self.get_logger().info('Close to dock, switching to alignment')
                    self.docking_state = DockingState.ALIGNING
        
        elif self.docking_state == DockingState.ALIGNING:
            # Fine-tune alignment before final approach
            # This is simplified - in a real system, you'd use multiple IR sensors for alignment
            cmd_vel.angular.z = self.dock_rotation_speed * 0.3  # Small rotation to check alignment
            
            # After a brief alignment period, move to final approach
            if self.get_clock().now().to_msg().sec - self.last_ir_time > 2.0:
                self.get_logger().info('Alignment complete, proceeding to final approach')
                self.docking_state = DockingState.FINAL_APPROACH
        
        elif self.docking_state == DockingState.FINAL_APPROACH:
            # Final slow approach to dock
            if self.last_ir_reading > self.dock_tolerance:
                cmd_vel.linear.x = self.dock_approach_speed * 0.3  # Very slow
            else:
                # We've reached the dock
                self.get_logger().info('Docking complete!')
                self.docking_state = DockingState.DOCKED
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = 0.0
        
        elif self.docking_state == DockingState.ERROR:
            # Handle error state - stop moving
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
        
        # Publish command velocity
        self.cmd_vel_pub.publish(cmd_vel)
    
    def execute_search_pattern(self, cmd_vel):
        """Execute search pattern to find the dock"""
        current_time = self.get_clock().now().to_msg().sec
        elapsed = current_time - self.search_start_time
        
        if self.search_pattern == 'rotate':
            # Simple rotation search
            cmd_vel.angular.z = self.dock_rotation_speed
            
            # If we've done a full rotation and not found the dock, try moving a bit
            if elapsed > (2 * math.pi / self.dock_rotation_speed):
                cmd_vel.angular.z = 0.0
                cmd_vel.linear.x = -0.1  # Back up slightly
                
                # Reset the search timer after backing up for a second
                if elapsed > (2 * math.pi / self.dock_rotation_speed) + 1.0:
                    self.search_start_time = current_time
        
        elif self.search_pattern == 'spiral':
            # Spiral search pattern
            self.search_angle += 0.1
            self.search_radius = 0.05 * elapsed
            
            # Limit the search radius
            if self.search_radius > 2.0:
                self.search_radius = 0.0
                self.search_angle = 0.0
                self.search_start_time = current_time
            
            # Calculate velocity commands for spiral motion
            cmd_vel.linear.x = 0.1 * math.cos(self.search_angle)
            cmd_vel.linear.y = 0.1 * math.sin(self.search_angle)
            cmd_vel.angular.z = 0.2
    
    def stop_robot(self):
        """Stop the robot immediately"""
        cmd_vel = Twist()
        self.cmd_vel_pub.publish(cmd_vel)
    
    def publish_state(self):
        """Publish current docking state"""
        msg = String()
        msg.data = self.docking_state.name
        self.state_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    
    auto_docking_node = AutoDockingNode()
    
    # Use a multi-threaded executor to handle the action server callbacks
    executor = MultiThreadedExecutor()
    executor.add_node(auto_docking_node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        auto_docking_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
