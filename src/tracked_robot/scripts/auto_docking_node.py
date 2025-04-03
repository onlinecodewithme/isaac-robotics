#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float64, String, Bool
from sensor_msgs.msg import Range, Image, PointCloud2, CameraInfo
from geometry_msgs.msg import Twist, PoseStamped, Pose, Point, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger
from tracked_robot_msgs.action import Dock  # Custom action type
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
import math
import time
import numpy as np
from enum import Enum
import threading
import cv2
from cv_bridge import CvBridge, CvBridgeError
import tf2_geometry_msgs
import tf2_ros
from visualization_msgs.msg import Marker, MarkerArray

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
        self.declare_parameter('use_zed_camera', True)  # Use ZED camera for dock detection
        self.declare_parameter('dock_marker_size', 0.15)  # Size of the ArUco marker on dock (meters)
        self.declare_parameter('aruco_dictionary', 'DICT_5X5_100')  # ArUco dictionary
        self.declare_parameter('dock_marker_id', 42)  # ArUco marker ID for dock
        
        # Get parameters
        self.dock_ir_topic = self.get_parameter('dock_ir_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.dock_tolerance = self.get_parameter('dock_tolerance').value
        self.dock_approach_speed = self.get_parameter('dock_approach_speed').value
        self.dock_rotation_speed = self.get_parameter('dock_rotation_speed').value
        self.dock_timeout = self.get_parameter('dock_timeout').value
        self.search_pattern = self.get_parameter('search_pattern').value
        self.use_zed_camera = self.get_parameter('use_zed_camera').value
        self.dock_marker_size = self.get_parameter('dock_marker_size').value
        self.aruco_dict_name = self.get_parameter('aruco_dictionary').value
        self.dock_marker_id = self.get_parameter('dock_marker_id').value
        
        # Initialize ArUco detector if ZED camera is used
        if self.use_zed_camera:
            self.bridge = CvBridge()
            # Set up ArUco detector
            self.aruco_dict = getattr(cv2.aruco, self.aruco_dict_name)
            self.aruco_params = cv2.aruco.DetectorParameters()
            self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
            # Camera matrix and distortion coefficients - will be updated from camera info
            self.camera_matrix = None
            self.dist_coeffs = np.zeros((5, 1), dtype=np.float32)
        
        # State variables
        self.docking_state = DockingState.IDLE
        self.last_ir_reading = 0.0
        self.last_ir_time = 0.0
        self.docking_start_time = 0.0
        self.dock_position = None
        self.search_start_time = 0.0
        self.search_angle = 0.0
        self.search_radius = 0.0
        self.dock_visible = False
        self.dock_pose = None
        self.dock_distance = 0.0
        self.dock_angle = 0.0
        self.dock_detection_count = 0
        self.dock_detection_threshold = 5  # Number of detections needed to confirm dock
        self.state_lock = threading.Lock()
        
        # TF2 for transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.state_pub = self.create_publisher(String, 'dock/state', 10)
        self.dock_marker_pub = self.create_publisher(MarkerArray, 'dock/markers', 10)
        self.dock_visible_pub = self.create_publisher(Bool, 'dock/visible', 10)
        
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
        
        # ZED camera subscribers if enabled
        if self.use_zed_camera:
            self.image_sub = self.create_subscription(
                Image,
                '/zed2i/rgb/image_rect_color',
                self.image_callback,
                10
            )
            
            self.camera_info_sub = self.create_subscription(
                CameraInfo,
                '/zed2i/rgb/camera_info',
                self.camera_info_callback,
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
    
    def camera_info_callback(self, msg):
        """Process camera info to get calibration data"""
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.get_logger().info('Camera calibration data received')
    
    def image_callback(self, msg):
        """Process ZED camera images for ArUco marker detection"""
        if not self.use_zed_camera or self.camera_matrix is None:
            return
            
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Detect ArUco markers
            corners, ids, rejected = self.aruco_detector.detectMarkers(cv_image)
            
            # Check if our dock marker was detected
            if ids is not None and self.dock_marker_id in ids:
                marker_idx = np.where(ids == self.dock_marker_id)[0][0]
                marker_corners = corners[marker_idx][0]
                
                # Estimate pose of the marker
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    [marker_corners], 
                    self.dock_marker_size, 
                    self.camera_matrix, 
                    self.dist_coeffs
                )
                
                # Extract position and orientation from the pose
                rvec = rvecs[0][0]
                tvec = tvecs[0][0]
                
                # Convert rotation vector to rotation matrix
                rotation_matrix, _ = cv2.Rodrigues(rvec)
                
                # Create transformation matrix
                transform_matrix = np.eye(4)
                transform_matrix[:3, :3] = rotation_matrix
                transform_matrix[:3, 3] = tvec
                
                # Calculate distance to marker (using Z component of translation vector)
                self.dock_distance = tvec[2]
                
                # Calculate angle to marker (approximate)
                self.dock_angle = math.atan2(tvec[0], tvec[2])
                
                # Increment detection counter
                self.dock_detection_count += 1
                
                # If we've seen the dock enough times, consider it confirmed
                if self.dock_detection_count >= self.dock_detection_threshold:
                    self.dock_visible = True
                    
                    # Publish dock transform
                    self.publish_dock_transform(transform_matrix, msg.header.stamp)
                    
                    # Publish marker for visualization
                    self.publish_dock_marker(tvec, rvec, msg.header.stamp)
                
                # If we're in searching state and dock is visible, transition to approaching
                if self.docking_state == DockingState.SEARCHING and self.dock_visible:
                    with self.state_lock:
                        self.get_logger().info(f'Dock marker detected at distance: {self.dock_distance:.2f}m, angle: {math.degrees(self.dock_angle):.1f} degrees')
                        self.docking_state = DockingState.APPROACHING
            else:
                # Reset detection counter if marker not seen
                self.dock_detection_count = max(0, self.dock_detection_count - 1)
                
                # If counter drops below threshold, dock is no longer considered visible
                if self.dock_detection_count < self.dock_detection_threshold:
                    self.dock_visible = False
                    
            # Publish dock visibility status
            visibility_msg = Bool()
            visibility_msg.data = self.dock_visible
            self.dock_visible_pub.publish(visibility_msg)
                
        except CvBridgeError as e:
            self.get_logger().error(f'CV Bridge error: {str(e)}')
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')
    
    def publish_dock_transform(self, transform_matrix, timestamp):
        """Publish the dock transform to TF tree"""
        try:
            # Extract translation and rotation
            translation = transform_matrix[:3, 3]
            rotation_matrix = transform_matrix[:3, :3]
            
            # Convert rotation matrix to quaternion
            quat = self.rotation_matrix_to_quaternion(rotation_matrix)
            
            # Create transform message
            transform_msg = TransformStamped()
            transform_msg.header.stamp = timestamp
            transform_msg.header.frame_id = 'zed2i_left_camera_optical_frame'
            transform_msg.child_frame_id = 'dock_frame'
            
            # Set translation
            transform_msg.transform.translation.x = translation[0]
            transform_msg.transform.translation.y = translation[1]
            transform_msg.transform.translation.z = translation[2]
            
            # Set rotation
            transform_msg.transform.rotation.x = quat[0]
            transform_msg.transform.rotation.y = quat[1]
            transform_msg.transform.rotation.z = quat[2]
            transform_msg.transform.rotation.w = quat[3]
            
            # Broadcast the transform
            self.tf_broadcaster.sendTransform(transform_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing dock transform: {str(e)}')
    
    def rotation_matrix_to_quaternion(self, rotation_matrix):
        """Convert a 3x3 rotation matrix to quaternion"""
        # Using the Eigen decomposition method
        q = np.zeros(4)
        trace = np.trace(rotation_matrix)
        
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            q[3] = 0.25 / s
            q[0] = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) * s
            q[1] = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) * s
            q[2] = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) * s
        else:
            if rotation_matrix[0, 0] > rotation_matrix[1, 1] and rotation_matrix[0, 0] > rotation_matrix[2, 2]:
                s = 2.0 * np.sqrt(1.0 + rotation_matrix[0, 0] - rotation_matrix[1, 1] - rotation_matrix[2, 2])
                q[3] = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) / s
                q[0] = 0.25 * s
                q[1] = (rotation_matrix[0, 1] + rotation_matrix[1, 0]) / s
                q[2] = (rotation_matrix[0, 2] + rotation_matrix[2, 0]) / s
            elif rotation_matrix[1, 1] > rotation_matrix[2, 2]:
                s = 2.0 * np.sqrt(1.0 + rotation_matrix[1, 1] - rotation_matrix[0, 0] - rotation_matrix[2, 2])
                q[3] = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) / s
                q[0] = (rotation_matrix[0, 1] + rotation_matrix[1, 0]) / s
                q[1] = 0.25 * s
                q[2] = (rotation_matrix[1, 2] + rotation_matrix[2, 1]) / s
            else:
                s = 2.0 * np.sqrt(1.0 + rotation_matrix[2, 2] - rotation_matrix[0, 0] - rotation_matrix[1, 1])
                q[3] = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) / s
                q[0] = (rotation_matrix[0, 2] + rotation_matrix[2, 0]) / s
                q[1] = (rotation_matrix[1, 2] + rotation_matrix[2, 1]) / s
                q[2] = 0.25 * s
        
        return q
    
    def publish_dock_marker(self, position, orientation, timestamp):
        """Publish marker for visualization in RViz"""
        marker_array = MarkerArray()
        
        # Create marker for the dock
        marker = Marker()
        marker.header.frame_id = 'zed2i_left_camera_optical_frame'
        marker.header.stamp = timestamp
        marker.ns = 'dock'
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        
        # Set position
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2]
        
        # Convert rotation vector to quaternion
        rotation_matrix, _ = cv2.Rodrigues(orientation)
        quat = self.rotation_matrix_to_quaternion(rotation_matrix)
        marker.pose.orientation.x = quat[0]
        marker.pose.orientation.y = quat[1]
        marker.pose.orientation.z = quat[2]
        marker.pose.orientation.w = quat[3]
        
        # Set scale
        marker.scale.x = self.dock_marker_size
        marker.scale.y = self.dock_marker_size
        marker.scale.z = 0.01  # Thin plate
        
        # Set color (bright green)
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.7
        
        marker_array.markers.append(marker)
        
        # Draw an arrow pointing to the dock center
        arrow = Marker()
        arrow.header.frame_id = 'zed2i_left_camera_optical_frame'
        arrow.header.stamp = timestamp
        arrow.ns = 'dock_direction'
        arrow.id = 1
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        
        # Arrow starts at camera and points to dock
        arrow.points.append(Point(x=0.0, y=0.0, z=0.0))
        arrow.points.append(Point(x=position[0], y=position[1], z=position[2]))
        
        # Set scale (shaft diameter and head diameter)
        arrow.scale.x = 0.02  # Shaft diameter
        arrow.scale.y = 0.04  # Head diameter
        arrow.scale.z = 0.0   # Not used
        
        # Set color (yellow)
        arrow.color.r = 1.0
        arrow.color.g = 1.0
        arrow.color.b = 0.0
        arrow.color.a = 0.7
        
        marker_array.markers.append(arrow)
        
        # Publish marker array
        self.dock_marker_pub.publish(marker_array)
    
    def ir_callback(self, msg):
        """Handle IR sensor readings for close-range detection"""
        self.last_ir_reading = msg.range
        self.last_ir_time = self.get_clock().now().to_msg().sec
        
        # If very close to dock (IR detection) and in APPROACHING state, switch to FINAL_APPROACH
        if self.docking_state == DockingState.APPROACHING and msg.range < 0.2:
            with self.state_lock:
                self.get_logger().info(f'IR sensor detected dock at close range: {msg.range}m')
                self.docking_state = DockingState.FINAL_APPROACH
    
    def odom_callback(self, msg):
        """Handle odometry data for position tracking"""
        # Store current position for search patterns
        self.current_pose = msg.pose.pose
    
    def start_docking_callback(self, request, response):
        """Service callback to start docking"""
        with self.state_lock:
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
        with self.state_lock:
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
        with self.state_lock:
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
                with self.state_lock:
                    self.docking_state = DockingState.IDLE
                goal_handle.canceled()
                result.success = False
                result.message = "Docking canceled"
                return result
            
            # Publish feedback
            feedback_msg.current_state = self.docking_state.name
            
            if self.use_zed_camera and self.dock_visible:
                feedback_msg.distance_to_dock = self.dock_distance
            else:
                feedback_msg.distance_to_dock = self.last_ir_reading
                
            goal_handle.publish_feedback(feedback_msg)
            
            # Check for timeout
            current_time = self.get_clock().now().to_msg().sec
            if current_time - self.docking_start_time > self.dock_timeout:
                self.get_logger().error('Docking timed out')
                self.stop_robot()
                with self.state_lock:
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
        self.dock_detection_count = 0
        self.dock_visible = False
    
    def docking_control_loop(self):
        """Main control loop for docking state machine"""
        with self.state_lock:
            if self.docking_state == DockingState.IDLE or self.docking_state == DockingState.DOCKED:
                return
            
            # Create command velocity message
            cmd_vel = Twist()
            
            # State machine
            if self.docking_state == DockingState.SEARCHING:
                self.execute_search_pattern(cmd_vel)
            
            elif self.docking_state == DockingState.APPROACHING:
                # Visual servoing approach using ZED camera
                if self.use_zed_camera and self.dock_visible:
                    # Calculate approach velocity based on distance
                    approach_speed = min(self.dock_approach_speed, self.dock_distance / 5.0)
                    cmd_vel.linear.x = max(0.05, approach_speed)  # At least 5cm/s forward
                    
                    # Calculate rotation to align with dock
                    # Angle is already calculated in image_callback
                    alignment_factor = 1.0  # How aggressive to correct (higher = more aggressive)
                    cmd_vel.angular.z = -self.dock_angle * alignment_factor  # Negative because positive angle means dock is to the left
                    
                    # If well-aligned and close, switch to aligning state
                    if abs(self.dock_angle) < 0.1 and self.dock_distance < 0.5:  # Within ~6 degrees and 50cm
                        self.get_logger().info('Close to dock and aligned, switching to alignment phase')
                        self.docking_state = DockingState.ALIGNING
                        
                # Backup IR approach if visual detection is not used or failed
                elif self.last_ir_time > 0:
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
                else:
                    # If we have no valid sensor input, fall back to search
                    self.get_logger().warn('No sensor input available, returning to search')
                    self.docking_state = DockingState.SEARCHING
                    self.search_start_time = self.get_clock().now().to_msg().sec
            
            elif self.docking_state == DockingState.ALIGNING:
                # Fine-tune alignment before final approach
                if self.use_zed_camera and self.dock_visible:
                    # Only rotate, no forward motion
                    cmd_vel.linear.x = 0.0
                    alignment_factor = 1.5  # More aggressive correction
                    cmd_vel.angular.z = -self.dock_angle * alignment_factor
                    
                    # If we're well aligned, move to final approach
                    if abs(self.dock_angle) < 0.05:  # Within ~3 degrees
                        self.get_logger().info('Alignment complete, proceeding to final approach')
                        self.docking_state = DockingState.FINAL_APPROACH
                else:
                    # Fallback alignment without camera
                    cmd_vel.angular.z = self.dock_rotation_speed * 0.3  # Small rotation to check alignment
                    
                    # After a brief alignment period, move to final approach
                    time_in_alignment = self.get_clock().now().to_msg().sec - self.last_ir_time
                    if time_in_alignment > 2.0:
                        self.get_logger().info('Alignment without camera complete, proceeding to final approach')
                        self.docking_state = DockingState.FINAL_APPROACH
            
            elif self.docking_state == DockingState.FINAL_APPROACH:
                # Final slow approach to dock
                if self.use_zed_camera and self.dock_visible:
                    # Very slow approach based on distance
                    approach_speed = min(self.dock_approach_speed * 0.3, self.dock_distance / 10.0)
                    cmd_vel.linear.x = max(0.03, approach_speed)  # At least 3cm/s forward
                    
                    # Minimal alignment correction
                    alignment_factor = 0.7  # Less aggressive to avoid oscillation
                    cmd_vel.angular.z = -self.dock_angle * alignment_factor
                    
                    # If we're very close, consider docked
                    if self.dock_distance < 0.1 or self.last_ir_reading < self.dock_tolerance:
                        self.get_logger().info('Docking complete!')
                        self.docking_state = DockingState.DOCKED
                        cmd_vel.linear.x = 0.0
                        cmd_vel.angular.z = 0.0
                elif self.last_ir_reading > self.dock_tolerance:
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
        
        # If we've found the dock visually, transition to approaching
        if self.use_zed_camera and self.dock_visible:
            self.get_logger().info(f'Dock marker found during search at distance: {self.dock_distance:.2f}m')
            self.docking_state = DockingState.APPROACHING
            return
        
        # Otherwise, continue with search pattern
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
