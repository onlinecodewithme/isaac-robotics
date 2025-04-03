#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the launch directory
    pkg_dir = get_package_share_directory('minimal_tracked_robot')
    
    # Specify xacro file location
    xacro_file = os.path.join(pkg_dir, 'urdf', 'simple_robot.xacro')
    
    # Use xacro to process the file
    robot_description_config = Command(['xacro ', xacro_file])
    
    # Create a parameter for the robot state publisher
    robot_description = {'robot_description': robot_description_config}
    
    # Launch arguments
    use_gui = LaunchConfiguration('use_gui', default='true')
    
    # Declare launch arguments
    declare_use_gui = DeclareLaunchArgument(
        'use_gui',
        default_value='true',
        description='Flag to enable joint_state_publisher_gui'
    )
    
    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )
    
    # Joint state publisher
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=UnlessCondition(use_gui)
    )
    
    # Launch description
    ld = LaunchDescription()
    
    # Add actions
    ld.add_action(declare_use_gui)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)
    
    return ld
