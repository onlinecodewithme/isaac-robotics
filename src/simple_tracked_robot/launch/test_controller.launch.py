#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the launch directory
    pkg_dir = get_package_share_directory('simple_tracked_robot')
    
    # Launch arguments
    use_simulation = LaunchConfiguration('simulation', default='true')
    
    # Declare launch arguments
    declare_simulation = DeclareLaunchArgument(
        'simulation',
        default_value='true',
        description='Use simulation mode (no actual ODrive hardware)'
    )
    
    # ODrive controller node
    odrive_controller_node = Node(
        package='simple_tracked_robot',
        executable='odrive_controller.py',
        name='odrive_controller',
        output='screen',
        parameters=[
            {'simulation_mode': use_simulation},
            {'wheel_radius': 0.15},
            {'track_width': 0.78},
            {'max_motor_current': 60.0},
            {'left_motor_index': 0},
            {'right_motor_index': 1},
            {'left_motor_reversed': False},
            {'right_motor_reversed': True}
        ]
    )
    
    # Test publisher node
    test_publisher_node = Node(
        package='simple_tracked_robot',
        executable='test_publisher.py',
        name='test_publisher',
        output='screen'
    )
    
    # Launch description
    ld = LaunchDescription()
    
    # Add declared arguments
    ld.add_action(declare_simulation)
    
    # Add nodes
    ld.add_action(odrive_controller_node)
    ld.add_action(test_publisher_node)
    
    return ld
