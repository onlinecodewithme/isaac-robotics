#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the launch directory
    pkg_dir = get_package_share_directory('tracked_robot')
    
    # Specify xacro file location and other paths
    xacro_file = os.path.join(pkg_dir, 'urdf', 'tracked_robot.urdf.xacro')
    robot_description_config = Command(['xacro ', xacro_file])
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RViz'
    )
    
    # Set robot state publisher parameters
    params = {'robot_description': robot_description_config, 'use_sim_time': use_sim_time}
    
    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[params],
        remappings=[],
    )
    
    # Joint state publisher
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
    )
    
    # ODrive motor controller node (simplified parameters)
    odrive_controller_node = Node(
        package='tracked_robot',
        executable='odrive_control_node.py',
        name='odrive_controller',
        output='screen',
        parameters=[
            {'wheel_radius': 0.15},
            {'track_width': 0.78},
            {'max_motor_current': 60.0},
            {'use_sim_time': use_sim_time}
        ],
    )
    
    # RViz (conditional)
    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_dir, 'config', 'robot.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
    )
    
    # Define LaunchDescription
    ld = LaunchDescription()
    
    # Add declared launch arguments
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_use_rviz)
    
    # Add nodes
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(odrive_controller_node)
    ld.add_action(rviz_node)
    
    return ld
