#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    # Get the launch directory
    pkg_dir = get_package_share_directory('simple_tracked_robot')
    
    # Specify xacro file location
    xacro_file = os.path.join(pkg_dir, 'urdf', 'tracked_robot.urdf.xacro')
    
    # Use xacro to process the file
    robot_description_config = Command(['xacro ', xacro_file])
    
    # Launch arguments
    use_simulation = LaunchConfiguration('simulation', default='true')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    
    # Declare launch arguments
    declare_simulation = DeclareLaunchArgument(
        'simulation',
        default_value='true',
        description='Use simulation mode (no actual ODrive hardware)'
    )
    
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Start RViz'
    )
    
    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_config}]
    )
    
    # Joint state publisher - disabled in simulation mode since our custom robot_state_publisher handles it
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(LaunchConfiguration('simulation', default='false'))
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
    
    # Robot state publisher for simulation
    robot_state_publisher_sim_node = Node(
        package='simple_tracked_robot',
        executable='robot_state_publisher.py',
        name='robot_state_publisher_sim',
        output='screen',
        parameters=[
            {'wheel_radius': 0.15},
            {'track_width': 0.78}
        ],
        condition=IfCondition(use_simulation)
    )
    
    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(use_rviz),
        arguments=['-d', os.path.join(pkg_dir, 'config', 'robot.rviz')] if os.path.exists(os.path.join(pkg_dir, 'config', 'robot.rviz')) else []
    )
    
    # Use our test publisher instead of teleop keyboard
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
    ld.add_action(declare_use_rviz)
    
    # Add nodes
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(odrive_controller_node)
    ld.add_action(robot_state_publisher_sim_node)
    ld.add_action(rviz_node)
    ld.add_action(test_publisher_node)
    
    return ld
