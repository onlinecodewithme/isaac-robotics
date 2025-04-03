#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    # Get the launch directory and package directory
    pkg_dir = get_package_share_directory('tracked_robot')
    nav2_launch_dir = None
    slam_launch_dir = None
    
    # Check for Nav2 and SLAM packages
    have_nav2 = False
    have_slam_toolbox = False
    
    try:
        nav2_launch_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
        have_nav2 = True
        print("Navigation2 packages found, enabling autonomous navigation capabilities")
    except PackageNotFoundError:
        print("Navigation2 packages not found, navigation capabilities will be limited")
    
    try:
        slam_launch_dir = os.path.join(get_package_share_directory('slam_toolbox'), 'launch')
        have_slam_toolbox = True
        print("SLAM Toolbox found, enabling mapping capabilities")
    except PackageNotFoundError:
        print("SLAM Toolbox not found, mapping capabilities will be limited")
    
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_slam = LaunchConfiguration('use_slam', default=str(have_slam_toolbox).lower())
    map_dir = LaunchConfiguration('map_dir', default=os.path.join(pkg_dir, 'maps'))
    map_file = LaunchConfiguration('map_file', default='map.yaml')
    use_rviz = LaunchConfiguration('use_rviz', default='false')
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )
    
    declare_use_slam = DeclareLaunchArgument(
        'use_slam',
        default_value=str(have_slam_toolbox).lower(),
        description='Whether to run SLAM (if installed)'
    )
    
    declare_map_dir = DeclareLaunchArgument(
        'map_dir',
        default_value=os.path.join(pkg_dir, 'maps'),
        description='Directory where maps are stored'
    )
    
    declare_map_file = DeclareLaunchArgument(
        'map_file',
        default_value='map.yaml',
        description='Name of the map yaml file'
    )
    
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Use RViz navigation visualization'
    )
    
    # SLAM toolbox if available 
    slam_toolbox_node = None
    if have_slam_toolbox:
        slam_toolbox_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                slam_launch_dir, '/online_async_launch.py'
            ]),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'slam_params_file': os.path.join(pkg_dir, 'config', 'slam_toolbox_params.yaml')
            }.items(),
            condition=IfCondition(use_slam)
        )
    
    # Nav2 bringup if available
    nav2_bringup_launch = None
    if have_nav2:
        nav2_bringup_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                nav2_launch_dir, '/bringup_launch.py'
            ]),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'map': os.path.join([map_dir, '/', map_file]),
                'params_file': os.path.join(pkg_dir, 'config', 'nav2_params.yaml'),
                'use_composition': 'True',
                'use_rviz': use_rviz
            }.items(),
            condition=UnlessCondition(use_slam)
        )
    
    # Navigation RVIZ (if requested)
    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2_nav',
        arguments=['-d', os.path.join(pkg_dir, 'config', 'navigation.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # Define LaunchDescription
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_use_slam)
    ld.add_action(declare_map_dir)
    ld.add_action(declare_map_file)
    ld.add_action(declare_use_rviz)
    
    # Add nodes conditionally
    if have_slam_toolbox:
        ld.add_action(slam_toolbox_node)
    else:
        ld.add_action(LogInfo(msg="SLAM Toolbox not found, skipping SLAM capabilities"))
    
    if have_nav2:
        ld.add_action(nav2_bringup_launch)
    else:
        ld.add_action(LogInfo(msg="Navigation2 packages not found, skipping Nav2 capabilities"))
    
    # Add RViz if requested
    ld.add_action(rviz_node)
    
    return ld
