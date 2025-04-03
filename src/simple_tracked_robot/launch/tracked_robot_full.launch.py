#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node

def generate_launch_description():
    # Get the launch directory
    pkg_dir = get_package_share_directory('simple_tracked_robot')
    
    # Detect available packages
    have_zed = False
    have_isaac_slam = False
    have_nav2 = False
    
    try:
        get_package_share_directory('zed_wrapper')
        have_zed = True
        print("ZED packages found, enabling ZED camera support")
    except PackageNotFoundError:
        print("ZED packages not found, disabling ZED camera support")
    
    try:
        get_package_share_directory('isaac_ros_visual_slam')
        have_isaac_slam = True
        print("NVIDIA Isaac Visual SLAM package found, enabling Isaac SLAM support")
    except PackageNotFoundError:
        print("NVIDIA Isaac Visual SLAM package not found, disabling Isaac SLAM support")
    
    try:
        get_package_share_directory('nav2_bringup')
        have_nav2 = True
        print("Navigation2 packages found, enabling autonomous navigation capabilities")
    except PackageNotFoundError:
        print("Navigation2 packages not found, navigation capabilities will be limited")
    
    # Specify xacro file location
    xacro_file = os.path.join(pkg_dir, 'urdf', 'tracked_robot.urdf.xacro')
    
    # Use xacro to process the file
    robot_description_config = Command(['xacro ', xacro_file])
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    use_zed = LaunchConfiguration('use_zed', default=str(have_zed).lower())
    use_isaac_slam = LaunchConfiguration('use_isaac_slam', default=str(have_isaac_slam).lower())
    use_nav2 = LaunchConfiguration('use_nav2', default=str(have_nav2).lower())
    use_simulation = LaunchConfiguration('simulation', default='true')
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Start RViz'
    )
    
    declare_use_zed = DeclareLaunchArgument(
        'use_zed',
        default_value=str(have_zed).lower(),
        description='Enable ZED camera if available'
    )
    
    declare_use_isaac_slam = DeclareLaunchArgument(
        'use_isaac_slam',
        default_value=str(have_isaac_slam).lower(),
        description='Enable NVIDIA Isaac Visual SLAM if available'
    )
    
    declare_use_nav2 = DeclareLaunchArgument(
        'use_nav2',
        default_value=str(have_nav2).lower(),
        description='Enable Nav2 navigation if available'
    )
    
    declare_use_simulation = DeclareLaunchArgument(
        'simulation',
        default_value='true',
        description='Enable simulation mode for ODrive controller'
    )
    
    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_config}]
    )
    
    # Joint state publisher (not needed if using real hardware or simulation that provides joint states)
    # But included for completeness when in full simulation mode
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=IfCondition(use_simulation),
        parameters=[{'use_sim_time': use_sim_time}]
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
            {'right_motor_reversed': True},
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # ZED wrapper node for stereo camera (if ZED is available and enabled)
    zed_wrapper_node = None
    if have_zed:
        # Try to find the ZED config file
        zed_config_file = os.path.join(pkg_dir, 'config', 'zed2i.yaml')
        if not os.path.exists(zed_config_file):
            # Try to find it in the tracked_robot package
            try:
                tracked_robot_pkg_dir = get_package_share_directory('tracked_robot')
                zed_config_file = os.path.join(tracked_robot_pkg_dir, 'config', 'zed2i.yaml')
            except PackageNotFoundError:
                print("Could not find zed2i.yaml config file")
                zed_config_file = ""
        
        if zed_config_file and os.path.exists(zed_config_file):
            zed_wrapper_node = Node(
                condition=IfCondition(use_zed),
                package='zed_wrapper',
                executable='zed_wrapper_node',
                name='zed2i',
                output='screen',
                parameters=[
                    zed_config_file,
                    {'use_sim_time': use_sim_time}
                ],
            )
    
    # NVIDIA Isaac Visual SLAM node (if available and enabled)
    isaac_visual_slam_node = None
    if have_isaac_slam:
        isaac_visual_slam_node = Node(
            condition=IfCondition(use_isaac_slam),
            package='isaac_ros_visual_slam',
            executable='visual_slam_node',
            name='visual_slam',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[
                ('rgb/camera_info', '/zed2i/rgb/camera_info'),
                ('rgb/image_rect', '/zed2i/rgb/image_rect_color'),
                ('depth/camera_info', '/zed2i/depth/camera_info'),
                ('depth/image_rect', '/zed2i/depth/depth_registered'),
            ],
        )
    
    # Nav2 launch (if available and enabled)
    nav2_launch = None
    if have_nav2:
        nav2_bringup_dir = get_package_share_directory('nav2_bringup')
        nav2_launch_dir = os.path.join(nav2_bringup_dir, 'launch')
        
        # Try to find a nav2 params file
        nav2_params_file = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
        if not os.path.exists(nav2_params_file):
            # Try to find it in the tracked_robot package
            try:
                tracked_robot_pkg_dir = get_package_share_directory('tracked_robot')
                nav2_params_file = os.path.join(tracked_robot_pkg_dir, 'config', 'nav2_params.yaml')
            except PackageNotFoundError:
                print("Could not find nav2_params.yaml config file")
                nav2_params_file = os.path.join(nav2_bringup_dir, 'params', 'nav2_params.yaml')
        
        # Create an empty map file if it doesn't exist
        map_yaml_path = os.path.join(pkg_dir, 'config', 'map.yaml')
        map_pgm_path = os.path.join(pkg_dir, 'config', 'map.pgm')
        
        if not os.path.exists(map_yaml_path):
            print("Creating a default map.yaml file")
            with open(map_yaml_path, 'w') as f:
                f.write("""image: map.pgm
resolution: 0.05
origin: [-10.0, -10.0, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
mode: scale
""")
        
        if not os.path.exists(map_pgm_path):
            print("Creating a default map.pgm file")
            # Create a 100x100 empty map image
            try:
                import numpy as np
                from PIL import Image
                # Create a white image (free space)
                img = np.ones((400, 400), dtype=np.uint8) * 254
                # Add a border
                img[0:10, :] = 0  # Top wall
                img[-10:, :] = 0  # Bottom wall
                img[:, 0:10] = 0  # Left wall
                img[:, -10:] = 0  # Right wall
                Image.fromarray(img).save(map_pgm_path)
            except ImportError:
                print("Could not create map.pgm, PIL or numpy not available")
                # Create an empty file
                with open(map_pgm_path, 'wb') as f:
                    # Simple P5 PGM header
                    f.write(b'P5\n400 400\n255\n')
                    # Fill with white pixels (254 = free space)
                    f.write(b'\xfe' * (400 * 400))
        
        nav2_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_dir, '/bringup_launch.py']),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': nav2_params_file,
                'map': map_yaml_path
            }.items(),
            condition=IfCondition(use_nav2)
        )
    
    # RViz
    rviz_config_file = os.path.join(pkg_dir, 'config', 'robot.rviz')
    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )
    
    # Teleop keyboard for manual control
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix = 'xterm -e'
    )
    
    # Launch description
    ld = LaunchDescription()
    
    # Add declared arguments
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_use_rviz)
    ld.add_action(declare_use_zed)
    ld.add_action(declare_use_isaac_slam)
    ld.add_action(declare_use_nav2)
    ld.add_action(declare_use_simulation)
    
    # Add required nodes
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(odrive_controller_node)
    ld.add_action(rviz_node)
    ld.add_action(teleop_node)
    
    # Add conditional nodes
    if zed_wrapper_node:
        ld.add_action(zed_wrapper_node)
    else:
        ld.add_action(LogInfo(msg="ZED camera support not available"))
    
    if isaac_visual_slam_node:
        ld.add_action(isaac_visual_slam_node)
    else:
        ld.add_action(LogInfo(msg="NVIDIA Isaac Visual SLAM support not available"))
    
    if nav2_launch:
        ld.add_action(nav2_launch)
    else:
        ld.add_action(LogInfo(msg="Navigation2 support not available"))
    
    return ld
