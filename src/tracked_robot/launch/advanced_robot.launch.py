#!/usr/bin/env python3

import os
import sys
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    # Get the launch directory
    pkg_dir = get_package_share_directory('tracked_robot')
    
    # Detect available packages
    have_zed = False
    have_isaac_slam = False
    have_isaac_perceptor = False
    
    try:
        get_package_share_directory('zed_wrapper')
        get_package_share_directory('zed_components')
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
        get_package_share_directory('isaac_ros_nova')
        have_isaac_perceptor = True
        print("NVIDIA Isaac Perceptor package found, enabling advanced perception")
    except PackageNotFoundError:
        print("NVIDIA Isaac Perceptor package not found, disabling advanced perception")
    
    # Specify xacro file location and other paths
    xacro_file = os.path.join(pkg_dir, 'urdf', 'tracked_robot.urdf.xacro')
    robot_description_config = Command(['xacro ', xacro_file])
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    use_zed = LaunchConfiguration('use_zed', default=str(have_zed).lower())
    use_isaac_slam = LaunchConfiguration('use_isaac_slam', default=str(have_isaac_slam).lower())
    use_isaac_perceptor = LaunchConfiguration('use_isaac_perceptor', default=str(have_isaac_perceptor).lower())
    use_auto_docking = LaunchConfiguration('use_auto_docking', default='true')
    rviz_config_file = LaunchConfiguration('rviz_config', default=os.path.join(pkg_dir, 'config', 'advanced.rviz'))
    
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
    
    declare_use_zed = DeclareLaunchArgument(
        'use_zed',
        default_value=str(have_zed).lower(),
        description='Use ZED camera if true'
    )
    
    declare_use_isaac_slam = DeclareLaunchArgument(
        'use_isaac_slam',
        default_value=str(have_isaac_slam).lower(),
        description='Use NVIDIA Isaac Visual SLAM if true'
    )
    
    declare_use_isaac_perceptor = DeclareLaunchArgument(
        'use_isaac_perceptor',
        default_value=str(have_isaac_perceptor).lower(),
        description='Use NVIDIA Isaac Perceptor if true'
    )
    
    declare_use_auto_docking = DeclareLaunchArgument(
        'use_auto_docking',
        default_value='true',
        description='Enable auto-docking capability if true'
    )
    
    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(pkg_dir, 'config', 'advanced.rviz'),
        description='Absolute path to rviz config file'
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
    
    # ODrive motor controller node - improved version with robust error handling
    odrive_controller_node = Node(
        package='tracked_robot',
        executable='odrive_control_node.py',
        name='odrive_controller',
        output='screen',
        parameters=[
            {'wheel_radius': 0.15},
            {'track_width': 0.78},
            {'gear_ratio': 1.0},
            {'encoder_cpr': 8192},
            {'max_motor_current': 60.0},  # Set for 1kW BLDC motors
            {'control_mode': 'velocity'},
            {'left_motor_index': 0},
            {'right_motor_index': 1},
            {'left_motor_reversed': False},
            {'right_motor_reversed': True},
            {'reconnect_attempts': 3},
            {'reconnect_interval': 1.0},
            {'use_sim_time': use_sim_time}
        ],
    )
    
    # Auto-docking node with ZED camera integration
    auto_docking_node = Node(
        condition=IfCondition(use_auto_docking),
        package='tracked_robot',
        executable='auto_docking_node.py',
        name='auto_docking_node',
        output='screen',
        parameters=[
            {'dock_ir_topic': '/dock_ir'},
            {'cmd_vel_topic': '/cmd_vel'},
            {'dock_tolerance': 0.05},
            {'dock_approach_speed': 0.1},
            {'dock_rotation_speed': 0.3},
            {'dock_timeout': 60.0},
            {'search_pattern': 'spiral'},
            {'use_zed_camera': have_zed},
            {'dock_marker_size': 0.15},
            {'aruco_dictionary': 'DICT_5X5_100'},
            {'dock_marker_id': 42},
            {'use_sim_time': use_sim_time}
        ],
    )
    
    # ZED wrapper node for stereo camera
    zed_wrapper_node = Node(
        condition=IfCondition(use_zed),
        package='zed_wrapper',
        executable='zed_wrapper_node',
        name='zed2i',
        output='screen',
        parameters=[
            os.path.join(pkg_dir, 'config', 'zed2i.yaml'),
            {'use_sim_time': use_sim_time}
        ],
    )
    
    # NVIDIA Isaac Visual SLAM node
    isaac_visual_slam_node = Node(
        condition=IfCondition(use_isaac_slam),
        package='isaac_ros_visual_slam',
        executable='visual_slam_node',
        name='visual_slam',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'denoise_input': True},
            {'rectified_images': True},
            {'enable_slam_visualization': True},
            {'enable_landmarks_view': True},
            {'enable_debug_mode': False},
            {'map_frame': 'map'},
            {'odom_frame': 'odom'},
            {'base_frame': 'base_link'},
            {'publish_tf': True},
            {'publish_odom_to_base_tf': False},
            {'localization_only_mode': False},
            {'enable_imu_fusion': True},
            {'vio_scale_factor': 1.0},
            {'map_update_interval': 5},
        ],
        remappings=[
            ('rgb/camera_info', '/zed2i/rgb/camera_info'),
            ('rgb/image_rect', '/zed2i/rgb/image_rect_color'),
            ('depth/camera_info', '/zed2i/depth/camera_info'),
            ('depth/image_rect', '/zed2i/depth/depth_registered'),
            ('visual_slam/tracking/odometry', '/odom'),
            ('imu', '/zed2i/imu/data'),
        ],
    )
    
    # NVIDIA Isaac Perceptor nodes for advanced perception
    isaac_perceptor_launch = None
    if have_isaac_perceptor:
        # This is a placeholder as the exact integration depends on the specific Isaac Perceptor modules being used
        # In a real implementation, this would include specific Nova nodes or other Isaac ROS modules
        isaac_perceptor_launch = LogInfo(
            condition=IfCondition(use_isaac_perceptor),
            msg="NVIDIA Isaac Perceptor integration would be launched here"
        )
    
    # RViz
    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
    )
    
    # Navigation launch
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_dir, 'launch', 'navigation.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_slam': 'true'  # Default to SLAM mode for mapping
        }.items()
    )
    
    # Define LaunchDescription
    ld = LaunchDescription()
    
    # Add declared launch arguments
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_use_rviz)
    ld.add_action(declare_use_zed)
    ld.add_action(declare_use_isaac_slam)
    ld.add_action(declare_use_isaac_perceptor)
    ld.add_action(declare_use_auto_docking)
    ld.add_action(declare_rviz_config)
    
    # Add core nodes
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(odrive_controller_node)
    
    # Add conditional nodes
    ld.add_action(auto_docking_node)
    
    if have_zed:
        ld.add_action(zed_wrapper_node)
    else:
        ld.add_action(LogInfo(msg="ZED packages not found, skipping ZED camera setup"))
    
    if have_isaac_slam:
        ld.add_action(isaac_visual_slam_node)
    else:
        ld.add_action(LogInfo(msg="NVIDIA Isaac Visual SLAM package not found, skipping Isaac SLAM setup"))
    
    if have_isaac_perceptor and isaac_perceptor_launch:
        ld.add_action(isaac_perceptor_launch)
    else:
        ld.add_action(LogInfo(msg="NVIDIA Isaac Perceptor package not found, skipping advanced perception"))
    
    # Add RViz and navigation
    ld.add_action(rviz_node)
    ld.add_action(navigation_launch)
    
    return ld
