# Simulation Setup for Tracked Robot

This guide provides instructions on setting up a simulation environment for the Advanced Tracked Robot using Gazebo. Simulation can be useful for testing navigation algorithms, sensor integration, and control strategies before deploying to the real hardware.

## Prerequisites

To use simulation with the tracked robot, you'll need:

1. ROS 2 Humble (already installed)
2. Gazebo Garden or newer
3. ROS-Gazebo integration packages

## Installing Gazebo and ROS-Gazebo Integration

Follow these steps to set up Gazebo simulation support:

```bash
# Add Gazebo package repository
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Update package lists
sudo apt-get update

# Install Gazebo Garden
sudo apt-get install -y gz-garden

# Install ROS 2 - Gazebo integration packages
sudo apt-get install -y ros-humble-ros-gz ros-humble-ros-gz-bridge ros-humble-ros-gz-interfaces ros-humble-ros-gz-sim
```

## Optional: Install Navigation Testing Dependencies

If you want to run the navigation system tests, you'll need additional packages:

```bash
# Install Gazebo ROS packages
sudo apt-get install -y ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control

# Install additional navigation dependencies
sudo apt-get install -y ros-humble-nav2-bringup ros-humble-navigation2 ros-humble-nav2-gazebo-spawner
```

## Setting Up COLCON_IGNORE for Simulation Packages

When working with the physical robot, you may want to skip building simulation-related packages:

```bash
# Skip simulation-related packages when building for real robot
touch src/nav2_system_tests/COLCON_IGNORE
```

## Running the Simulation

To launch the tracked robot in simulation:

```bash
# Source ROS 2
source /opt/ros/humble/setup.bash
source ~/isaac_xavier/install/setup.bash

# Launch the simulation
ros2 launch tracked_robot tracked_robot_sim.launch.py
```

### Creating a Simulation Launch File

Create a new launch file at `src/tracked_robot/launch/tracked_robot_sim.launch.py`:

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Get package directories
    tracked_robot_dir = get_package_share_directory('tracked_robot')
    
    # Launch Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
    )
    
    # Load and spawn robot model
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'tracked_robot',
            '-topic', 'robot_description',
            '-z', '0.1'
        ],
        output='screen'
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': open(os.path.join(tracked_robot_dir, 'urdf', 'tracked_robot.urdf.xacro')).read()},
            {'use_sim_time': True}
        ]
    )
    
    # ROS-Gazebo bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage@gz.msgs.TFMessage',
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'
        ],
        output='screen'
    )
    
    # Launch RVIZ
    rviz_config = os.path.join(tracked_robot_dir, 'config', 'robot.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    
    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot,
        bridge,
        rviz,
    ])
```

## Adapting URDF for Simulation

You may need to update your URDF files to include Gazebo-specific elements. Create or modify `src/tracked_robot/urdf/tracked_robot_gazebo.xacro` to add simulation-specific elements like:

- Inertial properties for each link
- Collision geometries
- Gazebo plugins for differential drive, sensors, etc.

## Next Steps

Once you have the simulation environment set up, you can:

1. Test navigation algorithms in simulation
2. Validate sensor fusion and obstacle avoidance
3. Tune control parameters before deploying to the real robot
4. Develop and test new features in a safe environment

When you're ready to switch back to the real robot, simply run your regular launch file:

```bash
ros2 launch tracked_robot advanced_robot.launch.py
