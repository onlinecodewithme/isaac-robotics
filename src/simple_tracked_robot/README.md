# Advanced Tracked Robot

This package provides a ROS2 implementation for an autonomous tracked robot with the following features:

- URDF model of a tracked robot with dimensions (780x1015x720 mm - W,L,H)
- Differential drive control using ODrive motor controllers for 1 kW BLDC motors
- Support for ZED 2i camera for visual perception
- Integration with NVIDIA Isaac ROS Perceptor for 3D mapping and visual SLAM
- Autonomous navigation using Nav2
- Auto-docking capability

## System Requirements

### Hardware

- NVIDIA Jetson Orin NX (or compatible Jetson device)
- ZED 2i camera (with ZED SDK installed)
- ODrive motor controllers
- Two 1 kW BLDC motors in differential setup

### Software

- ROS2 Humble
- NVIDIA Isaac ROS packages
- ZED SDK and ROS2 packages
- Navigation2 (Nav2) packages
- Additional ROS2 packages as needed (teleop, etc.)

## Installation

1. Install ROS2 Humble following the [official instructions](https://docs.ros.org/en/humble/Installation.html)
2. Install NVIDIA Isaac ROS packages: 
   - Follow instructions at [NVIDIA Isaac ROS](https://nvidia-isaac-ros.github.io/getting_started/index.html)
   - Install the Isaac Perceptor workflow: https://nvidia-isaac-ros.github.io/reference_workflows/isaac_perceptor/index.html
3. Install ZED SDK and ZED ROS2 packages:
   - Follow instructions at [Stereolabs ROS2 Documentation](https://www.stereolabs.com/docs/ros2)
4. Install Navigation2:
   ```
   sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
   ```
5. Install additional packages:
   ```
   sudo apt install ros-humble-slam-toolbox ros-humble-joint-state-publisher-gui ros-humble-xacro ros-humble-robot-state-publisher ros-humble-rviz2 ros-humble-teleop-twist-keyboard
   ```
6. Install Python dependencies for ODrive control:
   ```
   pip install odrive fibre numpy
   ```

## Build

1. Clone this repository into your ROS2 workspace
2. Build the packages:
   ```
   colcon build --symlink-install --packages-select simple_tracked_robot tracked_robot_msgs
   ```
3. Source the workspace:
   ```
   source install/setup.bash
   ```

## Usage

### Launch Files

The package includes several launch files for different scenarios:

1. **View Robot (Visualization Only)**
   ```
   export DISPLAY=:1  # If needed for display
   ros2 launch simple_tracked_robot view_robot.launch.py
   ```

2. **Basic Robot Control**
   ```
   ros2 launch simple_tracked_robot start_robot.launch.py simulation:=true
   ```
   Set `simulation:=false` to use real ODrive hardware.

3. **Full Robot with Navigation**
   ```
   ros2 launch simple_tracked_robot tracked_robot_full.launch.py
   ```
   
   Launch arguments:
   - `simulation:=true/false` - Enable/disable simulation mode for ODrive
   - `use_zed:=true/false` - Enable/disable ZED camera (if available)
   - `use_isaac_slam:=true/false` - Enable/disable NVIDIA Isaac SLAM (if available)
   - `use_nav2:=true/false` - Enable/disable Navigation2 (if available)
   - `use_rviz:=true/false` - Enable/disable RViz visualization

### Controlling the Robot

The robot can be controlled using the teleop keyboard:

1. Use the teleop window that appears when launching the robot
2. Use the following keys:
   - `i`, `j`, `l`, `,` - Move forward, left, right, backward
   - `u`, `o`, `m`, `.` - Diagonal movements
   - `k` - Stop the robot
   - `q`, `z` - Increase/decrease linear velocity
   - `w`, `x` - Increase/decrease angular velocity

### Navigation

When using the full configuration with Navigation2:

1. In RViz, use the "2D Pose Estimate" button to set the initial pose
2. Use the "2D Goal Pose" button to set a navigation goal
3. The robot will autonomously navigate to the goal while avoiding obstacles

## Architecture

The system architecture is built around the following components:

1. **Robot Model (URDF)** - Defines the physical structure of the robot
2. **ODrive Controller** - Interfaces with the ODrive motor controllers
3. **ZED Camera Integration** - Provides visual perception and depth data
4. **NVIDIA Isaac Visual SLAM** - Builds 3D maps and provides localization
5. **Navigation2** - Provides path planning and obstacle avoidance

The nodes communicate through standard ROS2 interfaces, with the ODrive controller subscribing to velocity commands (`cmd_vel`) and publishing joint states.

## Extending the System

To extend or customize the system:

1. Modify the URDF model in `urdf/tracked_robot.urdf.xacro` for different robot configurations
2. Adjust parameters in the launch files for your specific hardware
3. Add new sensors by updating the URDF and creating appropriate ROS2 interfaces
4. Modify the navigation parameters in `config/nav2_params.yaml` for different navigation behaviors

## Troubleshooting

1. **Display Issues**: Set the `DISPLAY` environment variable to your X display (e.g., `export DISPLAY=:1`)
2. **ODrive Connection**: Make sure the ODrive controllers are properly connected and configured
3. **ZED Camera**: Verify the ZED SDK is properly installed and the camera is connected
4. **Navigation Issues**: Check the Nav2 parameters and ensure the costmaps are properly configured
5. **SLAM Issues**: Verify the Isaac ROS packages are correctly installed and configured

## License

Apache License 2.0
