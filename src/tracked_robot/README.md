# Autonomous Tracked Robot with NVIDIA Isaac Perceptor

This ROS2 package implements an autonomous tracked robot (dimensions: 780x1015x720 mm - W,L,H) with capabilities for 3D mapping, navigation, obstacle avoidance, and auto-docking using NVIDIA Isaac ROS Perceptor and ZED 2i camera.

## Hardware Components

- **Robot Base**: Custom tracked platform (dimensions: 780x1015x720 mm)
- **Motors**: Two 1kW BLDC motors in differential drive configuration
- **Motor Controller**: ODrive controller
- **Computer**: NVIDIA Jetson Orin NX
- **Sensors**: ZED 2i Camera (with depth and tracking)
- **Auto-docking**: IR sensors for docking detection

## Software Architecture

The software stack is built on top of ROS2 and NVIDIA Isaac ROS, providing:

- Advanced perception using NVIDIA Isaac Perceptor
- Real-time 3D mapping and localization
- Autonomous navigation with dynamic obstacle avoidance
- Auto-docking capabilities

### Key Components

1. **Robot Control**: Interface with ODrive for differential-drive control
2. **3D Perception**: Integration with ZED 2i and NVIDIA Isaac for 3D mapping
3. **Autonomous Navigation**: Nav2-based path planning and execution
4. **Auto-Docking**: Custom implementation for finding and docking to charging station

## Building and Running

### Prerequisites

- ROS2 Humble or later
- NVIDIA Isaac ROS Perceptor (from https://nvidia-isaac-ros.github.io/reference_workflows/isaac_perceptor/index.html)
- ZED SDK and ROS2 wrapper (https://github.com/stereolabs/zed-ros2-wrapper)
- ODrive Python library (`pip install odrive`)

### Building

Clone this repository to your ROS2 workspace:

```bash
# Inside your ROS2 workspace
colcon build --symlink-install --packages-select tracked_robot tracked_robot_msgs
```

### Running the Robot

1. **Basic robot control and visualization**:
   ```bash
   ros2 launch tracked_robot tracked_robot.launch.py
   ```

2. **Navigation with NVIDIA Isaac Perceptor**:
   ```bash
   ros2 launch tracked_robot navigation.launch.py
   ```

3. **Auto-docking service**:
   ```bash
   # After navigation is running, trigger docking with:
   ros2 service call /dock/start std_srvs/srv/Trigger
   ```

## Key Nodes

- **odrive_control_node.py**: Interface with ODrive motor controller
- **auto_docking_node.py**: Handles docking sequence using IR sensors

## Configuration Files

- **zed2i.yaml**: ZED camera configuration
- **nav2_params.yaml**: Navigation stack parameters
- **slam_toolbox_params.yaml**: SLAM configuration

## Features

- **Real-time 3D Mapping**: Creates and updates 3D maps of the environment using ZED camera and NVIDIA Isaac SDK
- **Obstacle Avoidance**: Detects and avoids static and dynamic obstacles during navigation
- **Auto-Docking**: Autonomously locates and docks with charging station
- **Robust Navigation**: Handles varied terrain with tracked design and powerful motors

## Limitations and Future Work

- Currently requires calibration for each new environment
- Auto-docking IR sensor has limited range, future versions may use visual markers
- Adding integration with ROS2 Control for more standardized motor control
- Adding support for remote teleoperation

## Customizing the Robot

The physical parameters can be adjusted in `urdf/tracked_robot_properties.xacro`:
- Robot dimensions
- Motor power
- Wheel and track properties

Navigation parameters can be tuned in `config/nav2_params.yaml`.

## License

Apache License 2.0
