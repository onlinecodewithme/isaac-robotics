# Advanced Tracked Robot Implementation

This implementation enhances the original tracked robot with improved robustness, better integration with the ZED 2i camera, NVIDIA Isaac Perceptor, and advanced auto-docking capabilities. The robot now meets the specifications for an autonomous robot (dimensions: 780x1015x720 mm - W,L,H) capable of 3D mapping, navigation, obstacle avoidance, and auto-docking.

## Improvements and New Features

### ODrive Motor Controller Enhancements
- **Robust Error Handling**: Added proper error detection and recovery mechanisms
- **Automatic Reconnection**: Implemented retry logic for ODrive controller connection
- **Command Timeout Safety**: Added safety feature to stop motors if commands aren't received
- **Power Monitoring**: Added real-time monitoring of power consumption
- **Current Limiting**: Properly configured for 1kW BLDC motors

### Advanced Auto-Docking System
- **ZED Camera Integration**: Added visual servoing using the ZED 2i camera
- **ArUco Marker Detection**: Uses computer vision to detect the charging dock
- **Multi-Sensor Fusion**: Combines IR sensors and visual detection for reliable docking
- **Improved Search Patterns**: Implemented more efficient dock searching algorithms
- **Visual Feedback**: Added RViz visualization for dock detection and approach

### NVIDIA Isaac Perceptor Integration
- **Visual SLAM Integration**: Properly configured for ZED 2i camera
- **Enhanced Mapping**: Real-time 3D environment mapping
- **Visual Odometry**: Improved localization using visual cues
- **IMU Fusion**: Combined IMU and visual data for more robust pose estimation

### Navigation Improvements
- **Dynamic Obstacle Avoidance**: Better detection and avoidance of moving obstacles
- **Map Management**: Automatic map saving and loading
- **Path Planning**: More efficient path planning algorithms

## Hardware Requirements

- **Robot Base**: Dimensions 780x1015x720 mm (W,L,H)
- **Motors**: Two 1kW BLDC motors in differential drive configuration
- **Motor Controller**: ODrive controller with 1kW capacity
- **Computer**: NVIDIA Jetson Orin NX
- **Camera**: ZED 2i stereo camera
- **Charging Dock**: Custom dock with ArUco marker (ID 42)

## Software Dependencies

Ensure you have the following installed on your Jetson Orin NX:

1. ROS2 Humble
2. NVIDIA Isaac ROS packages:
   - isaac_ros_common
   - isaac_ros_visual_slam
   - isaac_ros_nitros
   - isaac_ros_nova (for perceptor)
3. ZED SDK and ROS2 wrapper
4. OpenCV with ArUco support
5. ODrive Python library: `pip install odrive`
6. Navigation2 stack

## Setting Up the Robot

### Hardware Setup

1. **Camera Mounting**: 
   - Mount the ZED 2i camera at the front-top of the robot
   - Ensure it has an unobstructed view and is properly calibrated

2. **ODrive Configuration**:
   - Connect the motors to the ODrive controller
   - Connect the ODrive to Jetson Orin NX via USB
   - Run ODrive calibration if needed

3. **Charging Dock**:
   - Print and attach the ArUco marker (ID 42, DICT_5X5_100 dictionary) to the charging dock
   - Ensure the marker size is 15cm x 15cm for proper detection

### Software Setup

1. Connect to your Jetson Orin NX (SSH or direct):
   ```bash
   ssh x4@192.168.1.83
   ```

2. Build the project:
   ```bash
   cd ~/isaac_robotics
   colcon build --symlink-install
   source install/setup.bash
   ```

## Running the Robot

### Basic Operation

Launch the advanced robot with all features enabled:

```bash
ros2 launch tracked_robot advanced_robot.launch.py
```

### Optional Parameters

You can configure the launch with various parameters:

```bash
ros2 launch tracked_robot advanced_robot.launch.py use_zed:=true use_isaac_slam:=true use_isaac_perceptor:=true use_auto_docking:=true
```

### Auto-Docking

To trigger the auto-docking sequence:

```bash
ros2 service call /dock/start std_srvs/srv/Trigger
```

To cancel docking:

```bash
ros2 service call /dock/stop std_srvs/srv/Trigger
```

Monitor docking status:

```bash
ros2 topic echo /dock/state
```

### Navigation

The robot can now create real-time 3D maps of its environment. To save a map:

```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: 'my_map'}}"
```

## Troubleshooting

### ODrive Connection Issues

If the ODrive controller doesn't connect:
1. Check USB connections
2. Verify the ODrive is powered
3. Run `ros2 topic echo /odrive/status` to see connection status
4. The new implementation will attempt to reconnect automatically

### ZED Camera Issues

If the ZED camera doesn't work properly:
1. Verify USB 3.0 connection
2. Check ZED SDK installation: `zed_depth_viewer`
3. Verify camera node is running: `ros2 topic echo /zed2i/rgb/camera_info`

### Auto-Docking Issues

If docking doesn't work:
1. Verify ArUco marker is properly visible
2. Check marker size configuration
3. Monitor marker detection: `ros2 topic echo /dock/visible`
4. Use RViz to visualize the detected marker

## Future Improvements

1. Integration with ROS2 Control for standardized motor control
2. Enhanced obstacle detection using 3D point cloud processing
3. Auto-calibration procedures for easier setup
4. Multi-floor mapping and navigation

## Credits

Built on NVIDIA Isaac ROS Perceptor, ZED SDK, and ODrive ecosystem.
