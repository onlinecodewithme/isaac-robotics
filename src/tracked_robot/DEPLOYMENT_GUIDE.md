# Deployment and Testing Guide for Advanced Tracked Robot

This guide provides step-by-step instructions to deploy and test the Advanced Tracked Robot implementation on your Jetson Orin NX with the connected ODrive controller and ZED 2i camera.

## Prerequisites

- Jetson Orin NX (IP: 192.168.1.83, Username: x4)
- ODrive controller connected via USB
- ZED 2i camera connected via USB 3.0
- ROS2 Humble installed on Jetson
- NVIDIA Isaac ROS packages installed
- ZED SDK installed

## 1. Deploy the Code to Jetson

Your project is already located at `/home/x4/isaac_xavier` on your Jetson. If you need to update it with the latest changes, you can use these methods:

### Option A: Update Existing Repository

```bash
# SSH to your Jetson
ssh x4@192.168.1.83

# Navigate to your project directory
cd /home/x4/isaac_xavier

# Pull latest changes if it's a git repository
git pull origin main

# OR, if you need to update specific files:
# cp -r /path/to/new/files/* .
```

### Option B: Transfer Files from Development Machine

```bash
# From your development machine, not the Jetson
# Transfer the updated files to Jetson using rsync
rsync -avz --exclude='.git/' /Users/randikaprasad/isaac_robotics/ x4@192.168.1.83:/home/x4/isaac_xavier/
```

### Option C: Create Backup and Deploy Fresh

If you need to start fresh while preserving your existing setup:

```bash
# SSH to your Jetson
ssh x4@192.168.1.83

# Backup your existing setup (optional)
cp -r /home/x4/isaac_xavier /home/x4/isaac_xavier_backup_$(date +%Y%m%d)

# Clean the directory (if needed)
# rm -rf /home/x4/isaac_xavier/*

# Then transfer new files from development machine
# rsync -avz --exclude='.git/' /Users/randikaprasad/isaac_robotics/ x4@192.168.1.83:/home/x4/isaac_xavier/
```

## 2. Build the Workspace on Jetson

SSH into your Jetson and build the workspace:

### Step 1: Install Dependencies

```bash
# SSH to your Jetson
ssh x4@192.168.1.83

# Navigate to workspace
cd /home/x4/isaac_xavier

# Install missing ROS2 build dependencies
sudo apt-get update
sudo apt-get install -y python3-pip python3-vcstool python3-rosdep \
    python3-colcon-common-extensions ros-humble-ament-cmake-core \
    ros-humble-ament-cmake-ros ros-humble-ament-cmake-python

# Install NVIDIA VPI (Vision Programming Interface)
# (Required for isaac_ros_common)
sudo apt-get install -y nvidia-vpi

# Make sure NVIDIA VPI is in your CMAKE_PREFIX_PATH
echo 'export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:/opt/nvidia/vpi1' >> ~/.bashrc
source ~/.bashrc

# Install ODrive Python library
pip install odrive fibre

# Initialize rosdep if not done before
sudo rosdep init  # Skip if already initialized
rosdep update

# Install ROS dependencies
rosdep install --from-paths src --ignore-src -r -y

# Install common dependencies that are needed
sudo apt-get install -y ros-humble-navigation2 ros-humble-nav2-bringup \
    ros-humble-slam-toolbox ros-humble-joint-state-publisher \
    ros-humble-robot-state-publisher ros-humble-xacro
```

### Step 2: Build with COLCON_IGNORE to Handle Dependencies

The best way to handle dependency issues is to use COLCON_IGNORE files to temporarily skip problematic packages:

```bash
# Source ROS environment
source /opt/ros/humble/setup.bash

# Create COLCON_IGNORE files in problematic package directories
touch src/isaac_ros_common/COLCON_IGNORE
touch src/isaac_ros_nitros/COLCON_IGNORE
touch src/isaac_ros_visual_slam/COLCON_IGNORE
touch src/isaac_ros_nova/COLCON_IGNORE
touch src/isaac_ros_image_pipeline/COLCON_IGNORE
touch src/isaac_ros_dnn_inference/COLCON_IGNORE
touch src/vision_msgs/COLCON_IGNORE
touch src/vision_opencv/COLCON_IGNORE
touch src/zed-ros2-wrapper/COLCON_IGNORE

# Now build only the tracked robot packages
colcon build --symlink-install --packages-select tracked_robot tracked_robot_msgs
```

This approach keeps all the proper dependencies in your package.xml but tells colcon to ignore building certain packages, allowing your core robot functionality to build successfully.

Alternatively, if you prefer a simpler approach:

```bash
# Build only specific packages without creating COLCON_IGNORE files
COLCON_IGNORE_SKIP_UNTIL_BUILD=true colcon build --symlink-install --packages-select tracked_robot tracked_robot_msgs
```

This should build our tracked_robot package without requiring all the dependencies that are causing issues.

### Step 3 (Optional): Try Building Full System

If you want to eventually build the full system with Isaac ROS, you'll need to properly install NVIDIA components:

```bash
# For Isaac ROS, follow the official installation guide:
# https://nvidia-isaac-ros.github.io/getting_started/index.html

# Example (specific steps may vary):
sudo apt-get install -y nvidia-container-toolkit nvidia-docker2
sudo systemctl restart docker

# Clone and build Isaac ROS packages separately following NVIDIA's instructions
```

## 3. Set Up Hardware and Test Components

### Step 1: Run the Setup Assistant

The setup assistant will help you configure each component:

```bash
# Source the ROS environment
source /opt/ros/humble/setup.bash

# Make sure setup scripts are executable
chmod +x /home/x4/isaac_xavier/src/tracked_robot/scripts/*.py

# Run the setup assistant
/home/x4/isaac_xavier/src/tracked_robot/scripts/setup_advanced_robot.py
```

Follow the interactive menu to:
1. Set up ODrive motor controller
2. Set up ZED 2i camera
3. Check ROS2 workspace
4. Test each component individually

### Step 2: Configure ODrive Controller

If you prefer to configure the ODrive separately:

```bash
# Run the ODrive setup script directly
/home/x4/isaac_xavier/src/tracked_robot/scripts/odrive_setup.py
```

This script will:
- Detect your ODrive
- Configure parameters for your 1kW BLDC motors
- Calibrate the motors (follow on-screen instructions)
- Test basic movement

### Step 3: Configure ZED Camera

If you prefer to configure the ZED camera separately:

```bash
# Run the ZED setup script directly
/home/x4/isaac_xavier/src/tracked_robot/scripts/zed_camera_setup.py
```

This script will:
- Test connection to your ZED 2i camera
- Save calibration parameters
- Test ROS2 integration
- Check for compatibility with NVIDIA Isaac

## 4. Test Full System Integration

After setting up the individual components, test the full system:

```bash
# Source ROS and workspace
source /opt/ros/humble/setup.bash
source /home/x4/isaac_xavier/install/setup.bash

# Launch the advanced robot with all components
ros2 launch tracked_robot advanced_robot.launch.py
```

### Verify Each Functionality

After launching the system, test each major functionality:

#### 1. ODrive Motor Control

```bash
# Move the robot forward for 2 seconds then stop
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
sleep 2
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Rotate the robot for 2 seconds then stop
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}"
sleep 2
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

#### 2. ZED Camera and Visual SLAM

In a separate terminal:

```bash
# View the camera feed
ros2 run image_tools showimage --ros-args -r image:=/zed2i/rgb/image_rect_color

# Monitor visual SLAM status
ros2 topic echo /visual_slam/status
```

#### 3. Auto-Docking

To test the docking system (ensure the ArUco marker is set up on the dock):

```bash
# Start the docking process
ros2 service call /dock/start std_srvs/srv/Trigger

# Monitor docking status
ros2 topic echo /dock/state

# If needed, stop the docking process
ros2 service call /dock/stop std_srvs/srv/Trigger
```

## 5. Common Troubleshooting

### ODrive Issues

1. **Cannot connect to ODrive**:
   - Check USB connection
   - Verify power supply
   - Try `sudo chmod 666 /dev/ttyUSB*`

2. **Motor errors during calibration**:
   - Ensure motors can move freely
   - Check power supply capacity (needs to handle 1kW motors)
   - Adjust current limit in `odrive_setup.py` if needed

### ZED Camera Issues

1. **Camera not detected**:
   - Ensure it's connected to a USB 3.0 port (blue)
   - Try `sudo lsusb` to verify USB connection
   - Check `zed-diagnostic` tool from ZED SDK

2. **Topics not publishing**:
   - Check if ZED wrapper is running: `ros2 node list | grep zed`
   - Verify camera permissions: `sudo chmod 666 /dev/video*`

### ROS2/Isaac Issues

1. **ROS nodes not starting**:
   - Check error messages in the launch output
   - Verify dependencies with `rosdep check --from-paths src`
   - Make sure your Jetson has enough resources (memory/CPU)

2. **Navigation errors**:
   - Verify your TF tree is correct: `ros2 run tf2_tools view_frames`
   - Check if map is being generated: `ros2 topic echo /map -n 1`

## 6. Monitoring System Performance

Monitor system resource usage while running:

```bash
# Monitor CPU/GPU usage and temperature
sudo tegrastats

# Monitor ROS2 topic rates (in a separate terminal)
ros2 topic hz /zed2i/rgb/image_rect_color
ros2 topic hz /visual_slam/tracking/odometry
ros2 topic hz /joint_states
```

## 7. Creating Maps and Saving Them

Once your robot is operational, you can create and save maps:

```bash
# Start SLAM mode
ros2 launch tracked_robot advanced_robot.launch.py use_slam:=true

# After mapping, save the map
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: 'my_map'}}"
```

## Next Steps

After successful deployment and testing:

1. Fine-tune navigation parameters in `config/nav2_params.yaml`
2. Adjust the auto-docking parameters if needed
3. Create a startup service to auto-launch on boot
