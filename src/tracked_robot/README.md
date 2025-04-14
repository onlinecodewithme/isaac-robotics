# ODrive ROS 2 Differential Drive Control

This package provides a complete solution for controlling ODrive-powered differential drive robots with ROS 2.

## Setup

1. Install the udev rules for ODrive:
```bash
python3 src/tracked_robot/scripts/setup_odrive.py
```

2. Test direct motor control (no encoder required):
```bash
python3 src/tracked_robot/scripts/direct_current_control.py --axis 0
```

3. Calibrate motors (if needed):
```bash
python3 src/tracked_robot/scripts/all_in_one_calibration.py --axis 0
```

## Scripts

### Motor Setup and Testing
- **setup_odrive.py**: Sets up permissions and tests basic functionality
- **direct_current_control.py**: Control motors directly with current commands (works even with uncalibrated encoders)
- **all_in_one_calibration.py**: Complete calibration script with error handling
- **improved_hall_calibration.py**: Specialized Hall sensor calibration
- **force_velocity_test.py**: Test velocity control with minimal calibration

### ROS 2 Integration
- **ros2_diff_drive.py**: The main ROS 2 node for controlling your differential drive robot

## Using the ROS 2 Node

The `ros2_diff_drive.py` script can be run in two modes:

### Standalone Mode (without ROS)
```bash
python3 src/tracked_robot/scripts/ros2_diff_drive.py --standalone --left_axis 0 --right_axis 1
```
This allows you to test the motors with keyboard controls.

### ROS 2 Mode
```bash
# In your ROS 2 workspace after building and sourcing
ros2 run tracked_robot ros2_diff_drive.py
```

## Troubleshooting

### Motors Not Moving
If your motors are not moving, try these steps:

1. Check power connections (48V) to the motors
2. Verify motor phases are correctly connected to ODrive
3. Try direct current control with higher current values:
   ```bash
   python3 src/tracked_robot/scripts/direct_current_control.py --axis 0
   ```
   Enter values like 2.0 or 3.0 to apply more current

4. Check for errors when attempting to enter closed loop control
5. Make sure the motor was successfully calibrated

### Permission Issues
If you see "Device permissions are not set up", run:
```bash
sudo bash -c "curl https://cdn.odriverobotics.com/files/odrive-udev-rules.rules > /etc/udev/rules.d/91-odrive.rules && udevadm control --reload-rules && udevadm trigger"
```
Then log out and log back in, or try reconnecting the ODrive USB cable.
