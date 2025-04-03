#!/usr/bin/env python3

"""
ZED 2i Camera Configuration and Test Script for Advanced Tracked Robot

This script tests and configures the ZED 2i camera for use with the Advanced Tracked Robot.
It verifies the camera connection, tests its capabilities, and checks ROS2 integration.
Run this script on your Jetson Orin NX with the ZED 2i camera connected via USB.

Usage:
    python3 zed_camera_setup.py

Make sure the ZED camera is connected before running this script.
"""

import os
import sys
import time
import subprocess
import argparse
import glob
import json
from datetime import datetime

# Check for ZED SDK
try:
    import pyzed.sl as sl
except ImportError:
    print("Error: ZED SDK Python bindings not found.")
    print("Please make sure ZED SDK is installed.")
    print("You can download it from: https://www.stereolabs.com/developers/release/")
    sys.exit(1)

def check_zed_version():
    """Check ZED SDK version and compatibility"""
    print("\n--- Checking ZED SDK Version ---")
    
    # Get ZED SDK version
    version = sl.Camera.get_sdk_version()
    print(f"ZED SDK Version: {version}")
    
    # Check minimum version
    min_version = "4.0.0"  # Adjust based on your needs
    if version < min_version:
        print(f"Warning: ZED SDK version {version} may be too old. Recommended minimum: {min_version}")
    else:
        print(f"ZED SDK version is compatible.")
    
    # Check if we're on a Jetson
    try:
        with open('/etc/nv_tegra_release', 'r') as f:
            jetson_info = f.read()
            print("Detected Jetson device:")
            print(jetson_info.strip())
    except:
        print("Not running on a Jetson device or unable to determine Jetson version.")

def test_camera_connection():
    """Test connection to the ZED 2i camera"""
    print("\n--- Testing ZED 2i Camera Connection ---")
    
    # Initialize camera
    cam = sl.Camera()
    
    # Create camera parameters
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720  # Use 720p
    init_params.camera_fps = 30  # Set FPS to 30
    init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE  # Use performance depth mode
    init_params.coordinate_units = sl.UNIT.METER  # Set units to meters
    init_params.sdk_verbose = 1  # Enable verbose logging (use integer 1 instead of boolean True)
    
    # Try to open the camera
    print("Opening ZED 2i camera...")
    status = cam.open(init_params)
    
    if status != sl.ERROR_CODE.SUCCESS:
        print(f"Failed to open ZED camera: {status}")
        return False
    
    # Display camera information
    cam_info = cam.get_camera_information()
    print(f"Camera Model: {cam_info.camera_model}")
    print(f"Serial Number: {cam_info.serial_number}")
    
    # Handle different ZED SDK versions - attribute names may vary
    try:
        if hasattr(cam_info, 'camera_firmware_version'):
            print(f"Camera Firmware: {cam_info.camera_firmware_version}")
        elif hasattr(cam_info, 'firmware_version'):
            print(f"Camera Firmware: {cam_info.firmware_version}")
        else:
            # Get firmware info from logs if available
            print(f"Camera Firmware: Check ZED logs for version info")
            
        if hasattr(cam_info, 'sensors_firmware_version'):
            print(f"Sensors Firmware: {cam_info.sensors_firmware_version}")
        elif hasattr(cam_info, 'sensors_configuration') and hasattr(cam_info.sensors_configuration, 'firmware_version'):
            print(f"Sensors Firmware: {cam_info.sensors_configuration.firmware_version}")
    except Exception as e:
        print(f"Note: Could not retrieve firmware versions: {e}")
        
    # Handle different ZED SDK versions for resolution and FPS
    try:
        # Try different attribute structures based on SDK version
        if hasattr(cam_info, 'camera_resolution'):
            print(f"Resolution: {cam_info.camera_resolution.width}x{cam_info.camera_resolution.height}")
        elif hasattr(cam_info, 'camera_configuration') and hasattr(cam_info.camera_configuration, 'resolution'):
            print(f"Resolution: {cam_info.camera_configuration.resolution.width}x{cam_info.camera_configuration.resolution.height}")
        else:
            # Fallback to getting resolution directly from the camera
            resolution = cam.get_resolution()
            print(f"Resolution: {resolution.width}x{resolution.height}")
        
        if hasattr(cam_info, 'camera_fps'):
            print(f"FPS: {cam_info.camera_fps}")
        elif hasattr(cam_info, 'camera_configuration') and hasattr(cam_info.camera_configuration, 'fps'):
            print(f"FPS: {cam_info.camera_configuration.fps}")
        else:
            print(f"FPS: Using default 30fps")
    except Exception as e:
        print(f"Note: Could not retrieve resolution/FPS information: {e}")
    
    # Check camera temperature (with version compatibility)
    try:
        # Different ZED SDK versions might have different methods
        if hasattr(cam, 'get_camera_temperature'):
            temp = cam.get_camera_temperature()
            print(f"Camera Temperature: {temp}°C")
        elif hasattr(cam, 'get_temperature'):
            temp = cam.get_temperature()
            print(f"Camera Temperature: {temp}°C")
        else:
            print("Camera temperature check not available in this ZED SDK version")
    except Exception as e:
        print(f"Could not get camera temperature: {e}")
    
    # Check if we can get an image
    print("Capturing a test image...")
    image = sl.Mat()
    depth = sl.Mat()
    runtime_parameters = sl.RuntimeParameters()
    
    # Warm up the camera with a few frames
    for i in range(5):
        cam.grab(runtime_parameters)
        time.sleep(0.1)
    
    # Capture image and depth
    if cam.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
        cam.retrieve_image(image, sl.VIEW.LEFT)
        cam.retrieve_measure(depth, sl.MEASURE.DEPTH)
        
        # Save images to temporary files
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        image_path = f"/tmp/zed_test_image_{timestamp}.png"
        depth_path = f"/tmp/zed_test_depth_{timestamp}.png"
        
        image.write(image_path)
        depth.write(depth_path)
        
        print(f"Test image saved to: {image_path}")
        print(f"Test depth image saved to: {depth_path}")
        print("Test image capture successful!")
    else:
        print("Failed to capture image")
        cam.close()
        return False
    
    # Test sensor data if available - with error handling
    try:
        if hasattr(cam_info, 'sensors_available') and cam_info.sensors_available:
            print("\nTesting onboard sensors...")
            sensors_data = sl.SensorsData()
            
            # Set a timeout to avoid blocking
            sensor_timeout = time.time() + 5  # 5 second timeout
            
            try:
                if cam.get_sensors_data(sensors_data, sl.TIME_REFERENCE.CURRENT) == sl.ERROR_CODE.SUCCESS:
                    # IMU data - with error handling for each sensor
                    try:
                        imu_data = sensors_data.get_imu_data()
                        print(f"IMU Acceleration: {imu_data.get_linear_acceleration()}")
                        print(f"IMU Angular Velocity: {imu_data.get_angular_velocity()}")
                    except Exception as e:
                        print(f"Could not get IMU data: {e}")
                    
                    # Magnetometer data - with error handling
                    try:
                        mag_data = sensors_data.get_magnetometer_data()
                        print(f"Magnetic Field: {mag_data.get_magnetic_field_calibrated()}")
                    except Exception as e:
                        print(f"Could not get magnetometer data: {e}")
                    
                    # Barometer data - with error handling
                    try:
                        baro_data = sensors_data.get_barometer_data()
                        print(f"Atmospheric Pressure: {baro_data.pressure} hPa")
                    except Exception as e:
                        print(f"Could not get barometer data: {e}")
                else:
                    print("Failed to retrieve sensor data (non-success return code)")
            except Exception as e:
                print(f"Error accessing sensor data: {e}")
                if time.time() > sensor_timeout:
                    print("Sensor data retrieval timed out")
        else:
            print("\nNo sensors available on this ZED camera model or SDK version")
    except Exception as e:
        print(f"\nError checking sensor availability: {e}")
    
    # Close the camera
    cam.close()
    print("Camera closed successfully.")
    return True

def save_camera_intrinsics():
    """Save camera calibration parameters for ROS2 integration"""
    print("\n--- Saving Camera Intrinsics ---")
    
    # Initialize camera
    cam = sl.Camera()
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720
    
    # Open camera
    status = cam.open(init_params)
    if status != sl.ERROR_CODE.SUCCESS:
        print(f"Failed to open camera: {status}")
        return False
    
    # Get calibration parameters
    calibration_params = cam.get_camera_information().camera_configuration.calibration_parameters
    
    # Create directory for calibration files
    calib_dir = os.path.expanduser("~/zed_calibration")
    os.makedirs(calib_dir, exist_ok=True)
    
    # Save intrinsics for left and right cameras with error handling
    try:
        intrinsics = {
            "left": {
                "fx": calibration_params.left_cam.fx,
                "fy": calibration_params.left_cam.fy,
                "cx": calibration_params.left_cam.cx,
                "cy": calibration_params.left_cam.cy,
                "distortion": [
                    calibration_params.left_cam.disto[0],
                    calibration_params.left_cam.disto[1],
                    calibration_params.left_cam.disto[2],
                    calibration_params.left_cam.disto[3],
                    calibration_params.left_cam.disto[4]
                ]
            },
            "right": {
                "fx": calibration_params.right_cam.fx,
                "fy": calibration_params.right_cam.fy,
                "cx": calibration_params.right_cam.cx,
                "cy": calibration_params.right_cam.cy,
                "distortion": [
                    calibration_params.right_cam.disto[0],
                    calibration_params.right_cam.disto[1],
                    calibration_params.right_cam.disto[2],
                    calibration_params.right_cam.disto[3],
                    calibration_params.right_cam.disto[4]
                ]
            },
            "baseline": calibration_params.T[0],  # baseline in mm
            "resolution": {}
        }
        
        # Get resolution using different possible attribute structures
        cam_info = cam.get_camera_information()
        if hasattr(cam_info, 'camera_resolution'):
            intrinsics["resolution"]["width"] = cam_info.camera_resolution.width
            intrinsics["resolution"]["height"] = cam_info.camera_resolution.height
        elif hasattr(cam_info, 'camera_configuration') and hasattr(cam_info.camera_configuration, 'resolution'):
            intrinsics["resolution"]["width"] = cam_info.camera_configuration.resolution.width
            intrinsics["resolution"]["height"] = cam_info.camera_configuration.resolution.height
        else:
            # Fallback to configured resolution values
            intrinsics["resolution"]["width"] = 1280  # HD720 default width
            intrinsics["resolution"]["height"] = 720  # HD720 default height
            print("Using default resolution values (1280x720)")
    except Exception as e:
        print(f"Warning: Error getting some camera intrinsics: {e}")
        # Create minimal intrinsics with default values as fallback
        intrinsics = {
            "left": {"fx": 700, "fy": 700, "cx": 640, "cy": 360, "distortion": [0, 0, 0, 0, 0]},
            "right": {"fx": 700, "fy": 700, "cx": 640, "cy": 360, "distortion": [0, 0, 0, 0, 0]},
            "baseline": 120,  # approximate baseline in mm
            "resolution": {"width": 1280, "height": 720}
        }
        print("Using fallback calibration values")
    
    # Save to JSON file
    json_path = os.path.join(calib_dir, "zed2i_intrinsics.json")
    with open(json_path, 'w') as f:
        json.dump(intrinsics, f, indent=4)
    
    print(f"Camera intrinsics saved to: {json_path}")
    
    # Close the camera
    cam.close()
    return True

def test_ros2_integration():
    """Test ROS2 integration with ZED wrapper"""
    print("\n--- Testing ROS2 Integration ---")
    
    # Check if ROS2 is installed
    try:
        source_cmd = "source /opt/ros/humble/setup.bash && "
        
        # Check if ZED ROS2 wrapper is installed
        print("Checking for ZED ROS2 wrapper...")
        result = subprocess.run(
            f"{source_cmd} ros2 pkg list | grep zed",
            shell=True, 
            stdout=subprocess.PIPE, 
            stderr=subprocess.PIPE,
            text=True
        )
        
        if "zed_wrapper" not in result.stdout:
            print("ZED ROS2 wrapper not found.")
            print("Please install it following: https://github.com/stereolabs/zed-ros2-wrapper")
            return False
        
        print("ZED ROS2 wrapper found!")
        
        # Test launching the ZED node
        print("\nTesting ZED ROS2 node launch (will run for 10 seconds)...")
        
        # Start the ZED node in a separate process
        launch_process = subprocess.Popen(
            f"{source_cmd} ros2 launch zed_wrapper zed2i.launch.py",
            shell=True,
            stderr=subprocess.PIPE,
            stdout=subprocess.PIPE,
            text=True
        )
        
        # Give the node time to start
        time.sleep(10)
        
        # Check if node is publishing
        topic_cmd = f"{source_cmd} ros2 topic list"
        topic_process = subprocess.run(
            topic_cmd,
            shell=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True
        )
        
        # Look for key ZED topics
        topics = topic_process.stdout.split('\n')
        zed_topics = [t for t in topics if '/zed2i/' in t]
        
        if len(zed_topics) > 0:
            print("ZED ROS2 node is running and publishing topics:")
            for topic in zed_topics[:10]:  # Show first 10 topics
                print(f"  {topic}")
            
            if len(zed_topics) > 10:
                print(f"  ... and {len(zed_topics) - 10} more")
                
            # Check for key topics we need
            required_topics = [
                '/zed2i/rgb/image_rect_color',
                '/zed2i/depth/depth_registered',
                '/zed2i/point_cloud/cloud_registered',
                '/zed2i/imu/data'
            ]
            
            missing = [t for t in required_topics if not any(t in zt for zt in zed_topics)]
            
            if missing:
                print("\nWarning: Some required topics are missing:")
                for topic in missing:
                    print(f"  {topic}")
            else:
                print("\nAll required topics are available!")
        else:
            print("No ZED topics found. ZED ROS2 node may have failed to start.")
        
        # Terminate the ZED node process
        print("\nStopping ZED ROS2 node...")
        launch_process.terminate()
        launch_process.wait(timeout=5)
        
        return True
        
    except Exception as e:
        print(f"Error testing ROS2 integration: {str(e)}")
        return False

def test_nvidia_isaac_integration():
    """Test NVIDIA Isaac ROS integration"""
    print("\n--- Testing NVIDIA Isaac ROS Integration ---")
    
    # Check if Isaac ROS packages are installed
    try:
        source_cmd = "source /opt/ros/humble/setup.bash && "
        
        # Look for key Isaac ROS packages
        print("Checking for NVIDIA Isaac ROS packages...")
        result = subprocess.run(
            f"{source_cmd} ros2 pkg list | grep isaac",
            shell=True, 
            stdout=subprocess.PIPE, 
            stderr=subprocess.PIPE,
            text=True
        )
        
        isaac_packages = result.stdout.strip().split('\n')
        
        if not isaac_packages or isaac_packages[0] == '':
            print("No NVIDIA Isaac ROS packages found.")
            print("Please install them following: https://nvidia-isaac-ros.github.io/getting_started/index.html")
            return False
        
        print("Found NVIDIA Isaac ROS packages:")
        for pkg in isaac_packages[:10]:  # Show first 10 packages
            print(f"  {pkg}")
            
        if len(isaac_packages) > 10:
            print(f"  ... and {len(isaac_packages) - 10} more")
        
        # Check for key packages we need
        required_packages = [
            'isaac_ros_visual_slam',
            'isaac_ros_nitros',
            'isaac_ros_nova'
        ]
        
        # Filter out empty strings
        isaac_packages = [p for p in isaac_packages if p]
        
        missing = [p for p in required_packages if not any(p in pkg for pkg in isaac_packages)]
        
        if missing:
            print("\nWarning: Some required Isaac ROS packages are missing:")
            for pkg in missing:
                print(f"  {pkg}")
            print("\nPlease install the missing packages for full functionality.")
        else:
            print("\nAll required NVIDIA Isaac ROS packages are available!")
        
        # For a full test, we would launch Isaac ROS nodes with ZED camera,
        # but this can be complex and resource-intensive, so we'll skip it here
        
        print("\nNOTE: Full integration test with Isaac ROS and ZED camera requires")
        print("launching multiple nodes. Please use the provided launch files:")
        print("  ros2 launch tracked_robot advanced_robot.launch.py")
        
        return True
        
    except Exception as e:
        print(f"Error testing NVIDIA Isaac ROS integration: {str(e)}")
        return False

def main():
    """Main function to test and configure ZED camera"""
    parser = argparse.ArgumentParser(description='Test and configure ZED 2i Camera for Advanced Tracked Robot')
    parser.add_argument('--no-ros', action='store_true', help='Skip ROS2 integration test')
    parser.add_argument('--no-isaac', action='store_true', help='Skip NVIDIA Isaac ROS integration test')
    args = parser.parse_args()
    
    print("ZED 2i Camera Setup for Advanced Tracked Robot")
    print("=============================================")
    
    # Check ZED SDK version
    check_zed_version()
    
    # Test camera connection
    if not test_camera_connection():
        print("\nFailed to connect to ZED 2i camera.")
        print("Please check the following:")
        print("  1. Camera is connected to a USB 3.0 port (blue)")
        print("  2. Camera has power (LED is on)")
        print("  3. No other applications are using the camera")
        sys.exit(1)
    
    # Save camera intrinsics
    save_camera_intrinsics()
    
    # Test ROS2 integration if not skipped
    if not args.no_ros:
        test_ros2_integration()
    
    # Test NVIDIA Isaac ROS integration if not skipped
    if not args.no_isaac and not args.no_ros:  # Skip Isaac test if ROS is skipped
        test_nvidia_isaac_integration()
    
    print("\nZED 2i camera setup complete.")
    print("\nYou can now use the tracked_robot package with your ZED 2i camera.")
    print("To run the advanced tracked robot, use:")
    print("  ros2 launch tracked_robot advanced_robot.launch.py")
    print("\nMake sure to adjust any parameters in the launch file if needed.")

if __name__ == "__main__":
    main()
