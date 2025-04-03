#!/usr/bin/env python3

"""
Main Setup Script for Advanced Tracked Robot

This script helps set up and configure all components of the Advanced Tracked Robot,
including ODrive motor controller, ZED 2i camera, and ROS2 integration.
It guides the user through the setup process and checks for any issues.

Usage:
    python3 setup_advanced_robot.py

Run this script on your Jetson Orin NX with all hardware components connected.
"""

import os
import sys
import time
import subprocess
import argparse
import platform
import shutil
from pathlib import Path

class SetupManager:
    def __init__(self):
        self.script_dir = Path(os.path.dirname(os.path.abspath(__file__)))
        self.repo_dir = self.script_dir.parent.parent  # Tracked robot repo root directory
        
        # Setup status
        self.hardware_detected = {
            'odrive': False,
            'zed': False,
            'jetson': False
        }
        
        self.packages_installed = {
            'ros2': False,
            'isaac_ros': False,
            'zed_sdk': False,
            'zed_ros': False,
            'odrive_lib': False
        }
    
    def check_hardware(self):
        """Detect connected hardware"""
        print("\n--- Checking Hardware ---")
        
        # Check if we're on a Jetson
        try:
            with open('/etc/nv_tegra_release', 'r') as f:
                self.hardware_detected['jetson'] = True
                jetson_info = f.read().strip()
                print(f"Detected Jetson: {jetson_info}")
        except:
            print("Not running on a Jetson device")
        
        # Check for ODrive
        try:
            import odrive
            self.packages_installed['odrive_lib'] = True
            try:
                odrv = odrive.find_any(timeout=3)
                self.hardware_detected['odrive'] = True
                print(f"Detected ODrive: {odrv.serial_number}")
            except:
                print("ODrive controller not found")
        except ImportError:
            print("ODrive Python library not installed")
        
        # Check for ZED camera
        try:
            import pyzed.sl as sl
            self.packages_installed['zed_sdk'] = True
            
            # Try to access the camera
            cam = sl.Camera()
            init_params = sl.InitParameters()
            init_params.camera_resolution = sl.RESOLUTION.HD720
            
            status = cam.open(init_params)
            if status == sl.ERROR_CODE.SUCCESS:
                self.hardware_detected['zed'] = True
                cam_info = cam.get_camera_information()
                print(f"Detected ZED Camera: {cam_info.camera_model} (Serial: {cam_info.serial_number})")
                cam.close()
            else:
                print("ZED camera detected but failed to open")
        except ImportError:
            print("ZED SDK not installed")
        except Exception as e:
            print(f"Error checking for ZED camera: {str(e)}")
    
    def check_software(self):
        """Check for required software packages"""
        print("\n--- Checking Software ---")
        
        # Check ROS2 installation
        try:
            result = subprocess.run(
                "source /opt/ros/humble/setup.bash && ros2 --version",
                shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True
            )
            if "humble" in result.stdout.lower():
                self.packages_installed['ros2'] = True
                print(f"ROS2 Humble detected: {result.stdout.strip()}")
            else:
                print(f"ROS2 version: {result.stdout.strip()}. ROS2 Humble recommended.")
        except:
            print("ROS2 not detected")
        
        # Check ZED ROS2 wrapper
        try:
            result = subprocess.run(
                "source /opt/ros/humble/setup.bash && ros2 pkg list | grep zed_wrapper",
                shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True
            )
            if "zed_wrapper" in result.stdout:
                self.packages_installed['zed_ros'] = True
                print("ZED ROS2 wrapper detected")
            else:
                print("ZED ROS2 wrapper not detected")
        except:
            print("Error checking for ZED ROS2 wrapper")
        
        # Check NVIDIA Isaac ROS packages
        try:
            result = subprocess.run(
                "source /opt/ros/humble/setup.bash && ros2 pkg list | grep isaac_ros",
                shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True
            )
            if "isaac_ros" in result.stdout:
                self.packages_installed['isaac_ros'] = True
                print("NVIDIA Isaac ROS packages detected")
            else:
                print("NVIDIA Isaac ROS packages not detected")
        except:
            print("Error checking for NVIDIA Isaac ROS packages")
        
        # Summary
        print("\nSoftware status summary:")
        for pkg, status in self.packages_installed.items():
            print(f"  {pkg}: {'Installed' if status else 'Not installed'}")
    
    def setup_odrive(self):
        """Set up ODrive controller"""
        print("\n--- Setting up ODrive Motor Controller ---")
        
        if not self.packages_installed['odrive_lib']:
            choice = input("ODrive Python library not detected. Install it? (y/n): ")
            if choice.lower() == 'y':
                subprocess.run("pip install odrive", shell=True)
                print("Installation completed. Please restart this script.")
                return
        
        if not self.hardware_detected['odrive']:
            print("\nODrive controller not detected.")
            print("Please make sure your ODrive is:")
            print("  1. Connected via USB")
            print("  2. Powered on")
            print("  3. Not in use by another application")
            print("\nWould you like to attempt setup anyway?")
            choice = input("Continue with ODrive setup? (y/n): ")
            if choice.lower() != 'y':
                return
        
        # Run ODrive setup script
        odrive_script = self.script_dir / "odrive_setup.py"
        
        print(f"\nRunning ODrive setup script: {odrive_script}")
        print("This will configure your ODrive for use with the tracked robot.")
        input("Press Enter to continue or Ctrl+C to cancel...")
        
        subprocess.run(f"python3 {odrive_script}", shell=True)
    
    def setup_zed_camera(self):
        """Set up ZED camera"""
        print("\n--- Setting up ZED Camera ---")
        
        if not self.packages_installed['zed_sdk']:
            print("\nZED SDK not detected. Please install it from:")
            print("https://www.stereolabs.com/developers/release/")
            return
        
        if not self.hardware_detected['zed']:
            print("\nZED camera not detected.")
            print("Please make sure your ZED camera is:")
            print("  1. Connected to a USB 3.0 port (blue)")
            print("  2. Not in use by another application")
            print("\nWould you like to attempt setup anyway?")
            choice = input("Continue with ZED camera setup? (y/n): ")
            if choice.lower() != 'y':
                return
        
        # Run ZED camera setup script
        zed_script = self.script_dir / "zed_camera_setup.py"
        
        print(f"\nRunning ZED camera setup script: {zed_script}")
        print("This will test your ZED camera and save calibration data.")
        input("Press Enter to continue or Ctrl+C to cancel...")
        
        subprocess.run(f"python3 {zed_script}", shell=True)
    
    def check_ros_workspace(self):
        """Check ROS workspace configuration"""
        print("\n--- Checking ROS2 Workspace ---")
        
        # Check if the workspace is built
        build_dir = self.repo_dir / "build"
        install_dir = self.repo_dir / "install"
        
        if not build_dir.exists() or not install_dir.exists():
            print("Workspace doesn't appear to be built yet.")
            print("You should build it with:")
            print("  cd /home/x4/isaac_robotics")
            print("  colcon build --symlink-install")
            return False
        
        # Check for tracked_robot package in the installation
        tracked_robot_install = install_dir / "tracked_robot"
        if not tracked_robot_install.exists():
            print("tracked_robot package not found in the installation directory.")
            print("You should build the workspace with:")
            print("  cd /home/x4/isaac_robotics")
            print("  colcon build --symlink-install")
            return False
        
        print("ROS2 workspace appears to be properly built.")
        return True
    
    def test_run_system(self):
        """Test run the full system"""
        print("\n--- Test Run System ---")
        
        if not self.check_ros_workspace():
            print("Cannot test run until the workspace is properly built.")
            return
        
        print("\nThis will launch the advanced robot system to test all components.")
        print("The system will run for 30 seconds and then be terminated.")
        choice = input("Proceed with test run? (y/n): ")
        
        if choice.lower() != 'y':
            return
        
        print("\nStarting advanced robot system...")
        
        # Source ROS workspace and launch advanced robot
        cmd = (
            "cd /home/x4/isaac_robotics && "
            "source /opt/ros/humble/setup.bash && "
            "source install/setup.bash && "
            "ros2 launch tracked_robot advanced_robot.launch.py"
        )
        
        print("\nLaunching: ros2 launch tracked_robot advanced_robot.launch.py")
        print("Press Ctrl+C after testing to stop the system.")
        
        subprocess.run(cmd, shell=True)
    
    def run_setup(self):
        """Run the complete setup process"""
        print("\n====================================================")
        print("      Advanced Tracked Robot Setup Assistant")
        print("====================================================")
        
        print("\nThis script will help you set up all components for the Advanced Tracked Robot.")
        print("Make sure your ODrive and ZED 2i camera are connected before proceeding.")
        
        # Check if we're on a Jetson Orin NX
        if platform.machine() != 'aarch64' or not os.path.exists('/etc/nv_tegra_release'):
            print("\nWarning: This does not appear to be a Jetson Orin NX.")
            print("Setup will proceed, but some components may not work correctly.")
            choice = input("Continue anyway? (y/n): ")
            if choice.lower() != 'y':
                return
        
        # Check hardware and software
        self.check_hardware()
        self.check_software()
        
        # Setup menu
        while True:
            print("\n--- Setup Menu ---")
            print("1. Set up ODrive motor controller")
            print("2. Set up ZED 2i camera")
            print("3. Check ROS2 workspace")
            print("4. Test run system")
            print("5. Exit")
            
            choice = input("\nEnter your choice (1-5): ")
            
            if choice == '1':
                self.setup_odrive()
            elif choice == '2':
                self.setup_zed_camera()
            elif choice == '3':
                self.check_ros_workspace()
            elif choice == '4':
                self.test_run_system()
            elif choice == '5':
                print("\nExiting setup assistant.")
                break
            else:
                print("Invalid choice. Please try again.")

def main():
    """Main entry point"""
    parser = argparse.ArgumentParser(description='Advanced Tracked Robot Setup')
    parser.add_argument('--no-interactive', action='store_true', help='Run all checks without interactive prompts')
    args = parser.parse_args()
    
    setup = SetupManager()
    
    if args.no_interactive:
        # Just run checks without interactive setup
        setup.check_hardware()
        setup.check_software()
        setup.check_ros_workspace()
    else:
        # Run interactive setup
        setup.run_setup()

if __name__ == "__main__":
    main()
