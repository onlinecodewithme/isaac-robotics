#!/usr/bin/env python3

"""
ODrive Setup Helper

This script helps set up ODrive by:
1. Checking permissions and installing udev rules if needed
2. Calibrating the motor with minimal configuration
3. Testing motor functionality with direct current control

Usage:
    python3 setup_odrive.py [--axis 0|1]
"""

import sys
import os
import time
import argparse
import subprocess
import signal

try:
    import odrive
    from odrive.enums import *
    HAS_ODRIVE = True
except ImportError:
    print("ODrive Python library not found. Installing...")
    try:
        subprocess.check_call([sys.executable, "-m", "pip", "install", "odrive"])
        print("ODrive installed. Please restart this script.")
        HAS_ODRIVE = False
    except Exception as e:
        print(f"Failed to install ODrive: {e}")
        print("Please install manually: pip install odrive")
        HAS_ODRIVE = False

def setup_udev_rules():
    """Set up udev rules for ODrive"""
    print("\n=== Step 1: Setting up ODrive permissions ===")
    
    # Check if we can run as root
    if os.geteuid() == 0:
        # We're already root
        install_rules()
    else:
        # Try with sudo
        print("This script needs to install udev rules for ODrive access.")
        print("You may be prompted for your password.")
        try:
            cmd = [
                "sudo", "bash", "-c",
                "curl https://cdn.odriverobotics.com/files/odrive-udev-rules.rules > /etc/udev/rules.d/91-odrive.rules && udevadm control --reload-rules && udevadm trigger"
            ]
            subprocess.run(cmd, check=True)
            print("✓ ODrive udev rules installed successfully")
            return True
        except subprocess.CalledProcessError as e:
            print(f"! Failed to install udev rules: {e}")
            print("\nPlease run the following command manually:")
            print("sudo bash -c \"curl https://cdn.odriverobotics.com/files/odrive-udev-rules.rules > /etc/udev/rules.d/91-odrive.rules && udevadm control --reload-rules && udevadm trigger\"")
            return False

def install_rules():
    """Install the rules directly (as root)"""
    try:
        # Download the rules file
        import urllib.request
        rules_url = "https://cdn.odriverobotics.com/files/odrive-udev-rules.rules"
        rules_file = "/etc/udev/rules.d/91-odrive.rules"
        
        print("Downloading udev rules...")
        urllib.request.urlretrieve(rules_url, rules_file)
        
        # Apply the rules
        subprocess.run(["udevadm", "control", "--reload-rules"], check=True)
        subprocess.run(["udevadm", "trigger"], check=True)
        
        print("✓ ODrive udev rules installed successfully")
        return True
    except Exception as e:
        print(f"! Failed to install udev rules: {e}")
        return False

def connect_to_odrive():
    """Attempt to connect to ODrive"""
    print("\n=== Step 2: Connecting to ODrive ===")
    print("Looking for ODrive...")
    
    try:
        odrv = odrive.find_any(timeout=10)
        print(f"✓ Connected to ODrive {odrv.serial_number}")
        return odrv
    except Exception as e:
        print(f"! Failed to connect to ODrive: {e}")
        print("\nTroubleshooting tips:")
        print("1. Make sure the ODrive is powered and connected via USB")
        print("2. You may need to disconnect and reconnect the USB cable")
        print("3. You may need to log out and back in for udev rules to take effect")
        return None

def simple_motor_test(odrv, axis_num):
    """Perform a simple motor test with current control"""
    print(f"\n=== Step 3: Testing motor {axis_num} ===")
    
    if not odrv:
        print("! No ODrive connected")
        return False
    
    # Get the axis object
    axis = getattr(odrv, f"axis{axis_num}")
    
    try:
        # Display current state
        print(f"Motor calibrated: {axis.motor.is_calibrated}")
        print(f"Encoder ready: {axis.encoder.is_ready}")
        print(f"Current state: {axis.current_state}")
        
        # Clear errors
        if hasattr(axis, 'error'):
            axis.error = 0
        if hasattr(axis.motor, 'error'):
            axis.motor.error = 0
        if hasattr(axis.encoder, 'error'):
            axis.encoder.error = 0
        if hasattr(axis.controller, 'error'):
            axis.controller.error = 0
        
        print("\nSetting to idle state...")
        axis.requested_state = AXIS_STATE_IDLE
        time.sleep(1.0)
        
        # Configure for current control
        print("Setting up current control mode...")
        axis.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL
        axis.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
        axis.motor.config.current_lim = 15.0  # Higher limit for 1000W motors
        
        # Enable motor
        print("Entering closed loop control...")
        axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        time.sleep(1.0)
        
        # Check if we succeeded
        if axis.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
            print(f"! Failed to enter closed loop control. State: {axis.current_state}")
            if hasattr(axis, 'error') and axis.error != 0:
                print(f"! Axis error: {axis.error}")
            
            # Try to forcibly clear errors and try again
            print("\nAttempting to force enter closed loop control...")
            axis.error = 0
            axis.motor.error = 0
            axis.encoder.error = 0
            axis.controller.error = 0
            
            # Try to enter closed loop control again
            axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            time.sleep(1.0)
            
            if axis.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
                print(f"! Second attempt failed. State: {axis.current_state}")
                return False
            else:
                print("✓ Forced into closed loop control")
        else:
            print("✓ Entered closed loop control")
        
        # Test with a gentle current
        print("\nTESTING MOTOR")
        print("Applying small current (1.0A) for 2 seconds...")
        
        axis.controller.input_torque = 1.0
        time.sleep(2.0)
        
        # Stop
        axis.controller.input_torque = 0.0
        print("Stopped motor.")
        
        # Prompt user for result
        result = input("\nDid the motor move? (y/n): ")
        if result.lower() == 'y':
            print("✓ Motor test successful!")
            return True
        else:
            print("! Motor did not move.")
            print("\nTroubleshooting tips:")
            print("1. Check motor power connections (48V)")
            print("2. Try increasing current: Run `python3 direct_current_control.py --axis {axis_num}`")
            print("3. Check that motor phases (U,V,W) are properly connected")
            return False
        
    except Exception as e:
        print(f"! Error during motor test: {e}")
        return False
    finally:
        # Always stop the motor
        try:
            if hasattr(axis, 'controller') and hasattr(axis.controller, 'input_torque'):
                axis.controller.input_torque = 0.0
            if hasattr(axis, 'requested_state'):
                axis.requested_state = AXIS_STATE_IDLE
        except:
            pass

def main():
    # Parse arguments
    parser = argparse.ArgumentParser(description="ODrive Setup Helper")
    parser.add_argument('--axis', type=int, default=0, choices=[0, 1],
                        help="Motor axis to test (0 or 1)")
    args = parser.parse_args()
    
    print("=" * 60)
    print("  ODRIVE SETUP HELPER")
    print("=" * 60)
    print("This script will help you set up and test your ODrive controller.")
    print("Make sure your ODrive is connected via USB and powered on.")
    
    # Skip odrive-specific steps if the library isn't available
    if not HAS_ODRIVE:
        sys.exit(1)
    
    # Set up udev rules
    setup_udev_rules()
    
    # Connect to ODrive
    odrv = connect_to_odrive()
    if not odrv:
        sys.exit(1)
    
    # Test motor
    simple_motor_test(odrv, args.axis)
    
    print("\n=== Setup Complete ===")
    print("You can now use the direct_current_control.py script for more testing:")
    print(f"python3 src/tracked_robot/scripts/direct_current_control.py --axis {args.axis}")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nSetup cancelled by user")
        sys.exit(0)
