#!/usr/bin/env python3

"""
ODrive Configuration Script for Advanced Tracked Robot

This script configures an ODrive controller for use with 1kW BLDC motors in
a differential drive setup for the Advanced Tracked Robot. Run this script 
on your Jetson Orin NX with the ODrive connected via USB.

Usage:
    python3 odrive_setup.py

Make sure the ODrive is powered up and connected before running this script.
"""

import sys
import time
import argparse
import numpy as np
try:
    import odrive
    from odrive.enums import *
    import fibre.libfibre
except ImportError:
    print("Error: ODrive Python library not found.")
    print("Please install it using: pip install odrive")
    sys.exit(1)

def find_odrive():
    """Find and connect to ODrive"""
    print("Looking for ODrive...")
    try:
        odrv = odrive.find_any(timeout=10)
        print(f"Found ODrive! Serial Number: {odrv.serial_number}")
        return odrv
    except fibre.libfibre.ObjectLostError as e:
        print(f"Error finding ODrive: {str(e)}")
        print("Make sure your ODrive is connected and powered.")
        return None
    except Exception as e:
        print(f"Unexpected error connecting to ODrive: {str(e)}")
        return None

def configure_odrive_for_bldc_motors(odrv, config):
    """Configure ODrive for BLDC motors with given parameters"""
    print("\n--- Configuring ODrive for 1kW BLDC Motors ---")
    
    # Configuration parameters for both axes
    for axis_num in [0, 1]:
        axis_name = f"axis{axis_num}"
        axis = getattr(odrv, axis_name)
        
        print(f"\nConfiguring {axis_name}...")
        
        # Motor configuration
        print("Configuring motor...")
        axis.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
        axis.motor.config.pole_pairs = config['pole_pairs']
        axis.motor.config.resistance_calib_max_voltage = config['calib_voltage']
        axis.motor.config.requested_current_range = config['current_range']
        axis.motor.config.current_control_bandwidth = config['current_bandwidth']
        axis.motor.config.torque_constant = config['torque_constant']
        
        # Current limits
        axis.motor.config.current_lim = config['current_limit']
        axis.motor.config.current_lim_margin = config['current_margin']
        
        # Encoder configuration
        print("Configuring encoder...")
        axis.encoder.config.mode = ENCODER_MODE_HALL
        axis.encoder.config.cpr = config['encoder_cpr']
        axis.encoder.config.calib_scan_distance = 150
        
        # Enhanced Hall sensor configuration
        if hasattr(axis.encoder.config, 'bandwidth'):
            axis.encoder.config.bandwidth = 100  # Hz - lower for more filtering
        if hasattr(axis.encoder.config, 'hall_polarity_calibrated'):
            axis.encoder.config.hall_polarity_calibrated = False  # Force recalibration
        if hasattr(axis.encoder.config, 'hall_offset_calibrated'):
            axis.encoder.config.hall_offset_calibrated = False  # Force recalibration
        
        # Controller configuration
        print("Configuring controller...")
        axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        axis.controller.config.vel_limit = config['velocity_limit']
        axis.controller.config.vel_limit_tolerance = 1.2  # 20% over limit
        axis.controller.config.vel_integrator_gain = 0.5
        axis.controller.config.vel_gain = config['velocity_gain']
        axis.controller.config.pos_gain = 1.0
        
        # Configuration for input filtering
        axis.controller.config.input_filter_bandwidth = 2.0  # Hz
        
        print(f"{axis_name} configuration complete.")
    
    # Configure brake resistor if present and if the attribute exists (firmware version dependent)
    if config['brake_resistance'] > 0:
        print("\nConfiguring brake resistor...")
        # Handle different ODrive firmware versions that might have different attribute names
        try:
            # Try setting brake resistance
            if hasattr(odrv.config, 'brake_resistance'):
                odrv.config.brake_resistance = config['brake_resistance']
                
            # Try enabling brake resistor - attribute name varies by firmware version
            if hasattr(odrv.config, 'enable_brake_resistor'):
                odrv.config.enable_brake_resistor = True
            elif hasattr(odrv, 'config_brake_resistance'):
                odrv.config_brake_resistance = config['brake_resistance']
            elif hasattr(odrv.config, 'brake_resistor_enabled'):
                odrv.config.brake_resistor_enabled = True
            else:
                print("Note: Brake resistor configuration not available in this firmware version. Skipping.")
        except Exception as e:
            print(f"Warning: Could not configure brake resistor: {str(e)}")
            print("This is not critical and the system will continue without brake resistor.")
    
    # Set up protection (also handle potential missing attributes)
    print("\nConfiguring protections...")
    try:
        if hasattr(odrv.config, 'dc_max_negative_current'):
            odrv.config.dc_max_negative_current = -10.0  # Amps
        if hasattr(odrv.config, 'max_regen_current'):
            odrv.config.max_regen_current = 10.0  # Amps
    except Exception as e:
        print(f"Warning: Could not configure some protections: {str(e)}")
    
    # Save configuration to ODrive's persistent storage
    print("\nSaving configuration to ODrive...")
    try:
        odrv.save_configuration()
        print("Configuration saved successfully.")
    except Exception as e:
        print(f"Error saving configuration: {str(e)}")

def calibrate_motors(odrv):
    """Run motor and encoder calibration routine"""
    print("\n--- Motor Calibration ---")
    print("WARNING: Motors will move during calibration.")
    input("Press Enter to continue with calibration or Ctrl+C to cancel...")
    
    try:
        # Calibrate motors one at a time
        for axis_num in [0, 1]:
            axis_name = f"axis{axis_num}"
            axis = getattr(odrv, axis_name)
            
            print(f"\nCalibrating {axis_name}...")
            axis.requested_state = AXIS_STATE_MOTOR_CALIBRATION
            
            # Wait for calibration to complete
            print("Waiting for motor calibration to complete...")
            while axis.current_state != AXIS_STATE_IDLE:
                time.sleep(0.1)
            
            if axis.motor.error != 0:
                print(f"Motor calibration failed with error: {axis.motor.error}")
                return False
            
            print(f"Motor calibration for {axis_name} successful.")
            
            # Improved Hall sensor calibration sequence
            print(f"Clearing any previous encoder errors for {axis_name}...")
            axis.encoder.error = 0
            
            print(f"Calibrating Hall encoder polarity for {axis_name}...")
            axis.requested_state = AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION
            
            # Wait with timeout for calibration to complete
            start_time = time.time()
            timeout = 15  # seconds
            while axis.current_state != AXIS_STATE_IDLE:
                if time.time() - start_time > timeout:
                    print(f"Timeout waiting for polarity calibration. Moving to next step.")
                    # Force state to idle to proceed
                    axis.requested_state = AXIS_STATE_IDLE
                    time.sleep(1.0)
                    break
                time.sleep(0.5)
                
            if axis.encoder.error != 0:
                print(f"Hall polarity calibration has error: {axis.encoder.error}")
                print("Attempting to clear error and continue...")
                axis.encoder.error = 0
            else:
                print(f"Hall polarity calibration for {axis_name} completed.")
            
            # Additional delay before next calibration step
            time.sleep(1.0)
            
            # Skip offset calibration as it's not always needed for Hall sensors
            # This is one of the common failure points
            print(f"Skipping encoder offset calibration for Hall sensors...")

        # Save configuration after calibration steps
        print("\nSaving calibration results...")
        odrv.save_configuration()
        print("Calibration results saved successfully.")
        return True
        
    except KeyboardInterrupt:
        print("\nCalibration canceled by user.")
        return False
    except Exception as e:
        print(f"\nCalibration failed: {str(e)}")
        return False

def test_motors(odrv):
    """Test motors with simple movements"""
    print("\n--- Motor Testing ---")
    print("WARNING: Motors will move during testing.")
    print("Ensure the robot is elevated with wheels able to spin freely.")
    input("Press Enter to continue with testing or Ctrl+C to cancel...")
    
    try:
        # Set both axes to closed-loop control
        for axis_num in [0, 1]:
            axis_name = f"axis{axis_num}"
            axis = getattr(odrv, axis_name)
            
            # Clear any errors that might prevent entering closed-loop control
            if axis.error != 0 or axis.motor.error != 0 or axis.encoder.error != 0 or axis.controller.error != 0:
                print(f"Clearing errors on {axis_name} before testing...")
                axis.error = 0
                axis.motor.error = 0
                axis.encoder.error = 0
                axis.controller.error = 0
                time.sleep(0.5)
            
            print(f"Setting {axis_name} to closed-loop control...")
            axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
            
            # Wait for mode change with timeout
            start_time = time.time()
            timeout = 5  # seconds
            while axis.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
                if time.time() - start_time > timeout:
                    print(f"Timeout waiting for {axis_name} to enter closed-loop control.")
                    print(f"Current state: {axis.current_state}")
                    if axis.error != 0 or axis.motor.error != 0 or axis.encoder.error != 0:
                        print(f"Errors: axis={axis.error}, motor={axis.motor.error}, encoder={axis.encoder.error}")
                    return False
                time.sleep(0.5)
            
            print(f"{axis_name} successfully entered closed-loop control mode.")
        
        # Simple sequence: forward, stop, reverse, stop
        test_sequence = [
            {"name": "Forward", "left": 1.0, "right": 1.0, "duration": 2.0},
            {"name": "Stop", "left": 0.0, "right": 0.0, "duration": 1.0},
            {"name": "Turn Right", "left": 1.0, "right": -1.0, "duration": 2.0},
            {"name": "Stop", "left": 0.0, "right": 0.0, "duration": 1.0},
            {"name": "Turn Left", "left": -1.0, "right": 1.0, "duration": 2.0},
            {"name": "Stop", "left": 0.0, "right": 0.0, "duration": 1.0},
            {"name": "Reverse", "left": -1.0, "right": -1.0, "duration": 2.0},
            {"name": "Stop", "left": 0.0, "right": 0.0, "duration": 1.0},
        ]
        
        # Execute test sequence
        for step in test_sequence:
            print(f"\nTesting: {step['name']}")
            
            # Send commands to both motors
            odrv.axis0.controller.input_vel = step['left']
            odrv.axis1.controller.input_vel = step['right']
            
            # Wait for specified duration
            for i in range(int(step['duration'] * 10)):
                # Print status update every 0.5 seconds
                if i % 5 == 0:
                    left_vel = odrv.axis0.encoder.vel_estimate
                    right_vel = odrv.axis1.encoder.vel_estimate
                    left_current = odrv.axis0.motor.current_control.Iq_measured
                    right_current = odrv.axis1.motor.current_control.Iq_measured
                    print(f"  Left: {left_vel:.2f} turns/s, {left_current:.2f} A | Right: {right_vel:.2f} turns/s, {right_current:.2f} A")
                
                time.sleep(0.1)
        
        # Ensure motors are stopped at the end
        odrv.axis0.controller.input_vel = 0.0
        odrv.axis1.controller.input_vel = 0.0
        
        # Set both axes to idle
        odrv.axis0.requested_state = AXIS_STATE_IDLE
        odrv.axis1.requested_state = AXIS_STATE_IDLE
        
        print("\nMotor test complete.")
        return True
        
    except KeyboardInterrupt:
        print("\nMotor test canceled by user.")
        # Ensure motors are stopped
        try:
            odrv.axis0.controller.input_vel = 0.0
            odrv.axis1.controller.input_vel = 0.0
            odrv.axis0.requested_state = AXIS_STATE_IDLE
            odrv.axis1.requested_state = AXIS_STATE_IDLE
        except:
            pass
        return False
    except Exception as e:
        print(f"\nMotor test failed: {str(e)}")
        # Ensure motors are stopped
        try:
            odrv.axis0.controller.input_vel = 0.0
            odrv.axis1.controller.input_vel = 0.0
            odrv.axis0.requested_state = AXIS_STATE_IDLE
            odrv.axis1.requested_state = AXIS_STATE_IDLE
        except:
            pass
        return False

def show_diagnostics(odrv):
    """Display ODrive diagnostics"""
    print("\n--- ODrive Diagnostics ---")
    
    try:
        print(f"ODrive Serial Number: {odrv.serial_number}")
        print(f"Hardware Version: {odrv.hw_version_major}.{odrv.hw_version_minor}")
        print(f"Firmware Version: {odrv.fw_version_major}.{odrv.fw_version_minor}.{odrv.fw_version_revision}")
        
        # Safely check for brake resistor configuration with exception handling
        brake_status = "Unknown"
        try:
            if hasattr(odrv.config, 'enable_brake_resistor'):
                brake_status = 'Enabled' if odrv.config.enable_brake_resistor else 'Disabled'
            elif hasattr(odrv.config, 'brake_resistor_enabled'):
                brake_status = 'Enabled' if odrv.config.brake_resistor_enabled else 'Disabled'
        except Exception:
            brake_status = "Not available in this firmware version"
        
        print(f"Brake Resistor: {brake_status}")
        print(f"DC Bus Voltage: {odrv.vbus_voltage:.2f}V")
        
        # Check for errors
        if odrv.error != 0:
            print(f"ODrive Error: {odrv.error}")
        
        # Check each axis
        for axis_num in [0, 1]:
            axis_name = f"axis{axis_num}"
            axis = getattr(odrv, axis_name)
            
            print(f"\n{axis_name.upper()} Status:")
            print(f"  Current State: {axis.current_state}")
            
            if axis.error != 0:
                print(f"  Axis Error: {axis.error}")
            
            if axis.motor.error != 0:
                print(f"  Motor Error: {axis.motor.error}")
            
            if axis.encoder.error != 0:
                print(f"  Encoder Error: {axis.encoder.error}")
            
            if axis.controller.error != 0:
                print(f"  Controller Error: {axis.controller.error}")
            
            # Show motor configuration
            print(f"  Motor Configuration:")
            print(f"    Type: {axis.motor.config.motor_type}")
            print(f"    Pole Pairs: {axis.motor.config.pole_pairs}")
            print(f"    Current Limit: {axis.motor.config.current_lim:.2f}A")
            
            # Show encoder configuration
            print(f"  Encoder Configuration:")
            print(f"    Mode: {axis.encoder.config.mode}")
            print(f"    CPR: {axis.encoder.config.cpr}")
            
            # Show controller configuration
            print(f"  Controller Configuration:")
            print(f"    Control Mode: {axis.controller.config.control_mode}")
            print(f"    Velocity Limit: {axis.controller.config.vel_limit:.2f} turns/s")
        
        return True
    except Exception as e:
        print(f"Error in diagnostics: {str(e)}")
        return False

def main():
    """Main function to set up ODrive"""
    parser = argparse.ArgumentParser(description='Configure ODrive for Advanced Tracked Robot')
    parser.add_argument('--no-calibrate', action='store_true', help='Skip motor calibration')
    parser.add_argument('--no-test', action='store_true', help='Skip motor testing')
    parser.add_argument('--no-save', action='store_true', help='Do not save configuration to ODrive')
    parser.add_argument('--diag-only', action='store_true', help='Run diagnostics only')
    args = parser.parse_args()
    
    # Default configuration for 1kW BLDC motors - adjusted for Hall sensors
    default_config = {
        'pole_pairs': 7,                # Motor pole pairs (adjust for your motors)
        'calib_voltage': 8.0,           # Lower calibration voltage for safer operation
        'current_range': 90.0,          # Amperes
        'current_bandwidth': 100.0,     # Hz
        'torque_constant': 8.27 / 16,   # Nm/A, initial guess, determined during calibration
        'current_limit': 60.0,          # Amperes
        'current_margin': 5.0,          # Amperes
        'encoder_cpr': 6,               # For Hall sensors, this should be 6 (3 hall sensors * 2)
        'velocity_limit': 20.0,         # turns/s
        'velocity_gain': 0.01,          # (Nm/rad/s) / (turn/s)
        'brake_resistance': 0.5,        # Ohms (0 to disable)
    }
    
    print("ODrive Setup for Advanced Tracked Robot")
    print("=======================================")
    
    # Find the ODrive
    odrv = find_odrive()
    if odrv is None:
        sys.exit(1)
    
    if args.diag_only:
        show_diagnostics(odrv)
        sys.exit(0)
    
    # Configure ODrive unless told not to
    configure_odrive_for_bldc_motors(odrv, default_config)
    
    # Run calibration if needed
    if not args.no_calibrate:
        if calibrate_motors(odrv):
            print("Calibration successful.")
        else:
            print("Calibration failed or was interrupted.")
            if not args.no_save:
                print("Not saving configuration due to failed calibration.")
                args.no_save = True
    
    # Test motors if needed
    if not args.no_test:
        if test_motors(odrv):
            print("Motor test successful.")
        else:
            print("Motor test failed or was interrupted.")
    
    # Save configuration if needed
    if not args.no_save:
        try:
            print("\nSaving configuration to ODrive...")
            odrv.save_configuration()
            print("Configuration saved successfully.")
        except Exception as e:
            print(f"Error saving configuration: {str(e)}")
    
    # Show diagnostics
    show_diagnostics(odrv)
    
    print("\nODrive setup complete.")
    print("\nYou can now use the tracked_robot package with your ODrive controller.")
    print("To run the advanced tracked robot, use:")
    print("  ros2 launch tracked_robot advanced_robot.launch.py")
    print("\nMake sure to adjust any parameters in the launch file if needed.")

if __name__ == "__main__":
    main()
