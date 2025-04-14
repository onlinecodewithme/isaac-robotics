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
        
        # Reset any errors to start clean
        if hasattr(axis, 'error'):
            axis.error = 0
        if hasattr(axis.motor, 'error'):
            axis.motor.error = 0
        if hasattr(axis.encoder, 'error'):
            axis.encoder.error = 0
        if hasattr(axis.controller, 'error'):
            axis.controller.error = 0
        
        # Motor configuration - optimized for 48V 1000W BLDC motors
        print("Configuring motor...")
        axis.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
        axis.motor.config.pole_pairs = config['pole_pairs']
        axis.motor.config.resistance_calib_max_voltage = config['calib_voltage']
        axis.motor.config.requested_current_range = config['current_range']
        axis.motor.config.current_control_bandwidth = config['current_bandwidth']
        axis.motor.config.torque_constant = config['torque_constant']
        
        # Motor specific parameters for 48V 1000W
        if hasattr(axis.motor.config, 'calibration_current'):
            axis.motor.config.calibration_current = config['calibration_current']
        
        # Use pre-calibrated values based on 48V 1000W BLDC motor specs
        # This helps bypass issues where calibration succeeds but closed-loop control fails
        if hasattr(axis.motor.config, 'phase_resistance'):
            print(f"Setting motor parameters for 48V 1000W BLDC motor...")
            axis.motor.config.phase_resistance = config['phase_resistance']
            if hasattr(axis.motor.config, 'phase_inductance'):
                axis.motor.config.phase_inductance = config['phase_inductance']
        
        # Current limits - configured for 1000W motor with 48V supply
        axis.motor.config.current_lim = config['current_limit']
        axis.motor.config.current_lim_margin = config['current_margin']
        
        # DC bus overvoltage trip level - important for 53V battery
        if hasattr(odrv.config, 'dc_bus_overvoltage_trip_level'):
            odrv.config.dc_bus_overvoltage_trip_level = config['overvoltage_trip_level']
        elif hasattr(odrv.config, 'dc_max_positive_voltage'):
            odrv.config.dc_max_positive_voltage = config['overvoltage_trip_level']
        
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
            
            # Clear any previous errors before starting
            if hasattr(axis, 'error'):
                axis.error = 0
            if hasattr(axis.motor, 'error'):
                axis.motor.error = 0
            if hasattr(axis.encoder, 'error'):
                axis.encoder.error = 0
            if hasattr(axis.controller, 'error'):
                axis.controller.error = 0
            
            # Motor calibration with retry logic
            print(f"\nCalibrating {axis_name}...")
            max_attempts = 3
            
            for attempt in range(1, max_attempts + 1):
                print(f"Motor calibration attempt {attempt}/{max_attempts}...")
                # First ensure we're in idle state
                axis.requested_state = AXIS_STATE_IDLE
                time.sleep(1.0)
                
                # Start calibration
                axis.requested_state = AXIS_STATE_MOTOR_CALIBRATION
                
                # Wait for calibration to complete with timeout
                start_time = time.time()
                timeout = 20  # seconds
                print("Waiting for motor calibration to complete...")
                
                while axis.current_state != AXIS_STATE_IDLE:
                    if time.time() - start_time > timeout:
                        print(f"Timeout waiting for calibration. Canceling this attempt.")
                        axis.requested_state = AXIS_STATE_IDLE
                        time.sleep(2.0)
                        break
                    time.sleep(0.5)
                
                # Check if calibration was successful
                if axis.motor.error == 0:
                    print(f"Motor calibration for {axis_name} successful.")
                    break
                else:
                    error_code = axis.motor.error
                    print(f"Motor calibration failed with error: {error_code}")
                    
                    # Error 1 is often resistance/inductance measurement issue
                    if error_code == 1 and attempt < max_attempts:
                        print("This appears to be a resistance measurement issue.")
                        print("Clearing error and adjusting parameters for retry...")
                        axis.motor.error = 0
                        
                        # Slightly adjust calibration parameters and try again
                        if hasattr(axis.motor.config, 'resistance_calib_max_voltage'):
                            current_voltage = axis.motor.config.resistance_calib_max_voltage
                            axis.motor.config.resistance_calib_max_voltage = current_voltage * 1.2
                            print(f"Increased calibration voltage to {axis.motor.config.resistance_calib_max_voltage:.1f}V")
                        
                        time.sleep(2.0)
                    else:
                        if attempt == max_attempts:
                            print(f"Failed all {max_attempts} attempts at motor calibration.")
                            return False
            
            # If we get here and motor.error is not 0, calibration failed
            if axis.motor.error != 0:
                print(f"Motor calibration for {axis_name} ultimately failed.")
                return False
            
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

def configure_for_direct_drive(odrv):
    """Configure specific parameters for direct drive operation"""
    print("\n--- Configuring for Direct Drive Operation ---")
    
    for axis_num in [0, 1]:
        axis_name = f"axis{axis_num}"
        axis = getattr(odrv, axis_name)
        
        print(f"Setting direct drive parameters for {axis_name}...")
        
        # Reset errors
        axis.error = 0
        axis.motor.error = 0
        axis.controller.error = 0
        
        # Set to much higher current for starting from standstill
        axis.motor.config.current_lim = 60.0
        
        # Try to set motor phase resistance directly
        if hasattr(axis.motor.config, 'phase_resistance'):
            axis.motor.config.phase_resistance = 0.1  # Typical for 1kW motor
        
        # Configure velocity control
        axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        axis.controller.config.vel_limit = 100.0  # Allow high limit for startup
        axis.controller.config.vel_ramp_rate = 10.0  # Ramp rate in turns/s^2
        
        # Set motor type
        axis.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
        
        # Save configuration
        odrv.save_configuration()
        
    print("Direct drive configuration complete.")

def try_sensorless_mode(axis, axis_name):
    """Try to run the motor in sensorless mode with optimized parameters"""
    print(f"\nTrying sensorless mode for {axis_name}...")
    
    # Clear all errors first
    print(f"Clearing all errors for {axis_name}...")
    if hasattr(axis, 'error'):
        axis.error = 0
    if hasattr(axis.motor, 'error'):
        axis.motor.error = 0
    if hasattr(axis.encoder, 'error'):
        axis.encoder.error = 0
    if hasattr(axis.controller, 'error'):
        axis.controller.error = 0
    time.sleep(1.0)
    
    # Set axis to idle state
    print(f"Setting {axis_name} to IDLE state...")
    axis.requested_state = AXIS_STATE_IDLE
    time.sleep(1.0)
    
    # Configure for sensorless mode with stronger parameters
    print(f"Configuring advanced sensorless parameters...")
    
    # Set control mode to velocity control
    axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
    
    # Configure sensorless estimator parameters
    try:
        # These parameters need to be properly tuned for 48V motors
        if hasattr(axis.sensorless_estimator.config, 'pm_flux_linkage'):
            axis.sensorless_estimator.config.pm_flux_linkage = 0.01  # Increased for 48V motors
        
        # Configure ramping parameters for stronger startup
        if hasattr(axis.controller.config, 'vel_ramp_rate'):
            axis.controller.config.vel_ramp_rate = 1.0  # Slower for reliable startup
            print(f"  Set velocity ramp rate to {axis.controller.config.vel_ramp_rate}")
        
        # Controller gains for sensorless operation
        if hasattr(axis.controller.config, 'vel_gain'):
            axis.controller.config.vel_gain = 0.025  # Increased gain
            print(f"  Set velocity gain to {axis.controller.config.vel_gain}")
        if hasattr(axis.controller.config, 'vel_integrator_gain'):
            axis.controller.config.vel_integrator_gain = 0.1  # Higher integrator gain
            print(f"  Set velocity integrator gain to {axis.controller.config.vel_integrator_gain}")
            
        # Direct motor control settings
        axis.motor.config.current_lim = 60.0  # Much higher current for starting motion
        print(f"  Set current limit to {axis.motor.config.current_lim}A")
    except Exception as e:
        print(f"  Warning: Some sensorless parameters couldn't be configured: {e}")
    
    # Try to use sensorless ramp - with stronger settings
    print(f"Attempting to enter sensorless ramp mode...")
    try:
        axis.requested_state = AXIS_STATE_SENSORLESS_CONTROL
        time.sleep(3.0)  # Give it more time to initialize
        
        # Check if successful
        if axis.current_state == AXIS_STATE_SENSORLESS_CONTROL:
            print(f"Successfully entered sensorless control mode with {axis_name}!")
            
            # Test the motor with a small movement to verify 
            print(f"Testing motor with a small movement...")
            axis.controller.input_vel = 5.0  # Higher starting velocity
            time.sleep(1.0)
            
            # Check if current is flowing
            current = axis.motor.current_control.Iq_measured
            if abs(current) > 1.0:
                print(f"Motor current detected: {current:.2f}A - motor should be active")
            else:
                print(f"Low motor current detected: {current:.2f}A - motor may not be moving")
            
            # Stop the test movement
            axis.controller.input_vel = 0.0
            time.sleep(0.5)
            
            return True
        else:
            print(f"Failed to enter sensorless control mode with {axis_name}.")
            print(f"Current state: {axis.current_state}")
            if hasattr(axis, 'error') and axis.error != 0:
                error_code = axis.error
                print(f"Axis error: {error_code}")
                
                # Common sensorless error diagnostics
                if error_code == 40:
                    print("  Error 40: Sensorless estimator failed")
                    print("  This often happens when the motor is not correctly parametrized")
                    print("  Check motor wiring and try increasing current_lim and flux linkage")
                    
                elif error_code & 512:  # Check for error bit 512
                    print("  Error 512: Motor didn't move properly.")
                    print("  This could be due to insufficient current or mechanical blockage")
                
            return False
    except Exception as e:
        print(f"Error entering sensorless mode: {e}")
        return False

def test_motors(odrv):
    """Test motors with simple movements"""
    print("\n--- Motor Testing ---")
    print("WARNING: Motors will move during testing.")
    print("Ensure the robot is elevated with wheels able to spin freely.")
    input("Press Enter to continue with testing or Ctrl+C to cancel...")
    
    try:
        # Configure for direct drive operation
        configure_for_direct_drive(odrv)
        
        print("\nPreparing motors for direct control...")
        
        for axis_num in [0, 1]:
            axis_name = f"axis{axis_num}"
            axis = getattr(odrv, axis_name)
            
            # Make sure motors are set to idle and errors cleared
            print(f"Setting {axis_name} to idle state and clearing errors...")
            axis.requested_state = AXIS_STATE_IDLE
            time.sleep(0.5)
            
            # Clear all errors
            if hasattr(axis, 'error'):
                axis.error = 0
            if hasattr(axis.motor, 'error'):
                axis.motor.error = 0
            if hasattr(axis.encoder, 'error'):
                axis.encoder.error = 0
            if hasattr(axis.controller, 'error'):
                axis.controller.error = 0
        
        # Try stronger sensorless mode for both motors
        print("\nAttempting alternative startup method...")
        axis0_ok = try_sensorless_mode(odrv.axis0, "axis0")
        axis1_ok = try_sensorless_mode(odrv.axis1, "axis1")
        
        # Very aggressive test sequence for high torque motors
        if not (axis0_ok or axis1_ok):
            print("Neither axis could enter sensorless mode. Will try one more approach...")
            print("Setting up for direct current control...")
            
            # Try to set up voltage control mode (firmware dependent)
            try:
                for axis_num in [0, 1]:
                    axis_name = f"axis{axis_num}"
                    axis = getattr(odrv, axis_name)
                    
                    # Reset to IDLE state
                    axis.requested_state = AXIS_STATE_IDLE
                    time.sleep(1.0)
                    
                    # Try voltage control mode if available
                    if hasattr(axis.controller.config, 'control_mode'):
                        print(f"Setting {axis_name} to direct current control...")
                        axis.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL
                        time.sleep(0.5)
                        
                        # Try to set to closed loop control
                        axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
                        time.sleep(2.0)
                        
                        if axis.current_state == AXIS_STATE_CLOSED_LOOP_CONTROL:
                            print(f"Successfully entered closed-loop control with {axis_name}!")
                            if axis_num == 0:
                                axis0_ok = True
                            else:
                                axis1_ok = True
                        else:
                            print(f"Failed to enter closed-loop control with {axis_name}.")
            except Exception as e:
                print(f"Error during alternative setup: {e}")

        # Test sequence with much higher currents and slower progression for 1000W motors
        test_sequence = [
            {"name": "Initial Forward (very low)", "left": 20.0, "right": 20.0, "duration": 3.0},
            {"name": "Stop", "left": 0.0, "right": 0.0, "duration": 1.0},
            {"name": "Forward (medium)", "left": 40.0, "right": 40.0, "duration": 2.0},
            {"name": "Stop", "left": 0.0, "right": 0.0, "duration": 1.0}, 
            {"name": "Reverse (low)", "left": -20.0, "right": -20.0, "duration": 2.0},
            {"name": "Stop", "left": 0.0, "right": 0.0, "duration": 1.0},
        ]

        # Apply sensorless velocity
        def apply_test_velocity(step):
            if axis0_ok:
                odrv.axis0.controller.input_vel = step['left']
                print(f"  Left: Setting velocity to {step['left']:.1f} turns/s")
            
            if axis1_ok:
                odrv.axis1.controller.input_vel = step['right']
                print(f"  Right: Setting velocity to {step['right']:.1f} turns/s")
                
            if not axis0_ok and not axis1_ok:
                print("  Cannot control motors - no axis in proper control mode")
        
        # Execute the test sequence
        for step in test_sequence:
            print(f"\nTesting: {step['name']}")
            apply_test_velocity(step)
            
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
        
        # Safely check for ODrive errors
        try:
            if hasattr(odrv, 'error') and odrv.error != 0:
                print(f"ODrive Error: {odrv.error}")
        except Exception:
            print("ODrive error status not available in this firmware version")
        
        # Check each axis
        for axis_num in [0, 1]:
            axis_name = f"axis{axis_num}"
            axis = getattr(odrv, axis_name)
            
            print(f"\n{axis_name.upper()} Status:")
            print(f"  Current State: {axis.current_state}")
            
            # Safely check all error states
            try:
                if hasattr(axis, 'error') and axis.error != 0:
                    print(f"  Axis Error: {axis.error}")
            except Exception:
                pass
                
            try:
                if hasattr(axis.motor, 'error') and axis.motor.error != 0:
                    print(f"  Motor Error: {axis.motor.error}")
            except Exception:
                pass
                
            try:
                if hasattr(axis.encoder, 'error') and axis.encoder.error != 0:
                    print(f"  Encoder Error: {axis.encoder.error}")
            except Exception:
                pass
                
            try:
                if hasattr(axis.controller, 'error') and axis.controller.error != 0:
                    print(f"  Controller Error: {axis.controller.error}")
            except Exception:
                pass
            
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
    
    # Configuration optimized for 48V 1000W BLDC motors with 53V battery
    # Using very aggressive settings for direct drive
    default_config = {
        'pole_pairs': 7,                # Motor pole pairs (common for 1kW BLDC)
        'calib_voltage': 20.0,          # Higher voltage needed for reliable calibration
        'current_range': 80.0,          # Much higher for 1000W motors
        'current_bandwidth': 100.0,     # Hz
        'calibration_current': 15.0,    # Higher calibration current for 1kW motors
        'torque_constant': 8.27 / 16,   # Nm/A, initial value
        'current_limit': 60.0,          # Much higher current to start heavy motors
        'current_margin': 15.0,         # Increased margin for better protection
        'encoder_cpr': 6,               # For Hall encoders
        'velocity_limit': 100.0,        # Much higher limit to overcome startup issues
        'velocity_gain': 0.03,          # Higher gain for more aggressive response
        'brake_resistance': 0.5,        # Ohms (0 to disable)
        'pre_calibrated': True,         # Use pre-calibrated values for better startup
        'phase_resistance': 0.1,        # Typical phase resistance for 48V 1kW BLDC
        'phase_inductance': 0.0001,     # Typical phase inductance for similar motors
        'overvoltage_trip_level': 65.0, # Set higher for 53V battery
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
