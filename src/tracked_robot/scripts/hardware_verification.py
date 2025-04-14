#!/usr/bin/env python3

"""
ODrive MY1020 Motor Hardware Verification Script

This script performs step-by-step hardware diagnostics and verification
to isolate why the motors are not spinning despite successful calibration.

Usage:
    python3 hardware_verification.py

This script requires interacting with the hardware components as it runs.
Follow the prompts and verify the physical connections as instructed.
"""

import sys
import time
import argparse
try:
    import odrive
    from odrive.enums import *
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
    except Exception as e:
        print(f"Error finding ODrive: {str(e)}")
        return None

def clear_errors(axis):
    """Clear all errors on a given axis"""
    if hasattr(axis, 'error'):
        axis.error = 0
    if hasattr(axis.motor, 'error'):
        axis.motor.error = 0
    if hasattr(axis.encoder, 'error'):
        axis.encoder.error = 0
    if hasattr(axis.controller, 'error'):
        axis.controller.error = 0

def verify_power_supply(odrv):
    """Verify the power supply is adequate"""
    print("\n===== POWER SUPPLY VERIFICATION =====")
    print("Current DC bus voltage: {:.2f}V".format(odrv.vbus_voltage))
    
    if odrv.vbus_voltage < 40.0:
        print("WARNING: Bus voltage is below 40V. This may be too low for 48V motors.")
    elif odrv.vbus_voltage < 50.0:
        print("Bus voltage is within acceptable range but lower than nominal 53V.")
    else:
        print("Bus voltage looks good at {:.2f}V.".format(odrv.vbus_voltage))
    
    # Verification steps for user
    print("\nPlease perform these manual checks:")
    print("1. Measure battery voltage directly with a multimeter")
    print("2. Check all power connections for tightness")
    print("3. Verify fuses are intact")
    
    input("\nPress Enter when you've verified the power supply...")

def verify_motor_wiring(odrv):
    """Verify motor wiring connections"""
    print("\n===== MOTOR WIRING VERIFICATION =====")
    print("Please check the following for both motors:")
    print("1. All three phase wires (A, B, C) are properly connected")
    print("2. Wires are securely fastened to both ODrive and motors")
    print("3. No damaged, frayed, or exposed wires")
    
    # Ask user to perform a manual test
    print("\nManual test: Disconnect motors from load and try turning by hand")
    print("Both motor shafts should rotate freely without excessive resistance")
    
    input("\nPress Enter when you've verified motor wiring and free rotation...")

def verify_hall_sensors(odrv):
    """Verify Hall sensor connections and functionality"""
    print("\n===== HALL SENSOR VERIFICATION =====")
    print("Please check Hall sensor connections:")
    print("1. Hall sensor wires are properly connected to ODrive")
    print("2. Check for 5V supply to Hall sensors")
    print("3. Verify ground connection to Hall sensors")
    
    # Try to test if Hall signals are changing
    print("\nPerforming Hall sensor test...")
    
    for axis_num in [0, 1]:
        axis = getattr(odrv, f"axis{axis_num}")
        print(f"\nTesting Hall sensors on axis{axis_num}...")
        
        # Configure axis
        clear_errors(axis)
        axis.requested_state = AXIS_STATE_IDLE
        axis.encoder.config.mode = ENCODER_MODE_HALL
        axis.encoder.config.cpr = 6
        
        # Ask user to rotate motor manually
        print(f"Please rotate axis{axis_num} motor shaft BY HAND slowly...")
        print("Watching for Hall state changes...")
        
        # Show initial Hall state
        initial_hall_state = None
        if hasattr(axis.encoder, 'hall_state'):
            initial_hall_state = axis.encoder.hall_state
            print(f"Initial Hall state: {initial_hall_state}")
        else:
            print("Hall state not available in this firmware")
            continue
        
        # Monitor for changes
        timeout = 10  # seconds
        start_time = time.time()
        hall_changed = False
        
        while time.time() - start_time < timeout:
            if hasattr(axis.encoder, 'hall_state'):
                current_hall = axis.encoder.hall_state
                if current_hall != initial_hall_state:
                    print(f"Hall state changed from {initial_hall_state} to {current_hall}!")
                    hall_changed = True
                    break
            time.sleep(0.1)
        
        if hall_changed:
            print(f"Hall sensors on axis{axis_num} are working properly.")
        else:
            print(f"No Hall state changes detected on axis{axis_num}.")
            print("Hall sensors may be faulty or not properly connected.")
    
    input("\nPress Enter to continue...")

def direct_motor_test(odrv):
    """Attempt direct motor control with maximum current"""
    print("\n===== DIRECT MOTOR TESTING =====")
    
    for axis_num in [0, 1]:
        axis = getattr(odrv, f"axis{axis_num}")
        print(f"\nDirect testing of axis{axis_num} motor...")
        
        # Configure for maximum torque
        clear_errors(axis)
        axis.requested_state = AXIS_STATE_IDLE
        time.sleep(0.5)
        
        # Set parameters for maximum power
        axis.motor.config.current_lim = 60.0  # Very high current
        axis.motor.config.calibration_current = 20.0
        axis.motor.config.pole_pairs = 8  # MY1020 motor
        axis.motor.config.torque_constant = 0.168  # Based on specs
        
        # Try to calibrate motor
        print(f"Calibrating axis{axis_num} motor...")
        axis.requested_state = AXIS_STATE_MOTOR_CALIBRATION
        
        # Wait for calibration
        timeout = 15
        start_time = time.time()
        while axis.current_state != AXIS_STATE_IDLE:
            if time.time() - start_time > timeout:
                print("Calibration timeout")
                break
            time.sleep(0.5)
        
        if axis.motor.error != 0:
            print(f"Motor calibration failed with error: {axis.motor.error}")
            continue
        
        print(f"Motor calibration successful on axis{axis_num}")
        
        # Try both closed loop modes
        
        # First try torque control
        print("\nTrying direct torque control...")
        
        try:
            # Set to torque control mode
            axis.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL
            time.sleep(0.5)
            
            # Try to enter closed loop
            clear_errors(axis)
            axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            time.sleep(2.0)
            
            if axis.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
                print(f"Failed to enter closed loop control. Error: {axis.error}")
                continue
            
            print("Applying maximum torque...")
            
            # Apply extreme torque for testing
            axis.controller.input_torque = 3.0  # Very high torque
            time.sleep(3.0)
            
            # Check current
            current = axis.motor.current_control.Iq_measured
            print(f"Current with max torque: {current:.2f}A")
            
            # Back to idle
            axis.requested_state = AXIS_STATE_IDLE
            
            input(f"\nDid the axis{axis_num} motor move? (Press Enter)...")
        except Exception as e:
            print(f"Error during torque test: {e}")
            axis.requested_state = AXIS_STATE_IDLE
    
    print("\nDirect motor tests complete.")

def motor_phasing_test(odrv):
    """Test if the motor phases are correctly connected"""
    print("\n===== MOTOR PHASING TEST =====")
    print("This test applies voltage directly to motor phases")
    print("WARNING: Motor might suddenly jerk if successful")
    
    for axis_num in [0, 1]:
        axis = getattr(odrv, f"axis{axis_num}")
        print(f"\nTesting axis{axis_num} motor phasing...")
        
        # Reset and configure
        clear_errors(axis)
        axis.requested_state = AXIS_STATE_IDLE
        time.sleep(0.5)
        
        try:
            if hasattr(axis, 'motor_phase_inductance'):
                print("Phase inductance (cached):", axis.motor_phase_inductance)
            
            # Run motor in open loop (voltage control)
            print("Attempting open loop voltage control...")
            
            # Set motor phase control to voltage
            if hasattr(axis.motor.config, 'enable_voltage_mode_vel_estimate'):
                # Some firmware versions have this
                axis.motor.config.enable_voltage_mode_vel_estimate = True
                print("Enabled voltage mode velocity estimate")
            
            # Set to open loop control if firmware supports it
            if hasattr(axis, 'controller'): 
                if hasattr(axis.controller.config, 'control_mode'):
                    try:
                        # If VOLTAGE_CONTROL is available as a control mode
                        axis.controller.config.control_mode = 3  # VOLTAGE_CONTROL (might not exist)
                        print("Set to voltage control mode")
                    except:
                        # If not, fall back to velocity control
                        print("Voltage control mode not available, using velocity control")
                        axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
            
            # Try to enter closed loop  
            clear_errors(axis)
            
            if hasattr(axis, 'requested_state'):
                try:
                    # Some versions have AXIS_STATE_MOTOR_CALIBRATION_VOLTAGE that we could try
                    # But we'll try the standard closed loop
                    print("Entering closed loop control...")
                    axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
                    time.sleep(2.0)
                    
                    if axis.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
                        print(f"Failed to enter closed loop control. Error: {axis.error}")
                        continue
                    
                    # Now try to apply voltage or current
                    print("Applying voltage command...")
                    try:
                        # Try direct PWM duty cycle if available
                        axis.motor.current_control.Iqd_setpoint = [0, 30]  # Maximum d-axis current
                        time.sleep(3.0)
                        print("Applied maximum d-axis current")
                    except:
                        # Otherwise try velocity
                        axis.controller.input_vel = 20.0
                        time.sleep(3.0)
                        print("Applied velocity command instead")
                    
                    # Back to idle
                    axis.requested_state = AXIS_STATE_IDLE
                    
                    # Ask user if motor moved
                    input(f"\nDid the axis{axis_num} motor move at all? (Press Enter)...")
                except Exception as e:
                    print(f"Error during voltage test: {e}")
                    if hasattr(axis, 'requested_state'):
                        axis.requested_state = AXIS_STATE_IDLE
        except Exception as e:
            print(f"Error during phasing test: {e}")
            if hasattr(axis, 'requested_state'):
                axis.requested_state = AXIS_STATE_IDLE

def diagnose_problems(odrv):
    """Perform analysis of known issues"""
    print("\n===== PROBLEM DIAGNOSIS =====")
    
    # Check for obvious issues
    print("Checking for common problems...")
    
    # Vbus voltage
    if odrv.vbus_voltage < 45.0:
        print("* ISSUE FOUND: Low voltage. The 48V motors need at least 45V to function properly.")
    
    # Check motor types
    for axis_num in [0, 1]:
        axis = getattr(odrv, f"axis{axis_num}")
        
        # Check if motor error still exists
        if axis.motor.error != 0:
            print(f"* ISSUE FOUND: Axis{axis_num} motor has error code {axis.motor.error}")
        
        # Check motor type
        if axis.motor.config.motor_type != MOTOR_TYPE_HIGH_CURRENT:
            print(f"* ISSUE FOUND: Axis{axis_num} not set to HIGH_CURRENT mode")
        
        # Check if the encoder has error
        if axis.encoder.error != 0:
            print(f"* ISSUE FOUND: Axis{axis_num} encoder has error code {axis.encoder.error}")
    
    # Provide summary of likely issues
    print("\nBased on all tests, the most likely issues are:")
    print("1. Motor phase wiring - incorrect connections or loose wires")
    print("2. Hall sensor wiring - incorrect connections or damaged sensors")
    print("3. Mechanical binding - motor shaft cannot move freely")
    print("4. Power supply - insufficient current available")
    
    print("\nRecommended hardware actions:")
    print("- Carefully check each motor phase wire connection")
    print("- Measure continuity of all connections with a multimeter")
    print("- Disconnect motors from load to test freely")
    print("- Try with a different power supply if available")
    print("- Swap motors if possible to isolate the problem")

def main():
    parser = argparse.ArgumentParser(description='ODrive Hardware Verification')
    parser.add_argument('--skip-verify', action='store_true', help='Skip verification prompts')
    args = parser.parse_args()
    
    # Find ODrive
    odrv = find_odrive()
    if odrv is None:
        print("No ODrive found. Exiting.")
        sys.exit(1)
    
    print("\nWelcome to ODrive Hardware Verification Tool")
    print("This interactive tool will help diagnose why motors aren't spinning")
    print("You'll need to interact with the hardware during the tests")
    
    # Run verification steps
    if not args.skip_verify:
        verify_power_supply(odrv)
        verify_motor_wiring(odrv)
        verify_hall_sensors(odrv)
    
    # Run direct motor testing
    direct_motor_test(odrv)
    
    # Run phase test 
    motor_phasing_test(odrv)
    
    # Analyze potential problems
    diagnose_problems(odrv)
    
    print("\nHardware verification complete.")
    print("If motors still don't move, consider:")
    print("1. Consulting ODrive forums or support at https://discourse.odriverobotics.com/")
    print("2. Testing with different motors to isolate the issue")
    print("3. Checking for mechanical transmission problems")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nVerification interrupted by user.")
        try:
            odrv0 = odrive.find_any(timeout=3)
            for axis_num in [0, 1]:
                axis = getattr(odrv0, f"axis{axis_num}")
                axis.requested_state = AXIS_STATE_IDLE
        except:
            pass
