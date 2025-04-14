#!/usr/bin/env python3

"""
Direct Motor Command - Simplest Possible Test

This script avoids any configuration or calibration and just
tries to directly control the motors with minimal steps.

Usage:
    python3 direct_motor_command.py
"""

import sys
import time
try:
    import odrive
    from odrive.enums import *
    print("ODrive library found")
except ImportError:
    print("Error: ODrive Python library not found.")
    print("Please install it using: pip install odrive")
    sys.exit(1)

def wait_for_idle(axis, timeout=20):
    """Wait for axis to return to idle state"""
    start_time = time.time()
    while axis.current_state != AXIS_STATE_IDLE:
        if time.time() - start_time > timeout:
            print("Timeout waiting for axis to return to idle")
            return False
        time.sleep(0.2)
    return True

print("\n===== DIRECT MOTOR COMMANDS =====")
print("This script will perform minimal calibration and attempt direct motor control")

# Connect to ODrive
print("\nLooking for ODrive...")
try:
    odrv = odrive.find_any(timeout=10)
    print(f"Found ODrive! Serial Number: {odrv.serial_number}")
except Exception as e:
    print(f"Error finding ODrive: {str(e)}")
    sys.exit(1)

# Basic settings without saving configuration
print("\nChecking system voltage...")
print(f"Vbus voltage: {odrv.vbus_voltage:.2f}V")

if odrv.vbus_voltage < 40.0:
    print("WARNING: Bus voltage may be too low for 48V motors")
    input("Press Enter to continue anyway...")

# Choose axis
print("\nWhich axis would you like to test? [0/1]")
print("(Default is 0, just press Enter)")
axis_choice = input()
if axis_choice == "1":
    axis_num = 1
else:
    axis_num = 0

axis = getattr(odrv, f"axis{axis_num}")
print(f"Using axis{axis_num}")

# Check and reset errors
def check_errors():
    """Check for errors on all components"""
    errors_exist = False
    
    if hasattr(axis, 'error') and axis.error != 0:
        print(f"Axis error: {axis.error}")
        errors_exist = True
    
    if hasattr(axis.motor, 'error') and axis.motor.error != 0:
        print(f"Motor error: {axis.motor.error}")
        errors_exist = True
    
    if hasattr(axis.encoder, 'error') and axis.encoder.error != 0:
        print(f"Encoder error: {axis.encoder.error}")
        errors_exist = True
    
    if hasattr(axis.controller, 'error') and axis.controller.error != 0:
        print(f"Controller error: {axis.controller.error}")
        errors_exist = True
    
    return errors_exist

def reset_errors():
    """Reset all errors"""
    if hasattr(axis, 'error'):
        axis.error = 0
    if hasattr(axis.motor, 'error'):
        axis.motor.error = 0
    if hasattr(axis.encoder, 'error'):
        axis.encoder.error = 0
    if hasattr(axis.controller, 'error'):
        axis.controller.error = 0

print("\nChecking for existing errors...")
if check_errors():
    print("Errors found. Clearing errors...")
    reset_errors()
    time.sleep(0.5)
    if check_errors():
        print("Warning: Some errors could not be cleared.")
else:
    print("No errors found.")

# Set axis to IDLE state
print("\nSetting to IDLE state...")
axis.requested_state = AXIS_STATE_IDLE
time.sleep(1.0)

# Set basic motor parameters (without saving configuration)
# These are conservative settings that should work with most motors
print("\nSetting basic motor parameters...")
axis.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
axis.motor.config.pole_pairs = 8  # MY1020 motor
axis.motor.config.resistance_calib_max_voltage = 12.0  # Very conservative
axis.motor.config.current_lim = 15.0  # Conservative current limit

# Try motor calibration
print("\nStarting motor calibration...")
print("WARNING: The motor will make noise and may attempt to move.")
input("Press Enter to continue or Ctrl+C to cancel...")

# Run calibration
reset_errors()
axis.requested_state = AXIS_STATE_MOTOR_CALIBRATION
print("Waiting for calibration to complete...")

if not wait_for_idle(axis):
    print("Failed to return to idle after calibration")
    check_errors()
    sys.exit(1)

# Check calibration result
if axis.motor.error != 0:
    print(f"Motor calibration failed with error: {axis.motor.error}")
    print("Testing cannot continue. Please check motor connections.")
    sys.exit(1)

print("Motor calibration successful!")

# Try Hall sensor calibration
print("\nCalibrating Hall sensors...")
reset_errors()
axis.requested_state = AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION
print("Waiting for Hall calibration to complete...")

if not wait_for_idle(axis):
    print("Failed to return to idle after Hall calibration")
    check_errors()
    sys.exit(1)

# Check Hall calibration result
if axis.encoder.error != 0:
    print(f"Hall calibration failed with error: {axis.encoder.error}")
    print("Hall sensors may be disconnected or faulty.")
    print("Warning: Proceeding anyway, but control may be unstable.")
else:
    print("Hall calibration successful!")

# Try simple movement
print("\n===== DIRECT MOTOR CONTROL =====")
print("Which control type would you like to try?")
print("1. Velocity control")
print("2. Torque control")
print("3. Sensorless control")
control_choice = input("Enter choice [1/2/3]: ")

if control_choice == "2":
    print("\nSetting up torque control...")
    axis.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL
    control_type = "torque"
elif control_choice == "3":
    print("\nSetting up sensorless control...")
    # Configure basic sensorless parameters
    if hasattr(axis.sensorless_estimator.config, 'pm_flux_linkage'):
        axis.sensorless_estimator.config.pm_flux_linkage = 0.008
    control_type = "sensorless"
else:
    print("\nSetting up velocity control...")
    axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
    control_type = "velocity"

# Enter closed loop control
print(f"\nAttempting to enter {'sensorless' if control_type == 'sensorless' else 'closed loop'} control...")
reset_errors()

# Set appropriate state
if control_type == "sensorless":
    axis.requested_state = AXIS_STATE_SENSORLESS_CONTROL
else:
    axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

time.sleep(2.0)  # Wait for state transition

# Check if state transition was successful
if control_type == "sensorless":
    target_state = AXIS_STATE_SENSORLESS_CONTROL
else:
    target_state = AXIS_STATE_CLOSED_LOOP_CONTROL

if axis.current_state != target_state:
    print(f"Failed to enter control mode. Current state: {axis.current_state}")
    check_errors()
    print("Setting back to idle...")
    axis.requested_state = AXIS_STATE_IDLE
    sys.exit(1)

print(f"Successfully entered {'sensorless' if control_type == 'sensorless' else 'closed loop'} control!")

# Function to send motor commands
def run_motor_test(command_type, values):
    print(f"\nRunning {command_type} test...")
    print("Press Ctrl+C at any time to stop the motor")
    
    try:
        for val in values:
            print(f"\nSetting {command_type} to {val}...")
            
            if command_type == "velocity":
                axis.controller.input_vel = val
            else:  # torque
                axis.controller.input_torque = val
            
            print(f"Monitoring for 3 seconds...")
            for i in range(6):
                current = axis.motor.current_control.Iq_measured
                vel = axis.encoder.vel_estimate if hasattr(axis.encoder, 'vel_estimate') else "unknown"
                print(f"  Current: {current:.2f}A, Velocity: {vel}")
                
                # Check if moving
                if isinstance(vel, (int, float)) and abs(vel) > 0.5:
                    print(f"  MOTOR IS MOVING! Detected velocity: {vel}")
                
                time.sleep(0.5)
            
            # Set back to zero between tests
            if command_type == "velocity":
                axis.controller.input_vel = 0.0
            else:  # torque
                axis.controller.input_torque = 0.0
            
            time.sleep(0.5)
        
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    finally:
        # Stop motor
        if command_type == "velocity":
            axis.controller.input_vel = 0.0
        else:  # torque
            axis.controller.input_torque = 0.0

# Test values
if control_type == "velocity":
    run_motor_test("velocity", [2.0, 5.0, 10.0, 20.0, 40.0])
elif control_type == "torque":
    run_motor_test("torque", [0.1, 0.2, 0.5, 1.0, 2.0, 3.0])
else:  # sensorless
    run_motor_test("velocity", [5.0, 10.0, 20.0, 50.0])

# Cleanup
print("\nTest complete. Setting motor back to idle state...")
axis.requested_state = AXIS_STATE_IDLE
time.sleep(0.5)

print("\n===== FINAL HARDWARE VERIFICATION =====")
print("Was the motor shaft completely free to turn during testing? [y/n]")
free_response = input().lower()
if free_response != 'y':
    print("Mechanical binding could be preventing motor rotation.")
    print("Try disconnecting the motor from any load before testing again.")

print("\nDid you hear any beeps or noises from the motor? [y/n]")
beep_response = input().lower()
if beep_response == 'y':
    print("Motor is electrically active but not rotating.")
    print("This suggests a mechanical binding issue or incorrect motor wiring.")
else:
    print("No motor activity detected. This suggests an electrical connection issue.")

print("\nPlease verify these hardware connections:")
print("1. All three motor phase wires are securely connected")
print("2. Hall sensor connections are secure (if using Hall sensors)")
print("3. Motor is free to rotate without any mechanical load")
print("4. Power supply is providing adequate voltage and current")

print("\nFor next steps, try:")
print("1. Disconnect motor from all mechanical loads and retry")
print("2. Check all wiring connections with a multimeter")
print("3. Try a different power supply if available")
print("4. Swap motors between axis0 and axis1 to isolate the issue")
