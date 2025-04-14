#!/usr/bin/env python3

"""
ODrive Encoder Error Diagnosis Tool

This script specifically focuses on diagnosing and resolving
Axis Error 9 (ENCODER_ERROR) when trying to control motors.

Usage:
    python3 encoder_diagnosis.py
"""

import sys
import time
try:
    import odrive
    from odrive.enums import *
except ImportError:
    print("Error: ODrive Python library not found.")
    print("Please install it using: pip install odrive")
    sys.exit(1)

def clear_errors(axis):
    """Clear all errors on axis"""
    if hasattr(axis, 'error'):
        axis.error = 0
    if hasattr(axis.motor, 'error'):
        axis.motor.error = 0
    if hasattr(axis.encoder, 'error'):
        axis.encoder.error = 0
    if hasattr(axis.controller, 'error'):
        axis.controller.error = 0
    time.sleep(0.5)

def print_error_state(axis):
    """Print detailed error information"""
    if hasattr(axis, 'error'):
        print(f"Axis error: {axis.error}")
    if hasattr(axis.motor, 'error'):
        print(f"Motor error: {axis.motor.error}")
    if hasattr(axis.encoder, 'error'):
        print(f"Encoder error: {axis.encoder.error}")
    if hasattr(axis.controller, 'error'):
        print(f"Controller error: {axis.controller.error}")

print("\n===== ODRIVE ENCODER DIAGNOSIS TOOL =====")
print("This script will diagnose and attempt to fix encoder issues")

# Connect to ODrive
print("\nLooking for ODrive...")
try:
    odrv = odrive.find_any(timeout=10)
    print(f"Found ODrive! Serial Number: {odrv.serial_number}")
except Exception as e:
    print(f"Error finding ODrive: {str(e)}")
    sys.exit(1)

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

# Print current error state
print("\nCurrent error states:")
print_error_state(axis)

# Clear errors
print("\nClearing all errors...")
clear_errors(axis)
time.sleep(0.5)

# Check if errors cleared
print("\nError states after clearing:")
print_error_state(axis)

# Set to idle state
print("\nSetting to IDLE state...")
axis.requested_state = AXIS_STATE_IDLE
time.sleep(1.0)

# Check encoder settings
print("\n===== ENCODER CONFIGURATION =====")
print(f"Encoder mode: {axis.encoder.config.mode}")
print(f"Encoder CPR: {axis.encoder.config.cpr}")
if hasattr(axis.encoder.config, 'bandwidth'):
    print(f"Encoder bandwidth: {axis.encoder.config.bandwidth}")

# Check if encoder works in open mode
print("\n===== HALL SENSOR TEST =====")
print("This will check if your Hall sensors are working correctly")
print("Please manually rotate the motor shaft BY HAND and watch the Hall state")
print("Press Ctrl+C to stop watching")

if hasattr(axis.encoder, 'hall_state'):
    initial_state = axis.encoder.hall_state
    print(f"Initial Hall state: {initial_state}")
    
    try:
        print("Watching for Hall state changes...")
        last_state = initial_state
        hall_changes = 0
        
        start_time = time.time()
        timeout = 15  # seconds
        
        while time.time() - start_time < timeout:
            current_state = axis.encoder.hall_state
            if current_state != last_state:
                hall_changes += 1
                print(f"Hall state changed: {last_state} -> {current_state}")
                last_state = current_state
            time.sleep(0.05)
            
        if hall_changes > 0:
            print(f"\nDetected {hall_changes} Hall state changes. Hall sensors are working!")
        else:
            print("\nNo Hall state changes detected. Hall sensors may be disconnected or faulty.")
    except KeyboardInterrupt:
        if hall_changes > 0:
            print(f"\nDetected {hall_changes} Hall state changes. Hall sensors are working!")
        else:
            print("\nNo Hall state changes detected. Hall sensors may be disconnected or faulty.")
else:
    print("Hall state information not available in this firmware.")

# Try a different encoder mode
print("\n===== TRYING DIFFERENT ENCODER CONFIGURATION =====")
print("1. Remain in HALL mode (default)")
print("2. Try INCREMENTAL encoder mode")
print("3. Try SENSORLESS mode")
encoder_choice = input("Enter choice [1/2/3]: ")

if encoder_choice == "2":
    print("\nSwitching to INCREMENTAL encoder mode...")
    axis.encoder.config.mode = ENCODER_MODE_INCREMENTAL
    print("Setting encoder CPR to 8192...")
    axis.encoder.config.cpr = 8192
    encoder_mode = "incremental"
elif encoder_choice == "3":
    print("\nPreparing for SENSORLESS mode...")
    if hasattr(axis.sensorless_estimator.config, 'pm_flux_linkage'):
        axis.sensorless_estimator.config.pm_flux_linkage = 0.01
        print("Set flux linkage parameter to 0.01")
    encoder_mode = "sensorless"
else:
    print("\nKeeping HALL encoder mode...")
    encoder_mode = "hall"

# Attempt motor calibration for incremental/hall
if encoder_mode != "sensorless":
    print("\nStarting motor calibration...")
    clear_errors(axis)
    axis.requested_state = AXIS_STATE_MOTOR_CALIBRATION
    
    # Wait for calibration
    timeout = 15
    start_time = time.time()
    while axis.current_state != AXIS_STATE_IDLE:
        if time.time() - start_time > timeout:
            print("Calibration timeout!")
            break
        time.sleep(0.5)
    
    # Check calibration result
    if axis.motor.error != 0:
        print(f"Motor calibration failed with error: {axis.motor.error}")
    else:
        print("Motor calibration successful!")
        
    # For non-sensorless modes, try encoder calibration
    if encoder_mode == "hall":
        print("\nCalibrating Hall encoder...")
        clear_errors(axis)
        axis.requested_state = AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION
    elif encoder_mode == "incremental":
        print("\nCalibrating incremental encoder...")
        clear_errors(axis)
        axis.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
    
    # Wait for encoder calibration
    timeout = 15
    start_time = time.time()
    while axis.current_state != AXIS_STATE_IDLE:
        if time.time() - start_time > timeout:
            print("Encoder calibration timeout!")
            break
        time.sleep(0.5)
    
    # Check encoder calibration result
    if axis.encoder.error != 0:
        print(f"Encoder calibration failed with error: {axis.encoder.error}")
    else:
        print("Encoder calibration successful!")

# Try to enter closed loop control or sensorless mode
print("\n===== MOTOR CONTROL ATTEMPT =====")
clear_errors(axis)

if encoder_mode == "sensorless":
    print("Attempting to enter SENSORLESS control mode...")
    axis.requested_state = AXIS_STATE_SENSORLESS_CONTROL
    control_mode = CONTROL_MODE_VELOCITY_CONTROL
    axis.controller.config.control_mode = control_mode
else:
    print("Attempting to enter CLOSED LOOP control...")
    control_mode = CONTROL_MODE_VELOCITY_CONTROL
    axis.controller.config.control_mode = control_mode
    axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

# Wait for mode transition
time.sleep(2.0)

# Check current state
target_state = AXIS_STATE_SENSORLESS_CONTROL if encoder_mode == "sensorless" else AXIS_STATE_CLOSED_LOOP_CONTROL
if axis.current_state != target_state:
    print(f"Failed to enter control mode. Current state: {axis.current_state}")
    print("Error information:")
    print_error_state(axis)
else:
    print(f"Successfully entered {'SENSORLESS' if encoder_mode == 'sensorless' else 'CLOSED LOOP'} control!")
    
    # Try basic motor movement
    print("\nTrying very gentle motor movement...")
    if encoder_mode == "sensorless" or control_mode == CONTROL_MODE_VELOCITY_CONTROL:
        for velocity in [2.0, 5.0, 10.0]:
            print(f"\nSetting velocity to {velocity} turns/s...")
            axis.controller.input_vel = velocity
            
            # Monitor for 3 seconds
            for i in range(6):
                current = axis.motor.current_control.Iq_measured
                print(f"  Current: {current:.2f}A")
                time.sleep(0.5)
            
            # Stop
            axis.controller.input_vel = 0.0
            time.sleep(0.5)
    else:
        for torque in [0.1, 0.2, 0.5]:
            print(f"\nSetting torque to {torque} Nm...")
            axis.controller.input_torque = torque
            
            # Monitor for 3 seconds
            for i in range(6):
                current = axis.motor.current_control.Iq_measured
                print(f"  Current: {current:.2f}A")
                time.sleep(0.5)
            
            # Stop
            axis.controller.input_torque = 0.0
            time.sleep(0.5)

# Return to idle state
print("\nSetting back to IDLE state...")
axis.requested_state = AXIS_STATE_IDLE
time.sleep(0.5)

# Final diagnostics
print("\n===== DIAGNOSIS RESULTS =====")

if hasattr(axis, 'error') and axis.error == 9:
    print("ENCODER ERROR 9 DETECTED.")
    print("\nThis specific error means one of these issues:")
    print("1. Hall sensors are not correctly connected")
    print("2. Hall sensor wiring is incorrect or damaged")
    print("3. Motor phases might be incorrectly connected")
    print("4. Encoder mode doesn't match the actual hardware")
    
    print("\nRECOMMENDED ACTIONS:")
    print("1. Check Hall sensor connections - all three Hall wires must be properly connected")
    print("2. Verify Hall sensor power (5V) and ground connections")
    print("3. Try running the motor without an encoder in SENSORLESS mode")
    print("4. If using this for a robot, try disconnecting the motor from all mechanical loads")
elif encoder_mode == "sensorless" and axis.current_state != AXIS_STATE_SENSORLESS_CONTROL:
    print("SENSORLESS MODE FAILED.")
    print("\nThis suggests the motor parameters might not be correctly configured.")
    print("Try running the motor calibration routine with the motor disconnected from any load.")
elif axis.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL and encoder_mode != "sensorless":
    print("CLOSED LOOP CONTROL FAILED.")
    print("\nThis suggests either incorrect encoder configuration or hardware problems.")
    print("Check your hardware connections and try again with different encoder settings.")
else:
    print("No critical errors detected.")
    print("If the motor still doesn't move, it's likely a mechanical binding issue.")
    print("Try disconnecting the motor from all mechanical loads and test again.")

print("\n===== HARDWARE VERIFICATION CHECKLIST =====")
print("☐ Hall sensor wires are properly connected to ODrive")
print("☐ All three motor phase wires are correctly connected")
print("☐ Motor can rotate freely without any mechanical load")
print("☐ Hall sensors change state when motor shaft is rotated by hand")
print("☐ Power supply is sufficient (current diagnostics show adequate voltage)")

print("\nThis completes the encoder diagnostic test.")
