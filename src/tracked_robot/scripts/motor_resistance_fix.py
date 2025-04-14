#!/usr/bin/env python3

"""
Motor Resistance Calibration Fix for MY1020 Motors

This script focuses specifically on resolving the motor calibration Error 1 
(resistance measurement issue) that's preventing closed loop control.

Usage:
    python3 motor_resistance_fix.py
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

def wait_for_idle(axis, timeout=20):
    """Wait for axis to return to idle state with timeout"""
    start_time = time.time()
    while axis.current_state != AXIS_STATE_IDLE:
        if time.time() - start_time > timeout:
            print("Timeout waiting for idle state")
            return False
        time.sleep(0.2)
    return True

print("\n===== MY1020 MOTOR RESISTANCE FIX =====")
print("This script will fix motor calibration by manually setting resistance values")

# Find ODrive
print("\nLooking for ODrive...")
try:
    odrv = odrive.find_any(timeout=10)
    print(f"Found ODrive! Serial Number: {odrv.serial_number}")
except Exception as e:
    print(f"Error finding ODrive: {str(e)}")
    sys.exit(1)

# Choose axis
print("\nWhich axis would you like to fix? [0/1]")
print("(Default is 0, just press Enter)")
axis_choice = input()
if axis_choice == "1":
    axis_num = 1
else:
    axis_num = 0

axis = getattr(odrv, f"axis{axis_num}")
print(f"Working on axis{axis_num}")

# Set to idle state
print("\nSetting to IDLE state...")
axis.requested_state = AXIS_STATE_IDLE
time.sleep(1.0)

# Clear any existing errors
print("Clearing any existing errors...")
clear_errors(axis)

# Check initial configuration
print("\n===== CURRENT CONFIGURATION =====")
print(f"Motor type: {axis.motor.config.motor_type}")
print(f"Pole pairs: {axis.motor.config.pole_pairs}")
print(f"Calibration voltage: {axis.motor.config.resistance_calib_max_voltage}V")
print(f"Current limit: {axis.motor.config.current_lim}A")

if hasattr(axis.motor.config, 'phase_resistance'):
    print(f"Phase resistance: {axis.motor.config.phase_resistance} ohms")
if hasattr(axis.motor.config, 'phase_inductance'):
    print(f"Phase inductance: {axis.motor.config.phase_inductance} H")

# Ask whether to use pre-configured or auto-detected values
print("\nHow would you like to configure the motor?")
print("1. Attempt calibration with higher voltage (recommended first)")
print("2. Use pre-configured values from MY1020 datasheet")
print("3. Use auto-detected values from previous calibration attempts")
config_choice = input("Enter choice [1/2/3]: ")

if config_choice == "2":
    # MY1020 typical values based on datasheet
    print("\nConfiguring with MY1020 datasheet values...")
    
    # Set motor parameters for MY1020 motor
    axis.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
    axis.motor.config.pole_pairs = 8  # MY1020 has 8 pole pairs
    axis.motor.config.torque_constant = 0.168  # Based on 3.5 N.m / 20.8A
    
    # Set fixed motor phase values (key for fixing error 1)
    # Calculate from datasheet: 48V / 20.8A = ~2.3 ohms total across phases
    # Phase resistance is roughly 2.3 / 3 = ~0.77 ohms per phase
    axis.motor.config.phase_resistance = 0.77  # ohms (calculated from datasheet)
    axis.motor.config.phase_inductance = 0.0001  # Typical value for this motor size
    
    # Skip calibration when using pre-configured values
    skip_calibration = True

elif config_choice == "3":
    # Use any existing phase resistance/inductance values (if available)
    print("\nChecking for existing auto-detected values...")
    
    if hasattr(axis.motor, 'phase_resistance') and axis.motor.phase_resistance > 0:
        res = axis.motor.phase_resistance
        print(f"Using auto-detected phase resistance: {res} ohms")
        axis.motor.config.phase_resistance = res
    else:
        print("No auto-detected phase resistance available.")
        axis.motor.config.phase_resistance = 0.77  # Default value
    
    if hasattr(axis.motor, 'phase_inductance') and axis.motor.phase_inductance > 0:
        ind = axis.motor.phase_inductance
        print(f"Using auto-detected phase inductance: {ind} H")
        axis.motor.config.phase_inductance = ind
    else:
        print("No auto-detected phase inductance available.")
        axis.motor.config.phase_inductance = 0.0001  # Default value
    
    # Skip calibration when using auto-detected values
    skip_calibration = True

else:  # Default to option 1
    # Set up for improved calibration
    print("\nConfiguring for higher voltage calibration...")
    
    # Increase calibration voltage (key to resolving Error 1)
    old_voltage = axis.motor.config.resistance_calib_max_voltage
    axis.motor.config.resistance_calib_max_voltage = 36.0  # Much higher than default
    print(f"Increased calibration voltage from {old_voltage}V to {axis.motor.config.resistance_calib_max_voltage}V")
    
    # Pre-calibration current limits (adjust for better calibration)
    old_current = axis.motor.config.current_lim
    axis.motor.config.current_lim = 40.0  # Higher than default
    print(f"Increased current limit from {old_current}A to {axis.motor.config.current_lim}A")
    
    # Make sure correct motor parameters are set
    axis.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
    axis.motor.config.pole_pairs = 8  # MY1020 has 8 pole pairs
    
    # Run calibration with higher voltage
    skip_calibration = False

# Save configuration
print("\nSaving configuration...")
try:
    odrv.save_configuration()
    print("Configuration saved successfully.")
except Exception as e:
    print(f"Warning: Could not save configuration - {e}")
    print("Continuing anyway...")

# Motor calibration (if not skipping)
if not skip_calibration:
    print("\nStarting motor calibration with higher voltage...")
    print("WARNING: The motor may make noise and twitch during calibration.")
    input("Press Enter to continue or Ctrl+C to cancel...")
    
    # Run motor calibration
    clear_errors(axis)
    axis.requested_state = AXIS_STATE_MOTOR_CALIBRATION
    
    # Wait for calibration to complete with timeout
    print("Waiting for calibration to complete...")
    if not wait_for_idle(axis, timeout=20):
        print("Calibration process timed out.")
    
    # Check calibration result
    if axis.motor.error != 0:
        print(f"Motor calibration failed with error: {axis.motor.error}")
        print("\nTrying backup approach: Pre-configuring resistance values...")
        
        # Set fixed motor phase values as backup
        axis.motor.config.phase_resistance = 0.77  # Default value for MY1020
        axis.motor.config.phase_inductance = 0.0001  # Default value for MY1020
        print("Set phase resistance to 0.77 ohms and inductance to 0.0001 H")
    else:
        print("Motor calibration successful!")
        
        # Store the measured values
        if hasattr(axis.motor, 'phase_resistance') and axis.motor.phase_resistance > 0:
            res = axis.motor.phase_resistance
            print(f"Measured phase resistance: {res} ohms")
            axis.motor.config.phase_resistance = res
        
        if hasattr(axis.motor, 'phase_inductance') and axis.motor.phase_inductance > 0:
            ind = axis.motor.phase_inductance
            print(f"Measured phase inductance: {ind} H")
            axis.motor.config.phase_inductance = ind

# Hall sensor calibration
print("\nCalibrating Hall sensors...")
clear_errors(axis)
axis.requested_state = AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION

# Wait for Hall calibration
if not wait_for_idle(axis, timeout=15):
    print("Hall calibration timed out.")

# Check Hall calibration result
if axis.encoder.error != 0:
    print(f"Hall calibration failed with error: {axis.encoder.error}")
else:
    print("Hall calibration successful!")

# Try to enter closed loop control
print("\n===== ATTEMPTING MOTOR CONTROL =====")
print("Setting up velocity control mode...")
axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL

# Extra settings for stable velocity control
axis.controller.config.vel_gain = 0.02  # Lower gain for stability
axis.controller.config.vel_integrator_gain = 0.1  # Moderate integrator gain

# Clear any accumulated errors
clear_errors(axis)

# Enter closed loop control
print("\nAttempting to enter closed loop control...")
axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
time.sleep(2.0)

if axis.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
    print(f"Failed to enter closed loop control. Current state: {axis.current_state}")
    if hasattr(axis, 'error') and axis.error != 0:
        print(f"Axis error: {axis.error}")
    
    print("\nTrying sensorless mode as fallback...")
    # Set sensorless parameters
    if hasattr(axis.sensorless_estimator.config, 'pm_flux_linkage'):
        axis.sensorless_estimator.config.pm_flux_linkage = 0.01
        print("Set flux linkage to 0.01")
    
    # Try to enter sensorless mode
    clear_errors(axis)
    axis.requested_state = AXIS_STATE_SENSORLESS_CONTROL
    time.sleep(2.0)
    
    if axis.current_state != AXIS_STATE_SENSORLESS_CONTROL:
        print(f"Failed to enter sensorless control. Current state: {axis.current_state}")
        if hasattr(axis, 'error') and axis.error != 0:
            print(f"Axis error: {axis.error}")
    else:
        print("Successfully entered sensorless control!")
        control_mode = "sensorless"
else:
    print("Successfully entered closed loop control!")
    control_mode = "closed_loop"

# Test motor movement if in control mode
if axis.current_state == AXIS_STATE_CLOSED_LOOP_CONTROL or axis.current_state == AXIS_STATE_SENSORLESS_CONTROL:
    print("\nTesting gentle motor movement...")
    
    # Generate increasing velocity commands
    values = [1.0, 2.0, 5.0, 10.0, 20.0, 30.0]
    
    try:
        for value in values:
            print(f"\nSetting velocity to {value} turns/s...")
            axis.controller.input_vel = value
            
            # Monitor for 2 seconds
            for i in range(4):
                current = axis.motor.current_control.Iq_measured
                vel = axis.encoder.vel_estimate if hasattr(axis.encoder, 'vel_estimate') else 0
                print(f"  Current: {current:.2f}A, Velocity: {vel:.2f} turns/s")
                
                # Check for movement
                if abs(vel) > 0.5:
                    print(f"  MOTOR IS MOVING! Detected velocity: {vel:.2f} turns/s")
                
                time.sleep(0.5)
            
            # Set velocity back to zero
            axis.controller.input_vel = 0.0
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nTest interrupted by user.")
    
    # Set back to idle state
    print("\nTesting complete. Setting back to idle state...")
    axis.controller.input_vel = 0.0
    axis.requested_state = AXIS_STATE_IDLE

else:
    print("\nSkipping movement test since control mode could not be entered.")

# Summarize results
print("\n===== MOTOR RESISTANCE FIX RESULTS =====")

if hasattr(axis.motor.config, 'phase_resistance'):
    print(f"Current phase resistance setting: {axis.motor.config.phase_resistance} ohms")
if hasattr(axis.motor.config, 'phase_inductance'):  
    print(f"Current phase inductance setting: {axis.motor.config.phase_inductance} H")

if axis.current_state == AXIS_STATE_CLOSED_LOOP_CONTROL:
    print("\nSUCCESS! Motor is now in closed loop control.")
    print("You can use the odrive_setup.py script or launch file for your robot now.")
elif axis.current_state == AXIS_STATE_SENSORLESS_CONTROL:
    print("\nPARTIAL SUCCESS! Motor is in sensorless control mode.")
    print("This mode doesn't use Hall sensors but should allow basic operation.")
    print("Recommended: Use sensorless mode in your application.")
else:
    print("\nMOTOR CONTROL STILL NOT WORKING.")
    print("\nTry these steps:")
    print("1. Disconnect motor from all mechanical loads")
    print("2. Check motor phase connections for loose wires")
    print("3. Try a different control mode (velocity/torque) in odrivetool")
    print("4. Consider updating ODrive firmware if possible")

print("\nSave these values for your configuration:")
print(f"- Phase Resistance: {axis.motor.config.phase_resistance} ohms")
print(f"- Phase Inductance: {axis.motor.config.phase_inductance} H")
print(f"- Pole Pairs: {axis.motor.config.pole_pairs}")
print(f"- Torque Constant: {axis.motor.config.torque_constant}")

print("\nIf the motor moved at any point during testing, the electrical connection is good.")
print("If it did not move, check all wiring and connections again.")
