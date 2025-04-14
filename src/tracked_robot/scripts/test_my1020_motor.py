#!/usr/bin/env python3

"""
MY1020 1000W BLDC Motor Direct Test Script

This script focuses on testing a single MY1020 motor with maximum torque
to verify if the motor can be driven. It uses direct torque commands instead
of velocity control.

Usage:
    python3 test_my1020_motor.py
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

def reset_errors(axis):
    """Reset all errors on axis"""
    print("Clearing errors...")
    if hasattr(axis, 'error'):
        axis.error = 0
    if hasattr(axis.motor, 'error'):
        axis.motor.error = 0
    if hasattr(axis.encoder, 'error'):
        axis.encoder.error = 0
    if hasattr(axis.controller, 'error'):
        axis.controller.error = 0
    time.sleep(0.5)

def prepare_motor(axis, config):
    """Configure the MY1020 motor with maximum performance settings"""
    print("\nPreparing motor with maximum performance settings...")
    
    # Set motor to high current mode
    axis.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
    
    # MY1020 specific parameters
    axis.motor.config.pole_pairs = config['pole_pairs']
    axis.motor.config.torque_constant = config['torque_constant']
    axis.motor.config.resistance_calib_max_voltage = config['calib_voltage']
    
    # VERY high current for startup
    axis.motor.config.current_lim = config['current_limit']
    axis.motor.config.calibration_current = config['calibration_current']
    
    # Direct phase values
    if hasattr(axis.motor.config, 'phase_resistance'):
        axis.motor.config.phase_resistance = config['phase_resistance']
        print(f"  Set phase resistance: {config['phase_resistance']}")
    if hasattr(axis.motor.config, 'phase_inductance'):
        axis.motor.config.phase_inductance = config['phase_inductance']
        print(f"  Set phase inductance: {config['phase_inductance']}")
    
    # Set to Hall encoder mode
    axis.encoder.config.mode = ENCODER_MODE_HALL
    axis.encoder.config.cpr = 6  # Hall sensors
    
    # Save configuration
    print("Saving configuration...")
    try:
        odrv.save_configuration()
    except:
        print("Warning: Couldn't save configuration")

def calibrate_motor(axis):
    """Run motor calibration with maximum current"""
    print("\nCalibrating motor with maximum current...")
    
    # Reset errors
    reset_errors(axis)
    
    # Start from IDLE state
    axis.requested_state = AXIS_STATE_IDLE
    time.sleep(1.0)
    
    # Run calibration
    print("Starting motor calibration...")
    axis.requested_state = AXIS_STATE_MOTOR_CALIBRATION
    
    # Wait for calibration to complete
    timeout = 20  # seconds
    start_time = time.time()
    while axis.current_state != AXIS_STATE_IDLE:
        if time.time() - start_time > timeout:
            print("Calibration timeout. Canceling...")
            axis.requested_state = AXIS_STATE_IDLE
            time.sleep(1.0)
            break
        time.sleep(0.5)
    
    # Check if calibration was successful
    if axis.motor.error != 0:
        print(f"Motor calibration failed with error: {axis.motor.error}")
        return False
    else:
        print("Motor calibration successful!")
        return True

def test_with_direct_torque(axis):
    """Test motor with direct torque commands"""
    print("\nTesting motor with direct torque...")
    
    # Reset errors
    reset_errors(axis)
    
    # Set to torque control mode
    print("Setting torque control mode...")
    axis.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL
    time.sleep(0.5)
    
    # Try to enter closed loop control
    print("Entering CLOSED_LOOP_CONTROL...")
    axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    
    # Wait for state change
    timeout = 5  # seconds
    start_time = time.time()
    while axis.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
        if time.time() - start_time > timeout:
            print(f"Timeout entering closed loop control. Current state: {axis.current_state}")
            if axis.error != 0:
                print(f"Error: {axis.error}")
            return False
        time.sleep(0.5)
    
    print("Successfully entered CLOSED_LOOP_CONTROL!")
    
    # Apply progressively higher torque
    print("\nApplying progressive torque...")
    
    # Test torque steps
    torque_steps = [
        {"value": 0.2, "duration": 2.0, "description": "Low torque"},
        {"value": 0.5, "duration": 2.0, "description": "Medium torque"},
        {"value": 1.0, "duration": 2.0, "description": "High torque"},
        {"value": 2.0, "duration": 2.0, "description": "Maximum torque"},
        {"value": 0.0, "duration": 1.0, "description": "Stop"}
    ]
    
    for step in torque_steps:
        print(f"\nSetting {step['description']}: {step['value']} Nm")
        axis.controller.input_torque = step['value']
        
        # Monitor during the step
        for i in range(int(step['duration'] * 2)):
            current = axis.motor.current_control.Iq_measured
            vel = axis.encoder.vel_estimate
            print(f"  Current: {current:.2f}A, Velocity: {vel:.2f} turns/sec")
            time.sleep(0.5)
    
    # Set torque to 0 at the end
    axis.controller.input_torque = 0.0
    
    # Back to idle state
    print("Setting back to IDLE state...")
    axis.requested_state = AXIS_STATE_IDLE
    
    return True

def test_with_sensorless(axis, config):
    """Test motor with sensorless mode"""
    print("\nTesting motor in sensorless mode...")
    
    # Reset errors
    reset_errors(axis)
    
    # Set to idle state
    axis.requested_state = AXIS_STATE_IDLE
    time.sleep(1.0)
    
    # Configure sensorless mode
    print("Configuring sensorless mode...")
    
    # Set velocity control
    axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
    
    # Set flux linkage
    if hasattr(axis.sensorless_estimator.config, 'pm_flux_linkage'):
        # Try 3 different values - start with low
        flux_values = [0.005, 0.015, 0.03]
        flux_idx = 0
        
        while flux_idx < len(flux_values):
            flux = flux_values[flux_idx]
            print(f"\nTrying flux linkage value: {flux}")
            axis.sensorless_estimator.config.pm_flux_linkage = flux
            
            # Try to enter sensorless control
            try:
                print("Entering sensorless control...")
                axis.requested_state = AXIS_STATE_SENSORLESS_CONTROL
                time.sleep(3.0)
                
                if axis.current_state != AXIS_STATE_SENSORLESS_CONTROL:
                    print(f"Failed to enter sensorless control. Error: {axis.error}")
                    print("Trying next flux value...")
                    flux_idx += 1
                    continue
                
                print("Successfully entered sensorless control!")
                
                # Try to apply velocity
                for vel in [10, 20, 40, 60]:
                    print(f"Testing velocity: {vel} turns/s")
                    axis.controller.input_vel = vel
                    time.sleep(2.0)
                    
                    # Monitor
                    current = axis.motor.current_control.Iq_measured
                    actual_vel = axis.encoder.vel_estimate
                    print(f"  Current: {current:.2f}A, Velocity: {actual_vel:.2f} turns/sec")
                    
                    # If we see significant current or movement, we've succeeded
                    if abs(current) > 3.0 or abs(actual_vel) > 1.0:
                        print("Motor is responding!")
                        return True
                
                print("Motor not responding with this flux linkage value.")
                axis.requested_state = AXIS_STATE_IDLE
                time.sleep(1.0)
                flux_idx += 1
            except Exception as e:
                print(f"Error testing with flux linkage {flux}: {e}")
                axis.requested_state = AXIS_STATE_IDLE
                time.sleep(1.0)
                flux_idx += 1
    
    return False

def main():
    parser = argparse.ArgumentParser(description='Advanced MY1020 Motor Test')
    parser.add_argument('--axis', type=int, default=0, help='Axis to test (0 or 1)')
    args = parser.parse_args()
    
    # MY1020 specific configuration - maximum settings
    my1020_config = {
        'pole_pairs': 8,
        'torque_constant': 0.168,  # 3.5 Nm / 20.8A
        'calib_voltage': 30.0,     # Higher for reliable calibration
        'current_limit': 60.0,     # Much higher for startup
        'calibration_current': 20.0,  # Higher for reliable calibration
        'phase_resistance': 0.15,  # ~2.3 ohms / 3 phases
        'phase_inductance': 0.0001,
    }
    
    # Select the axis
    axis_num = args.axis
    print(f"Testing MY1020 motor on axis{axis_num}...")
    
    # Find ODrive
    odrv = find_odrive()
    if odrv is None:
        print("No ODrive found. Exiting.")
        sys.exit(1)
    
    # Get the axis
    axis = getattr(odrv, f"axis{axis_num}")
    
    # Prepare motor
    prepare_motor(axis, my1020_config)
    
    # Calibrate motor
    if not calibrate_motor(axis):
        print("Motor calibration failed. Exiting.")
        sys.exit(1)
    
    # Test methods
    test_methods = [
        {"name": "Direct Torque Control", "func": lambda: test_with_direct_torque(axis)},
        {"name": "Sensorless Mode", "func": lambda: test_with_sensorless(axis, my1020_config)},
    ]
    
    # Try each method
    for method in test_methods:
        print(f"\n------ Testing with {method['name']} ------")
        if method["func"]():
            print(f"SUCCESS with {method['name']}!")
            break
        else:
            print(f"Method {method['name']} failed to move the motor.")
    
    print("\nTest complete. Set axis back to idle.")
    axis.requested_state = AXIS_STATE_IDLE

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nTest interrupted by user.")
        try:
            odrv0 = odrive.find_any(timeout=3)
            print("Setting motor back to IDLE state...")
            if hasattr(odrv0, 'axis0'):
                odrv0.axis0.requested_state = AXIS_STATE_IDLE
            if hasattr(odrv0, 'axis1'):
                odrv0.axis1.requested_state = AXIS_STATE_IDLE
        except:
            pass
