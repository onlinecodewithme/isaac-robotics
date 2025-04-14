#!/usr/bin/env python3

"""
Gentle Motor Test for MY1020 Motors

This script uses more conservative parameters that match the working calibration
parameters from the main odrive_setup.py script.

Usage:
    python3 gentle_motor_test.py [--axis 0]
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
    """Reset all errors on the specified axis"""
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
    """Configure with gentle parameters that are known to work"""
    print("\nConfiguring with gentle parameters...")
    
    # Set motor to high current mode
    axis.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
    
    # Use more conservative values that worked in the main setup
    axis.motor.config.pole_pairs = config['pole_pairs']
    axis.motor.config.resistance_calib_max_voltage = config['calib_voltage']
    
    # Lower current for safer calibration
    axis.motor.config.current_lim = config['current_limit']
    
    # Try to set calibration current if available
    if hasattr(axis.motor.config, 'calibration_current'):
        axis.motor.config.calibration_current = config['calibration_current']
        print(f"  Set calibration current: {config['calibration_current']}")
    
    # Set phase resistance directly
    if hasattr(axis.motor.config, 'phase_resistance'):
        axis.motor.config.phase_resistance = config['phase_resistance']
        print(f"  Set phase resistance: {config['phase_resistance']}")
        
    if hasattr(axis.motor.config, 'phase_inductance'):
        axis.motor.config.phase_inductance = config['phase_inductance']
        print(f"  Set phase inductance: {config['phase_inductance']}")
    
    # Encoder configuration
    axis.encoder.config.mode = ENCODER_MODE_HALL
    axis.encoder.config.cpr = config['encoder_cpr']
    
    # Controller configuration
    axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
    axis.controller.config.vel_limit = config['velocity_limit']
    axis.controller.config.vel_gain = config['velocity_gain']
    
    # Try to save configuration
    print(f"Saving configuration...")
    try:
        odrv.save_configuration()
        print("Configuration saved successfully.")
    except Exception as e:
        print(f"Warning: Could not save configuration - {e}")

def calibrate_motor(axis):
    """Run motor calibration with parameters known to work"""
    print("\nCalibrating motor with gentle parameters...")
    
    # Reset errors
    reset_errors(axis)
    
    # Set to idle state
    axis.requested_state = AXIS_STATE_IDLE
    time.sleep(1.0)
    
    # Run calibration
    print(f"Starting motor calibration...")
    axis.requested_state = AXIS_STATE_MOTOR_CALIBRATION
    
    # Wait for calibration to complete with timeout
    start_time = time.time()
    timeout = 20
    while axis.current_state != AXIS_STATE_IDLE:
        if time.time() - start_time > timeout:
            print(f"Calibration timeout. Canceling...")
            axis.requested_state = AXIS_STATE_IDLE
            time.sleep(1.0)
            break
        time.sleep(0.5)
    
    # Check if calibration was successful
    if axis.motor.error != 0:
        print(f"Motor calibration failed with error: {axis.motor.error}")
        return False
    
    print(f"Motor calibration successful!")
    
    # Run Hall sensor calibration
    print(f"Calibrating Hall encoder...")
    axis.requested_state = AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION
    
    # Wait for Hall calibration
    start_time = time.time()
    while axis.current_state != AXIS_STATE_IDLE:
        if time.time() - start_time > timeout:
            print(f"Hall calibration timeout. Canceling...")
            axis.requested_state = AXIS_STATE_IDLE
            time.sleep(1.0)
            break
        time.sleep(0.5)
    
    if axis.encoder.error != 0:
        print(f"Hall calibration failed with error: {axis.encoder.error}")
        return False
    
    print(f"Hall calibration successful!")
    return True

def test_motor_movement(axis):
    """Test motor movement using different modes"""
    print("\nTesting motor movement...")
    
    # Reset errors
    reset_errors(axis)
    
    # Try closed loop velocity control first
    print(f"\n1. Testing velocity control mode...")
    axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
    
    try:
        # Enter closed loop control
        print(f"Entering closed loop control...")
        axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        time.sleep(1.0)
        
        if axis.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
            print(f"Failed to enter closed loop control. Error: {axis.error}")
            return False
        
        print(f"Successfully entered closed loop control!")
        
        # Start with very low velocity
        velocities = [2.0, 5.0, 10.0, 20.0, 30.0]
        for vel in velocities:
            print(f"\nTesting velocity: {vel} turns/s...")
            axis.controller.input_vel = vel
            
            # Monitor for 2 seconds
            for i in range(4):
                current = axis.motor.current_control.Iq_measured
                actual_vel = axis.encoder.vel_estimate
                print(f"  Current: {current:.2f}A, Velocity: {actual_vel:.2f} turns/s")
                time.sleep(0.5)
                
                # Check for mechanical movement
                if abs(actual_vel) > 0.5:
                    print(f"Motor is moving! Detected velocity: {actual_vel:.2f} turns/s")
            
            # Pause between tests
            axis.controller.input_vel = 0.0
            time.sleep(0.5)
        
        # Back to idle
        axis.requested_state = AXIS_STATE_IDLE
        time.sleep(0.5)
        
    except Exception as e:
        print(f"Error during velocity test: {e}")
        axis.requested_state = AXIS_STATE_IDLE
    
    # Try torque control
    print(f"\n2. Testing torque control mode...")
    axis.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL
    
    try:
        # Enter closed loop control
        print(f"Entering closed loop control...")
        axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        time.sleep(1.0)
        
        if axis.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
            print(f"Failed to enter closed loop control. Error: {axis.error}")
            return False
        
        print(f"Successfully entered closed loop control!")
        
        # Apply progressive torque
        torques = [0.1, 0.2, 0.5, 1.0, 2.0]
        for torque in torques:
            print(f"\nTesting torque: {torque} Nm...")
            axis.controller.input_torque = torque
            
            # Monitor for 2 seconds
            for i in range(4):
                current = axis.motor.current_control.Iq_measured
                actual_vel = axis.encoder.vel_estimate
                print(f"  Current: {current:.2f}A, Velocity: {actual_vel:.2f} turns/s")
                time.sleep(0.5)
                
                # Check for mechanical movement
                if abs(actual_vel) > 0.5:
                    print(f"Motor is moving! Detected velocity: {actual_vel:.2f} turns/s")
            
            # Pause between tests
            axis.controller.input_torque = 0.0
            time.sleep(0.5)
        
        # Back to idle
        axis.requested_state = AXIS_STATE_IDLE
        
    except Exception as e:
        print(f"Error during torque test: {e}")
        axis.requested_state = AXIS_STATE_IDLE
    
    # Try sensorless mode if everything else fails
    print(f"\n3. Testing sensorless mode...")
    
    try:
        # Configure sensorless
        if hasattr(axis.sensorless_estimator.config, 'pm_flux_linkage'):
            axis.sensorless_estimator.config.pm_flux_linkage = 0.005
            print(f"  Set flux linkage to 0.005")
        
        # Try to enter sensorless mode
        print(f"Entering sensorless control...")
        axis.requested_state = AXIS_STATE_SENSORLESS_CONTROL
        time.sleep(3.0)
        
        if axis.current_state != AXIS_STATE_SENSORLESS_CONTROL:
            print(f"Failed to enter sensorless control. Error: {axis.error}")
            return False
        
        print(f"Successfully entered sensorless control!")
        
        # Test with velocity
        velocities = [10.0, 20.0, 40.0]
        for vel in velocities:
            print(f"\nTesting velocity: {vel} turns/s...")
            axis.controller.input_vel = vel
            
            # Monitor for 2 seconds
            for i in range(4):
                current = axis.motor.current_control.Iq_measured
                print(f"  Current: {current:.2f}A")
                time.sleep(0.5)
        
        # Stop motion
        axis.controller.input_vel = 0.0
        
        # Back to idle
        axis.requested_state = AXIS_STATE_IDLE
        
    except Exception as e:
        print(f"Error during sensorless test: {e}")
        axis.requested_state = AXIS_STATE_IDLE
    
    return True

def check_hardware(axis):
    """Basic hardware checks"""
    print("\nPerforming hardware checks...")
    
    # Check motor phase values
    if hasattr(axis.motor, 'phase_resistance'):
        print(f"Measured phase resistance: {axis.motor.phase_resistance}")
    
    if hasattr(axis.motor, 'phase_inductance'):
        print(f"Measured phase inductance: {axis.motor.phase_inductance}")
    
    # Check for obvious mechanical issues
    print("\nMECHANICAL CHECK INSTRUCTIONS:")
    print("1. Make sure ODrive has power but motor is in IDLE state")
    print("2. Try turning the motor shaft BY HAND")
    print("3. Is there significant resistance to turning? [y/n]")
    response = input().lower()
    
    if response == 'y':
        print("Mechanical binding detected! You need to check:")
        print("- Motor shaft coupling")
        print("- Gearbox connection")
        print("- Any mechanical load connected to motor")
        print("- Try disconnecting motor from all mechanical loads")
    else:
        print("Motor shaft rotates freely. Mechanical binding is not the issue.")
    
    # Check Hall sensors
    print("\nHALL SENSOR CHECK:")
    print("1. Rotate motor shaft by hand while watching Hall state")
    print("2. Initial Hall state:", end=" ")
    
    if hasattr(axis.encoder, 'hall_state'):
        initial_state = axis.encoder.hall_state
        print(initial_state)
        
        print("3. Now rotate motor shaft and watch for state changes...")
        print("   Press Ctrl+C when done")
        
        try:
            while True:
                if hasattr(axis.encoder, 'hall_state'):
                    current_state = axis.encoder.hall_state
                    if current_state != initial_state:
                        print(f"   Hall state changed to: {current_state}")
                        initial_state = current_state
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\nHall sensor check complete.")
    else:
        print("Hall state information not available in this firmware.")
    
    return True

def main():
    parser = argparse.ArgumentParser(description='Gentle MY1020 Motor Test')
    parser.add_argument('--axis', type=int, default=0, help='Axis to test (0 or 1)')
    args = parser.parse_args()
    
    # Parameters known to work with this motor
    gentle_config = {
        'pole_pairs': 8,                # Based on MY1020 specs
        'calib_voltage': 24.0,          # More conservative
        'current_limit': 25.0,          # More conservative
        'calibration_current': 10.0,    # More conservative
        'phase_resistance': 0.15,       # From spec
        'phase_inductance': 0.0001,     # From spec
        'encoder_cpr': 6,               # Hall sensors
        'velocity_limit': 40.0,         # Conservative
        'velocity_gain': 0.02,          # Conservative
    }
    
    # Select axis
    axis_num = args.axis
    print(f"Testing MY1020 motor on axis{axis_num} with gentle parameters...")
    
    # Find ODrive
    odrv = find_odrive()
    if odrv is None:
        print("No ODrive found. Exiting.")
        sys.exit(1)
    
    # Get axis
    axis = getattr(odrv, f"axis{axis_num}")
    
    # Prepare motor
    prepare_motor(axis, gentle_config)
    
    # Calibrate motor
    if not calibrate_motor(axis):
        print("\nPROBLEM: Motor calibration failed.")
        print("Let's check the hardware before proceeding...")
        check_hardware(axis)
        sys.exit(1)
    
    # Test motor movement
    test_motor_movement(axis)
    
    # Perform hardware checks
    check_hardware(axis)
    
    # Final cleanup
    print("\nTest complete. Setting motor to idle state.")
    axis.requested_state = AXIS_STATE_IDLE

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nTest interrupted by user.")
        try:
            odrv0 = odrive.find_any(timeout=3)
            if hasattr(odrv0, f'axis0'):
                odrv0.axis0.requested_state = AXIS_STATE_IDLE
            if hasattr(odrv0, f'axis1'):
                odrv0.axis1.requested_state = AXIS_STATE_IDLE
        except:
            pass
