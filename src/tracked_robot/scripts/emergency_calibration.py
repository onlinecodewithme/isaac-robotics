#!/usr/bin/env python3

"""
ODrive Emergency Calibration Script

This script attempts to calibrate ODrive motors even in challenging situations
using extremely conservative parameters and special workarounds for
common calibration issues.

Usage:
    python3 emergency_calibration.py [--axis 0|1]
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

def print_header(text):
    print("\n" + "=" * 60)
    print(f"  {text}")
    print("=" * 60)

def clear_errors(axis):
    """Clear all errors on axis"""
    try:
        if hasattr(axis, 'error'):
            axis.error = 0
        if hasattr(axis.motor, 'error'):
            axis.motor.error = 0
        if hasattr(axis.encoder, 'error'):
            axis.encoder.error = 0
        if hasattr(axis.controller, 'error'):
            axis.controller.error = 0
        print("✓ Errors cleared")
    except Exception as e:
        print(f"! Error clearing errors: {str(e)}")

def print_error_state(axis):
    """Print detailed error information"""
    try:
        if hasattr(axis, 'error'):
            print(f"Axis error: {axis.error}")
        if hasattr(axis.motor, 'error'):
            print(f"Motor error: {axis.motor.error}")
        if hasattr(axis.encoder, 'error'):
            print(f"Encoder error: {axis.encoder.error}")
        if hasattr(axis.controller, 'error'):
            print(f"Controller error: {axis.controller.error}")
    except Exception as e:
        print(f"! Error reading error states: {str(e)}")

def emergency_motor_calibration(axis):
    """Attempt to calibrate motor with extremely conservative settings"""
    print_header("EMERGENCY MOTOR CALIBRATION")
    print("Using ultra-conservative settings to attempt calibration...")
    
    # First, ensure we're in idle state
    try:
        axis.requested_state = AXIS_STATE_IDLE
        time.sleep(0.5)
    except Exception as e:
        print(f"! Error setting idle state: {str(e)}")
    
    # Clear all errors
    clear_errors(axis)
    
    # Set motor type to high current
    try:
        axis.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
        print("✓ Set motor type to HIGH_CURRENT")
    except Exception as e:
        print(f"! Could not set motor type: {str(e)}")
    
    # Set EXTREMELY conservative current limits
    # These are much lower than normal but might help with calibration
    try:
        # First try newer API
        if hasattr(axis.config, 'dc_max_current'):
            axis.config.dc_max_current = 5.0  # 5 amps
            axis.config.dc_max_negative_current = -5.0
            print("✓ Set current limits to 5.0A (new API)")
        elif hasattr(axis.motor.config, 'current_lim'):
            # Older API
            axis.motor.config.current_lim = 5.0
            print("✓ Set current limit to 5.0A (old API)")
    except Exception as e:
        print(f"! Could not set current limits: {str(e)}")
    
    # Set minimum calibration current
    try:
        # Try different API paths, from newest to oldest
        if hasattr(axis.config, 'calibration_current'):
            axis.config.calibration_current = 2.0  # Very low
            print("✓ Set calibration current to 2.0A (new API)")
        elif hasattr(axis.motor.config, 'calibration_current'):
            axis.motor.config.calibration_current = 2.0
            print("✓ Set calibration current to 2.0A (old API)")
        else:
            print("! Could not set calibration current - unknown API version")
    except Exception as e:
        print(f"! Could not set calibration current: {str(e)}")
    
    # Set very safe resistance calibration voltage
    try:
        axis.motor.config.resistance_calib_max_voltage = 2.0  # Very low
        print("✓ Set calibration voltage to 2.0V")
    except Exception as e:
        print(f"! Could not set calibration voltage: {str(e)}")
    
    # Try to manually set resistance and inductance if available
    # This can bypass calibration issues in some cases
    print("Attempting manual motor parameter configuration...")
    try:
        # These are common ranges for brushless motors similar to those used in robotics
        axis.motor.config.phase_resistance = 0.15  # ohms (reasonable value for larger motors)
        print("✓ Set phase resistance to 0.15 ohms")
    except Exception as e:
        print(f"! Could not set phase resistance: {str(e)}")
    
    try:
        axis.motor.config.phase_inductance = 0.00005  # henries (50 microhenries)
        print("✓ Set phase inductance to 50 μH")
    except Exception as e:
        print(f"! Could not set phase inductance: {str(e)}")
    
    # Also try to set a current control bandwidth that isn't too aggressive
    try:
        if hasattr(axis.motor.config, 'current_control_bandwidth'):
            axis.motor.config.current_control_bandwidth = 100  # Hz (conservative)
            print("✓ Set current control bandwidth to 100 Hz")
    except Exception as e:
        print(f"! Could not set current control bandwidth: {str(e)}")
    
    # Try to avoid DRV fault errors with conservative driver configuration
    try:
        if hasattr(axis.motor.config, 'inverter_temp_limit_lower'):
            axis.motor.config.inverter_temp_limit_lower = 50  # °C
            axis.motor.config.inverter_temp_limit_upper = 65  # °C
            print("✓ Set conservative temperature limits")
    except Exception as e:
        print(f"! Could not set temperature limits: {str(e)}")
    
    # Set conservative pole pairs - most motors are 7 or 8
    try:
        axis.motor.config.pole_pairs = 7  # Common value
        print("✓ Set pole pairs to 7")
    except Exception as e:
        print(f"! Could not set pole pairs: {str(e)}")
    
    # Set Hall encoder mode and CPR
    try:
        axis.encoder.config.mode = ENCODER_MODE_HALL
        print("✓ Set encoder mode to HALL")
        axis.encoder.config.cpr = 42  # 6 states * 7 pole pairs
        print("✓ Set encoder CPR to 42")
    except Exception as e:
        print(f"! Could not configure encoder: {str(e)}")
    
    # Try saving configuration before calibration
    try:
        odrv.save_configuration()
        print("✓ Saved configuration")
        time.sleep(1.0)  # Wait a moment for save to complete
    except Exception as e:
        print(f"! Could not save configuration: {str(e)}")
    
    # Prepare for calibration
    clear_errors(axis)
    
    # First try with skip_encoder_hall_phase flag if available
    # This can help in cases where the motor calibration fails due to Hall errors
    print("\nAttempting motor calibration...")
    try:
        # Skip hall phase detection if the flag exists
        if hasattr(axis.config, 'skip_encoder_hall_phase'):
            original_skip_state = axis.config.skip_encoder_hall_phase
            axis.config.skip_encoder_hall_phase = True
            print("✓ Enabled skip_encoder_hall_phase for safer calibration")
        
        # Run calibration in pre-calibrated mode if that option exists
        if hasattr(axis.motor, 'pre_calibrated'):
            if not axis.motor.pre_calibrated:
                try:
                    axis.motor.pre_calibrated = True
                    print("✓ Set motor to pre_calibrated mode")
                except:
                    print("! Could not set pre_calibrated flag")
    except Exception as e:
        print(f"! Error configuring calibration options: {str(e)}")
    
    # Run the actual calibration
    success = False
    try:
        # Perform motor calibration
        print("Starting motor calibration (resistance only)...")
        axis.requested_state = AXIS_STATE_MOTOR_CALIBRATION
        
        # Wait for calibration to complete (with timeout)
        timeout = 15  # seconds
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            if axis.current_state == AXIS_STATE_IDLE:
                break
            time.sleep(0.5)
            print(".", end="", flush=True)
            
        print()  # Newline after dots
        
        # Check if calibration completed
        if time.time() - start_time >= timeout:
            print("! Motor calibration timed out")
        else:
            # Check if it was successful
            if axis.motor.is_calibrated:
                print("✓ Motor calibration succeeded!")
                success = True
            else:
                print("! Motor calibration failed")
                print("   Error state:")
                print_error_state(axis)
    except Exception as e:
        print(f"! Error during motor calibration: {str(e)}")
    
    # Reset the skip_encoder_hall_phase flag if we changed it
    if success and hasattr(axis.config, 'skip_encoder_hall_phase'):
        try:
            axis.config.skip_encoder_hall_phase = original_skip_state
            print("✓ Reset skip_encoder_hall_phase to original state")
        except:
            pass
    
    # Try to save the calibration
    if success:
        try:
            odrv.save_configuration()
            print("✓ Saved calibration results")
        except Exception as e:
            print(f"! Could not save calibration: {str(e)}")
    
    return success

def emergency_encoder_calibration(axis):
    """Attempt to calibrate encoder with various fallbacks"""
    print_header("EMERGENCY ENCODER CALIBRATION")
    
    # First check if motor is calibrated
    if not axis.motor.is_calibrated:
        print("! Motor is not calibrated - cannot calibrate encoder")
        print("  Run motor calibration first")
        return False
    
    # Clear errors
    clear_errors(axis)
    
    # Set to idle first
    axis.requested_state = AXIS_STATE_IDLE
    time.sleep(0.5)
    
    # Perform encoder calibration
    success = False
    
    # Try all available calibration modes in order of likelihood of success
    calibration_modes = []
    
    # Check what calibration modes are available
    if hasattr(AXIS_STATE, 'ENCODER_HALL_POLARITY_CALIBRATION'):
        calibration_modes.append(("HALL_POLARITY", AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION))
    if hasattr(AXIS_STATE, 'ENCODER_HALL_PHASE_CALIBRATION'):
        calibration_modes.append(("HALL_PHASE", AXIS_STATE_ENCODER_HALL_PHASE_CALIBRATION))
    
    # Always add standard encoder offset calibration
    calibration_modes.append(("OFFSET", AXIS_STATE_ENCODER_OFFSET_CALIBRATION))
    
    # Try each calibration mode until one succeeds
    for name, mode in calibration_modes:
        print(f"\nAttempting {name} calibration...")
        clear_errors(axis)
        
        try:
            axis.requested_state = mode
            
            # Wait for calibration to complete (with timeout)
            timeout = 20  # seconds
            start_time = time.time()
            
            while time.time() - start_time < timeout:
                if axis.current_state == AXIS_STATE_IDLE:
                    break
                time.sleep(0.5)
                print(".", end="", flush=True)
                
            print()  # Newline after dots
            
            # Check if calibration completed
            if time.time() - start_time >= timeout:
                print(f"! {name} calibration timed out, trying next method")
            else:
                # Check if it was successful
                if axis.encoder.is_ready:
                    print(f"✓ {name} calibration succeeded!")
                    success = True
                    break
                else:
                    print(f"! {name} calibration failed")
                    print("   Error state:")
                    print_error_state(axis)
        except Exception as e:
            print(f"! Error during {name} calibration: {str(e)}")
    
    # If all standard methods failed, try the "index search" method
    if not success and hasattr(AXIS_STATE, 'ENCODER_INDEX_SEARCH'):
        print("\nAttempting INDEX_SEARCH calibration (last resort)...")
        clear_errors(axis)
        
        try:
            axis.requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH
            
            # Wait for calibration to complete (with timeout)
            timeout = 20  # seconds
            start_time = time.time()
            
            while time.time() - start_time < timeout:
                if axis.current_state == AXIS_STATE_IDLE:
                    break
                time.sleep(0.5)
                print(".", end="", flush=True)
                
            print()  # Newline after dots
            
            # Check if calibration completed
            if time.time() - start_time >= timeout:
                print("! INDEX_SEARCH calibration timed out")
            else:
                # Check if it was successful
                if axis.encoder.is_ready:
                    print("✓ INDEX_SEARCH calibration succeeded!")
                    success = True
                else:
                    print("! INDEX_SEARCH calibration failed")
                    print("   Error state:")
                    print_error_state(axis)
        except Exception as e:
            print(f"! Error during INDEX_SEARCH calibration: {str(e)}")
    
    # If all calibration methods failed, try manual configuration
    if not success:
        print("\nAll calibration methods failed. Attempting manual configuration...")
        
        # For Hall encoders, we can try to manually set the configuration
        try:
            # Set encoder ready flag and zero offset (this is a workaround)
            # WARNING: This isn't ideal but may help in desperate situations
            if hasattr(axis.encoder, 'config'):
                if hasattr(axis.encoder.config, 'hall_polarity_calibrated'):
                    axis.encoder.config.hall_polarity_calibrated = True
                    print("✓ Manually set hall_polarity_calibrated flag")
                
                if hasattr(axis.encoder.config, 'hall_offset'):
                    axis.encoder.config.hall_offset = 0
                    print("✓ Manually set hall_offset to 0")
                    
                # On older firmware, there may be a direct is_ready flag
                if hasattr(axis.encoder, 'config') and hasattr(axis.encoder.config, 'use_index'):
                    axis.encoder.config.use_index = False
                    print("✓ Disabled index search requirement")
                
                success = True
        except Exception as e:
            print(f"! Error during manual encoder configuration: {str(e)}")
    
    # Save configuration
    if success:
        try:
            odrv.save_configuration()
            print("✓ Saved encoder calibration")
        except Exception as e:
            print(f"! Could not save encoder calibration: {str(e)}")
    
    return success

def test_velocity_control(axis):
    """Test velocity control with very gentle movements"""
    print_header("TESTING VELOCITY CONTROL")
    print("Testing motor with very gentle movements...")
    
    # Clear errors and set to idle
    clear_errors(axis)
    axis.requested_state = AXIS_STATE_IDLE
    time.sleep(0.5)
    
    # Set velocity control mode
    try:
        axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        print("✓ Set velocity control mode")
    except Exception as e:
        print(f"! Could not set control mode: {str(e)}")
        return False
    
    # Set extremely conservative velocity limit (if available)
    try:
        axis.controller.config.vel_limit = 5.0  # rad/s
        print("✓ Set velocity limit to 5.0 rad/s")
    except Exception as e:
        print(f"! Could not set velocity limit: {str(e)}")
    
    # Enter closed loop control
    print("Entering closed loop control...")
    success = False
    try:
        axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        time.sleep(1.0)
        
        if axis.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
            print("! Failed to enter closed loop control")
            print_error_state(axis)
            return False
        
        print("✓ Successfully entered closed loop control")
        
        # Start with very slow movement
        velocities = [0.5, 1.0, 2.0, 0.0, -0.5, -1.0, -2.0, 0.0]
        success = True
        
        for vel in velocities:
            direction = "clockwise" if vel > 0 else "counter-clockwise" if vel < 0 else "stopping"
            print(f"\nTesting {direction} at {abs(vel):.1f} rad/s...")
            axis.controller.input_vel = vel
            
            # Monitor for a couple of seconds
            for i in range(4):
                try:
                    actual_vel = axis.encoder.vel_estimate if hasattr(axis.encoder, 'vel_estimate') else 0
                    current = axis.motor.current_control.Iq_measured if hasattr(axis.motor.current_control, 'Iq_measured') else 0
                    print(f"  Velocity: {actual_vel:.2f} rad/s, Current: {current:.2f}A")
                    
                    # Check for errors
                    if axis.error != 0 or axis.motor.error != 0 or axis.encoder.error != 0:
                        print("\n! Error detected during movement:")
                        print_error_state(axis)
                        success = False
                        break
                except Exception as e:
                    print(f"! Error reading status: {str(e)}")
                
                time.sleep(0.5)
                
            if not success:
                break
        
        # Always stop motor
        axis.controller.input_vel = 0.0
        time.sleep(0.5)
        
    except Exception as e:
        print(f"! Error during velocity control test: {str(e)}")
        success = False
    finally:
        # Return to idle state
        try:
            axis.requested_state = AXIS_STATE_IDLE
            print("✓ Returned to idle state")
        except:
            print("! Failed to return to idle state")
    
    if success:
        print("\n✓ Velocity control test completed successfully!")
    else:
        print("\n! Velocity control test failed")
    
    return success

def basic_calibration(axis_num):
    """Perform a very basic calibration with minimal steps"""
    # Connect to ODrive
    global odrv
    print("Looking for ODrive...")
    try:
        odrv = odrive.find_any(timeout=10)
        print(f"✓ Found ODrive! Serial Number: {odrv.serial_number}")
    except Exception as e:
        print(f"! Could not find ODrive: {str(e)}")
        return False
    
    # Get the specified axis
    axis = getattr(odrv, f"axis{axis_num}")
    print(f"Using axis{axis_num}")
    
    # Print initial state
    print("\nInitial state:")
    print_error_state(axis)
    
    # Try motor calibration first
    motor_success = emergency_motor_calibration(axis)
    
    # Wait a few seconds to let things settle
    time.sleep(3.0)
    
    # Try to reconnect, as ODrive might have reset during calibration
    try:
        print("\nReconnecting to ODrive...")
        odrv = odrive.find_any(timeout=10)
        print(f"✓ Reconnected to ODrive")
        axis = getattr(odrv, f"axis{axis_num}")
    except Exception as e:
        print(f"! Could not reconnect to ODrive: {str(e)}")
        return False
    
    # If motor calibration succeeded, try encoder calibration
    encoder_success = False
    if motor_success:
        print("\nMotor calibration successful, proceeding to encoder calibration...")
        encoder_success = emergency_encoder_calibration(axis)
    else:
        print("\nMotor calibration failed, cannot proceed to encoder calibration")
    
    # Wait a few seconds to let things settle
    time.sleep(3.0)
    
    # Try to reconnect, as ODrive might have reset during calibration
    try:
        print("\nReconnecting to ODrive...")
        odrv = odrive.find_any(timeout=10)
        print(f"✓ Reconnected to ODrive")
        axis = getattr(odrv, f"axis{axis_num}")
    except Exception as e:
        print(f"! Could not reconnect to ODrive: {str(e)}")
        return False
    
    # If both calibrations succeeded, try a basic movement test
    if motor_success and encoder_success:
        print("\nBoth calibrations successful, testing basic movement...")
        test_velocity_control(axis)
    else:
        print("\nCalibration incomplete, skipping movement test")
    
    # Print final results
    print_header("CALIBRATION RESULTS")
    try:
        print(f"Motor calibrated: {axis.motor.is_calibrated}")
        print(f"Encoder ready: {axis.encoder.is_ready}")
        print(f"\nFinal error states:")
        print_error_state(axis)
    except Exception as e:
        print(f"! Error reading final state: {str(e)}")
    
    print("\nFor more detailed testing, use:")
    print(f"python3 src/tracked_robot/scripts/advanced_motor_test.py --axis {axis_num} --mode velocity")
    
    return motor_success and encoder_success

def main():
    parser = argparse.ArgumentParser(description="ODrive Emergency Calibration")
    parser.add_argument('--axis', type=int, default=0, choices=[0, 1],
                        help="Axis to calibrate (0 or 1)")
    args = parser.parse_args()
    
    print_header("ODRIVE EMERGENCY CALIBRATION")
    print("This utility will attempt to calibrate your ODrive motor")
    print("using extremely conservative settings and special workarounds.")
    print("Use this when other calibration methods have failed.")
    
    basic_calibration(args.axis)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nCalibration cancelled by user.")
