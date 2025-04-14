#!/usr/bin/env python3

"""
Advanced ODrive Hall Sensor Calibration

This script is specialized for properly configuring and calibrating
Hall sensors on ODrive controllers, particularly for high-power
48V motors used in tracked robots.

Usage:
    python3 hall_sensor_calibration.py [--axis 0|1]
"""

import sys
import time
import argparse
import math
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
        time.sleep(0.2)
        print("✓ Errors cleared")
    except Exception as e:
        print(f"! Error clearing errors: {str(e)}")

def print_error_state(axis, prefix=""):
    """Print detailed error information"""
    if prefix:
        print(f"\n{prefix}")
    
    try:
        print(f"  Axis error: {axis.error}")
        print(f"  Motor error: {axis.motor.error}")
        print(f"  Encoder error: {axis.encoder.error}")
        print(f"  Controller error: {axis.controller.error}")
    except Exception as e:
        print(f"  Error reading error states: {str(e)}")

def detect_pole_pairs(axis):
    """Try to detect the actual number of pole pairs by testing Hall transitions"""
    print("\nDetecting pole pairs from Hall transitions...")
    
    if not hasattr(axis.encoder, 'hall_state'):
        print("! Cannot access Hall state - firmware may not support this feature")
        return 7  # Default for many brushless motors
    
    # Save current state and prepare for testing
    original_mode = axis.encoder.config.mode
    axis.requested_state = AXIS_STATE_IDLE
    clear_errors(axis)
    
    # Set to Hall mode temporarily for testing
    axis.encoder.config.mode = ENCODER_MODE_HALL
    axis.encoder.config.cpr = 6  # Standard for basic Hall setup
    
    try:
        print("Rotate the motor EXACTLY ONE FULL REVOLUTION by hand, slowly and steadily.")
        print("Press Enter when ready to start counting...")
        input()
        
        print("Counting Hall state changes. Turn the motor ONE FULL REVOLUTION now...")
        start_hall = axis.encoder.hall_state
        transitions = []
        last_state = start_hall
        
        # Give user 30 seconds to rotate the motor one full turn
        timeout = time.time() + 30
        states_seen = {last_state}
        
        while time.time() < timeout:
            current_state = axis.encoder.hall_state
            if current_state != last_state:
                transitions.append((last_state, current_state))
                last_state = current_state
                states_seen.add(current_state)
                print(f"  Hall transition: {transitions[-1][0]} → {transitions[-1][1]}")
                
            # If we see the starting state again after seeing at least 3 different states,
            # the user might have completed one revolution
            if len(transitions) > 5 and current_state == start_hall and len(states_seen) >= 3:
                verify = input("\nLooks like you completed one revolution. Is this correct? (y/n): ")
                if verify.lower() in ('y', 'yes'):
                    break
            
            time.sleep(0.05)
        
        # Return to original encoder mode
        axis.encoder.config.mode = original_mode
        
        # Calculate pole pairs based on Hall transitions
        # Each pole pair typically causes 6 Hall state transitions in one revolution
        if len(transitions) < 3:
            print("! Too few transitions detected. Using default pole pairs value.")
            return 7  # Default value
            
        # Most common brushless motors have transitions that are a multiple of 6
        pole_pairs = round(len(transitions) / 6)
        if pole_pairs < 1:
            pole_pairs = 7  # Fallback
            
        print(f"\nDetected approximately {len(transitions)} Hall transitions in one revolution.")
        print(f"This suggests {pole_pairs} pole pairs.")
        
        # Verify with user
        user_pp = input(f"Enter the pole pairs to use [default: {pole_pairs}]: ")
        if user_pp.strip() and user_pp.isdigit() and int(user_pp) > 0:
            return int(user_pp)
        return pole_pairs
        
    except KeyboardInterrupt:
        print("\nPole pair detection cancelled. Using default value.")
        # Return to original encoder mode
        axis.encoder.config.mode = original_mode
        return 7  # Default
    except Exception as e:
        print(f"\nError during pole pair detection: {str(e)}")
        # Return to original encoder mode
        axis.encoder.config.mode = original_mode
        return 7  # Default

def configure_hall_encoder(axis, pole_pairs):
    """Configure encoder for Hall sensors with correct parameters"""
    print_header("CONFIGURING HALL ENCODER")
    print(f"Using {pole_pairs} pole pairs")
    
    try:
        # Set Hall mode
        axis.encoder.config.mode = ENCODER_MODE_HALL
        print("✓ Set encoder mode to HALL")
        
        # Set counts per revolution to 6 * pole_pairs
        cpr = 6 * pole_pairs
        axis.encoder.config.cpr = cpr
        print(f"✓ Set encoder CPR to {cpr} (6 states * {pole_pairs} pole pairs)")
        
        # Set appropriate bandwidth
        if hasattr(axis.encoder.config, 'bandwidth'):
            axis.encoder.config.bandwidth = 100  # Conservative value for stability
            print("✓ Set encoder bandwidth to 100 Hz")
            
        # Set calib_scan_distance if available (for direction detection)
        if hasattr(axis.encoder.config, 'calib_scan_distance'):
            axis.encoder.config.calib_scan_distance = 150  # degrees
            print("✓ Set calibration scan distance to 150 degrees")
            
        # Additional settings if needed
        if hasattr(axis.encoder.config, 'hall_polarity_calibrated'):
            # Reset polarity calibration flag to ensure we do a fresh calibration
            try:
                axis.encoder.config.hall_polarity_calibrated = False
                print("✓ Reset Hall polarity calibration flag")
            except:
                print("! Could not reset Hall polarity calibration flag")
        
        # Additional settings for better performance
        if hasattr(axis.config, 'enable_watchdog'):
            axis.config.enable_watchdog = False
            print("✓ Disabled watchdog during calibration")
            
        # Save configuration
        try:
            odrv.save_configuration()
            print("✓ Configuration saved")
        except:
            print("! Could not save configuration")
        
        return True
    except Exception as e:
        print(f"! Error configuring Hall encoder: {str(e)}")
        return False

def calibrate_motor(axis):
    """Perform motor calibration"""
    print_header("MOTOR CALIBRATION")
    
    # Set motor calibration current
    # Using a conservative value for high-power motors
    try:
        # Try API v0.5.1+
        if hasattr(axis.config, 'calibration_current'):
            axis.config.calibration_current = 10.0  # Amps
            print("✓ Set calibration current to 10.0A (new API)")
        # Older API
        elif hasattr(axis.motor.config, 'calibration_current'):
            axis.motor.config.calibration_current = 10.0  # Amps
            print("✓ Set calibration current to 10.0A (old API)")
        else:
            print("! Could not set calibration current")
    except Exception as e:
        print(f"! Error setting calibration current: {str(e)}")
    
    # Clear errors and start calibration
    clear_errors(axis)
    print("Starting motor calibration...")
    
    try:
        axis.requested_state = AXIS_STATE_MOTOR_CALIBRATION
        
        # Wait for calibration to complete
        timeout = 15  # seconds
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            if axis.current_state == AXIS_STATE_IDLE:
                break
            time.sleep(0.2)
            
        if time.time() - start_time >= timeout:
            print("! Motor calibration timed out")
            return False
        
        # Check if successful
        if axis.motor.is_calibrated:
            print("✓ Motor calibration successful!")
            return True
        else:
            print(f"! Motor calibration failed")
            print_error_state(axis, "Motor Errors:")
            return False
    except Exception as e:
        print(f"! Error during motor calibration: {str(e)}")
        return False

def calibrate_hall_sensors(axis):
    """Perform Hall sensor calibration"""
    print_header("HALL SENSOR CALIBRATION")
    
    if not axis.motor.is_calibrated:
        print("! Motor must be calibrated before Hall sensors")
        return False
    
    # Clear errors and start calibration
    clear_errors(axis)
    print("Starting Hall sensor calibration...")
    
    try:
        # Choose the right calibration state based on firmware
        if hasattr(AXIS_STATE, 'ENCODER_HALL_POLARITY_CALIBRATION'):
            print("Using HALL_POLARITY_CALIBRATION")
            axis.requested_state = AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION
        else:
            print("Using standard ENCODER_OFFSET_CALIBRATION")
            axis.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
        
        # Wait for calibration to complete
        timeout = 30  # seconds - Hall calibration might take longer
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            if axis.current_state == AXIS_STATE_IDLE:
                break
            time.sleep(0.2)
            
        if time.time() - start_time >= timeout:
            print("! Hall sensor calibration timed out")
            return False
        
        # Check if successful
        if axis.encoder.is_ready:
            print("✓ Hall sensor calibration successful!")
            return True
        else:
            print(f"! Hall sensor calibration failed")
            print_error_state(axis, "Encoder Errors:")
            return False
    except Exception as e:
        print(f"! Error during Hall sensor calibration: {str(e)}")
        return False

def verify_hall_sensor_operation(axis):
    """Verify Hall sensors are working properly"""
    print_header("HALL SENSOR VERIFICATION")
    
    if not hasattr(axis.encoder, 'hall_state'):
        print("! Hall state information not available")
        return False
    
    print("Checking Hall sensor operation...")
    print("Please rotate the motor shaft BY HAND and watch the Hall state")
    print("Press Ctrl+C to stop watching")
    
    try:
        initial_state = axis.encoder.hall_state
        print(f"Initial Hall state: {initial_state}")
        
        print("Watching for Hall state changes...")
        last_state = initial_state
        hall_changes = 0
        
        start_time = time.time()
        timeout = 15  # seconds
        
        while time.time() - start_time < timeout:
            current_state = axis.encoder.hall_state
            if current_state != last_state:
                hall_changes += 1
                print(f"  Hall state changed: {last_state} → {current_state}")
                last_state = current_state
            time.sleep(0.05)
            
        if hall_changes > 5:
            print(f"\n✓ Detected {hall_changes} Hall state changes. Hall sensors are working properly!")
            return True
        else:
            print(f"\n! Only {hall_changes} Hall state changes detected. Hall sensors may not be working optimally.")
            return False
    except KeyboardInterrupt:
        if hall_changes > 0:
            print(f"\n✓ Detected {hall_changes} Hall state changes. Hall sensors appear to be working!")
            return True
        else:
            print("\n! No Hall state changes detected. Hall sensors may be disconnected or faulty.")
            return False

def test_closed_loop_control(axis):
    """Test if motor works in closed loop control"""
    print_header("TESTING CLOSED LOOP CONTROL")
    
    # Make sure it's calibrated
    if not axis.motor.is_calibrated or not axis.encoder.is_ready:
        print("! Motor or encoder not calibrated")
        return False
    
    # Clear errors
    clear_errors(axis)
    
    try:
        # Configure for velocity control
        axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        print("✓ Set to velocity control mode")
        
        # Set conservative current limits for testing
        try:
            # Try API v0.5.1+
            if hasattr(axis.config, 'dc_max_current'):
                axis.config.dc_max_current = 20.0  # Amps
                axis.config.dc_max_negative_current = -20.0  # Amps
                print("✓ Set conservative current limits: 20.0A (new API)")
            # Older API
            elif hasattr(axis.motor.config, 'current_lim'):
                axis.motor.config.current_lim = 20.0  # Amps
                print("✓ Set conservative current limits: 20.0A (old API)")
        except Exception as e:
            print(f"! Could not set current limits: {str(e)}")
        
        # Set velocity limit
        if hasattr(axis.controller.config, 'vel_limit'):
            axis.controller.config.vel_limit = 10.0  # rad/s, conservative limit
            print("✓ Set velocity limit to 10.0 rad/s")
        
        print("\nAttempting to enter closed loop control...")
        axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        time.sleep(1.0)
        
        if axis.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
            print("! Failed to enter closed loop control")
            print_error_state(axis, "Control Errors:")
            axis.requested_state = AXIS_STATE_IDLE
            return False
        
        print("✓ Successfully entered closed loop control!")
        
        # Gentle movement test
        print("\nTesting gentle movement...")
        for velocity in [1.0, 2.0, -1.0, -2.0, 0.0]:
            if velocity == 0.0:
                print("\nStopping motor...")
            else:
                direction = "clockwise" if velocity > 0 else "counter-clockwise"
                print(f"\nMoving {direction} at {abs(velocity)} rad/s...")
                
            axis.controller.input_vel = velocity
            
            # Monitor for 2 seconds
            for i in range(4):
                curr_vel = axis.encoder.vel_estimate if hasattr(axis.encoder, 'vel_estimate') else "unknown"
                curr = axis.motor.current_control.Iq_measured if hasattr(axis.motor.current_control, 'Iq_measured') else "unknown"
                
                print(f"  Velocity: {curr_vel}, Current: {curr}")
                
                # Check for errors
                if axis.error != 0 or axis.motor.error != 0 or axis.encoder.error != 0:
                    print("\n! Error detected during movement:")
                    print_error_state(axis)
                    break
                    
                time.sleep(0.5)
        
        # Return to idle
        print("\nReturning to idle state...")
        axis.requested_state = AXIS_STATE_IDLE
        print("✓ Movement test complete!")
        return True
        
    except Exception as e:
        print(f"! Error during closed loop testing: {str(e)}")
        try:
            axis.controller.input_vel = 0.0
            axis.requested_state = AXIS_STATE_IDLE
        except:
            pass
        return False

def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(description="ODrive Hall Sensor Calibration")
    parser.add_argument('--axis', type=int, choices=[0, 1], default=0, 
                        help="Axis to calibrate (default: 0)")
    args = parser.parse_args()

    print_header("ODRIVE HALL SENSOR CALIBRATION UTILITY")
    print("This utility will properly calibrate your Hall sensors")
    print("for more reliable operation with ODrive controllers.")
    
    # Connect to ODrive
    print("\nLooking for ODrive...")
    try:
        global odrv
        odrv = odrive.find_any(timeout=10)
        print(f"✓ Found ODrive! Serial Number: {odrv.serial_number}")
    except Exception as e:
        print(f"! Error finding ODrive: {str(e)}")
        sys.exit(1)
    
    # Get the axis
    axis_num = args.axis
    axis = getattr(odrv, f"axis{axis_num}")
    print(f"Using axis{axis_num}")
    
    # Initial error state
    print_error_state(axis, "Initial error states:")
    
    # Clear all errors
    clear_errors(axis)
    
    # Set to idle state to start
    print("\nSetting to IDLE state...")
    axis.requested_state = AXIS_STATE_IDLE
    time.sleep(1.0)
    
    # Print initial configuration
    print("\nCurrent configuration:")
    print(f"  Encoder mode: {axis.encoder.config.mode}")
    print(f"  Encoder CPR: {axis.encoder.config.cpr}")
    if hasattr(axis.encoder.config, 'bandwidth'):
        print(f"  Encoder bandwidth: {axis.encoder.config.bandwidth}")
    
    # Detect pole pairs
    pole_pairs = detect_pole_pairs(axis)
    
    # Configure Hall encoder
    if not configure_hall_encoder(axis, pole_pairs):
        print("! Failed to configure Hall encoder properly.")
        sys.exit(1)
    
    # Perform motor calibration
    if not calibrate_motor(axis):
        print("! Motor calibration failed. Cannot proceed with Hall calibration.")
        sys.exit(1)
    
    # Save configuration before proceeding
    try:
        odrv.save_configuration()
        print("✓ Configuration saved after motor calibration")
    except:
        print("! Could not save configuration")
    
    # Perform Hall sensor calibration
    if not calibrate_hall_sensors(axis):
        print("! Hall sensor calibration failed.")
        print("  Check Hall sensor connections and try again.")
        sys.exit(1)
    
    # Verify Hall sensors operation
    verify_hall_sensor_operation(axis)
    
    # Save final configuration
    try:
        odrv.save_configuration()
        print("\n✓ Final configuration saved")
    except:
        print("\n! Could not save final configuration")
    
    # Test closed loop control
    test_closed_loop_control(axis)
    
    print_header("CALIBRATION COMPLETE")
    print("Your Hall sensors have been calibrated.")
    print("You can now use the ODrive in your robot!")
    
    # Final error check
    clear_errors(axis)
    print_error_state(axis, "Final error state:")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nCalibration cancelled by user.")
        sys.exit(0)
