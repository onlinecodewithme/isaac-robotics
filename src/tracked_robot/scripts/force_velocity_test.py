#!/usr/bin/env python3

"""
ODrive Force Velocity Test

This script allows testing motor velocity control even without fully
calibrated Hall sensors by using open-loop control or bypassing
normal checks.

Usage:
    python3 force_velocity_test.py [--axis 0|1] [--mode bypass|openloop]
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
    header = "\n" + "=" * 60 + "\n"
    header += f"  {text}\n"
    header += "=" * 60
    print(header)

def connect_to_odrive(timeout=10, retry_count=3):
    """Connect to ODrive with retries"""
    print("Connecting to ODrive...")
    
    for attempt in range(retry_count):
        try:
            odrv = odrive.find_any(timeout=timeout)
            print(f"✓ Connected to ODrive {odrv.serial_number}")
            return odrv
        except Exception as e:
            if attempt < retry_count - 1:
                print(f"Connection attempt {attempt+1} failed. Retrying...")
                time.sleep(1)
            else:
                print(f"Failed to connect to ODrive after {retry_count} attempts: {str(e)}")
                return None
    
    return None

def force_encoder_ready(axis):
    """Force the encoder ready flag (DANGEROUS but useful for testing)"""
    # WARNING: This is not recommended for production use, only for testing
    print("WARNING: Forcing encoder ready flag. This is for TESTING ONLY.")
    print("         Use with caution and at your own risk!")

    # Clear any errors
    if hasattr(axis, 'error'):
        axis.error = 0
    if hasattr(axis.motor, 'error'):
        axis.motor.error = 0
    if hasattr(axis.encoder, 'error'):
        axis.encoder.error = 0
    if hasattr(axis.controller, 'error'):
        axis.controller.error = 0
        
    # Set count_in_cpr manually if available
    # This helps with position feedback
    if hasattr(axis.encoder, 'count_in_cpr'):
        axis.encoder.count_in_cpr = 0
        print("✓ Reset encoder position")
    
    # Try to force is_ready on the encoder
    try:
        if hasattr(axis.encoder, 'config'):
            # Older ODrive firmware may have this flag
            if hasattr(axis.encoder.config, 'pre_calibrated'):
                axis.encoder.config.pre_calibrated = True
                print("✓ Set encoder.config.pre_calibrated = True")
                
            # Try to set other flags that might help
            if hasattr(axis.encoder.config, 'hall_polarity_calibrated'):
                axis.encoder.config.hall_polarity_calibrated = True
                print("✓ Set encoder.config.hall_polarity_calibrated = True")
                
            # Save configuration
            try:
                odrv.save_configuration()
                print("✓ Saved encoder configuration")
                time.sleep(3.0)  # Give device time to reset if needed
            except Exception as e:
                print(f"! Could not save configuration: {e}")
                
            # Need to reconnect after saving configuration
            return connect_to_odrive()
    except Exception as e:
        print(f"! Error forcing encoder ready: {e}")
        
    return None

def test_velocity_open_loop(axis, speeds):
    """Test velocity in open loop mode"""
    print_header("OPEN LOOP VELOCITY TEST")
    print("This mode bypasses encoder feedback and directly controls the motor")
    
    # Set open loop values
    axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
    print("✓ Set to VELOCITY control mode")
    
    # Set conservative current limits
    try:
        axis.motor.config.current_lim = 10.0  # Amps
        print(f"✓ Set current limit to {axis.motor.config.current_lim}A")
    except:
        print("! Could not set current limit")
    
    # Set velocity limit
    try:
        axis.controller.config.vel_limit = 10.0  # rad/s
        print(f"✓ Set velocity limit to {axis.controller.config.vel_limit} rad/s")
    except:
        print("! Could not set velocity limit")
    
    # Enter open loop control
    print("\nEntering control mode...")
    
    try:
        # Use closed loop, but we forced calibration
        axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        time.sleep(1.0)
    except Exception as e:
        print(f"! Error entering control mode: {e}")
        return False
    
    if axis.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
        print(f"! Failed to enter control mode. Current state: {axis.current_state}")
        print("Trying to force the controller...")
        
        # Try to force it anyway
        try:
            axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            time.sleep(1.0)
        except:
            print("! Second attempt failed")
            return False
    
    print("✓ Successfully entered control mode")
    
    # Test various speeds
    for speed in speeds:
        direction = "forward" if speed > 0 else "backward" if speed < 0 else "stopped"
        print(f"\nTesting {direction} at {abs(speed):.1f} rad/s...")
        
        try:
            axis.controller.input_vel = speed
            
            # Monitor for 3 seconds
            for i in range(6):
                # Try to read current and velocity if available
                try:
                    current = 0
                    actual_vel = 0
                    
                    if hasattr(axis.motor, 'current_control') and hasattr(axis.motor.current_control, 'Iq_measured'):
                        current = axis.motor.current_control.Iq_measured
                        
                    if hasattr(axis.encoder, 'vel_estimate'):
                        actual_vel = axis.encoder.vel_estimate
                        
                    print(f"  Commanded: {speed:.1f} rad/s, Actual: {actual_vel:.1f} rad/s, Current: {current:.2f}A")
                except Exception as e:
                    print(f"  ! Error reading feedback: {e}")
                    
                time.sleep(0.5)
                
                # Check for errors
                try:
                    if axis.error != 0:
                        print(f"! Error detected: axis.error = {axis.error}")
                        break
                except:
                    pass
        except Exception as e:
            print(f"! Error controlling velocity: {e}")
            
        # Set speed to 0 between tests
        try:
            axis.controller.input_vel = 0.0
            time.sleep(1.0)
        except:
            pass
    
    # Return to idle state
    try:
        axis.requested_state = AXIS_STATE_IDLE
        print("\n✓ Returned to idle state")
    except Exception as e:
        print(f"\n! Error returning to idle state: {e}")
    
    return True

def start_test(axis_num, mode):
    """Run the specified test mode"""
    print(f"Testing axis {axis_num} in {mode} mode")
    
    # Connect to ODrive
    odrv = connect_to_odrive()
    if not odrv:
        print("! Could not connect to ODrive")
        return False
    
    # Get axis
    try:
        axis = getattr(odrv, f"axis{axis_num}")
    except Exception as e:
        print(f"! Error accessing axis{axis_num}: {e}")
        return False
    
    # Print initial status
    print("\nInitial status:")
    print(f"  Motor calibrated: {axis.motor.is_calibrated}")
    print(f"  Encoder ready: {axis.encoder.is_ready}")
    
    # Check if the motor is calibrated
    if not axis.motor.is_calibrated:
        print("! Motor is not calibrated")
        print("  Please run calibration script first to at least calibrate the motor")
        return False
    
    # Force encoder ready if in bypass mode
    if mode == "bypass":
        # Force encoder ready
        print("\nForcing encoder to ready state...")
        new_odrv = force_encoder_ready(axis)
        
        if new_odrv:
            odrv = new_odrv
            axis = getattr(odrv, f"axis{axis_num}")
            print("\nStatus after bypass:")
            print(f"  Motor calibrated: {axis.motor.is_calibrated}")
            print(f"  Encoder ready: {axis.encoder.is_ready}")
    
    # Test velocity control
    if mode == "bypass" or mode == "openloop":
        # Predefined speed sequence for testing
        speeds = [1.0, 3.0, 5.0, 0.0, -1.0, -3.0, -5.0, 0.0]
        test_velocity_open_loop(axis, speeds)
    
    return True

def main():
    parser = argparse.ArgumentParser(description="ODrive Force Velocity Test")
    parser.add_argument('--axis', type=int, default=0, choices=[0, 1],
                        help="Axis to test (0 or 1)")
    parser.add_argument('--mode', type=str, default='bypass', choices=['bypass', 'openloop'],
                        help="Test mode (bypass or openloop)")
    args = parser.parse_args()
    
    print_header("FORCE VELOCITY TEST")
    print("This script tests motor velocity control using measures to")
    print("bypass normal encoder calibration requirements.")
    print("IMPORTANT: Use with caution! This is for testing purposes only.")
    
    # Execute the selected test
    start_test(args.axis, args.mode)
    
    print_header("TEST COMPLETE")
    print("If the motor did not move, check physical connections.")
    print("Ensure motor phases and Hall sensor wires are properly connected.")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nTest canceled by user")
        # Try to make sure motor is stopped and in idle mode
        try:
            odrv = odrive.find_any(timeout=3)
            for axis_num in [0, 1]:
                try:
                    axis = getattr(odrv, f"axis{axis_num}")
                    axis.controller.input_vel = 0.0
                    axis.requested_state = AXIS_STATE_IDLE
                except:
                    pass
        except:
            pass
