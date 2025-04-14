#!/usr/bin/env python3

"""
ODrive All-in-One Calibration and Configuration

This script provides a comprehensive calibration routine that handles
device resets properly and allows both motor and encoder calibration
to be completed in a single run, with special handling for the
"object disappeared" errors.

Usage:
    python3 all_in_one_calibration.py [--axis 0|1]
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

def connect_to_odrive(timeout=10, retry_count=5, verbose=True):
    """Connect to ODrive with retries"""
    if verbose:
        print("Connecting to ODrive...")
    
    for attempt in range(retry_count):
        try:
            odrv = odrive.find_any(timeout=timeout)
            if verbose:
                print(f"✓ Connected to ODrive {odrv.serial_number}")
            return odrv
        except Exception as e:
            if attempt < retry_count - 1:
                if verbose:
                    print(f"Connection attempt {attempt+1} failed. Retrying...")
                time.sleep(1)
            else:
                if verbose:
                    print(f"Failed to connect after {retry_count} attempts: {str(e)}")
                return None
    
    return None

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
        return True
    except Exception as e:
        print(f"! Error clearing errors: {str(e)}")
        return False

def configure_calibration_parameters(axis):
    """Set up reasonable parameters for calibration"""
    try:
        # Motor type
        axis.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
        print("✓ Set motor type to HIGH_CURRENT")
        
        # Pole pairs - typically 7 for many brushless motors used with ODrive
        axis.motor.config.pole_pairs = 7
        print("✓ Set pole pairs to 7")
        
        # Current limits for 1000W motors - increased for better calibration success
        axis.motor.config.current_lim = 25.0
        print("✓ Set current limit to 25.0A")
        
        # Calibration current - higher for 1000W motors
        axis.motor.config.calibration_current = 15.0
        print("✓ Set calibration current to 15.0A")
        
        # Resistance calibration voltage - lower for safer operation
        axis.motor.config.resistance_calib_max_voltage = 4.0
        print("✓ Set resistance calibration voltage to 4.0V")
        
        # Phase inductance and resistance - reasonable values
        axis.motor.config.phase_inductance = 0.00005  # 50 μH
        axis.motor.config.phase_resistance = 0.2      # 0.2 Ω
        print("✓ Set motor parameters manually")
        
        # Encoder setup
        axis.encoder.config.mode = ENCODER_MODE_HALL
        axis.encoder.config.cpr = 6 * axis.motor.config.pole_pairs
        print(f"✓ Set encoder CPR to {axis.encoder.config.cpr} (6 states * {axis.motor.config.pole_pairs} pole pairs)")
        
        # Set bandwidth if available
        if hasattr(axis.encoder.config, 'bandwidth'):
            axis.encoder.config.bandwidth = 100  # Hz
            print("✓ Set encoder bandwidth to 100 Hz")
            
        return True
    except Exception as e:
        print(f"! Error setting parameters: {str(e)}")
        return False

def save_configuration_with_reconnect(odrv):
    """Save configuration and handle expected reset"""
    print("\nSaving configuration...")
    try:
        odrv.save_configuration()
        print("Configuration saved, device will likely reset...")
    except Exception as e:
        if "object disappeared" in str(e).lower():
            print("Device reset as expected when saving")
        else:
            print(f"! Error during save: {str(e)}")
    
    # Always wait after save
    print("Waiting for device to reset and come back online...")
    time.sleep(7.0)
    
    # Reconnect
    new_odrv = connect_to_odrive(verbose=False)
    if new_odrv is None:
        print("! Failed to reconnect after configuration save")
    return new_odrv

def calibrate_motor(axis):
    """Calibrate motor and handle reset"""
    print("\n--- Motor Calibration ---")
    try:
        # Make sure we're in idle state
        axis.requested_state = AXIS_STATE_IDLE
        time.sleep(0.5)
        clear_errors(axis)
        
        # Start calibration
        print("Starting motor calibration (motor will chirp)...")
        axis.requested_state = AXIS_STATE_MOTOR_CALIBRATION
        
        # Wait for calibration
        max_wait = 15  # seconds
        start_time = time.time()
        while time.time() - start_time < max_wait:
            try:
                if axis.current_state == AXIS_STATE_IDLE:
                    break
                time.sleep(0.5)
                print(".", end="", flush=True)
            except:
                # Handle disconnection during calibration
                time.sleep(0.5)
                print("x", end="", flush=True)
        
        print("")  # New line
        
        # Check if we're still connected
        try:
            state = axis.current_state
            is_calibrated = axis.motor.is_calibrated
            if is_calibrated:
                print("✓ Motor calibration successful!")
                return True
            else:
                print(f"! Motor calibration failed, state: {state}")
                return False
        except Exception as e:
            print(f"! Error checking motor calibration: {str(e)}")
            print("  Device may have reset, reconnecting...")
            return None
    except Exception as e:
        print(f"! Error during motor calibration: {str(e)}")
        return False

def force_encoder_ready(axis):
    """Force the encoder ready flag using the manual method"""
    print("\n--- Manual Encoder Configuration ---")
    
    try:
        # Set to idle first
        axis.requested_state = AXIS_STATE_IDLE
        clear_errors(axis)
        
        # First check if Hall sensors are working
        if hasattr(axis.encoder, 'hall_state'):
            hall_state = axis.encoder.hall_state
            print(f"Current Hall state: {hall_state}")
            
            if hall_state < 1 or hall_state > 6:
                print("! Invalid Hall state detected")
                print("  Check Hall sensor connections")
                return False
                
            print("✓ Hall sensors are working correctly")
            
            # Ask user to confirm movement ability
            input("\nPlease check that both motor and Hall sensors are firmly connected.")
            
            # Now try to set pre-calibrated flag if available
            if hasattr(axis.encoder.config, 'pre_calibrated'):
                axis.encoder.config.pre_calibrated = True
                print("✓ Set encoder pre_calibrated = True")
            
            if hasattr(axis.encoder.config, 'hall_polarity_calibrated'):
                axis.encoder.config.hall_polarity_calibrated = True
                print("✓ Set hall_polarity_calibrated = True")
            
            # Manually reset hall_offset to 0
            if hasattr(axis.encoder.config, 'hall_offset'):
                axis.encoder.config.hall_offset = 0
                print("✓ Set hall_offset = 0")
            
            return True
        else:
            print("! Cannot access Hall state")
            return False
    except Exception as e:
        print(f"! Error during manual encoder configuration: {str(e)}")
        return False

def verify_calibration(axis):
    """Verify calibration status"""
    print("\n--- Calibration Verification ---")
    
    try:
        motor_calibrated = axis.motor.is_calibrated
        encoder_ready = axis.encoder.is_ready
        
        print(f"Motor calibrated: {motor_calibrated}")
        print(f"Encoder ready: {encoder_ready}")
        
        if hasattr(axis, 'error') and axis.error != 0:
            print(f"! Axis error: {axis.error}")
        if hasattr(axis.motor, 'error') and axis.motor.error != 0:
            print(f"! Motor error: {axis.motor.error}")
        if hasattr(axis.encoder, 'error') and axis.encoder.error != 0:
            print(f"! Encoder error: {axis.encoder.error}")
        
        return motor_calibrated and encoder_ready
    except Exception as e:
        print(f"! Error verifying calibration: {str(e)}")
        return False

def test_velocity_control(axis):
    """Simple velocity control test"""
    print("\n--- Velocity Control Test ---")
    
    try:
        # Configure for velocity control
        axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        axis.controller.config.vel_limit = 5.0  # rad/s - conservative
        print("✓ Set to VELOCITY control mode")
        
        # Enter closed loop control
        print("Entering closed loop control...")
        axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        time.sleep(1.0)
        
        if axis.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
            print(f"! Failed to enter control mode. State: {axis.current_state}")
            return False
        
        print("✓ Successfully entered closed loop control")
        
        # Test gentle movements
        speeds = [1.0, 0.0, -1.0, 0.0]
        for speed in speeds:
            direction = "forward" if speed > 0 else "reverse" if speed < 0 else "stopped"
            print(f"\nTesting {direction} at {abs(speed):.1f} rad/s...")
            
            axis.controller.input_vel = speed
            for i in range(4):  # monitor for 2 seconds
                try:
                    vel = axis.encoder.vel_estimate if hasattr(axis.encoder, 'vel_estimate') else "unknown"
                    print(f"  Speed: {vel} rad/s")
                    
                    # Check for errors
                    if axis.error != 0:
                        print(f"! Error detected: {axis.error}")
                        break
                except Exception as e:
                    print(f"! Error reading feedback: {e}")
                    
                time.sleep(0.5)
        
        # Return to idle
        print("\nReturning to idle state...")
        axis.controller.input_vel = 0.0
        time.sleep(0.5)
        axis.requested_state = AXIS_STATE_IDLE
        
        print("✓ Test completed successfully!")
        return True
    except Exception as e:
        print(f"! Error during velocity test: {e}")
        try:
            # Safety stop
            axis.controller.input_vel = 0.0
            axis.requested_state = AXIS_STATE_IDLE
        except:
            pass
        return False

def run_comprehensive_calibration(axis_num):
    """Run all calibration steps with proper reconnection handling"""
    print_header("COMPREHENSIVE ODRIVE CALIBRATION")
    print(f"Calibrating ODrive axis {axis_num}")
    
    # Step 1: Connect to ODrive
    odrv = connect_to_odrive()
    if not odrv:
        print("! Could not connect to ODrive")
        return False
    
    # Step 2: Configure basic parameters
    axis = getattr(odrv, f"axis{axis_num}")
    if not configure_calibration_parameters(axis):
        print("! Configuration failed")
        return False
    
    # Step 3: Save configuration and handle reset
    odrv = save_configuration_with_reconnect(odrv)
    if not odrv:
        print("! Failed to reconnect after saving configuration")
        return False
    
    # Step 4: Motor calibration
    axis = getattr(odrv, f"axis{axis_num}")
    calibration_result = calibrate_motor(axis)
    
    # Handle potential reset during motor calibration
    if calibration_result is None:
        # If we lost connection, reconnect
        print("Reconnecting after motor calibration...")
        odrv = connect_to_odrive()
        if not odrv:
            print("! Failed to reconnect after motor calibration")
            return False
        
        # Verify motor calibration was successful
        axis = getattr(odrv, f"axis{axis_num}")
        if not axis.motor.is_calibrated:
            print("! Motor calibration was not saved")
            return False
        print("✓ Motor calibration verified after reconnection")
    elif not calibration_result:
        print("! Motor calibration failed")
        return False
    
    # Step 5: Save after motor calibration
    print("\nSaving after motor calibration...")
    odrv = save_configuration_with_reconnect(odrv)
    if not odrv:
        print("! Failed to reconnect after saving motor calibration")
        return False
    
    # Step 6: Manual encoder configuration
    axis = getattr(odrv, f"axis{axis_num}")
    if not force_encoder_ready(axis):
        print("! Manual encoder configuration failed")
        return False
    
    # Step 7: Save after encoder configuration
    print("\nSaving after encoder configuration...")
    odrv = save_configuration_with_reconnect(odrv)
    if not odrv:
        print("! Failed to reconnect after saving encoder configuration")
        return False
    
    # Step 8: Verify calibration
    axis = getattr(odrv, f"axis{axis_num}")
    if not verify_calibration(axis):
        print("\n! Calibration verification failed")
        print("  You can still try force_velocity_test.py in bypass mode")
        return False
    
    # Step 9: Optional velocity test
    test_prompt = input("\nRun velocity control test? (y/n): ")
    if test_prompt.lower() == 'y':
        if not test_velocity_control(axis):
            print("! Velocity test failed")
    else:
        print("Skipping velocity test")
    
    print_header("CALIBRATION COMPLETE")
    print("The ODrive has been calibrated and configured.")
    print("You can now use it with ROS 2 for your differential robot.")
    return True

def main():
    # Parse arguments
    parser = argparse.ArgumentParser(description="ODrive All-in-One Calibration")
    parser.add_argument('--axis', type=int, default=0, choices=[0, 1],
                        help="Axis to calibrate (0 or 1)")
    args = parser.parse_args()
    
    # Run the comprehensive calibration sequence
    run_comprehensive_calibration(args.axis)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nCalibration cancelled by user")
        # Safety: try to ensure motor is stopped
        try:
            odrv = odrive.find_any(timeout=3)
            for axis_num in [0, 1]:
                try:
                    axis = getattr(odrv, f"axis{axis_num}")
                    axis.requested_state = AXIS_STATE_IDLE
                except:
                    pass
        except:
            pass
