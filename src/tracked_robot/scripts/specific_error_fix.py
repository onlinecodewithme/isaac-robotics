#!/usr/bin/env python3

"""
ODrive Error 0x801 (AXIS_ERROR_MOTOR_FAILED) Targeted Fix

This script attempts to work around the specific 0x801 error
when Hall sensors are confirmed working but motor calibration fails.

IMPORTANT: Run with higher current settings and hardware checks
performed according to HARDWARE_TROUBLESHOOTING.md

Usage:
    python3 specific_error_fix.py
"""

import sys
import time
import argparse
import logging
import signal

try:
    import odrive
    from odrive.enums import *
except ImportError:
    print("Error: ODrive Python library not found.")
    print("Please install it using: pip install odrive")
    sys.exit(1)

# Configure logging
logging.basicConfig(level=logging.INFO, 
                    format='%(asctime)s [%(levelname)s] %(message)s',
                    datefmt='%H:%M:%S')
logger = logging.getLogger("odrive_error_fix")

def connect_to_odrive(timeout=10):
    """Connect to ODrive with retries"""
    logger.info("Connecting to ODrive...")
    
    try:
        odrv = odrive.find_any(timeout=timeout)
        logger.info(f"Connected to ODrive {odrv.serial_number}")
        return odrv
    except Exception as e:
        logger.error(f"Failed to connect to ODrive: {str(e)}")
        return None

def clear_errors(axis):
    """Clear all errors on the axis"""
    try:
        logger.info("Clearing all errors...")
        
        if hasattr(axis, 'error'):
            axis.error = 0
        if hasattr(axis.motor, 'error'):
            axis.motor.error = 0
        if hasattr(axis.encoder, 'error'):
            axis.encoder.error = 0
        if hasattr(axis.controller, 'error'):
            axis.controller.error = 0
            
        logger.info("Errors cleared")
        return True
    except Exception as e:
        logger.error(f"Error clearing errors: {str(e)}")
        return False

def reset_to_factory_defaults(odrv):
    """Reset ODrive to factory defaults"""
    logger.info("RESETTING ODRIVE TO FACTORY DEFAULTS")
    logger.info("This will erase all configuration!")
    
    choice = input("Continue with factory reset? (y/n): ")
    if choice.lower() != 'y':
        logger.info("Factory reset cancelled")
        return False
    
    try:
        logger.info("Erasing configuration...")
        odrv.erase_configuration()
        logger.info("Configuration erased. Device will reboot.")
        logger.info("Waiting for reboot...")
        time.sleep(5.0)
        
        # Reconnect
        new_odrv = connect_to_odrive()
        if new_odrv is None:
            logger.error("Failed to reconnect after factory reset")
            return False
            
        logger.info("Successfully reset and reconnected")
        return True, new_odrv
    except Exception as e:
        logger.error(f"Error during factory reset: {str(e)}")
        return False, None

def check_firmware_version(odrv):
    """Check firmware version"""
    try:
        fw_major = odrv.fw_version_major
        fw_minor = odrv.fw_version_minor
        fw_revision = odrv.fw_version_revision
        
        logger.info(f"Firmware version: {fw_major}.{fw_minor}.{fw_revision}")
        
        if fw_major < 0 or (fw_major == 0 and fw_minor < 5):
            logger.warning("Firmware may be outdated. Consider updating!")
        
        return True
    except Exception as e:
        logger.error(f"Error checking firmware: {str(e)}")
        return False

def check_hall_sensors(axis):
    """Check hall sensor functionality"""
    logger.info("Testing Hall sensors...")
    logger.info("Please rotate the motor shaft by hand...")
    
    hall_states = set()
    tries = 0
    
    try:
        logger.info("Reading hall states (rotate motor slowly)...")
        logger.info("Press Ctrl+C when finished.")
        
        # Collect hall states until user presses Ctrl+C
        try:
            while tries < 100:  # Safety limit
                hall_state = axis.encoder.hall_state
                hall_states.add(hall_state)
                
                # Status update
                if tries % 5 == 0:
                    logger.info(f"Current hall state: {hall_state}")
                    logger.info(f"Distinct states seen: {sorted(list(hall_states))}")
                
                tries += 1
                time.sleep(0.5)
        except KeyboardInterrupt:
            logger.info("\nHall sensor test stopped by user")
        
        # Analyze results
        logger.info(f"Hall states detected: {sorted(list(hall_states))}")
        
        if len(hall_states) >= 3:
            logger.info("✓ Hall sensors appear to be working correctly!")
            return True
        elif len(hall_states) > 1:
            logger.warning("! Limited hall states detected. Not all hall sensors may be working.")
            return True
        else:
            logger.error("! No change in hall state. Sensors may be disconnected or faulty.")
            return False
            
    except Exception as e:
        logger.error(f"Error testing hall sensors: {str(e)}")
        return False

def apply_aggressive_fix(odrv, axis, axis_num, high_current=15.0, attempts=3):
    """Apply aggressive fix for Error 0x801"""
    logger.info("Applying aggressive fix for Error 0x801...")
    
    for attempt in range(attempts):
        logger.info(f"Attempt {attempt+1}/{attempts}...")
        
        try:
            # 1. Set to idle
            axis.requested_state = AXIS_STATE_IDLE
            time.sleep(0.5)
            
            # 2. Clear all errors
            clear_errors(axis)
            
            # 3. Set motor type to GIMBAL and then back to HIGH_CURRENT
            # This can sometimes reset internal ODrive state
            logger.info("Cycling motor type...")
            axis.motor.config.motor_type = MOTOR_TYPE_GIMBAL
            time.sleep(0.2)
            axis.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
            logger.info("Set motor type to HIGH_CURRENT")
            
            # 4. Increase current limits substantially
            logger.info(f"Setting high current limits: {high_current}A")
            axis.motor.config.current_lim = high_current
            axis.motor.config.calibration_current = high_current * 0.7
            
            # 5. Increase resistance calibration max voltage
            axis.motor.config.resistance_calib_max_voltage = 6.0
            logger.info("Increased resistance calibration voltage")
            
            # 6. Bypass encoder concerns
            if hasattr(axis.encoder.config, 'use_index'):
                axis.encoder.config.use_index = False
            if hasattr(axis.encoder.config, 'calib_scan_omega'):
                axis.encoder.config.calib_scan_omega = 20.0  # rad/s
            if hasattr(axis.encoder.config, 'bandwidth'):
                axis.encoder.config.bandwidth = 100.0  # Hz
                
            # 7. Save configuration
            logger.info("Saving aggressive configuration...")
            try:
                odrv.save_configuration()
                time.sleep(3.0)
                
                # If we get here, the device didn't reset (which is unusual)
                logger.info("Configuration saved without device reset")
            except Exception as e:
                if "object disappeared" in str(e).lower():
                    logger.info("Device reset during save (expected)")
                    time.sleep(5.0)
                    # Reconnect
                    new_odrv = connect_to_odrive()
                    if new_odrv is None:
                        logger.error("Failed to reconnect after save")
                        continue
                    new_axis = getattr(new_odrv, f"axis{axis_num}")
                    # Update our references
                    odrv = new_odrv
                    axis = new_axis
                else:
                    logger.error(f"Error during save: {str(e)}")
                    continue
            
            # 8. Try motor calibration with high current
            logger.info("Attempting motor calibration with high current...")
            axis.requested_state = AXIS_STATE_MOTOR_CALIBRATION
            
            # 9. Wait for calibration to complete
            start_time = time.time()
            while time.time() - start_time < 15.0:
                try:
                    if axis.current_state == AXIS_STATE_IDLE:
                        break
                    time.sleep(0.5)
                    print(".", end="", flush=True)
                except:
                    print("x", end="", flush=True)
            
            print("")  # New line after progress indicators
            
            # 10. Check if calibration succeeded
            try:
                motor_calibrated = axis.motor.is_calibrated
                if motor_calibrated:
                    logger.info("✓ MOTOR CALIBRATION SUCCESSFUL!")
                    
                    # Try to save the calibration
                    try:
                        odrv.save_configuration()
                        logger.info("Configuration saved!")
                    except Exception as e:
                        logger.warning(f"Error saving after successful calibration: {str(e)}")
                        logger.warning("But calibration succeeded, so we can proceed")
                    
                    return True
                else:
                    logger.warning(f"! Calibration failed on attempt {attempt+1}")
                    
                    # Check error code
                    if hasattr(axis, 'error') and axis.error != 0:
                        err = axis.error
                        logger.error(f"Axis error after calibration: 0x{err:X}")
                    if hasattr(axis.motor, 'error') and axis.motor.error != 0:
                        err = axis.motor.error
                        logger.error(f"Motor error after calibration: 0x{err:X}")
            except Exception as e:
                logger.error(f"Error checking calibration result: {str(e)}")
                # Assume device reset, reconnect
                odrv = connect_to_odrive()
                if odrv is None:
                    logger.error("Failed to reconnect after calibration")
                    continue
                axis = getattr(odrv, f"axis{axis_num}")
                
        except Exception as e:
            logger.error(f"Error during aggressive fix: {str(e)}")
    
    logger.warning("All aggressive fix attempts failed")
    return False
    
def attempt_direct_phase_control(axis):
    """Attempt direct phase control bypassing all safety checks"""
    logger.info("=" * 50)
    logger.info("ATTEMPTING DIRECT PHASE CONTROL")
    logger.info("This mode bypasses all safety checks!")
    logger.info("=" * 50)
    
    print("\nIMPORTANT: This mode ignores sensor feedback and directly commands motor phases.")
    print("          Be prepared to disconnect power if the motor vibrates excessively.")
    choice = input("Continue with direct phase control? (y/n): ")
    if choice.lower() != 'y':
        logger.info("Direct phase control cancelled")
        return False
    
    try:
        # 1. Set to idle
        axis.requested_state = AXIS_STATE_IDLE
        time.sleep(0.5)
        
        # 2. Clear errors
        clear_errors(axis)
        
        # 3. Set motor type to GIMBAL
        axis.motor.config.motor_type = MOTOR_TYPE_GIMBAL
        logger.info("Set motor type to GIMBAL")
        
        # 4. Limited current for safety
        axis.motor.config.current_lim = 5.0
        logger.info("Set current limit to 5.0A for safety")
        
        # 5. Enter SENSORLESS velocity control
        if hasattr(axis.controller.config, 'control_mode'):
            axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
            logger.info("Set control mode to VELOCITY_CONTROL")
        
        # 6. Enter sensorless ramp mode if available
        has_sensorless = False
        if hasattr(axis.controller.config, 'input_mode'):
            if hasattr(INPUT_MODE_SENSORLESS_RAMP, 'value'):
                axis.controller.config.input_mode = INPUT_MODE_SENSORLESS_RAMP.value
                has_sensorless = True
                logger.info("Set input mode to SENSORLESS_RAMP")
            elif 'INPUT_MODE_SENSORLESS_RAMP' in dir(odrive.enums):
                axis.controller.config.input_mode = odrive.enums.INPUT_MODE_SENSORLESS_RAMP
                has_sensorless = True
                logger.info("Set input mode to SENSORLESS_RAMP")
        
        if not has_sensorless:
            logger.warning("Sensorless ramp mode not available, using passthrough")
            axis.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
        
        # 7. Configure sensorless ramp parameters if available
        try:
            if hasattr(axis.controller.config, 'vel_ramp_rate'):
                axis.controller.config.vel_ramp_rate = 1.0  # rad/s²
                logger.info("Set velocity ramp rate to 1.0 rad/s²")
            if hasattr(axis.sensorless_estimator.config, 'pm_flux_linkage'):
                axis.sensorless_estimator.config.pm_flux_linkage = 5.51e-03
                logger.info("Set PM flux linkage to 5.51e-03")
        except Exception as e:
            logger.warning(f"Error setting sensorless parameters: {str(e)}")
        
        # 8. Attempt to enter closed loop control
        logger.info("Entering closed loop control...")
        axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        time.sleep(1.0)
        
        # 9. Check if we succeeded
        if axis.current_state == AXIS_STATE_CLOSED_LOOP_CONTROL:
            logger.info("✓ Successfully entered SENSORLESS control mode!")
            
            # 10. Try to move the motor
            logger.info("Applying gentle velocity command...")
            
            if has_sensorless:
                logger.info("Starting sensorless control with increasing speed...")
                for speed in [2.0, 4.0, 6.0, 0.0, -2.0, -4.0, -6.0, 0.0]:
                    logger.info(f"Setting velocity to {speed} rad/s")
                    axis.controller.input_vel = float(speed)
                    time.sleep(2.0)
            else:
                # Direct velocity control
                logger.info("Setting velocity to 2.0 rad/s")
                axis.controller.input_vel = 2.0
                time.sleep(2.0)
                logger.info("Setting velocity to 0.0 rad/s")
                axis.controller.input_vel = 0.0
            
            logger.info("Direct phase control test completed")
            return True
        else:
            logger.error(f"Failed to enter closed loop control. State: {axis.current_state}")
            
            if hasattr(axis, 'error') and axis.error != 0:
                err = axis.error
                logger.error(f"Axis error: 0x{err:X}")
            
            return False
    except Exception as e:
        logger.error(f"Error during direct phase control: {str(e)}")
        return False
    finally:
        # Safety: try to stop motor and return to idle
        try:
            if hasattr(axis.controller, 'input_vel'):
                axis.controller.input_vel = 0.0
            if hasattr(axis, 'requested_state'):
                axis.requested_state = AXIS_STATE_IDLE
            logger.info("Motor stopped and returned to idle state")
        except Exception as e:
            logger.error(f"Error stopping motor: {str(e)}")

if __name__ == "__main__":
    # Parse arguments
    parser = argparse.ArgumentParser(description="ODrive Error 0x801 Fix")
    parser.add_argument('--axis', type=int, default=0, choices=[0, 1],
                      help="Motor axis (0 or 1)")
    parser.add_argument('--current', type=float, default=15.0,
                      help="Maximum calibration current in Amps")
    args = parser.parse_args()
    
    axis_num = args.axis
    max_current = args.current
    
    # Welcome message
    print(f"=" * 80)
    print(f"  ODRIVE ERROR 0x801 (AXIS_ERROR_MOTOR_FAILED) FIX UTILITY")
    print(f"=" * 80)
    print(f"This utility targets the specific 0x801 error when hall sensors are working")
    print(f"but motor calibration fails. Ensure power and wiring are properly checked.")
    print(f"Working on axis {axis_num} with max current {max_current}A\n")
    
    # Connect to ODrive
    odrv = connect_to_odrive()
    if odrv is None:
        sys.exit(1)
    
    # Check firmware version
    check_firmware_version(odrv)
    
    # Get axis reference
    axis = getattr(odrv, f"axis{axis_num}")
    
    # Display current state
    print(f"\n=== CURRENT STATE ===")
    try:
        motor_type = axis.motor.config.motor_type
        motor_type_str = "HIGH_CURRENT" if motor_type == 0 else "GIMBAL" if motor_type == 2 else str(motor_type)
        print(f"Motor Type: {motor_type_str}")
        print(f"Current Limit: {axis.motor.config.current_lim}A")
        print(f"Calibration Current: {axis.motor.config.calibration_current}A")
        print(f"Motor State: {axis.current_state}")
        print(f"Motor Calibrated: {axis.motor.is_calibrated}")
        print(f"Encoder Ready: {axis.encoder.is_ready}")
        
        if hasattr(axis, 'error') and axis.error != 0:
            err = axis.error
            print(f"Axis Error: 0x{err:X}")
        if hasattr(axis.motor, 'error') and axis.motor.error != 0:
            err = axis.motor.error
            print(f"Motor Error: 0x{err:X}")
    except Exception as e:
        print(f"Error reading state: {str(e)}")
    
    # Menu
    while True:
        print("\n" + "=" * 50)
        print("ERROR 0x801 FIX MENU")
        print("=" * 50)
        print("1. Check Hall sensors")
        print("2. Apply aggressive fix (high current, etc.)")
        print("3. Reset to factory defaults")
        print("4. Attempt direct phase control (last resort)")
        print("5. Exit")
        
        choice = input("\nEnter choice (1-5): ")
        
        if choice == '1':
            check_hall_sensors(axis)
        elif choice == '2':
            apply_aggressive_fix(odrv, axis, axis_num, high_current=max_current)
            # Reconnect to get fresh state
            odrv = connect_to_odrive()
            if odrv is not None:
                axis = getattr(odrv, f"axis{axis_num}")
        elif choice == '3':
            success, new_odrv = reset_to_factory_defaults(odrv)
            if success and new_odrv is not None:
                odrv = new_odrv
                axis = getattr(odrv, f"axis{axis_num}")
        elif choice == '4':
            attempt_direct_phase_control(axis)
            # Reconnect to get fresh state
            odrv = connect_to_odrive()
            if odrv is not None:
                axis = getattr(odrv, f"axis{axis_num}")
        elif choice == '5':
            print("Exiting...")
            sys.exit(0)
        else:
            print("Invalid choice")
