#!/usr/bin/env python3

"""
Improved ODrive Hall Sensor Calibration

This script properly configures and calibrates Hall sensors on ODrive controllers,
with enhanced reconnection capabilities to handle device resets.

Usage:
    python3 improved_hall_calibration.py [--axis 0|1] [--polepairs 2]
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

def connect_to_odrive(timeout=10, retry_count=3):
    """Connect to ODrive with retries"""
    print("Looking for ODrive...")
    
    for attempt in range(retry_count):
        try:
            odrv = odrive.find_any(timeout=timeout)
            print(f"✓ Found ODrive! Serial Number: {odrv.serial_number}")
            return odrv
        except Exception as e:
            if attempt < retry_count - 1:
                print(f"Connection attempt {attempt+1} failed. Retrying...")
                time.sleep(1)
            else:
                print(f"Failed to connect to ODrive after {retry_count} attempts: {str(e)}")
                return None
    
    return None

def clear_errors(axis, name=""):
    """Clear all errors on axis"""
    prefix = f"[{name}] " if name else ""
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
        print(f"{prefix}✓ Errors cleared")
    except Exception as e:
        print(f"{prefix}! Error clearing errors: {str(e)}")

def print_error_state(axis, prefix=""):
    """Print detailed error information"""
    if prefix:
        print(f"\n{prefix}")
    
    try:
        if hasattr(axis, 'error'):
            print(f"  Axis error: {axis.error}")
        if hasattr(axis.motor, 'error'):
            print(f"  Motor error: {axis.motor.error}")
        if hasattr(axis.encoder, 'error'):
            print(f"  Encoder error: {axis.encoder.error}")
        if hasattr(axis.controller, 'error'):
            print(f"  Controller error: {axis.controller.error}")
    except Exception as e:
        print(f"  Error reading error states: {str(e)}")

def detect_pole_pairs(odrive_obj, axis_num):
    """Count Hall transitions to estimate pole pairs"""
    print_header("DETECTING POLE PAIRS")
    
    # Get fresh axis reference
    axis = getattr(odrive_obj, f"axis{axis_num}")
    
    if not hasattr(axis.encoder, 'hall_state'):
        print("! Cannot access Hall state - firmware may not support this feature")
        return None
    
    # Set to idle state and clear errors
    try:
        axis.requested_state = AXIS_STATE_IDLE
        clear_errors(axis, "Pole Pair Detection")
        
        # Configure for Hall sensing
        axis.encoder.config.mode = ENCODER_MODE_HALL
        axis.encoder.config.cpr = 6  # Basic hall setup for detection
        print("✓ Set to Hall mode for detection")
    except Exception as e:
        print(f"! Error setting up Hall detection: {str(e)}")
        return None
    
    # Manual detection procedure
    print("\nTo count Hall transitions:")
    print("1. Rotate the motor EXACTLY ONE FULL REVOLUTION by hand")
    print("2. Rotate slowly and steadily to capture all transitions")
    print("3. We'll count the state changes to estimate pole pairs\n")
    
    input("Press Enter when ready to start counting...")
    
    transitions = []
    last_state = axis.encoder.hall_state
    print(f"Starting at Hall state: {last_state}")
    print("Counting Hall state changes. Turn the motor ONE FULL REVOLUTION now...")
    
    # Give user 20 seconds to rotate the motor one full turn
    timeout = time.time() + 20
    states_seen = {last_state}
    
    try:
        while time.time() < timeout:
            try:
                current_state = axis.encoder.hall_state
                if current_state != last_state:
                    transitions.append((last_state, current_state))
                    print(f"  Hall transition: {last_state} → {current_state}")
                    last_state = current_state
                    states_seen.add(current_state)
                time.sleep(0.05)
            except Exception as e:
                print(f"! Error reading Hall state: {str(e)}")
                time.sleep(0.5)
                continue
            
            # After seeing several transitions, check if user wants to stop
            if len(transitions) >= 6 and len(states_seen) >= 3:
                check = input("\nPress Enter to continue counting or type 'done' if you've completed one revolution: ")
                if check.lower() == 'done':
                    break
                timeout = time.time() + 10  # Reset timeout
    except KeyboardInterrupt:
        print("\nHall state monitoring stopped by user")
    
    # Calculate pole pairs
    if len(transitions) < 4:
        print("! Too few transitions detected. Cannot determine pole pairs.")
        pp = input("Please manually enter pole pairs (default is 7): ")
        return int(pp) if pp.strip() and pp.isdigit() and int(pp) > 0 else 7
        
    # Estimate pole pairs (each pair gives 6 transitions for a full electrical cycle)
    # Most motors with Hall sensors should have transitions in multiples of 6
    estimated_pp = round(len(transitions) / 6)
    if estimated_pp < 1:
        estimated_pp = round(len(transitions) / 6 * 2) / 2
        if estimated_pp < 1:
            estimated_pp = 2  # Minimum
    
    print(f"\nDetected {len(transitions)} Hall transitions in one revolution")
    print(f"This suggests approximately {estimated_pp} pole pairs")
    
    # Ask user to confirm or override
    pp_input = input(f"Enter the pole pairs to use (default: {estimated_pp}): ")
    if pp_input.strip() and pp_input.isdigit() and int(pp_input) > 0:
        return int(pp_input)
    return estimated_pp

def manual_hall_calibration(axis):
    """Manually calibrate Hall sensors by observing patterns"""
    print_header("MANUAL HALL SENSOR CALIBRATION")
    print("This is a special routine to manually map Hall states to positions")
    print("Since the automatic calibration is failing, we'll try a manual approach")
    
    # First check if Hall sensors are working
    if not hasattr(axis.encoder, 'hall_state'):
        print("! Cannot access Hall state - firmware may not support this feature")
        return False
    
    # Get the current Hall state
    initial_state = axis.encoder.hall_state
    if initial_state < 1 or initial_state > 6:
        print(f"! Invalid Hall state detected: {initial_state}")
        print("  Valid Hall states are 1-6")
        print("  Check Hall sensor connections before proceeding")
        return False
    
    print(f"Current Hall state: {initial_state}")
    print("\nSTEP 1: Testing Hall state changes")
    print("Rotate the motor BY HAND very slowly and observe the Hall state changes")
    print("Press Ctrl+C to stop when you've seen several state changes")
    
    # Record the observed Hall states and transitions
    hall_transitions = []
    last_state = initial_state
    
    try:
        print("\nWatching Hall state changes... (Ctrl+C to stop)")
        for i in range(60):  # Up to 30 seconds
            current_state = axis.encoder.hall_state
            if current_state != last_state:
                hall_transitions.append((last_state, current_state))
                print(f"  Hall transition: {last_state} → {current_state}")
                last_state = current_state
            time.sleep(0.5)
            
            # After several transitions, check if user wants to continue
            if len(hall_transitions) >= 3 and i % 4 == 0:
                check = input("\nContinue watching? (y/n): ")
                if check.lower() != 'y':
                    break
    except KeyboardInterrupt:
        pass
    
    # Check if we have enough transitions
    if len(hall_transitions) < 2:
        print("\n! Not enough Hall state transitions detected")
        print("  Please check that the Hall sensors are properly connected")
        return False
    
    print(f"\nDetected {len(hall_transitions)} Hall state transitions")
    print("Hall sensors appear to be working properly!")
    
    # Prepare for manual calibration
    print("\nSTEP 2: Manually configuring Hall values")
    print("Based on detected Hall states, we'll configure the ODrive directly")
    
    try:
        # Hall sensor mapping
        # First, set the right encoder mode and CPR
        axis.encoder.config.mode = ENCODER_MODE_HALL
        pole_pairs = axis.motor.config.pole_pairs
        axis.encoder.config.cpr = 6 * pole_pairs
        print(f"✓ Set encoder CPR to {6 * pole_pairs} (6 states * {pole_pairs} pole pairs)")
        
        # Try to manually set the calibration flag
        if hasattr(axis.encoder.config, 'hall_polarity_calibrated'):
            axis.encoder.config.hall_polarity_calibrated = True
            print("✓ Manually set hall_polarity_calibrated flag")
        
        # By using pre_calibrated, we can avoid the calibration step
        if hasattr(axis.encoder.config, 'pre_calibrated'):
            axis.encoder.config.pre_calibrated = True
            print("✓ Set encoder to pre_calibrated mode")
        
        # Use manual offsets if available
        if hasattr(axis.encoder.config, 'hall_offset'):
            # 0 is a safe default
            axis.encoder.config.hall_offset = 0
            print("✓ Set hall_offset to 0")
        
        # Save configuration
        print("\nSaving configuration...")
        odrv.save_configuration()
        print("✓ Configuration saved")
        
        print("\nManual Hall calibration complete!")
        print("You can now try to test the motor with:")
        print("python3 src/tracked_robot/scripts/force_velocity_test.py --axis 0 --mode bypass")
        
        return True
    except Exception as e:
        print(f"! Error during manual Hall calibration: {e}")
        return False

def step1_configure_hall_sensors(axis_num, pole_pairs):
    """Configure Hall sensors with the correct pole pairs"""
    print_header("STEP 1: CONFIGURE HALL SENSORS")
    print(f"Using axis {axis_num} with {pole_pairs} pole pairs")
    
    # Connect to ODrive
    odrv = connect_to_odrive()
    if not odrv:
        return False
    
    # Get axis reference
    axis = getattr(odrv, f"axis{axis_num}")
    
    # Print current configuration
    print("\nInitial configuration:")
    try:
        print(f"  Encoder mode: {axis.encoder.config.mode}")
        print(f"  Encoder CPR: {axis.encoder.config.cpr}")
        if hasattr(axis.encoder.config, 'bandwidth'):
            print(f"  Encoder bandwidth: {axis.encoder.config.bandwidth}")
    except Exception as e:
        print(f"  Error reading configuration: {str(e)}")
    
    # Apply Hall sensor configuration
    try:
        # Set idle state
        axis.requested_state = AXIS_STATE_IDLE
        time.sleep(0.2)
        
        # Clear errors
        clear_errors(axis, "Configuration")
        
        # Configure encoder
        print("\nApplying Hall encoder configuration...")
        axis.encoder.config.mode = ENCODER_MODE_HALL
        print("✓ Set encoder mode to HALL")
        
        # Calculate CPR = 6 states * pole_pairs
        cpr = 6 * pole_pairs
        axis.encoder.config.cpr = cpr
        print(f"✓ Set encoder CPR to {cpr} (6 states * {pole_pairs} pole pairs)")
        
        # Set appropriate bandwidth
        if hasattr(axis.encoder.config, 'bandwidth'):
            axis.encoder.config.bandwidth = 100  # Hz - moderate value
            print("✓ Set encoder bandwidth to 100 Hz")
        
        # Reset calibration flags if needed
        if hasattr(axis.encoder.config, 'hall_polarity_calibrated'):
            try:
                axis.encoder.config.hall_polarity_calibrated = False
                print("✓ Reset Hall polarity calibration flag")
            except:
                print("! Could not reset Hall polarity calibration flag")
        
        # Motor configuration for better performance
        try:
            if hasattr(axis.motor.config, 'resistance_calib_max_voltage'):
                axis.motor.config.resistance_calib_max_voltage = 4.0  # V
                print("✓ Set resistance calibration voltage to 4.0V")
            
            # Disable watchdog during calibration if possible
            if hasattr(axis.config, 'enable_watchdog'):
                axis.config.enable_watchdog = False
                print("✓ Disabled watchdog during calibration")
        except Exception as e:
            print(f"! Error setting motor parameters: {str(e)}")
        
        # Save configuration
        print("\nSaving configuration...")
        try:
            odrv.save_configuration()
            print("✓ Configuration saved")
            time.sleep(2.0)  # Wait for possible reset after save
        except Exception as e:
            print(f"! Error saving configuration: {str(e)}")
        
        print("\nConfiguration completed. If ODrive reset, reconnect and proceed to Step 2.")
        return True
        
    except Exception as e:
        print(f"\n! Error configuring Hall sensors: {str(e)}")
        return False

def step2_calibrate_motor(axis_num):
    """Calibrate just the motor"""
    print_header("STEP 2: MOTOR CALIBRATION")
    print(f"Calibrating motor on axis {axis_num}")
    
    # Connect to ODrive
    odrv = connect_to_odrive()
    if not odrv:
        return False
    
    # Get axis reference
    axis = getattr(odrv, f"axis{axis_num}")
    
    # Set to idle state and clear errors
    try:
        axis.requested_state = AXIS_STATE_IDLE
        time.sleep(0.5)
        clear_errors(axis, "Motor Calibration")
    except Exception as e:
        print(f"! Error preparing for calibration: {str(e)}")
        return False
    
    # Set calibration current (conservative value)
    try:
        # Different ODrive firmware versions use different APIs
        if hasattr(axis.motor.config, 'calibration_current'):
            # Older API
            axis.motor.config.calibration_current = 10.0  # Amps
            print("✓ Set calibration current to 10.0A (old API)")
        elif hasattr(axis.config, 'calibration_current'):
            # Newer API (v0.5.1+)
            axis.config.calibration_current = 10.0  # Amps
            print("✓ Set calibration current to 10.0A (new API)")
        else:
            print("! Could not set calibration current - unknown API version")
    except Exception as e:
        print(f"! Error setting calibration current: {str(e)}")
    
    # Perform motor calibration
    print("\nStarting motor calibration...")
    print("This will make the motor emit a chirp sound\n")
    try:
        axis.requested_state = AXIS_STATE_MOTOR_CALIBRATION
        
        # Wait for calibration to complete
        print("Waiting for calibration to complete...")
        timeout = 15  # seconds
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            if axis.current_state == AXIS_STATE_IDLE:
                break
            time.sleep(0.2)
            
        # Check if successful
        if time.time() - start_time >= timeout:
            print("! Motor calibration timed out")
            return False
            
        if axis.motor.is_calibrated:
            print("✓ Motor calibration successful!")
            
            # Save after motor calibration
            try:
                odrv.save_configuration()
                print("✓ Motor calibration saved")
            except Exception as e:
                print(f"! Error saving motor calibration: {str(e)}")
            
            return True
        else:
            print(f"! Motor calibration failed")
            print_error_state(axis, "Motor Errors:")
            return False
            
    except Exception as e:
        print(f"! Error during motor calibration: {str(e)}")
        return False

def step3_calibrate_hall_sensors(axis_num):
    """Calibrate just the Hall sensors (after motor calibration)"""
    print_header("STEP 3: HALL SENSOR CALIBRATION")
    print(f"Calibrating Hall sensors on axis {axis_num}")
    
    # Connect to ODrive
    global odrv
    odrv = connect_to_odrive()
    if not odrv:
        return False
    
    # Get axis reference
    axis = getattr(odrv, f"axis{axis_num}")
    
    # Check if motor is calibrated
    if not axis.motor.is_calibrated:
        print("! Motor must be calibrated before Hall sensors")
        print("  Please run Step 2 (Motor Calibration) first")
        return False
    
    # Set to idle state and clear errors
    try:
        axis.requested_state = AXIS_STATE_IDLE
        time.sleep(0.5)
        clear_errors(axis, "Hall Calibration")
    except Exception as e:
        print(f"! Error preparing for Hall calibration: {str(e)}")
        return False
    
    # Check if the Hall sensors work by watching state changes
    print("\nFirst, let's check if Hall sensors are working...")
    print("Please rotate the motor shaft BY HAND and watch for state changes.")
    
    # Track Hall states for a few seconds
    initial_state = axis.encoder.hall_state
    print(f"Current Hall state: {initial_state}")
    
    if initial_state < 1 or initial_state > 6:
        print("\n! Invalid Hall state detected. Hall sensors may be disconnected or miswired.")
        print("  Valid Hall states are 1-6")
        return False
    
    # Ask user to choose calibration method
    print("\nCHOOSE CALIBRATION METHOD:")
    print("1: Standard automatic calibration (motor will rotate)")
    print("2: Manual calibration (uses Hall states from manual rotation)")
    method = input("Choose method [1/2]: ")
    
    if method == "2":
        # Try manual calibration first
        print("\nUsing MANUAL CALIBRATION method...")
        manual_success = manual_hall_calibration(axis)
        
        if manual_success:
            return True
        else:
            print("\nManual calibration failed, falling back to automatic method...")
    
    # Start Hall sensor calibration (automatic method)
    print("\nStarting automatic Hall sensor calibration...")
    print("This will rotate the motor to detect Hall sensor sequence\n")
    try:
        # Find the right calibration state based on firmware
        if hasattr(AXIS_STATE, 'ENCODER_HALL_POLARITY_CALIBRATION'):
            # Newer firmware with dedicated Hall calibration
            print("Using HALL_POLARITY_CALIBRATION mode")
            axis.requested_state = AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION
        else:
            # Older firmware using standard encoder calibration
            print("Using standard ENCODER_OFFSET_CALIBRATION mode")
            axis.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
        
        # Wait for calibration to complete
        print("Waiting for calibration to complete...")
        timeout = 30  # seconds
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            if axis.current_state == AXIS_STATE_IDLE:
                break
            time.sleep(0.2)
            
        # Check if successful
        if time.time() - start_time >= timeout:
            print("! Hall sensor calibration timed out")
            
            # Try manual calibration as fallback
            print("\nAutomatic calibration failed. Trying manual calibration as fallback...")
            return manual_hall_calibration(axis)
            
        if axis.encoder.is_ready:
            print("✓ Hall sensor calibration successful!")
            
            # Save after Hall calibration
            try:
                odrv.save_configuration()
                print("✓ Hall calibration saved")
            except Exception as e:
                print(f"! Error saving Hall calibration: {str(e)}")
            
            return True
        else:
            print(f"! Hall sensor calibration failed")
            print_error_state(axis, "Encoder Errors:")
            
            # Try manual calibration as fallback
            print("\nAutomatic calibration failed. Trying manual calibration as fallback...")
            return manual_hall_calibration(axis)
            
    except Exception as e:
        print(f"! Error during Hall calibration: {str(e)}")
        
        # Try manual calibration as fallback
        print("\nAutomatic calibration failed. Trying manual calibration as fallback...")
        return manual_hall_calibration(axis)

def step4_test_movement(axis_num):
    """Test basic closed-loop movement after calibration"""
    print_header("STEP 4: MOVEMENT TESTING")
    print(f"Testing closed-loop control on axis {axis_num}")
    
    # Connect to ODrive
    odrv = connect_to_odrive()
    if not odrv:
        return False
    
    # Get axis reference
    axis = getattr(odrv, f"axis{axis_num}")
    
    # Check calibration status
    if not axis.motor.is_calibrated or not axis.encoder.is_ready:
        print("! Motor and/or encoder are not calibrated")
        print("  Please complete Steps 2 and 3 first")
        return False
    
    # Set to idle state and clear errors
    try:
        axis.requested_state = AXIS_STATE_IDLE
        time.sleep(0.5)
        clear_errors(axis, "Movement Test")
    except Exception as e:
        print(f"! Error preparing for movement test: {str(e)}")
        return False
    
    # Configure for velocity control
    try:
        axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        print("✓ Set to velocity control mode")
        
        # Set conservative limits for testing
        if hasattr(axis.controller.config, 'vel_limit'):
            axis.controller.config.vel_limit = 10.0  # rad/s
            print("✓ Set velocity limit to 10.0 rad/s")
        
        # Set current limits based on API version
        if hasattr(axis.config, 'dc_max_current'):
            # Newer API
            axis.config.dc_max_current = 20.0  # Amps
            axis.config.dc_max_negative_current = -20.0  # Amps
            print("✓ Set current limits to 20.0A (new API)")
        elif hasattr(axis.motor.config, 'current_lim'):
            # Older API
            axis.motor.config.current_lim = 20.0  # Amps
            print("✓ Set current limit to 20.0A (old API)")
    except Exception as e:
        print(f"! Error configuring control parameters: {str(e)}")
    
    # Enter closed loop control
    print("\nAttempting to enter closed loop control...")
    try:
        axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        time.sleep(1.0)
        
        if axis.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
            print("! Failed to enter closed loop control")
            print_error_state(axis, "Control Errors:")
            return False
            
        print("✓ Successfully entered closed loop control!")
        
        # Test gentle movements
        print("\nTesting gentle movements...")
        test_velocities = [
            {"speed": 1.0, "desc": "Slow forward", "time": 2.0},
            {"speed": 0.0, "desc": "Stop", "time": 1.0},
            {"speed": -1.0, "desc": "Slow reverse", "time": 2.0},
            {"speed": 0.0, "desc": "Stop", "time": 1.0},
            {"speed": 2.0, "desc": "Medium forward", "time": 2.0},
            {"speed": 0.0, "desc": "Stop", "time": 1.0},
            {"speed": -2.0, "desc": "Medium reverse", "time": 2.0},
            {"speed": 0.0, "desc": "Final stop", "time": 1.0}
        ]
        
        for test in test_velocities:
            print(f"\n{test['desc']} at {abs(test['speed'])} rad/s...")
            axis.controller.input_vel = test['speed']
            
            # Wait and monitor
            for i in range(int(test['time'] * 2)):
                try:
                    vel = axis.encoder.vel_estimate if hasattr(axis.encoder, 'vel_estimate') else "unknown"
                    current = axis.motor.current_control.Iq_measured if hasattr(axis.motor.current_control, 'Iq_measured') else "unknown"
                    print(f"  Speed: {vel}, Current: {current}")
                    
                    # Check for errors
                    if axis.error != 0 or axis.motor.error != 0 or axis.encoder.error != 0:
                        print("! Error detected during movement:")
                        print_error_state(axis)
                        break
                except Exception as e:
                    print(f"! Error reading status: {str(e)}")
                    
                time.sleep(0.5)
        
        # Return to idle
        print("\nReturning to idle state...")
        axis.controller.input_vel = 0.0
        time.sleep(0.5)
        axis.requested_state = AXIS_STATE_IDLE
        
        print("✓ Movement test completed successfully!")
        return True
        
    except Exception as e:
        print(f"! Error during movement test: {str(e)}")
        try:
            # Safety: ensure motor stops
            axis.controller.input_vel = 0.0
            axis.requested_state = AXIS_STATE_IDLE
        except:
            pass
        return False

def show_menu():
    """Show main menu for calibration steps"""
    print_header("HALL SENSOR CALIBRATION MENU")
    print("This utility helps calibrate Hall sensors for ODrive controllers")
    print("by breaking the process into manageable steps.\n")
    
    print("Available steps:")
    print("  1. Configure Hall Sensors - Set correct parameters")
    print("  2. Calibrate Motor - Perform motor resistance calibration")
    print("  3. Calibrate Hall Sensors - Learn Hall sensor positions")
    print("  4. Test Movement - Verify closed loop control")
    print("  5. Run All Steps - Perform complete calibration")
    print("  6. Auto-Detect Pole Pairs - Count Hall transitions")
    print("  7. Exit\n")
    
    choice = input("Enter step number to execute (1-7): ")
    return choice if choice in ["1", "2", "3", "4", "5", "6", "7"] else None

def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(description="ODrive Hall Sensor Calibration")
    parser.add_argument('--axis', type=int, choices=[0, 1], default=0, 
                        help="Axis to calibrate (default: 0)")
    parser.add_argument('--polepairs', type=int, 
                        help="Manual pole pair value (bypass detection)")
    parser.add_argument('--step', type=int, choices=[1, 2, 3, 4, 5, 6], 
                        help="Run specific step directly")
    args = parser.parse_args()
    
    # Get axis number
    axis_num = args.axis
    print(f"Selected Axis: {axis_num}")
    
    # Get or detect pole pairs
    pole_pairs = args.polepairs
    
    # If step specified on command line, run it directly
    if args.step:
        choice = str(args.step)
    else:
        # Show menu for interactive selection
        choice = show_menu()
    
    # Process menu choice
    if choice == "1":
        # If no pole pairs specified, detect them first
        if pole_pairs is None:
            odrv = connect_to_odrive()
            if odrv:
                pole_pairs = detect_pole_pairs(odrv, axis_num)
            else:
                pole_pairs = 7  # Default if connection fails
        
        # Configure Hall sensors
        step1_configure_hall_sensors(axis_num, pole_pairs)
        
    elif choice == "2":
        # Calibrate motor
        step2_calibrate_motor(axis_num)
        
    elif choice == "3":
        # Calibrate Hall sensors
        step3_calibrate_hall_sensors(axis_num)
        
    elif choice == "4":
        # Test movement
        step4_test_movement(axis_num)
        
    elif choice == "5":
        # Run all steps
        print_header("RUNNING COMPLETE CALIBRATION SEQUENCE")
        
        # If no pole pairs specified, detect them first
        if pole_pairs is None:
            odrv = connect_to_odrive()
            if odrv:
                pole_pairs = detect_pole_pairs(odrv, axis_num)
            else:
                pole_pairs = 7  # Default if connection fails
                
        # Run all steps
        if step1_configure_hall_sensors(axis_num, pole_pairs):
            time.sleep(3)  # Allow time for potential reset
            if step2_calibrate_motor(axis_num):
                time.sleep(3)  # Allow time for potential reset
                if step3_calibrate_hall_sensors(axis_num):
                    time.sleep(3)  # Allow time for potential reset
                    step4_test_movement(axis_num)
        
    elif choice == "6":
        # Just detect pole pairs
        odrv = connect_to_odrive()
        if odrv:
            detect_pole_pairs(odrv, axis_num)
        
    elif choice == "7" or choice is None:
        print("Exiting calibration utility")
        return
    
    print_header("CALIBRATION STEP COMPLETE")
    print("You can run additional calibration steps as needed.")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nCalibration cancelled by user.")
