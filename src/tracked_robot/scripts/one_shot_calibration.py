#!/usr/bin/env python3

"""
ODrive One-Shot Calibration

This is an ultra-minimal calibration script designed to avoid the
ODrive "EmptyInterface" errors during calibration. It uses the most
basic approach possible, reconnecting between every step.

Usage:
    python3 one_shot_calibration.py --axis 0|1
"""

import sys
import time
import argparse
import subprocess
try:
    import odrive
    from odrive.enums import *
except ImportError:
    print("Error: ODrive Python library not found.")
    print("Please install it using: pip install odrive")
    sys.exit(1)

def connect_odrive():
    """Connect to ODrive, with retries"""
    print("Connecting to ODrive...")
    max_attempts = 5
    
    for attempt in range(max_attempts):
        try:
            odrv = odrive.find_any(timeout=10)
            print(f"Connected to ODrive {odrv.serial_number}")
            return odrv
        except Exception as e:
            if attempt < max_attempts - 1:
                print(f"Connection attempt {attempt+1} failed, retrying...")
                time.sleep(2)
            else:
                print(f"Failed to connect after {max_attempts} attempts")
                return None

def run_command(command):
    """Run a Python command in a new process to avoid interface issues"""
    print(f"Running: {command}")
    try:
        result = subprocess.run(['python3', '-c', command], 
                                capture_output=True, text=True, timeout=15)
        print(result.stdout)
        if result.stderr:
            print("ERRORS:", result.stderr)
        return result.returncode == 0
    except subprocess.TimeoutExpired:
        print("Command timed out")
        return False

def set_config_step(axis_num, config_name, value, type_convert=None):
    """Set a configuration value in a separate process"""
    # Convert value to appropriate string representation
    if type_convert == "float":
        value_str = f"{float(value)}"
    elif type_convert == "int":
        value_str = f"{int(value)}"
    elif type_convert == "bool":
        value_str = "True" if value else "False"
    else:
        value_str = str(value)
    
    # Create command to set configuration
    command = f"""
import odrive
import time
from odrive.enums import *
try:
    odrv = odrive.find_any(timeout=10)
    axis = getattr(odrv, f"axis{axis_num}")
    {config_name} = {value_str}
    print(f"Successfully set {config_name} = {value_str}")
    time.sleep(0.5)  # Brief pause to let it register
    odrv.save_configuration()
    print("Saved configuration")
except Exception as e:
    print(f"Error: {{e}}")
    exit(1)
"""
    return run_command(command)

def perform_calibration_step(axis_num, state):
    """Perform a calibration step in a separate process"""
    command = f"""
import odrive
import time
from odrive.enums import *
try:
    odrv = odrive.find_any(timeout=10)
    print(f"Connected to ODrive {{odrv.serial_number}}")
    axis = getattr(odrv, f"axis{axis_num}")
    
    # Clear errors
    if hasattr(axis, 'error'):
        axis.error = 0
    if hasattr(axis.motor, 'error'):
        axis.motor.error = 0
    if hasattr(axis.encoder, 'error'):
        axis.encoder.error = 0
    if hasattr(axis.controller, 'error'):
        axis.controller.error = 0
    
    # Set to IDLE first
    axis.requested_state = AXIS_STATE_IDLE
    time.sleep(1.0)
    
    # Request the calibration state
    print(f"Starting calibration state: {{state}}")
    axis.requested_state = {state}
    
    # Wait for it to finish
    max_wait = 20  # seconds
    start_time = time.time()
    dots = 0
    
    while time.time() - start_time < max_wait:
        try:
            if axis.current_state == AXIS_STATE_IDLE:
                break
            dots = (dots % 3) + 1
            print("." * dots + " " * (3 - dots), end="\\r")
            time.sleep(0.5)
        except:
            # If we lose connection, just wait
            time.sleep(0.5)
    
    print("")  # Newline after dots
    
    # Check result
    print(f"Calibration completed in {{time.time() - start_time:.1f}} seconds")
    if axis.current_state == AXIS_STATE_IDLE:
        if "{state}" == "AXIS_STATE_MOTOR_CALIBRATION" and axis.motor.is_calibrated:
            print("✓ Motor successfully calibrated!")
            exit(0)
        elif "{state}" == "AXIS_STATE_ENCODER_OFFSET_CALIBRATION" and axis.encoder.is_ready:
            print("✓ Encoder successfully calibrated!")
            exit(0)
        elif "{state}" == "AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION" and axis.encoder.is_ready:
            print("✓ Hall sensors successfully calibrated!")
            exit(0)
        else:
            print("Calibration may have failed")
            exit(1)
    else:
        print(f"Axis in unexpected state: {{axis.current_state}}")
        exit(1)
except Exception as e:
    print(f"Error: {{e}}")
    exit(1)
"""
    return run_command(command)

def check_status(axis_num):
    """Check current status in a separate process"""
    command = f"""
import odrive
import time
from odrive.enums import *
try:
    odrv = odrive.find_any(timeout=10)
    print(f"Connected to ODrive {{odrv.serial_number}}")
    axis = getattr(odrv, f"axis{axis_num}")
    
    print(f"AXIS {axis_num} STATUS:")
    print(f"  Motor calibrated: {{axis.motor.is_calibrated}}")
    print(f"  Encoder ready: {{axis.encoder.is_ready}}")
    
    print("\\nERROR STATES:")
    if hasattr(axis, 'error'):
        print(f"  Axis error: {{axis.error}}")
    if hasattr(axis.motor, 'error'):
        print(f"  Motor error: {{axis.motor.error}}")
    if hasattr(axis.encoder, 'error'):
        print(f"  Encoder error: {{axis.encoder.error}}")
    if hasattr(axis.controller, 'error'):
        print(f"  Controller error: {{axis.controller.error}}")
        
    if axis.motor.is_calibrated and axis.encoder.is_ready:
        print("\\n✓ CALIBRATION COMPLETE: Both motor and encoder are calibrated!")
    else:
        print("\\n! CALIBRATION INCOMPLETE: Further steps needed")
except Exception as e:
    print(f"Error: {{e}}")
    exit(1)
"""
    return run_command(command)

def one_shot_calibration(axis_num):
    """Perform calibration with the most minimal approach possible"""
    print("\n=== STEP 1: Configure Motor ===")
    print("Setting up minimal configuration for calibration...")
    
    # Configure motor type
    set_config_step(axis_num, f"axis.motor.config.motor_type", "MOTOR_TYPE_HIGH_CURRENT")
    
    # Set pole pairs (7 is common for many larger motors)
    set_config_step(axis_num, f"axis.motor.config.pole_pairs", 7, "int")
    
    # Set conservative current limits (different APIs based on firmware)
    print("Setting current limits...")
    # Try the old API first
    set_config_step(axis_num, f"axis.motor.config.current_lim", 5.0, "float")
    set_config_step(axis_num, f"axis.motor.config.calibration_current", 3.0, "float")
    
    # Try new API too (one of them will work)
    set_config_step(axis_num, f"axis.config.dc_max_current", 5.0, "float")
    set_config_step(axis_num, f"axis.config.dc_max_negative_current", -5.0, "float")
    set_config_step(axis_num, f"axis.config.calibration_current", 3.0, "float")
    
    # Set minimal calibration voltage
    set_config_step(axis_num, f"axis.motor.config.resistance_calib_max_voltage", 2.0, "float")
    
    # Configure encoder
    print("Configuring encoder...")
    set_config_step(axis_num, f"axis.encoder.config.mode", "ENCODER_MODE_HALL")
    set_config_step(axis_num, f"axis.encoder.config.cpr", 42, "int")  # 6 states * 7 poles
    
    # Set manual motor parameters (can help with calibration)
    set_config_step(axis_num, f"axis.motor.config.phase_resistance", 0.15, "float")
    set_config_step(axis_num, f"axis.motor.config.phase_inductance", 0.00005, "float")
    
    print("\n=== STEP 2: Motor Calibration ===")
    print("Starting motor calibration (resistance measurement)...")
    perform_calibration_step(axis_num, "AXIS_STATE_MOTOR_CALIBRATION")
    
    print("\n=== STEP 3: Encoder Calibration ===")
    print("Starting Hall sensor calibration...")
    # Try two different methods - one might work
    if not perform_calibration_step(axis_num, "AXIS_STATE_ENCODER_HALL_POLARITY_CALIBRATION"):
        print("Hall polarity calibration failed, trying standard encoder calibration...")
        perform_calibration_step(axis_num, "AXIS_STATE_ENCODER_OFFSET_CALIBRATION")
    
    print("\n=== STEP 4: Checking Final Status ===")
    check_status(axis_num)
    
    print("\n=== CALIBRATION PROCESS COMPLETE ===")
    print("If calibration was successful, you can now run:")
    print(f"python3 src/tracked_robot/scripts/advanced_motor_test.py --axis {axis_num} --mode velocity")

def main():
    parser = argparse.ArgumentParser(description="ODrive One-Shot Calibration")
    parser.add_argument('--axis', type=int, default=0, choices=[0, 1],
                        help="Axis to calibrate (0 or 1)")
    args = parser.parse_args()
    
    print("=" * 60)
    print("  ODRIVE ONE-SHOT CALIBRATION")
    print("=" * 60)
    print("This script runs an ultra-minimal calibration sequence")
    print("designed to avoid ODrive connection issues during calibration.")
    print("It will perform minimal configuration and calibration steps.")
    
    one_shot_calibration(args.axis)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nCalibration cancelled by user")
