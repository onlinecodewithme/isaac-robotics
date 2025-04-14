#!/usr/bin/env python3

"""
Advanced Motor Test Script for ODrive Controllers

This script provides comprehensive motor testing with different speeds, 
position control, and trajectory following. It includes robust connection
handling to avoid issues when the ODrive resets.

Usage:
    python3 advanced_motor_test.py [--axis 0|1] [--mode velocity|position|trajectory|all]

Example:
    python3 advanced_motor_test.py --axis 1 --mode all
"""

import sys
import time
import math
import argparse
import numpy as np
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

def prepare_for_test(axis_num):
    """Prepare axis for testing, returns axis object if successful"""
    # Connect to ODrive
    odrv = connect_to_odrive()
    if not odrv:
        print("! Cannot connect to ODrive")
        return None
    
    # Get axis object
    try:
        axis = getattr(odrv, f"axis{axis_num}")
    except Exception as e:
        print(f"! Error accessing axis{axis_num}: {str(e)}")
        return None
    
    # Check if calibrated
    try:
        if not axis.motor.is_calibrated:
            print("! Motor is not calibrated")
            print("  Run calibration script first")
            return None
            
        if not axis.encoder.is_ready:
            print("! Encoder is not calibrated")
            print("  Run calibration script first")
            return None
    except Exception as e:
        print(f"! Error checking calibration status: {str(e)}")
        return None
    
    # Clear errors and set to idle
    try:
        axis.requested_state = AXIS_STATE_IDLE
        time.sleep(0.5)
        clear_errors(axis, "Preparation")
    except Exception as e:
        print(f"! Error preparing axis: {str(e)}")
        return None
    
    return axis, odrv

def enter_closed_loop_control(axis):
    """Enter closed loop control mode safely"""
    try:
        print("Entering closed loop control...")
        axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        time.sleep(1.0)
        
        if axis.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
            print("! Failed to enter closed loop control")
            print_error_state(axis, "Control Errors:")
            return False
            
        print("✓ Successfully entered closed loop control")
        return True
    except Exception as e:
        print(f"! Error entering closed loop control: {str(e)}")
        return False

def exit_closed_loop_control(axis):
    """Safely exit control mode"""
    try:
        # Set velocity/position to 0 to stop movement
        if hasattr(axis.controller, 'input_vel'):
            axis.controller.input_vel = 0.0
        if hasattr(axis.controller, 'input_pos'):
            axis.controller.input_pos = axis.encoder.pos_estimate
            
        time.sleep(0.5)
        axis.requested_state = AXIS_STATE_IDLE
        print("✓ Returned to idle state")
        return True
    except Exception as e:
        print(f"! Error exiting control mode: {str(e)}")
        try:
            axis.requested_state = AXIS_STATE_IDLE
        except:
            pass
        return False

def test_velocity_control(axis_num):
    """Test velocity control with various speeds"""
    print_header("VELOCITY CONTROL TEST")
    
    # Prepare for test
    result = prepare_for_test(axis_num)
    if not result:
        return False
    axis, odrv = result
    
    try:
        # Configure velocity control
        axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        print("✓ Set control mode to VELOCITY_CONTROL")
        
        # Set conservative limits
        if hasattr(axis.controller.config, 'vel_limit'):
            axis.controller.config.vel_limit = 30.0  # rad/s
            print("✓ Set velocity limit to 30.0 rad/s")
        
        # Set current limits based on API
        if hasattr(axis.config, 'dc_max_current'):
            # Newer API
            axis.config.dc_max_current = 30.0  # Amps
            axis.config.dc_max_negative_current = -30.0  # Amps
            print("✓ Set current limits to 30.0A (new API)")
        elif hasattr(axis.motor.config, 'current_lim'):
            # Older API
            axis.motor.config.current_lim = 30.0  # Amps
            print("✓ Set current limit to 30.0A (old API)")
            
        # Enter closed loop control
        if not enter_closed_loop_control(axis):
            return False
            
        print("\nRunning velocity control tests...")
        
        # Slow acceleration test
        print("\n1. Acceleration Test (0 to 20 rad/s):")
        print("   Starting from 0...")
        axis.controller.input_vel = 0.0
        time.sleep(1.0)
        
        print("   Accelerating to 20 rad/s over 5 seconds...")
        start_time = time.time()
        end_time = start_time + 5.0
        
        while time.time() < end_time:
            progress = (time.time() - start_time) / 5.0  # 0.0 to 1.0
            target_vel = 20.0 * progress
            axis.controller.input_vel = target_vel
            
            vel = axis.encoder.vel_estimate if hasattr(axis.encoder, 'vel_estimate') else 0
            current = axis.motor.current_control.Iq_measured if hasattr(axis.motor.current_control, 'Iq_measured') else 0
            
            print(f"   Time: {progress*5:.1f}s, Target: {target_vel:.1f}, Actual: {vel:.1f}, Current: {current:.2f}A")
            time.sleep(0.5)
            
        # Hold at max speed
        print("\n   Holding at 20 rad/s for 2 seconds...")
        axis.controller.input_vel = 20.0
        for i in range(4):
            vel = axis.encoder.vel_estimate if hasattr(axis.encoder, 'vel_estimate') else 0
            current = axis.motor.current_control.Iq_measured if hasattr(axis.motor.current_control, 'Iq_measured') else 0
            print(f"   Actual: {vel:.1f} rad/s, Current: {current:.2f}A")
            time.sleep(0.5)
            
        # Deceleration
        print("\n   Decelerating to 0 over 3 seconds...")
        start_time = time.time()
        end_time = start_time + 3.0
        
        while time.time() < end_time:
            progress = (time.time() - start_time) / 3.0  # 0.0 to 1.0
            target_vel = 20.0 * (1.0 - progress)
            axis.controller.input_vel = target_vel
            
            vel = axis.encoder.vel_estimate if hasattr(axis.encoder, 'vel_estimate') else 0
            current = axis.motor.current_control.Iq_measured if hasattr(axis.motor.current_control, 'Iq_measured') else 0
            
            print(f"   Time: {progress*3:.1f}s, Target: {target_vel:.1f}, Actual: {vel:.1f}, Current: {current:.2f}A")
            time.sleep(0.5)
            
        # Stop completely
        axis.controller.input_vel = 0.0
        time.sleep(1.0)
        
        # Direction change test
        print("\n2. Direction Change Test:")
        
        # Forward
        print("\n   Forward at 10 rad/s for 2 seconds...")
        axis.controller.input_vel = 10.0
        for i in range(4):
            vel = axis.encoder.vel_estimate if hasattr(axis.encoder, 'vel_estimate') else 0
            current = axis.motor.current_control.Iq_measured if hasattr(axis.motor.current_control, 'Iq_measured') else 0
            print(f"   Actual: {vel:.1f} rad/s, Current: {current:.2f}A")
            time.sleep(0.5)
            
        # Quick direction change
        print("\n   Changing direction to -10 rad/s...")
        axis.controller.input_vel = -10.0
        for i in range(6):
            vel = axis.encoder.vel_estimate if hasattr(axis.encoder, 'vel_estimate') else 0
            current = axis.motor.current_control.Iq_measured if hasattr(axis.motor.current_control, 'Iq_measured') else 0
            print(f"   Actual: {vel:.1f} rad/s, Current: {current:.2f}A")
            time.sleep(0.5)
            
        # Stop
        axis.controller.input_vel = 0.0
        time.sleep(1.0)
        
        # Speed ramp test
        print("\n3. Speed Ramp Test:")
        speeds = [5.0, 10.0, 15.0, 20.0, 25.0, 20.0, 15.0, 10.0, 5.0, 0.0]
        
        for speed in speeds:
            print(f"\n   Setting velocity to {speed} rad/s...")
            axis.controller.input_vel = speed
            
            # Monitor for 1.5 seconds
            for i in range(3):
                vel = axis.encoder.vel_estimate if hasattr(axis.encoder, 'vel_estimate') else 0
                current = axis.motor.current_control.Iq_measured if hasattr(axis.motor.current_control, 'Iq_measured') else 0
                print(f"   Actual: {vel:.1f} rad/s, Current: {current:.2f}A")
                time.sleep(0.5)
                
        # Sinusoidal velocity profile
        print("\n4. Sinusoidal Velocity Profile:")
        print("   Running sine wave pattern for 10 seconds...")
        
        start_time = time.time()
        end_time = start_time + 10.0
        
        while time.time() < end_time:
            elapsed = time.time() - start_time
            # Sine wave with 15 rad/s amplitude, 0.2 Hz
            target_vel = 15.0 * math.sin(2 * math.pi * 0.2 * elapsed)
            axis.controller.input_vel = target_vel
            
            vel = axis.encoder.vel_estimate if hasattr(axis.encoder, 'vel_estimate') else 0
            current = axis.motor.current_control.Iq_measured if hasattr(axis.motor.current_control, 'Iq_measured') else 0
            
            print(f"   Time: {elapsed:.1f}s, Target: {target_vel:.1f}, Actual: {vel:.1f}, Current: {current:.2f}A")
            time.sleep(0.5)
            
        # Stop
        axis.controller.input_vel = 0.0
        time.sleep(0.5)
        
        # Exit closed loop control
        exit_closed_loop_control(axis)
        print("\n✓ Velocity control test completed successfully!")
        return True
        
    except Exception as e:
        print(f"\n! Error during velocity test: {str(e)}")
        try:
            axis.controller.input_vel = 0.0
            axis.requested_state = AXIS_STATE_IDLE
        except:
            pass
        return False

def test_position_control(axis_num):
    """Test position control mode"""
    print_header("POSITION CONTROL TEST")
    
    # Prepare for test
    result = prepare_for_test(axis_num)
    if not result:
        return False
    axis, odrv = result
    
    try:
        # Configure position control
        axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
        print("✓ Set control mode to POSITION_CONTROL")
        
        # Set conservative limits
        if hasattr(axis.controller.config, 'vel_limit'):
            axis.controller.config.vel_limit = 20.0  # rad/s
            print("✓ Set velocity limit to 20.0 rad/s")
            
        # Set position gains if available
        if hasattr(axis.controller.config, 'pos_gain'):
            axis.controller.config.pos_gain = 20.0
            print("✓ Set position gain to 20.0")
            
        if hasattr(axis.controller.config, 'vel_gain'):
            axis.controller.config.vel_gain = 0.5  # Conservative
            print("✓ Set velocity gain to 0.5")
            
        # Enter closed loop control
        if not enter_closed_loop_control(axis):
            return False
            
        # Get initial position
        try:
            initial_pos = axis.encoder.pos_estimate
            print(f"\nInitial position: {initial_pos:.2f} rad")
        except Exception as e:
            print(f"! Error reading initial position: {str(e)}")
            initial_pos = 0.0
            
        print("\nRunning position control tests...")
        
        # Simple absolute position moves
        print("\n1. Absolute Position Movement:")
        positions = [
            initial_pos + 2.0,  # 2 radians forward
            initial_pos - 2.0,  # 2 radians backward
            initial_pos,        # back to start
        ]
        
        for pos in positions:
            print(f"\n   Moving to position {pos:.2f} rad...")
            axis.controller.input_pos = pos
            
            # Monitor until position is reached
            start_time = time.time()
            timeout = 5.0  # seconds
            
            while time.time() - start_time < timeout:
                current_pos = axis.encoder.pos_estimate if hasattr(axis.encoder, 'pos_estimate') else 0
                pos_error = abs(current_pos - pos)
                current = axis.motor.current_control.Iq_measured if hasattr(axis.motor.current_control, 'Iq_measured') else 0
                
                print(f"   Current: {current_pos:.2f} rad, Error: {pos_error:.3f}, Current: {current:.2f}A")
                
                # Check if position reached (within 0.1 rad)
                if pos_error < 0.1:
                    print(f"   ✓ Position reached!")
                    break
                    
                time.sleep(0.5)
                
            time.sleep(1.0)  # Pause at position
            
        # Relative movements
        print("\n2. Relative Position Increments:")
        current_pos = axis.encoder.pos_estimate
        
        for increment in [1.0, 2.0, -3.0, 0.0]:
            target_pos = current_pos + increment
            print(f"\n   Moving {increment:+.1f} rad to {target_pos:.2f} rad...")
            axis.controller.input_pos = target_pos
            
            # Monitor movement
            start_time = time.time()
            timeout = 5.0  # seconds
            
            while time.time() - start_time < timeout:
                current_pos = axis.encoder.pos_estimate if hasattr(axis.encoder, 'pos_estimate') else 0
                pos_error = abs(current_pos - target_pos)
                current = axis.motor.current_control.Iq_measured if hasattr(axis.motor.current_control, 'Iq_measured') else 0
                
                print(f"   Current: {current_pos:.2f} rad, Error: {pos_error:.3f}, Current: {current:.2f}A")
                
                # Check if position reached (within 0.1 rad)
                if pos_error < 0.1:
                    print(f"   ✓ Position reached!")
                    break
                    
                time.sleep(0.5)
                
            current_pos = axis.encoder.pos_estimate  # Update for next increment
            time.sleep(1.0)  # Pause at position
            
        # Multi-revolution test
        print("\n3. Multi-Revolution Test:")
        
        # One full revolution forward
        target_pos = current_pos + 2*math.pi
        print(f"\n   Moving one full revolution forward to {target_pos:.2f} rad...")
        axis.controller.input_pos = target_pos
        
        # Monitor movement
        start_time = time.time()
        timeout = 10.0  # seconds
        
        while time.time() - start_time < timeout:
            current_pos = axis.encoder.pos_estimate if hasattr(axis.encoder, 'pos_estimate') else 0
            pos_error = abs(current_pos - target_pos)
            current = axis.motor.current_control.Iq_measured if hasattr(axis.motor.current_control, 'Iq_measured') else 0
            
            print(f"   Current: {current_pos:.2f} rad, Error: {pos_error:.3f}, Current: {current:.2f}A")
            
            # Check if position reached (within 0.1 rad)
            if pos_error < 0.1:
                print(f"   ✓ Position reached!")
                break
                
            time.sleep(0.5)
            
        # One full revolution backward
        current_pos = axis.encoder.pos_estimate
        target_pos = current_pos - 2*math.pi
        print(f"\n   Moving one full revolution backward to {target_pos:.2f} rad...")
        axis.controller.input_pos = target_pos
        
        # Monitor movement
        start_time = time.time()
        timeout = 10.0  # seconds
        
        while time.time() - start_time < timeout:
            current_pos = axis.encoder.pos_estimate if hasattr(axis.encoder, 'pos_estimate') else 0
            pos_error = abs(current_pos - target_pos)
            current = axis.motor.current_control.Iq_measured if hasattr(axis.motor.current_control, 'Iq_measured') else 0
            
            print(f"   Current: {current_pos:.2f} rad, Error: {pos_error:.3f}, Current: {current:.2f}A")
            
            # Check if position reached (within 0.1 rad)
            if pos_error < 0.1:
                print(f"   ✓ Position reached!")
                break
                
            time.sleep(0.5)
            
        # Return to initial position
        print(f"\n   Returning to initial position {initial_pos:.2f} rad...")
        axis.controller.input_pos = initial_pos
        
        # Monitor movement
        start_time = time.time()
        timeout = 10.0  # seconds
        
        while time.time() - start_time < timeout:
            current_pos = axis.encoder.pos_estimate if hasattr(axis.encoder, 'pos_estimate') else 0
            pos_error = abs(current_pos - initial_pos)
            current = axis.motor.current_control.Iq_measured if hasattr(axis.motor.current_control, 'Iq_measured') else 0
            
            print(f"   Current: {current_pos:.2f} rad, Error: {pos_error:.3f}, Current: {current:.2f}A")
            
            # Check if position reached (within 0.1 rad)
            if pos_error < 0.1:
                print(f"   ✓ Position reached!")
                break
                
            time.sleep(0.5)
            
        # Exit closed loop control
        exit_closed_loop_control(axis)
        print("\n✓ Position control test completed successfully!")
        return True
        
    except Exception as e:
        print(f"\n! Error during position test: {str(e)}")
        try:
            axis.requested_state = AXIS_STATE_IDLE
        except:
            pass
        return False

def test_trajectory_following(axis_num):
    """Test trajectory following with smooth movements"""
    print_header("TRAJECTORY FOLLOWING TEST")
    
    # Prepare for test
    result = prepare_for_test(axis_num)
    if not result:
        return False
    axis, odrv = result
    
    try:
        # Configure position control with trap trajectory
        if hasattr(axis.controller.config, 'input_mode'):
            axis.controller.config.input_mode = INPUT_MODE_TRAP_TRAJ
            print("✓ Set input mode to TRAP_TRAJ (Trapezoidal Trajectory)")
        else:
            print("! Trap trajectory mode not available on this firmware")
            print("  Using standard position control instead")
            
        axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
        print("✓ Set control mode to POSITION_CONTROL")
        
        # Set conservative limits
        if hasattr(axis.controller.config, 'vel_limit'):
            axis.controller.config.vel_limit = 15.0  # rad/s
            print("✓ Set velocity limit to 15.0 rad/s")
            
        # Configure trap traj (if available)
        if hasattr(axis, 'trap_traj') and hasattr(axis.trap_traj, 'config'):
            if hasattr(axis.trap_traj.config, 'vel_limit'):
                axis.trap_traj.config.vel_limit = 15.0  # rad/s
                print("✓ Set trajectory velocity limit to 15.0 rad/s")
                
            if hasattr(axis.trap_traj.config, 'accel_limit'):
                axis.trap_traj.config.accel_limit = 10.0  # rad/s^2
                print("✓ Set trajectory acceleration limit to 10.0 rad/s^2")
                
            if hasattr(axis.trap_traj.config, 'decel_limit'):
                axis.trap_traj.config.decel_limit = 10.0  # rad/s^2
                print("✓ Set trajectory deceleration limit to 10.0 rad/s^2")
                
        # Enter closed loop control
        if not enter_closed_loop_control(axis):
            return False
            
        # Get initial position
        try:
            initial_pos = axis.encoder.pos_estimate
            print(f"\nInitial position: {initial_pos:.2f} rad")
        except Exception as e:
            print(f"! Error reading initial position: {str(e)}")
            initial_pos = 0.0
            
        print("\nRunning trajectory following tests...")
        
        # Single smooth move test
        print("\n1. Single Smooth Move:")
        target_pos = initial_pos + 6.0  # 6 radians (about 1 revolution)
        print(f"\n   Moving to position {target_pos:.2f} rad...")
        axis.controller.input_pos = target_pos
        
        # Monitor movement
        start_time = time.time()
        timeout = 10.0  # seconds
        
        while time.time() - start_time < timeout:
            current_pos = axis.encoder.pos_estimate if hasattr(axis.encoder, 'pos_estimate') else 0
            current_vel = axis.encoder.vel_estimate if hasattr(axis.encoder, 'vel_estimate') else 0
            pos_error = abs(current_pos - target_pos)
            current = axis.motor.current_control.Iq_measured if hasattr(axis.motor.current_control, 'Iq_measured') else 0
            
            # For trajectory mode, also print trajectory info if available
            traj_info = ""
            if hasattr(axis, 'trap_traj') and hasattr(axis.trap_traj, 'config'):
                if hasattr(axis.trap_traj, 'vel_setpoint'):
                    traj_vel = axis.trap_traj.vel_setpoint
                    traj_info = f", Traj Vel: {traj_vel:.2f}"
                    
            print(f"   Pos: {current_pos:.2f} rad, Vel: {current_vel:.2f} rad/s, Error: {pos_error:.3f}, Current: {current:.2f}A{traj_info}")
            
            # Check if position reached (within 0.1 rad) and velocity near zero
            if pos_error < 0.1 and abs(current_vel) < 0.5:
                print(f"   ✓ Position reached and settled!")
                break
                
            time.sleep(0.5)
            
        time.sleep(1.0)  # Pause at position
        
        # Back to initial position with trap trajectory
        print("\n   Returning to initial position...")
        axis.controller.input_pos = initial_pos
        
        # Monitor movement
        start_time = time.time()
        timeout = 10.0  # seconds
        
        while time.time() - start_time < timeout:
            current_pos = axis.encoder.pos_estimate if hasattr(axis.encoder, 'pos_estimate') else 0
            current_vel = axis.encoder.vel_estimate if hasattr(axis.encoder, 'vel_estimate') else 0
            pos_error = abs(current_pos - initial_pos)
            current = axis.motor.current_control.Iq_measured if hasattr(axis.motor.current_control, 'Iq_measured') else 0
            
            # For trajectory mode, also print trajectory info if available
            traj_info = ""
            if hasattr(axis, 'trap_traj') and hasattr(axis.trap_traj, 'config'):
                if hasattr(axis.trap_traj, 'vel_setpoint'):
                    traj_vel = axis.trap_traj.vel_setpoint
                    traj_info = f", Traj Vel: {traj_vel:.2f}"
                    
            print(f"   Pos: {current_pos:.2f} rad, Vel: {current_vel:.2f} rad/s, Error: {pos_error:.3f}, Current: {current:.2f}A{traj_info}")
            
            # Check if position reached (within 0.1 rad) and velocity near zero
            if pos_error < 0.1 and abs(current_vel) < 0.5:
                print(f"   ✓ Position reached and settled!")
                break
                
            time.sleep(0.5)
            
        time.sleep(1.0)  # Pause at position
        
        # Multi-point trajectory
        print("\n2. Multi-Point Trajectory:")
        print("   Will move to 5 different positions with smooth transitions")
        
        # Generate 5 positions around the current position
        current_pos = axis.encoder.pos_estimate
        positions = [
            current_pos + 2.0,    # 2 rad forward
            current_pos - 1.0,    # 1 rad backward from start
            current_pos + 4.0,    # 4 rad forward from start
            current_pos,          # back to start
            current_pos - 3.0,    # 3 rad backward from start
            current_pos,          # back to start
        ]
        
        for i, pos in enumerate(positions):
            print(f"\n   Point {i+1}: Moving to {pos:.2f} rad...")
            axis.controller.input_pos = pos
            
            # Monitor until reasonably close to target
            start_time = time.time()
            timeout = 5.0  # seconds
            
            while time.time() - start_time < timeout:
                current_pos = axis.encoder.pos_estimate if hasattr(axis.encoder, 'pos_estimate') else 0
                current_vel = axis.encoder.vel_estimate if hasattr(axis.encoder, 'vel_estimate') else 0
                pos_error = abs(current_pos - pos)
                
                        # For trajectory mode, also print trajectory info if available
                traj_info = ""
                if hasattr(axis, 'trap_traj') and hasattr(axis.trap_traj, 'config'):
                    if hasattr(axis.trap_traj, 'vel_setpoint'):
                        traj_vel = axis.trap_traj.vel_setpoint
                        traj_info = f", Traj Vel: {traj_vel:.2f}"
                        
                print(f"   Pos: {current_pos:.2f} rad, Vel: {current_vel:.2f} rad/s, Error: {pos_error:.3f}{traj_info}")
                
                # Check if position is reached
                if pos_error < 0.2:
                    print(f"   ✓ Position reached, moving to next point")
                    break
                    
                time.sleep(0.5)
                
        # Exit closed loop control
        exit_closed_loop_control(axis)
        print("\n✓ Trajectory following test completed successfully!")
        return True
        
    except Exception as e:
        print(f"\n! Error during trajectory test: {str(e)}")
        try:
            axis.requested_state = AXIS_STATE_IDLE
        except:
            pass
        return False

def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(description="Advanced Motor Test")
    parser.add_argument('--axis', type=int, default=0, choices=[0, 1],
                        help="Axis to test (0 or 1)")
    parser.add_argument('--mode', type=str, default='all', 
                        choices=['velocity', 'position', 'trajectory', 'all'],
                        help="Test mode to run")
    args = parser.parse_args()
    
    print_header("ADVANCED MOTOR TEST")
    print(f"Testing axis {args.axis} in {args.mode} mode")
    
    # Run the selected test(s)
    if args.mode == 'velocity' or args.mode == 'all':
        test_velocity_control(args.axis)
        time.sleep(2.0)  # Pause between tests
        
    if args.mode == 'position' or args.mode == 'all':
        test_position_control(args.axis)
        time.sleep(2.0)  # Pause between tests
        
    if args.mode == 'trajectory' or args.mode == 'all':
        test_trajectory_following(args.axis)
        
    print_header("TEST COMPLETE")
    print("All tests have been completed.")
    print("Ensure motors are in IDLE state before disconnecting.")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nTest interrupted by user.")
        print("Attempting to set motor to idle state...")
        try:
            odrv = connect_to_odrive(timeout=3)
            if odrv:
                if hasattr(odrv, 'axis0'):
                    odrv.axis0.requested_state = AXIS_STATE_IDLE
                if hasattr(odrv, 'axis1'):
                    odrv.axis1.requested_state = AXIS_STATE_IDLE
        except:
            pass
