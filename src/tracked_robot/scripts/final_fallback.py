#!/usr/bin/env python3

"""
ODrive Absolute Last Resort Control

This script is the final attempt to make the ODrive motors move when all else fails.
It uses a very aggressive approach to bypass ODrive's normal safety checks and 
directly controls the motor, even with axis errors. It specifically addresses
the error code 2049 seen in the setup attempt.

USE WITH EXTREME CAUTION - This is a last resort.

Usage:
    python3 final_fallback.py [--axis 0|1] [--current 5.0]
"""

import sys
import time
import argparse
import logging
import struct
import signal

try:
    import odrive
    from odrive.enums import *
    import fibre.libfibre
except ImportError:
    print("Error: ODrive Python library not found.")
    print("Please install it using: pip install odrive")
    sys.exit(1)

# Configure logging
logging.basicConfig(level=logging.INFO, 
                    format='%(asctime)s [%(levelname)s] %(message)s',
                    datefmt='%H:%M:%S')
logger = logging.getLogger("odrive_fallback")

class ODriveFallbackControl:
    def __init__(self, axis_num=0, max_current=5.0):
        self.axis_num = axis_num
        self.max_current = max_current
        self.odrive = None
        self.axis = None
        self.connected = False
        self.tried_motor_calibration = False
        
    def connect(self, timeout=10):
        """Connect to ODrive with timeout"""
        logger.info("Connecting to ODrive...")
        try:
            self.odrive = odrive.find_any(timeout=timeout)
            logger.info(f"Connected to ODrive {self.odrive.serial_number}")
            
            # Get axis reference
            self.axis = getattr(self.odrive, f"axis{self.axis_num}")
            self.connected = True
            
            logger.info(f"Using axis {self.axis_num}")
            return True
        except Exception as e:
            logger.error(f"Failed to connect: {str(e)}")
            self.connected = False
            return False
            
    def reset_errors(self):
        """Reset all errors aggressively"""
        logger.info("Clearing all errors...")
        try:
            # Set to idle first
            self.axis.requested_state = AXIS_STATE_IDLE
            time.sleep(0.5)
            
            # Clear errors at all levels
            if hasattr(self.axis, 'error'):
                self.axis.error = 0
            if hasattr(self.axis.motor, 'error'):
                self.axis.motor.error = 0
            if hasattr(self.axis.encoder, 'error'):
                self.axis.encoder.error = 0
            if hasattr(self.axis.controller, 'error'):
                self.axis.controller.error = 0
                
            logger.info("All errors cleared")
            return True
        except Exception as e:
            logger.error(f"Error clearing errors: {str(e)}")
            return False
    
    def try_motor_calibration(self):
        """Attempt motor calibration"""
        if self.tried_motor_calibration:
            logger.info("Skipping motor calibration (already attempted)")
            return False
            
        logger.info("Attempting motor calibration...")
        try:
            # Reset to idle first
            self.axis.requested_state = AXIS_STATE_IDLE
            time.sleep(0.5)
            self.reset_errors()
            
            # Set reasonable current limits
            self.axis.motor.config.calibration_current = 5.0
            self.axis.motor.config.current_lim = self.max_current
            
            # Start calibration
            logger.info("Starting motor calibration (motor will chirp)...")
            self.axis.requested_state = AXIS_STATE_MOTOR_CALIBRATION
            
            # Wait for calibration
            start_time = time.time()
            while time.time() - start_time < 10.0:
                try:
                    if self.axis.current_state == AXIS_STATE_IDLE:
                        break
                    time.sleep(0.2)
                    print(".", end="", flush=True)
                except:
                    print("!", end="", flush=True)
                    
            print("")  # New line
            
            # Mark as tried even if it failed
            self.tried_motor_calibration = True
            
            # Check result
            if hasattr(self.axis.motor, 'is_calibrated') and self.axis.motor.is_calibrated:
                logger.info("Motor calibration successful!")
                return True
            else:
                logger.warning("Motor calibration did not succeed")
                return False
                
        except Exception as e:
            logger.error(f"Error during motor calibration: {str(e)}")
            self.tried_motor_calibration = True
            return False
            
    def bypass_encoder_check(self):
        """Force encoder to be considered ready"""
        logger.info("Bypassing encoder checks...")
        try:
            # Set flags to pretend encoder is ready
            if hasattr(self.axis.encoder.config, 'pre_calibrated'):
                self.axis.encoder.config.pre_calibrated = True
                logger.info("Set pre_calibrated = True")
                
            # Also set hall encoder polarity
            if hasattr(self.axis.encoder.config, 'hall_polarity_calibrated'):
                self.axis.encoder.config.hall_polarity_calibrated = True
                logger.info("Set hall_polarity_calibrated = True")
                
            # Set offset to reasonable value
            if hasattr(self.axis.encoder.config, 'hall_offset'):
                self.axis.encoder.config.hall_offset = 0
                logger.info("Set hall_offset = 0")
                
            # Try to save configuration
            try:
                self.odrive.save_configuration()
                logger.info("Saved encoder bypass configuration")
                # Device might reset here, but we'll handle that
            except Exception as e:
                if "object disappeared" in str(e).lower():
                    logger.info("ODrive reset while saving - this is normal")
                    # Reconnect
                    time.sleep(3.0)
                    self.connect()
                else:
                    logger.warning(f"Non-reset error during save: {str(e)}")
                    
            return True
        except Exception as e:
            logger.error(f"Failed to bypass encoder: {str(e)}")
            return False
    
    def try_force_closed_loop(self):
        """Force the motor into closed loop control"""
        logger.info("Forcing closed loop control...")
        try:
            # First reset to idle
            self.axis.requested_state = AXIS_STATE_IDLE
            time.sleep(0.5)
            self.reset_errors()
            
            # This is a specific fix for error 2049 (0x801) seen in setup
            # This error is AXIS_ERROR_MOTOR_FAILED
            logger.info("Applying fixes for error code 0x801 (MOTOR_FAILED)")
            try:
                # Make sure motor type is correct
                self.axis.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
                
                # Set torque control mode
                self.axis.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL
                self.axis.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
                
                # Force encoder ready
                self.bypass_encoder_check()
                
                # Save configuration (may cause reset)
                try:
                    self.odrive.save_configuration()
                    logger.info("Saved control mode configuration")
                except Exception as e:
                    if "object disappeared" in str(e).lower():
                        logger.info("ODrive reset while saving - this is normal")
                        # Reconnect
                        time.sleep(3.0)
                        self.connect()
                    else:
                        logger.warning(f"Non-reset error during save: {str(e)}")
            except Exception as e:
                logger.error(f"Error configuring control mode: {str(e)}")
            
            # Now try to enter closed loop control
            logger.info("Entering closed loop control...")
            self.axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            time.sleep(1.0)
            
            # Check if successful
            state = self.axis.current_state
            if state == AXIS_STATE_CLOSED_LOOP_CONTROL:
                logger.info("✓ Successfully entered closed loop control!")
                return True
            else:
                logger.warning(f"Failed to enter closed loop control. State: {state}")
                
                # Check for specific errors
                if hasattr(self.axis, 'error') and self.axis.error != 0:
                    error_code = self.axis.error
                    logger.error(f"Axis error: 0x{error_code:X}")
                    
                    # Final desperate attempt - force the controller state
                    logger.info("Making final desperate attempt to move motor...")
                    return self.absolute_last_resort()
                    
                return False
        except Exception as e:
            logger.error(f"Error forcing closed loop: {str(e)}")
            return False
            
    def absolute_last_resort(self):
        """
        Absolute last resort method to try to move the motor.
        This directly applies torque by using MOTOR_TYPE_GIMBAL,
        which doesn't require encoder feedback.
        """
        logger.info("=" * 50)
        logger.info("ABSOLUTE LAST RESORT MODE ACTIVATED")
        logger.info("=" * 50)
        logger.info("This bypasses many safety features!")
        
        try:
            # Go to idle
            self.axis.requested_state = AXIS_STATE_IDLE
            time.sleep(0.5)
            
            # Clear errors
            self.reset_errors()
            
            # Try gimbal mode as a last resort
            logger.info("Setting motor to GIMBAL mode to bypass current checks")
            self.axis.motor.config.motor_type = MOTOR_TYPE_GIMBAL
            
            # Reduced current for safety
            self.axis.motor.config.current_lim = min(5.0, self.max_current)
            logger.info(f"Limited current to {self.axis.motor.config.current_lim}A for safety")
            
            # Set torque control mode
            self.axis.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL
            self.axis.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
            
            # Enter closed loop control
            logger.info("Attempting closed loop control with GIMBAL motor type...")
            self.axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            time.sleep(1.0)
            
            # Check result
            if self.axis.current_state == AXIS_STATE_CLOSED_LOOP_CONTROL:
                logger.info("✓ Last resort mode SUCCESSFUL!")
                return True
            else:
                logger.error(f"Last resort mode failed. State: {self.axis.current_state}")
                return False
                
        except Exception as e:
            logger.error(f"Error in last resort mode: {str(e)}")
            return False
            
    def apply_current(self, current_value):
        """Apply current to move motor"""
        if not self.connected:
            logger.error("Not connected")
            return False
            
        try:
            # Apply current based on control mode
            if hasattr(self.axis.controller.config, 'control_mode'):
                mode = self.axis.controller.config.control_mode
                
                if mode == CONTROL_MODE_TORQUE_CONTROL:
                    # Direct current control
                    self.axis.controller.input_torque = float(current_value)
                    logger.info(f"Applied torque current: {current_value}A")
                else:
                    # Try velocity for other modes
                    self.axis.controller.input_vel = float(current_value)
                    logger.info(f"Applied velocity: {current_value}")
            else:
                logger.error("Cannot determine control mode")
                return False
                
            return True
        except Exception as e:
            logger.error(f"Error applying current: {str(e)}")
            return False
            
    def stop(self):
        """Stop the motor"""
        try:
            if not self.connected:
                return
                
            logger.info("Stopping motor...")
            
            # Try to zero out input
            if hasattr(self.axis.controller, 'input_torque'):
                self.axis.controller.input_torque = 0.0
            if hasattr(self.axis.controller, 'input_vel'):
                self.axis.controller.input_vel = 0.0
                
            # Go to idle
            self.axis.requested_state = AXIS_STATE_IDLE
            
            logger.info("Motor stopped")
        except Exception as e:
            logger.error(f"Error stopping motor: {str(e)}")
        
def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(description="ODrive Last Resort Control")
    parser.add_argument('--axis', type=int, default=0, choices=[0, 1],
                        help="Motor axis (0 or 1)")
    parser.add_argument('--current', type=float, default=5.0,
                        help="Maximum current in Amps")
    args = parser.parse_args()
    
    # Create controller
    controller = ODriveFallbackControl(args.axis, args.current)
    
    # Connect to ODrive
    if not controller.connect():
        print("Failed to connect to ODrive. Exiting.")
        sys.exit(1)
    
    # Setup signal handler for Ctrl+C
    def signal_handler(sig, frame):
        print("\nStopping motor and exiting...")
        controller.stop()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    # Display current status
    try:
        print("\n=== CURRENT STATUS ===")
        print(f"Motor Type: {controller.axis.motor.config.motor_type}")
        print(f"Control Mode: {controller.axis.controller.config.control_mode}")
        print(f"Current Limit: {controller.axis.motor.config.current_lim}A")
        print(f"Motor State: {controller.axis.current_state}")
        print(f"Motor Calibrated: {controller.axis.motor.is_calibrated}")
        print(f"Encoder Ready: {controller.axis.encoder.is_ready}")
        
        if hasattr(controller.axis, 'error') and controller.axis.error != 0:
            err = controller.axis.error
            print(f"Axis Error: 0x{err:X}")
        if hasattr(controller.axis.motor, 'error') and controller.axis.motor.error != 0:
            err = controller.axis.motor.error
            print(f"Motor Error: 0x{err:X}")
        if hasattr(controller.axis.encoder, 'error') and controller.axis.encoder.error != 0:
            err = controller.axis.encoder.error
            print(f"Encoder Error: 0x{err:X}")
    except Exception as e:
        print(f"Error getting status: {e}")
    
    # Try to force the motor to move
    print("\n=== ATTEMPTING MOTOR CONTROL ===")
    
    # Step 1: Reset errors
    controller.reset_errors()
    
    # Step 2: Try motor calibration if needed
    if not controller.axis.motor.is_calibrated:
        print("\nMotor needs calibration")
        controller.try_motor_calibration()
    
    # Step 3: Force closed loop
    success = controller.try_force_closed_loop()
    
    if not success:
        print("\n! Failed to achieve control mode.")
        choice = input("Continue with limited functionality? (y/n): ")
        if choice.lower() != 'y':
            controller.stop()
            sys.exit(1)
    
    # Main control loop
    print("\n=== MOTOR CONTROL READY ===")
    print("Control options:")
    print("  <number>: Apply current (e.g. 3 or -2)")
    print("  c: Check motor status")
    print("  r: Reset all errors")
    print("  q: Quit")
    
    while True:
        cmd = input("\nEnter command: ").strip().lower()
        
        if cmd == 'q':
            break
        elif cmd == 'c':
            # Check status
            try:
                state = controller.axis.current_state
                print(f"Motor State: {state}")
                if hasattr(controller.axis, 'error') and controller.axis.error != 0:
                    err = controller.axis.error
                    print(f"Axis Error: 0x{err:X}")
            except Exception as e:
                print(f"Error getting status: {e}")
        elif cmd == 'r':
            # Reset errors
            controller.reset_errors()
        else:
            # Try to parse as current
            try:
                value = float(cmd)
                
                # Limit current
                if abs(value) > controller.max_current:
                    print(f"Limiting current to {controller.max_current}A")
                    value = controller.max_current if value > 0 else -controller.max_current
                
                # Apply for 1 second then stop
                print(f"Applying {'forward' if value > 0 else 'reverse'} current...")
                controller.apply_current(value)
                time.sleep(1.0)
                controller.apply_current(0.0)
                print("Stopped")
            except ValueError:
                print("Invalid command")
    
    # Clean shutdown
    controller.stop()
    print("Motor stopped and program exited")

if __name__ == "__main__":
    main()
