#!/usr/bin/env python3

"""
ODrive Direct Current Control

This script allows direct control of ODrive motors using current control mode,
which bypasses the need for encoder calibration. This is useful when you have
motors that aren't fully calibrated but you need to test their movement.

Usage:
    python3 direct_current_control.py [--axis 0|1] [--current 1.0]
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
logger = logging.getLogger("odrive_current_control")

class ODriveCurrentController:
    def __init__(self, axis_num=0):
        self.axis_num = axis_num
        self.odrive = None
        self.axis = None
        self.connected = False
        
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
            
    def clear_errors(self):
        """Clear all errors on the axis"""
        try:
            if hasattr(self.axis, 'error'):
                self.axis.error = 0
            if hasattr(self.axis.motor, 'error'):
                self.axis.motor.error = 0
            if hasattr(self.axis.encoder, 'error'):
                self.axis.encoder.error = 0
            if hasattr(self.axis.controller, 'error'):
                self.axis.controller.error = 0
            logger.info("Errors cleared")
            return True
        except Exception as e:
            logger.error(f"Error clearing errors: {str(e)}")
            return False
            
    def set_current_control_mode(self):
        """Switch to current control mode"""
        try:
            # First set to idle
            self.axis.requested_state = AXIS_STATE_IDLE
            time.sleep(0.5)
            
            # Clear any errors
            self.clear_errors()
            
            # Configure for current control
            self.axis.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL
            logger.info("Set to TORQUE_CONTROL mode")
            
            # Make sure current limit is appropriate
            if hasattr(self.axis.motor.config, 'current_lim'):
                current_lim = self.axis.motor.config.current_lim
                logger.info(f"Current limit is {current_lim}A")
                if current_lim < 10.0:
                    self.axis.motor.config.current_lim = 10.0
                    logger.info(f"Increased current limit to 10.0A")
                
            # Enter closed loop control
            logger.info("Entering closed loop control...")
            self.axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            time.sleep(0.5)
            
            # Check if we succeeded
            if self.axis.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
                logger.error(f"Failed to enter closed loop control. State: {self.axis.current_state}")
                if hasattr(self.axis, 'error') and self.axis.error != 0:
                    logger.error(f"Axis error: {self.axis.error}")
                return False
                
            logger.info("Successfully entered current control mode")
            return True
        except Exception as e:
            logger.error(f"Error setting current control mode: {str(e)}")
            return False
            
    def apply_current(self, current):
        """Apply current to the motor"""
        try:
            if not self.connected:
                logger.error("Not connected to ODrive")
                return False
                
            if self.axis.current_state != AXIS_STATE_CLOSED_LOOP_CONTROL:
                logger.warning("Not in closed loop control mode, attempting to enter it")
                self.set_current_control_mode()
                
            logger.info(f"Applying {current:.2f}A current")
            self.axis.controller.input_torque = float(current)
            return True
        except Exception as e:
            logger.error(f"Error applying current: {str(e)}")
            return False
            
    def stop(self):
        """Stop the motor"""
        try:
            if not self.connected:
                return
                
            logger.info("Stopping motor")
            self.axis.controller.input_torque = 0.0
            
            # Go back to idle
            time.sleep(0.5)
            self.axis.requested_state = AXIS_STATE_IDLE
            logger.info("Motor stopped")
        except Exception as e:
            logger.error(f"Error stopping motor: {str(e)}")

def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='ODrive Direct Current Control')
    parser.add_argument('--axis', type=int, default=0, choices=[0, 1],
                        help='Motor axis (0 or 1)')
    parser.add_argument('--current', type=float, default=1.0,
                        help='Current to apply in Amps')
    args = parser.parse_args()
    
    # Create controller
    controller = ODriveCurrentController(args.axis)
    
    # Connect to ODrive
    if not controller.connect():
        print("Failed to connect to ODrive. Exiting.")
        sys.exit(1)
    
    # Set up signal handler for graceful exit
    def signal_handler(sig, frame):
        print("\nStopping motor and exiting...")
        controller.stop()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    # Display motor state and error info
    try:
        print(f"\nMotor State: {controller.axis.current_state}")
        print(f"Motor Calibrated: {controller.axis.motor.is_calibrated}")
        print(f"Encoder Ready: {controller.axis.encoder.is_ready}")
        
        if hasattr(controller.axis, 'error'):
            print(f"Axis Error: {controller.axis.error}")
        if hasattr(controller.axis.motor, 'error'):
            print(f"Motor Error: {controller.axis.motor.error}")
        if hasattr(controller.axis.encoder, 'error'):
            print(f"Encoder Error: {controller.axis.encoder.error}")
    except Exception as e:
        print(f"Error reading status: {str(e)}")
    
    # Switch to current control mode
    if not controller.set_current_control_mode():
        print("Failed to enter current control mode. Exiting.")
        sys.exit(1)
    
    print("\nDirect Current Control Mode")
    print("--------------------------")
    print("Commands:")
    print("  +N: Apply +N Amps (e.g. +2)")
    print("  -N: Apply -N Amps (e.g. -1.5)")
    print("  0: Stop (apply 0 current)")
    print("  q: Quit")
    
    try:
        while True:
            cmd = input("\nEnter current command: ").strip().lower()
            
            # Check for quit
            if cmd == 'q':
                break
                
            # Parse current value
            try:
                current = float(cmd)
                
                # Apply current
                controller.apply_current(current)
                
                # Display live data if available
                try:
                    print(f"Applied Current: {current:.2f}A")
                    if hasattr(controller.axis.motor.current_control, 'Iq_measured'):
                        print(f"Measured Current: {controller.axis.motor.current_control.Iq_measured:.2f}A")
                    if hasattr(controller.axis.encoder, 'vel_estimate'):
                        print(f"Velocity: {controller.axis.encoder.vel_estimate:.2f} rad/s")
                except Exception as e:
                    print(f"Error reading feedback: {str(e)}")
            except ValueError:
                print("Invalid input. Please enter a number or 'q' to quit.")
    
    finally:
        # Always stop the motor
        controller.stop()
        print("Motor stopped. Goodbye!")

if __name__ == "__main__":
    main()
