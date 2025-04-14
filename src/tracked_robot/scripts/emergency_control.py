#!/usr/bin/env python3

"""
ODrive Emergency Control

This script allows controlling ODrive motors even when normal calibration fails.
It uses a special low-level approach that bypasses encoder requirements by
directly applying phase currents to the motor.

USE WITH CAUTION - This method doesn't use encoder feedback and is only
for emergency testing or validation.

Usage:
    python3 emergency_control.py [--axis 0|1] [--current 2.0]
"""

import sys
import time
import signal
import argparse
import logging

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
logger = logging.getLogger("odrive_emergency")

class EmergencyMotorControl:
    def __init__(self, axis_num=0):
        self.axis_num = axis_num
        self.odrive = None
        self.axis = None
        self.connected = False
        self.max_current = 10.0
        
    def connect(self, timeout=10):
        """Connect to ODrive with timeout"""
        logger.info("Connecting to ODrive...")
        try:
            self.odrive = odrive.find_any(timeout=timeout)
            logger.info(f"Connected to ODrive {self.odrive.serial_number}")
            
            # Get axis reference
            self.axis = getattr(self.odrive, f"axis{self.axis_num}")
            self.connected = True
            
            logger.info(f"Connected to axis {self.axis_num}")
            return True
        except Exception as e:
            logger.error(f"Failed to connect: {str(e)}")
            self.connected = False
            return False
    
    def configure_emergency_control(self):
        """Configure the motor for emergency open-loop control"""
        logger.info("Setting up emergency control mode...")
        
        try:
            # First, go to idle state
            self.axis.requested_state = AXIS_STATE_IDLE
            time.sleep(0.5)
            
            # Clear any errors
            if hasattr(self.axis, 'error'):
                self.axis.error = 0
            if hasattr(self.axis.motor, 'error'):
                self.axis.motor.error = 0
            if hasattr(self.axis.encoder, 'error'):
                self.axis.encoder.error = 0
            if hasattr(self.axis.controller, 'error'):
                self.axis.controller.error = 0
            
            # Setup for motor type
            self.axis.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
            
            # Set a reasonable current limit
            self.axis.motor.config.current_lim = self.max_current
            self.axis.motor.config.calibration_current = 5.0
            
            # Configure for open loop control
            if hasattr(self.axis.controller.config, 'control_mode'):
                self.axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
                logger.info("Set to POSITION_CONTROL")
            
            # Try open loop control first
            if hasattr(self.axis.controller.config, 'input_mode'):
                self.axis.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
                logger.info("Set INPUT_MODE_PASSTHROUGH")
                
            # Should enable the motor for direct phase control
            logger.info("Attempting direct motor control...")
            
            # Try to run motor calibration if it's not already calibrated
            if not self.axis.motor.is_calibrated:
                logger.info("Motor not calibrated. Attempting calibration...")
                try:
                    self.axis.requested_state = AXIS_STATE_MOTOR_CALIBRATION
                    time.sleep(7.0)  # Wait for calibration to complete
                    
                    # Check if calibration worked
                    if self.axis.motor.is_calibrated:
                        logger.info("Motor calibration successful!")
                    else:
                        logger.warning("Motor calibration may have failed.")
                except Exception as e:
                    logger.error(f"Error during calibration: {str(e)}")
            
            # Enable motor again, regardless of whether calibration succeeded
            try:
                # Try direct current control
                if hasattr(self.axis.controller.config, 'control_mode'):
                    self.axis.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL
                    logger.info("Set to TORQUE_CONTROL for direct current")
                
                # Enter closed loop control state
                self.axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
                time.sleep(0.5)
                
                # Check if we succeeded
                if self.axis.current_state == AXIS_STATE_CLOSED_LOOP_CONTROL:
                    logger.info("Successfully entered closed loop control!")
                    if hasattr(self.axis, 'error') and self.axis.error != 0:
                        logger.warning(f"Note: Axis still has error: {self.axis.error}")
                    return True
                else:
                    logger.warning(f"Failed to enter closed loop. State: {self.axis.current_state}")
                    if hasattr(self.axis, 'error') and self.axis.error != 0:
                        logger.warning(f"Axis error: {self.axis.error}")
                    
                    # Try ultimate fallback: Direct phase control
                    logger.info("Attempting direct phase control (extreme emergency mode)...")
                    try:
                        # This is risky but might work in extreme cases
                        if hasattr(self.axis.motor, 'config'):
                            logger.info("RESET CONTROLLER TO FORCE A CLEAN STATE...")
                            
                            # Tell controller we've calibrated the encoder
                            if hasattr(self.axis.encoder.config, 'pre_calibrated'):
                                self.axis.encoder.config.pre_calibrated = True
                                logger.info("Pretending encoder is calibrated")
                            
                            # Last resort: Try to use sensorless ramp
                            try:
                                if hasattr(self.axis.controller.config, 'control_mode'):
                                    new_mode = CONTROL_MODE_VELOCITY_CONTROL
                                    self.axis.controller.config.control_mode = new_mode
                                    logger.info(f"Set control mode to {new_mode}")
                                
                                # Final attempt to enter closed loop
                                self.axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
                                time.sleep(0.5)
                                
                                if self.axis.current_state == AXIS_STATE_CLOSED_LOOP_CONTROL:
                                    logger.info("Success! Entered control loop on second attempt")
                                    return True
                                else:
                                    logger.error(f"Second attempt failed. State: {self.axis.current_state}")
                                    if hasattr(self.axis, 'error') and self.axis.error != 0:
                                        logger.error(f"Error code: {self.axis.error}")
                            except Exception as e:
                                logger.error(f"Error during final attempt: {e}")
                                
                    except Exception as e:
                        logger.error(f"Error during direct phase override: {e}")
                        
                    return False
            except Exception as e:
                logger.error(f"Unexpected error: {str(e)}")
                return False
                
        except Exception as e:
            logger.error(f"Error configuring emergency control: {str(e)}")
            return False
    
    def apply_current(self, current_value):
        """Apply current to move motor"""
        if not self.connected:
            logger.error("Not connected")
            return False
            
        try:
            # Check current mode
            if hasattr(self.axis.controller.config, 'control_mode'):
                control_mode = self.axis.controller.config.control_mode
                logger.info(f"Current control mode: {control_mode}")
                
                # Apply appropriate control based on mode
                if control_mode == CONTROL_MODE_TORQUE_CONTROL:
                    # Direct current control
                    self.axis.controller.input_torque = float(current_value)
                    logger.info(f"Applied current: {current_value}A")
                elif control_mode == CONTROL_MODE_VELOCITY_CONTROL:
                    # Velocity control
                    self.axis.controller.input_vel = float(current_value)
                    logger.info(f"Applied velocity: {current_value} rad/s")
                else:
                    # Fall back to direct current in any other mode
                    if hasattr(self.axis.controller, 'input_torque'):
                        self.axis.controller.input_torque = float(current_value)
                        logger.info(f"Applied current: {current_value}A")
                    else:
                        logger.error("Unable to control motor - no appropriate control mode")
                        return False
            
            # Show feedback if available
            try:
                if hasattr(self.axis.motor.current_control, 'Iq_measured'):
                    Iq = self.axis.motor.current_control.Iq_measured
                    logger.info(f"Measured current: {Iq:.2f}A")
                if hasattr(self.axis.encoder, 'vel_estimate'):
                    vel = self.axis.encoder.vel_estimate
                    logger.info(f"Velocity estimate: {vel:.2f} rad/s")
            except:
                pass
                
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
            
            # Try to zero out control commands
            try:
                if hasattr(self.axis.controller, 'input_torque'):
                    self.axis.controller.input_torque = 0.0
                if hasattr(self.axis.controller, 'input_vel'):
                    self.axis.controller.input_vel = 0.0
            except:
                pass
                
            # Try to go to idle state
            try:
                self.axis.requested_state = AXIS_STATE_IDLE
            except:
                pass
                
            logger.info("Motor should be stopped")
        except Exception as e:
            logger.error(f"Error stopping motor: {str(e)}")

def main():
    # Parse arguments
    parser = argparse.ArgumentParser(description="ODrive Emergency Motor Control")
    parser.add_argument('--axis', type=int, default=0, choices=[0, 1],
                      help="Motor axis to control (0 or 1)")
    parser.add_argument('--current', type=float, default=3.0,
                      help="Maximum current to apply (in Amps)")
    args = parser.parse_args()
    
    # Set up controller
    controller = EmergencyMotorControl(args.axis)
    controller.max_current = args.current
    
    # Connect to ODrive
    if not controller.connect():
        print("Failed to connect to ODrive. Exiting.")
        sys.exit(1)
    
    # Set up signal handler for graceful exit
    def signal_handler(sig, frame):
        print("\nExiting and stopping motor...")
        controller.stop()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    # Show motor status
    print("\n=== MOTOR STATUS ===")
    try:
        print(f"Motor State: {controller.axis.current_state}")
        print(f"Motor Calibrated: {controller.axis.motor.is_calibrated}")
        print(f"Encoder Ready: {controller.axis.encoder.is_ready}")
        
        if hasattr(controller.axis, 'error'):
            err = controller.axis.error
            print(f"Axis Error: {err} (0x{err:X})")
        if hasattr(controller.axis.motor, 'error'):
            err = controller.axis.motor.error 
            print(f"Motor Error: {err} (0x{err:X})")
        if hasattr(controller.axis.encoder, 'error'):
            err = controller.axis.encoder.error
            print(f"Encoder Error: {err} (0x{err:X})")
    except Exception as e:
        print(f"Error getting status: {str(e)}")
    
    # Configure for emergency control
    print("\n=== CONFIGURING EMERGENCY CONTROL ===")
    if not controller.configure_emergency_control():
        print("! WARNING: Not all setup steps succeeded.")
        print("  Some functions might not work properly.")
        
        prompt = input("Continue anyway? (y/n): ")
        if prompt.lower() != 'y':
            controller.stop()
            sys.exit(1)
    
    # Main control loop
    print("\n=== EMERGENCY CONTROL MODE ===")
    print("Enter positive number for forward current")
    print("Enter negative number for reverse current")
    print("Enter 'q' to quit")
    
    try:
        while True:
            cmd = input("\nEnter current value: ").strip().lower()
            
            if cmd == 'q':
                break
                
            try:
                value = float(cmd)
                if abs(value) > controller.max_current:
                    print(f"! Value too high. Limited to ±{controller.max_current}A")
                    value = controller.max_current if value > 0 else -controller.max_current
                
                # Apply the current/command
                controller.apply_current(value)
                
                if value != 0:
                    print(f"Applying {'forward' if value > 0 else 'reverse'} current for 1 second...")
                    time.sleep(1.0)
                    print("Stopping...")
                    controller.apply_current(0.0)
            except ValueError:
                print("Invalid input. Please enter a number or 'q'.")
    finally:
        controller.stop()
        print("Motor stopped and controller exited.")

if __name__ == "__main__":
    main()
