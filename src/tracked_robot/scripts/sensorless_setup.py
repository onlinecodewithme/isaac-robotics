#!/usr/bin/env python3

"""
ODrive Sensorless Setup Script

This script configures the ODrive for sensorless operation mode, which
doesn't require encoder feedback. This can be useful when:
1. Hall sensors are not working properly
2. Motor calibration fails due to encoder issues
3. You want to run in a simpler configuration mode

Based on the approach from: https://rbts.co/Blog/Electronics/odrive-3.6-sensorless-setup/

Usage:
    python3 sensorless_setup.py [--axis 0|1] [--velocity 2.0]
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
logger = logging.getLogger("sensorless")

class SensorlessODrive:
    def __init__(self, axis_num=0, velocity=2.0):
        self.axis_num = axis_num
        self.velocity = velocity
        self.odrv = None
        self.axis = None
        self.connected = False
        
    def connect(self, timeout=10):
        """Connect to ODrive"""
        logger.info("Connecting to ODrive...")
        try:
            self.odrv = odrive.find_any(timeout=timeout)
            logger.info(f"Connected to ODrive {self.odrv.serial_number}")
            
            # Get axis reference
            self.axis = getattr(self.odrv, f"axis{self.axis_num}")
            self.connected = True
            
            logger.info(f"Connected to axis {self.axis_num}")
            return True
        except Exception as e:
            logger.error(f"Connection failed: {str(e)}")
            return False
            
    def check_errors(self):
        """Check for errors and report them"""
        try:
            axis_error = self.axis.error if hasattr(self.axis, 'error') else 0
            motor_error = self.axis.motor.error if hasattr(self.axis.motor, 'error') else 0
            controller_error = self.axis.controller.error if hasattr(self.axis.controller, 'error') else 0
            
            if axis_error != 0:
                logger.warning(f"Axis error: 0x{axis_error:X}")
            if motor_error != 0:
                logger.warning(f"Motor error: 0x{motor_error:X}")
            if controller_error != 0:
                logger.warning(f"Controller error: 0x{controller_error:X}")
                
            return axis_error, motor_error, controller_error
        except Exception as e:
            logger.error(f"Error checking errors: {str(e)}")
            return None, None, None
            
    def clear_errors(self):
        """Clear all errors on axis"""
        try:
            logger.info("Clearing all errors...")
            
            self.axis.error = 0
            self.axis.motor.error = 0
            self.axis.controller.error = 0
            if hasattr(self.axis, 'encoder'):
                self.axis.encoder.error = 0
                
            logger.info("Errors cleared")
            return True
        except Exception as e:
            logger.error(f"Error clearing errors: {str(e)}")
            return False
            
    def configure_for_sensorless(self):
        """Configure ODrive for sensorless mode"""
        
        # Step 1: Set to idle state first
        try:
            logger.info("Setting to idle state...")
            self.axis.requested_state = AXIS_STATE_IDLE
            time.sleep(0.5)
            
            # Clear any existing errors
            self.clear_errors()
            
            # Step 2: Set motor type to HIGH_CURRENT
            logger.info("Configuring ODrive for sensorless operation...")
            
            # Basic motor configuration
            self.axis.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
            logger.info("Set motor type to HIGH_CURRENT")
            
            # Motor current limits
            self.axis.motor.config.current_lim = 25.0  # Higher for 1000W motor
            self.axis.motor.config.calibration_current = 15.0
            logger.info(f"Set current limit to 25.0A and calibration current to 15.0A")
            
            # Step 3: Configure sensorless settings
            self.axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
            logger.info("Set control mode to VELOCITY_CONTROL")
            
            # Step 4: Configure for sensorless ramp
            # This is the key part - using sensorless ramp mode
            try:
                # First, check if the enum is available directly
                if 'INPUT_MODE_SENSORLESS_RAMP' in dir(odrive.enums):
                    self.axis.controller.config.input_mode = odrive.enums.INPUT_MODE_SENSORLESS_RAMP
                    logger.info("Set input mode to SENSORLESS_RAMP using enum")
                else:
                    # For ODrive firmware v0.5.6, sensorless ramp mode is input_mode=5
                    # This hardcoded value works with that specific firmware version
                    self.axis.controller.config.input_mode = 5  # SENSORLESS_RAMP value for v0.5.6
                    logger.info("Set input mode to SENSORLESS_RAMP (value=5)")
            except Exception as e:
                logger.error(f"Error setting input mode: {str(e)}")
                return False
            
            # Step 5: Configure sensorless parameters
            # These parameters are critical for correct sensorless operation
            
            # How quickly to ramp up the velocity
            self.axis.controller.config.vel_ramp_rate = 1.0  # 1 turn/s²
            logger.info(f"Set velocity ramp rate to 1.0 turn/s²")
            
            # Parameters for sensorless estimator
            if hasattr(self.axis, 'sensorless_estimator'):
                logger.info("Configuring sensorless estimator parameters...")
                
                if hasattr(self.axis.sensorless_estimator.config, 'pm_flux_linkage'):
                    # This is a motor-specific parameter, but 5.51e-3 is a reasonable guess
                    # Affects how well the ODrive can estimate the rotor position
                    self.axis.sensorless_estimator.config.pm_flux_linkage = 5.51e-3
                    logger.info("Set PM flux linkage to 5.51e-3")
                    
            # Set a reasonable velocity limit
            self.axis.controller.config.vel_limit = 10.0  # 10 turns/s
            logger.info(f"Set velocity limit to 10.0 turns/s")
            
            # Save the configuration
            logger.info("Saving configuration...")
            try:
                self.odrv.save_configuration()
                logger.info("Configuration saved!")
                
                # Wait a moment as the ODrive will likely reset
                logger.info("Waiting for device reset...")
                time.sleep(5.0)
                
                # Reconnect
                self.connect()
                logger.info("Reconnected after configuration save")
                
            except Exception as e:
                if "object disappeared" in str(e).lower():
                    logger.info("Device reset during save (expected)")
                    logger.info("Reconnecting...")
                    time.sleep(5.0)
                    self.connect()
                else:
                    logger.error(f"Error during save: {str(e)}")
                    return False
            
            logger.info("Sensorless configuration complete!")
            return True
                
        except Exception as e:
            logger.error(f"Error configuring sensorless mode: {str(e)}")
            return False
            
    def calibrate_motor_only(self):
        """Run only motor calibration (not encoder calibration)"""
        logger.info("Running motor-only calibration...")
        try:
            # Make sure in idle state
            self.axis.requested_state = AXIS_STATE_IDLE
            time.sleep(0.5)
            
            # Clear errors
            self.clear_errors()
            
            # Calibrate motor phases
            logger.info("Starting motor calibration...")
            self.axis.requested_state = AXIS_STATE_MOTOR_CALIBRATION
            
            # Wait for calibration to complete
            start_time = time.time()
            max_wait = 15.0
            while time.time() - start_time < max_wait:
                try:
                    if self.axis.current_state == AXIS_STATE_IDLE:
                        break
                    time.sleep(0.5)
                    print(".", end="", flush=True)
                except:
                    print("!", end="", flush=True)
                    
            print("")  # New line after progress indicators
            
            # Check if calibration succeeded
            if hasattr(self.axis.motor, 'is_calibrated') and self.axis.motor.is_calibrated:
                logger.info("✓ Motor calibration successful!")
                
                # Save calibration results
                try:
                    self.odrv.save_configuration()
                    logger.info("Configuration saved!")
                except Exception as e:
                    logger.warning(f"Error saving configuration: {str(e)}")
                    logger.warning("But motor calibration succeeded, so we can continue")
                    
                return True
            else:
                logger.error("Motor calibration failed!")
                self.check_errors()
                return False
        except Exception as e:
            logger.error(f"Error during motor calibration: {str(e)}")
            return False
            
    def enter_closed_loop_control(self):
        """Enter closed loop control in sensorless mode"""
        logger.info("Entering closed loop sensorless mode...")
        try:
            self.axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            time.sleep(1.0)
            
            actual_state = self.axis.current_state
            if actual_state == AXIS_STATE_CLOSED_LOOP_CONTROL:
                logger.info("✓ Successfully entered closed loop control!")
                return True
            else:
                logger.error(f"Failed to enter closed loop control. State: {actual_state}")
                self.check_errors()
                return False
        except Exception as e:
            logger.error(f"Error entering closed loop control: {str(e)}")
            return False
            
    def run_motor(self, velocity=None, duration=5.0):
        """Run the motor at specified velocity for duration seconds"""
        if velocity is None:
            velocity = self.velocity
            
        logger.info(f"Running motor at velocity {velocity} turns/s for {duration} seconds...")
        try:
            # Set the velocity target
            self.axis.controller.input_vel = float(velocity)
            
            # Monitor for the specified duration
            start_time = time.time()
            while time.time() - start_time < duration:
                try:
                    # Get current estimated velocity
                    if hasattr(self.axis, 'sensorless_estimator') and \
                       hasattr(self.axis.sensorless_estimator, 'vel_estimate'):
                        vel = self.axis.sensorless_estimator.vel_estimate
                        logger.info(f"Velocity: {vel:.2f} turns/s")
                    elif hasattr(self.axis.encoder, 'vel_estimate'):
                        vel = self.axis.encoder.vel_estimate
                        logger.info(f"Velocity: {vel:.2f} turns/s")
                    
                    # Check for any errors
                    if self.axis.error != 0:
                        logger.warning(f"Error detected: 0x{self.axis.error:X}")
                    
                    time.sleep(0.5)
                except Exception as e:
                    logger.error(f"Error monitoring: {str(e)}")
                    break
                    
            # Stop the motor
            self.axis.controller.input_vel = 0.0
            logger.info("Motor stopped")
            return True
        except Exception as e:
            logger.error(f"Error running motor: {str(e)}")
            return False
        finally:
            # Safety stop
            try:
                self.axis.controller.input_vel = 0.0
            except:
                pass
                
    def stop(self):
        """Stop the motor and return to idle state"""
        logger.info("Stopping motor and returning to idle...")
        try:
            # Set velocity to zero
            if hasattr(self.axis.controller, 'input_vel'):
                self.axis.controller.input_vel = 0.0
                
            # Return to idle state
            self.axis.requested_state = AXIS_STATE_IDLE
            
            logger.info("Motor stopped and in idle state")
            return True
        except Exception as e:
            logger.error(f"Error stopping motor: {str(e)}")
            return False
            
    def test_sensorless(self):
        """Run a comprehensive test of the sensorless setup"""
        print("\n" + "=" * 60)
        print(" ODRIVE SENSORLESS MODE SETUP AND TEST")
        print("=" * 60)
        
        # Connect to ODrive
        if not self.connect():
            print("Failed to connect to ODrive. Exiting.")
            return False
            
        # Step 1: Configure for sensorless operation
        print("\n" + "=" * 50)
        print("SENSORLESS CONFIGURATION")
        print("=" * 50)
        if not self.configure_for_sensorless():
            print("Failed to configure sensorless mode. Exiting.")
            return False
            
        # Step 2: Motor-only calibration
        print("\n" + "=" * 50)
        print("MOTOR CALIBRATION")
        print("=" * 50)
        if not self.calibrate_motor_only():
            print("\nMotor calibration failed. Proceeding anyway to test sensorless mode...")
        
        # Step 3: Enter closed loop control
        print("\n" + "=" * 50)
        print("ENTERING CLOSED LOOP CONTROL")
        print("=" * 50)
        if not self.enter_closed_loop_control():
            print("Failed to enter closed loop control. Exiting.")
            self.stop()
            return False
            
        # Step 4: Run motor test
        print("\n" + "=" * 50)
        print("MOTOR TEST")
        print("=" * 50)
        print("The motor will now run. Ensure it can spin freely.")
        print("Press Ctrl+C at any time to stop the test.")
        
        try:
            # First try a low speed
            self.run_motor(velocity=2.0, duration=3.0)
            
            # If successful, try reversing
            self.run_motor(velocity=-2.0, duration=3.0)
            
            # Then try the requested speed
            if abs(self.velocity) > 2.0:
                self.run_motor(velocity=self.velocity, duration=5.0)
        except KeyboardInterrupt:
            print("\nTest interrupted by user.")
        finally:
            self.stop()
            
        # Final status
        print("\n" + "=" * 50)
        print("TEST COMPLETE")
        print("=" * 50)
        print("Sensorless mode has been configured and tested.")
        print("\nTo use sensorless mode with your ROS 2 controller:")
        print("1. Start the ODrive in closed loop control:")
        print(f"   odrv0.axis{self.axis_num}.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL")
        print("2. Use input_vel to control velocity:")
        print(f"   odrv0.axis{self.axis_num}.controller.input_vel = <velocity>")
        print("\nImportant considerations for sensorless mode:")
        print("- The motor must be spinning at sufficient velocity (typically >0.5 turns/s)")
        print("- Sensorless mode cannot provide accurate position control")
        print("- Motor will fail to track very low velocities")
        
        return True
        
def main():
    # Parse arguments
    parser = argparse.ArgumentParser(description="ODrive Sensorless Setup")
    parser.add_argument('--axis', type=int, default=0, choices=[0, 1],
                        help="Motor axis (0 or 1)")
    parser.add_argument('--velocity', type=float, default=2.0,
                        help="Test velocity in turns/s")
    args = parser.parse_args()
    
    # Set up signal handler for Ctrl+C
    def signal_handler(sig, frame):
        print("\nExiting...")
        try:
            controller.stop()
        except:
            pass
        sys.exit(0)
        
    signal.signal(signal.SIGINT, signal_handler)
    
    # Create controller and run test
    controller = SensorlessODrive(args.axis, args.velocity)
    controller.test_sensorless()

if __name__ == "__main__":
    main()
