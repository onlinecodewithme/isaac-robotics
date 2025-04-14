#!/usr/bin/env python3

"""
ODrive Hardware Diagnostic Tool

This script provides a series of targeted hardware tests
to diagnose why motor calibration is failing with Error 0x801.
It focuses on checking power supply, motor phases, and other hardware.

Usage:
    python3 hardware_diagnostics.py [--axis 0|1]
"""

import sys
import time
import argparse
import numpy as np
import logging
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
logger = logging.getLogger("hardware_diagnostics")

class HardwareDiagnostics:
    def __init__(self, axis_num=0):
        self.axis_num = axis_num
        self.odrv = None
        self.axis = None
        self.connected = False
        self.motor_data = {}
        
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
    
    def get_version_info(self):
        """Get firmware and hardware version info"""
        logger.info("Checking ODrive version info...")
        
        try:
            fw_major = self.odrv.fw_version_major
            fw_minor = self.odrv.fw_version_minor
            fw_revision = self.odrv.fw_version_revision
            hw_version = self.odrv.hw_version if hasattr(self.odrv, 'hw_version') else "unknown"
            
            logger.info(f"Firmware version: {fw_major}.{fw_minor}.{fw_revision}")
            logger.info(f"Hardware version: {hw_version}")
            
            # Store in results
            self.motor_data['fw_version'] = f"{fw_major}.{fw_minor}.{fw_revision}"
            self.motor_data['hw_version'] = hw_version
            
            return fw_major, fw_minor, fw_revision
        except Exception as e:
            logger.error(f"Error getting version info: {str(e)}")
            return None, None, None
    
    def check_dc_voltage(self):
        """Check DC bus voltage"""
        logger.info("Checking DC bus voltage...")
        
        try:
            vbus_voltage = self.odrv.vbus_voltage
            logger.info(f"DC bus voltage: {vbus_voltage:.1f}V")
            
            # Store in results
            self.motor_data['vbus_voltage'] = vbus_voltage
            
            # Analyze
            if vbus_voltage < 10.0:
                logger.error("! DC voltage too low. Power supply not connected?")
            elif vbus_voltage < 40.0:
                logger.warning("! DC voltage below 40V. Recommended 48V for 1000W motors.")
            elif vbus_voltage > 56.0:
                logger.warning("! DC voltage high (>56V). Check power supply.")
            else:
                logger.info("✓ DC voltage in good range for 48V system")
                
            return vbus_voltage
        except Exception as e:
            logger.error(f"Error reading DC voltage: {str(e)}")
            return None
    
    def measure_current(self, duration=2.0):
        """Measure actual current flowing through motor"""
        logger.info("Measuring current control capability...")
        
        # First, make sure in idle state
        try:
            self.axis.requested_state = AXIS_STATE_IDLE
            time.sleep(0.2)
            
            # Clear errors
            if hasattr(self.axis, 'error'):
                self.axis.error = 0
            if hasattr(self.axis.motor, 'error'):
                self.axis.motor.error = 0
            
            # Set current control mode
            self.axis.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL
            self.axis.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
            
            # Set calibration current to low value
            orig_calib_current = self.axis.motor.config.calibration_current
            orig_current_lim = self.axis.motor.config.current_lim
            
            # Safer values to start
            self.axis.motor.config.current_lim = 5.0
            logger.info(f"Set current limit to 5.0A for testing")
            
            # Try to enter closed loop control without calibration
            # This may fail due to encoder not ready, which is expected
            self.axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            time.sleep(0.5)
            
            current_measurements = []
            commanded_current = None
            max_measured_current = 0.0
            actual_state = self.axis.current_state
            
            # Execute based on state
            if actual_state == AXIS_STATE_CLOSED_LOOP_CONTROL:
                logger.info("Entered closed loop control - applying test current")
                
                # Try with very small current for safety
                commanded_current = 1.0
                self.axis.controller.input_torque = commanded_current
                
                start_time = time.time()
                while time.time() - start_time < duration:
                    try:
                        if hasattr(self.axis.motor.current_control, 'Iq_measured'):
                            current = self.axis.motor.current_control.Iq_measured
                            current_measurements.append(current)
                            max_measured_current = max(max_measured_current, abs(current))
                            logger.info(f"Current reading: {current:.2f}A")
                    except:
                        pass
                    time.sleep(0.1)
                
                # Stop the current
                self.axis.controller.input_torque = 0.0
                time.sleep(0.2)
                logger.info("Current test complete")
                
            else:
                # We couldn't enter closed loop control - try to read current during calibration
                logger.info("Could not enter closed loop control - checking current during calibration")
                logger.info("Starting motor calibration for current measurement...")
                self.axis.requested_state = AXIS_STATE_MOTOR_CALIBRATION
                
                start_time = time.time()
                while time.time() - start_time < min(5.0, duration * 2):
                    try:
                        if hasattr(self.axis.motor.current_control, 'Iq_measured'):
                            current = self.axis.motor.current_control.Iq_measured
                            current_measurements.append(current)
                            max_measured_current = max(max_measured_current, abs(current))
                            logger.info(f"Calibration current: {current:.2f}A")
                    except Exception as e:
                        logger.error(f"Error reading current: {str(e)}")
                    time.sleep(0.1)
            
            # Restore original values
            self.axis.motor.config.calibration_current = orig_calib_current
            self.axis.motor.config.current_lim = orig_current_lim
            self.axis.requested_state = AXIS_STATE_IDLE
            
            # Analyze results
            self.motor_data['current_test'] = {
                'commanded_current': commanded_current,
                'max_measured_current': max_measured_current,
                'measurements': current_measurements
            }
            
            if len(current_measurements) == 0:
                logger.error("! No current measurements obtained")
                logger.error("! This indicates a serious hardware problem with current sensing or motor phases")
                return False
            
            if commanded_current is not None:
                # If we were able to command current
                if max_measured_current < 0.5 * commanded_current:
                    logger.error(f"! Very low current measurement. Expected ~{commanded_current}A, measured max {max_measured_current:.2f}A")
                    logger.error("! Check motor phase connections and power supply")
                    return False
                else:
                    logger.info(f"✓ Motor can produce current: commanded {commanded_current}A, measured max {max_measured_current:.2f}A")
                    return True
            else:
                # If we only did calibration
                if max_measured_current < 0.5:
                    logger.error(f"! Very low current during calibration: max {max_measured_current:.2f}A")
                    logger.error("! Check motor phase connections and power supply")
                    return False
                else:
                    logger.info(f"✓ Motor shows current during calibration: max {max_measured_current:.2f}A")
                    return True
            
        except Exception as e:
            logger.error(f"Error in current test: {str(e)}")
            return False
        finally:
            # Safety: try to return to idle
            try:
                self.axis.requested_state = AXIS_STATE_IDLE
            except:
                pass
    
    def check_error_code(self):
        """Analyze the specific error code"""
        logger.info("Analyzing error codes...")
        
        try:
            axis_error = self.axis.error if hasattr(self.axis, 'error') else 0
            motor_error = self.axis.motor.error if hasattr(self.axis.motor, 'error') else 0
            encoder_error = self.axis.encoder.error if hasattr(self.axis.encoder, 'error') else 0
            controller_error = self.axis.controller.error if hasattr(self.axis.controller, 'error') else 0
            
            logger.info(f"Axis error: 0x{axis_error:X}")
            logger.info(f"Motor error: 0x{motor_error:X}")
            logger.info(f"Encoder error: 0x{encoder_error:X}")
            logger.info(f"Controller error: 0x{controller_error:X}")
            
            # Store in results
            self.motor_data['errors'] = {
                'axis': axis_error,
                'motor': motor_error,
                'encoder': encoder_error,
                'controller': controller_error
            }
            
            # Specific analysis of error 0x801 (AXIS_ERROR_MOTOR_FAILED)
            if axis_error == 0x801:
                logger.error("! Error 0x801 (AXIS_ERROR_MOTOR_FAILED) detected")
                logger.error("  This error indicates a hardware issue with the motor or its connections")
                
                if motor_error == 0x1:
                    logger.error("  Motor error 0x1: MOTOR_ERROR_PHASE_RESISTANCE_OUT_OF_RANGE")
                    logger.error("  Likely causes:")
                    logger.error("  1. One or more motor phase wires are disconnected")
                    logger.error("  2. Poor connection at the ODrive terminal blocks")
                    logger.error("  3. Motor windings have incorrect resistance for your configuration")
                    
                elif motor_error == 0x2:
                    logger.error("  Motor error 0x2: MOTOR_ERROR_PHASE_INDUCTANCE_OUT_OF_RANGE")
                    logger.error("  Likely causes:")
                    logger.error("  1. Motor inductance doesn't match configuration")
                    logger.error("  2. Try manually setting phase_inductance and phase_resistance")
                    
                elif motor_error == 0x4:
                    logger.error("  Motor error 0x4: MOTOR_ERROR_DRV_FAULT")
                    logger.error("  This indicates the motor driver chip detected a problem:")
                    logger.error("  1. Short circuit between phases or phase to ground")
                    logger.error("  2. Damaged ODrive board")
                    logger.error("  3. Motor phase wires have damaged insulation")
                    
                elif motor_error == 0x8:
                    logger.error("  Motor error 0x8: MOTOR_ERROR_CONTROL_DEADLINE_MISSED")
                    logger.error("  This indicates a firmware timing issue. Try updating firmware.")
                
                elif motor_error == 0x0:
                    logger.error("  No specific motor error reported with AXIS_ERROR_MOTOR_FAILED")
                    logger.error("  This could indicate an issue with current measurement or power supply")
                    
                else:
                    logger.error(f"  Unknown motor error code: 0x{motor_error:X}")
                
            return axis_error
            
        except Exception as e:
            logger.error(f"Error analyzing error codes: {str(e)}")
            return None
    
    def test_hall_sensors(self, max_wait=15):
        """Test Hall sensor functionality"""
        logger.info("Testing Hall sensors")
        
        hall_states = set()
        start_time = time.time()
        initial_state = None
        max_consecutive_same = 0
        consecutive_same = 0
        
        print("\nPlease rotate the motor shaft by hand during this test.")
        print("Press Ctrl+C when finished or wait for timeout.")
        
        try:
            # Collect hall states while user rotates motor
            prev_state = None
            
            while time.time() - start_time < max_wait:
                try:
                    hall_state = self.axis.encoder.hall_state
                    hall_states.add(hall_state)
                    
                    if initial_state is None:
                        initial_state = hall_state
                    
                    if prev_state == hall_state:
                        consecutive_same += 1
                    else:
                        max_consecutive_same = max(max_consecutive_same, consecutive_same)
                        consecutive_same = 1
                        prev_state = hall_state
                    
                    sys.stdout.write(f"\rHall state: {hall_state}   ")
                    sys.stdout.flush()
                    time.sleep(0.2)
                except KeyboardInterrupt:
                    break
                except Exception as e:
                    logger.error(f"Error reading hall state: {str(e)}")
                    break
            
            print("\n")  # New line after readings
            
            # Store results
            self.motor_data['hall_test'] = {
                'states_seen': sorted(list(hall_states)),
                'max_consecutive_same': max_consecutive_same
            }
            
            # Analyze results
            if len(hall_states) == 0:
                logger.error("! No Hall sensor states could be read")
                logger.error("! Check Hall sensor connections")
                return False
            elif len(hall_states) < 3:
                logger.warning(f"! Only {len(hall_states)} Hall states detected: {sorted(list(hall_states))}")
                logger.warning("! Hall sensors may not be working correctly")
                return False
            elif len(hall_states) < 6:
                logger.warning(f"! Only {len(hall_states)} Hall states detected: {sorted(list(hall_states))}")
                logger.warning("! Some Hall sensors may not be working correctly")
                return True
            else:
                logger.info(f"✓ All 6 Hall states detected: {sorted(list(hall_states))}")
                return True
            
        except Exception as e:
            logger.error(f"Error in Hall sensor test: {str(e)}")
            return False
    
    def measure_motor_resistance(self):
        """Try to measure motor resistance manually"""
        logger.info("Measuring motor phase resistance...")
        
        try:
            # First check if we can read calibration settings
            if hasattr(self.axis.motor, 'config'):
                logger.info("Current motor configuration:")
                
                if hasattr(self.axis.motor.config, 'phase_resistance'):
                    phase_resistance = self.axis.motor.config.phase_resistance
                    logger.info(f"Configured phase resistance: {phase_resistance:.6f} ohms")
                
                if hasattr(self.axis.motor.config, 'phase_inductance'):
                    phase_inductance = self.axis.motor.config.phase_inductance
                    logger.info(f"Configured phase inductance: {phase_inductance:.6f} H")
            
            # Try to measure resistance using a more careful calibration
            logger.info("Attempting careful phase resistance measurement...")
            
            # Set to idle first
            self.axis.requested_state = AXIS_STATE_IDLE
            time.sleep(0.2)
            
            # Clear errors
            if hasattr(self.axis, 'error'):
                self.axis.error = 0
            if hasattr(self.axis.motor, 'error'):
                self.axis.motor.error = 0
            
            # Reduce max calibration voltage to be safer
            if hasattr(self.axis.motor.config, 'resistance_calib_max_voltage'):
                orig_voltage = self.axis.motor.config.resistance_calib_max_voltage
                self.axis.motor.config.resistance_calib_max_voltage = 2.0
                logger.info("Set resistance_calib_max_voltage = 2.0V")
            
            # Check if we can measure directly
            can_measure = False
            measured_resistance = None
            
            # We have to use calibration to measure, but only up to the resistance part
            logger.warning("Starting partial calibration - motor will make a sound")
            
            self.axis.requested_state = AXIS_STATE_MOTOR_CALIBRATION
            
            # Wait briefly for resistance measurement
            states_sequence = []
            time.sleep(0.5)  # Wait a bit for resistance measurement to start
            
            # Check if we can get resistance from the partial calibration
            resistance_measured = False
            
            # Save the resistance if we see it
            start_time = time.time()
            while time.time() - start_time < 5.0:  # Don't wait too long
                try:
                    # Try to grab the resistance if we can
                    if hasattr(self.axis.motor, 'phase_resistance'):
                        measured_resistance = self.axis.motor.phase_resistance
                        if measured_resistance > 0:
                            logger.info(f"Measured phase resistance: {measured_resistance:.6f} ohms")
                            resistance_measured = True
                            break
                except:
                    pass
                    
                # If we're out of calibration, stop waiting
                try:
                    current_state = self.axis.current_state
                    if current_state != AXIS_STATE_MOTOR_CALIBRATION:
                        break
                except:
                    pass
                
                time.sleep(0.2)
            
            # Restore original voltage limit
            try:
                if hasattr(self.axis.motor.config, 'resistance_calib_max_voltage'):
                    self.axis.motor.config.resistance_calib_max_voltage = orig_voltage
            except:
                pass
            
            # Try to go back to idle state
            try:
                self.axis.requested_state = AXIS_STATE_IDLE
            except:
                pass
            
            # Store measurement
            self.motor_data['resistance_test'] = {
                'measured_resistance': measured_resistance,
                'resistance_measured': resistance_measured
            }
            
            # Analyze results
            if not resistance_measured or measured_resistance is None:
                logger.error("! Failed to measure phase resistance")
                logger.error("! This indicates a problem with motor wiring or power supply")
                return False
            elif measured_resistance < 0.01:
                logger.error(f"! Measured resistance very low: {measured_resistance:.6f} ohms")
                logger.error("! Possible short circuit between motor phases")
                return False
            elif measured_resistance > 5.0:
                logger.error(f"! Measured resistance very high: {measured_resistance:.6f} ohms")
                logger.error("! Possible open circuit or disconnected motor phase")
                return False
            else:
                logger.info(f"✓ Phase resistance in reasonable range: {measured_resistance:.6f} ohms")
                return True
            
        except Exception as e:
            logger.error(f"Error in resistance measurement: {str(e)}")
            return False
        finally:
            # Safety: try to return to idle
            try:
                self.axis.requested_state = AXIS_STATE_IDLE
            except:
                pass

    def run_all_tests(self):
        """Run all hardware diagnostic tests"""
        print("\n" + "=" * 60)
        print("  ODRIVE HARDWARE DIAGNOSTICS")
        print("=" * 60)
        
        if not self.connect():
            print("Failed to connect to ODrive. Exiting.")
            return False
        
        results = {}
        
        # Check firmware version
        print("\n" + "=" * 50)
        print("SYSTEM INFORMATION")
        print("=" * 50)
        fw_major, fw_minor, fw_revision = self.get_version_info()
        
        # Check DC voltage
        print("\n" + "=" * 50)
        print("POWER SUPPLY TEST")
        print("=" * 50)
        vbus_voltage = self.check_dc_voltage()
        results['power_supply'] = "PASS" if vbus_voltage and vbus_voltage >= 40.0 else "FAIL"
        
        # Test error codes
        print("\n" + "=" * 50)
        print("ERROR CODE ANALYSIS")
        print("=" * 50)
        error_code = self.check_error_code()
        results['error_code'] = "Pass" if error_code == 0 else "FAIL"
        
        # Test Hall sensors
        print("\n" + "=" * 50)
        print("HALL SENSOR TEST")
        print("=" * 50)
        hall_ok = self.test_hall_sensors()
        results['hall_sensors'] = "PASS" if hall_ok else "FAIL"
        
        # Test motor resistance
        print("\n" + "=" * 50)
        print("MOTOR RESISTANCE TEST")
        print("=" * 50)
        resistance_ok = self.measure_motor_resistance()
        results['resistance'] = "PASS" if resistance_ok else "FAIL"
        
        # Test current control
        print("\n" + "=" * 50)
        print("CURRENT CONTROL TEST")
        print("=" * 50)
        current_ok = self.measure_current()
        results['current'] = "PASS" if current_ok else "FAIL"
        
        # Summarize results
        print("\n" + "=" * 50)
        print("DIAGNOSTIC SUMMARY")
        print("=" * 50)
        
        # Store in results
        self.motor_data['test_results'] = results
        
        print(f"Power Supply Test: {results['power_supply']}")
        print(f"Error Code Analysis: {results['error_code']}")
        print(f"Hall Sensor Test: {results['hall_sensors']}")
        print(f"Motor Resistance Test: {results['resistance']}")
        print(f"Current Control Test: {results['current']}")
        
        # Calculate overall assessment
        critical_failures = 0
        for test, result in results.items():
            if result != "PASS":
                if test in ['power_supply', 'resistance', 'current']:
                    critical_failures += 1
        
        if critical_failures > 0:
            print("\nDIAGNOSIS: HARDWARE ISSUE DETECTED")
            print("Several critical tests failed. Please check:")
            
            if results['power_supply'] != "PASS":
                print("- Verify 48V power supply is properly connected and can deliver 20A+")
            
            if results['resistance'] != "PASS":
                print("- Check motor phase wires (U,V,W) for proper connections")
                print("- Look for loose connections at the terminal blocks")
                print("- Try swapping any two phase wires to rule out incorrect wiring")
            
            if results['current'] != "PASS":
                print("- Ensure ODrive has adequate cooling")
                print("- Check for any visible damage to the ODrive board")
            
            if results['hall_sensors'] != "PASS":
                print("- Inspect Hall sensor connections")
                print("- Check that the encoder cable is not damaged")
            
            print("\nNEXT STEPS:")
            print("1. Fix the identified hardware issues")
            print("2. Re-run this diagnostic")
            print("3. Once all tests pass, try the all_in_one_calibration.py script again")
            
        else:
            print("\nDIAGNOSIS: CONFIGURATION ISSUE")
            print("Hardware appears to be working, but calibration still fails.")
            print("This suggests a configuration mismatch with your specific motor.")
            
            print("\nRECOMMENDED ACTION:")
            print("Try setting manual motor parameters:")
            print("1. Get your specific motor's parameters from the datasheet")
            print("2. Configure them explicitly in the ODrive:")
            print("   - Motor pole pairs")
            print("   - Phase resistance")
            print("   - Phase inductance")
            print("   - Current limits")
            print("3. Use the all_in_one_calibration.py script with the --manual option")
        
        return results
        
def main():
    parser = argparse.ArgumentParser(description="ODrive Hardware Diagnostics")
    parser.add_argument('--axis', type=int, default=0, choices=[0, 1],
                      help="Motor axis to test (0 or 1)")
    args = parser.parse_args()
    
    diagnostics = HardwareDiagnostics(args.axis)
    diagnostics.run_all_tests()

if __name__ == "__main__":
    main()
