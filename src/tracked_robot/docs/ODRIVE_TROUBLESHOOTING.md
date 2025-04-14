# ODrive MY1020 1000W Motor Troubleshooting Guide

## Summary of Current Status
- Motor calibration is successful for both motors
- Hall encoder polarity calibration works correctly 
- Axis0 successfully enters sensorless mode but motor doesn't move
- Axis1 fails to enter sensorless mode with error 40
- Very low current readings during velocity commands
- No detected motor movement

## Hardware Checks

### 1. Power Supply Verification
- **Battery Voltage**: Confirm the 53V battery is properly connected and providing power to ODrive
- **Voltage under load**: Measure battery voltage while attempting motor operation (should remain >48V)
- **Power connections**: Check for loose power cables, oxidation, or poor contact points
- **Fuses**: Check if any fuses are blown in the power path

### 2. Motor Wiring Checks
- **Phase Connections**: Verify all three motor phase wires are properly connected for each motor
   - **A/B/C Phases**: Ensure correct phase sequence (swap any two phases if direction is reversed)
   - **Tighten Connections**: Check for loose or intermittent connections at the ODrive and motors
   - **Continuity Test**: Use a multimeter to check continuity between ODrive terminals and motor phases

- **Hall Sensor Wiring**:
   - **Hall A/B/C**: Verify hall sensor wires are properly connected
   - **Hall power**: Check 5V supply to hall sensors
   - **Hall ground**: Verify ground connection to hall sensors

### 3. Direct Motor Testing
```
# Test by directly applying 5A current (without sensorless mode)
odrv0.axis0.motor.config.current_lim = 5.0
odrv0.axis0.requested_state = AXIS_STATE_MOTOR_CALIBRATION
# Now try direct current
odrv0.axis0.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL
odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
# If successful, try:
odrv0.axis0.controller.input_torque = 0.5
# Look for shaft movement or current flowing
```

### 4. Mechanical Tests
- **Free Movement**: Disconnect motors from wheels/gears and check if shaft spins freely by hand
- **Shaft Binding**: Check if shaft is binding - should turn easily by hand
- **Gear Blockage**: Inspect for any mechanical blockage in power transmission
- **Shaft Coupling**: Verify shaft coupling isn't slipping if motors are connected via couplers

### 5. Specific MY1020 Motor Checks
- **Motor Specs Verification**: Confirm motors are truly MY1020 1000W 48V (instead of 36V variant)
- **Rated Current**: Verify if rated current of 20.8A is correct for your specific model
- **Pole Pairs**: Confirm if 8 pole pairs is correct (may be 7 in some variants)
- **Motor Resistance**: Measure phase-to-phase resistance with multimeter (should be around 0.77 ohms per phase for 48V)

## Software Adjustments to Try

If hardware checks pass, try these software modifications:

### 1. Increase Current Limit
```python
# Modify the ODrive setup script to use higher current
default_config['current_limit'] = 60.0  # Try 60A (3x rated current for startup)
```

### 2. Try Different Flux Linkage Value
```python
# In the try_sensorless_mode function:
axis.sensorless_estimator.config.pm_flux_linkage = 0.0025  # Try much lower value
# or
axis.sensorless_estimator.config.pm_flux_linkage = 0.025   # Try much higher value
```

### 3. Try Basic Motor Test via ODriveTool

```bash
# Start ODriveTool
odrivetool

# Configure axis
odrv0.axis0.motor.config.current_lim = 60.0
odrv0.axis0.motor.config.torque_constant = 0.168
odrv0.axis0.motor.config.pole_pairs = 8
odrv0.save_configuration()

# Calibrate motor
odrv0.axis0.requested_state = AXIS_STATE_MOTOR_CALIBRATION
# Check if successful:
odrv0.axis0.motor.error  # Should be 0

# Try direct CLOSED_LOOP_CONTROL with torque mode
odrv0.axis0.controller.config.control_mode = CONTROL_MODE_TORQUE_CONTROL
odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
# Apply torque (should spin motor or at least see current)
odrv0.axis0.controller.input_torque = 1.0
# Check current
odrv0.axis0.motor.current_control.Iq_measured
```

## Last Resort - Factory Reset

If all else fails, try a factory reset:

```bash
odrivetool
odrv0.erase_configuration()
# Then power cycle the ODrive and reconfigure
```

## Common Error Codes

- **Error 40 (Sensorless Estimator Failed)**: 
  - Incorrect motor parameters
  - Motor not properly connected
  - Insufficient current for startup
  - Wrong pole pairs setting

- **Error 1 (Motor Calibration)**:
  - Low calibration voltage
  - High resistance in wiring
  - Loose connections

Remember that hardware issues are the most likely cause if the software configuration seems correct but motors don't move. Start with the power and wiring checks first.
