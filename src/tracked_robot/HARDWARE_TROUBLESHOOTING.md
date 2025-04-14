# ODrive Hardware Troubleshooting Guide

Based on the persistent `AXIS_ERROR_MOTOR_FAILED` (0x801) error that couldn't be resolved with software, this guide focuses on checking the physical hardware setup.

## Quick Hardware Checklist

1. ✓ ODrive is powered and communicating via USB (confirmed)
2. ? 48V power supply connected correctly and functioning
3. ? Motor phase wires (U, V, W) connected correctly
4. ? Hall sensor connections are correct
5. ? Motor is physically able to turn (not obstructed)

## Step 1: Check Power Supply

The error 0x801 is often related to power supply issues:

1. **Confirm 48V is available**:
   - Verify voltage with a multimeter at the ODrive DC input terminals
   - Should read 48V ± 2V
   - Check for loose connections

2. **Check power supply capacity**:
   - For 1000W motors, the power supply should be rated at least 20A at 48V
   - Insufficient power can cause motor failures during startup/calibration
   - The ODrive needs 150W minimum just for calibration

## Step 2: Check Motor Wiring

Incorrect motor wiring is a common cause of the 0x801 error:

1. **Phase Wires (U, V, W)**:
   - Ensure all three phase wires are securely connected
   - Try swapping any two phase wires (e.g., swap U and V) as this can solve some calibration issues
   - Check for damaged insulation or shorts between phases

2. **Resistance Check**:
   - With power OFF, measure resistance between any two motor phases
   - Should be low (typically 0.1-2.0 ohms) but not zero
   - All phase-to-phase measurements should be approximately equal

## Step 3: Hall Sensor Checks

1. **Verify Hall Sensor Connections**:
   - Confirm all Hall sensor wires are connected (typically 5 wires: power, ground, and three signals)
   - Ensure cable shielding is properly grounded
   - Check for bent pins in connectors

2. **Test Hall Sensor Functionality**:
   - Run this command to see if Hall sensors are responding:
     ```
     python3 -c "import odrive; odrv = odrive.find_any(); print(f'Hall state: {odrv.axis0.encoder.hall_state}')"
     ```
   - Rotate the motor shaft by hand slowly
   - Hall state should change between values 1-6 as the motor rotates
   - If Hall state doesn't change, the sensors may be defective

## Step 4: Motor Physical Checks

1. **Rotation Test**:
   - With the ODrive powered off, rotate the motor by hand
   - The motor should turn smoothly without unusual resistance
   - Listen for grinding or clicking sounds
   
2. **Visual Inspection**:
   - Check for obvious physical damage
   - Look for signs of overheating or burnt components
   - Inspect ODrive board for damaged components

## Step 5: ODrive Board Checks

1. **Reset to Factory Default**:
   ```python
   import odrive
   odrv = odrive.find_any()
   odrv.erase_configuration()
   # Device will reboot
   ```

2. **Check ODrive Temperature**:
   - ODrive may shut down if overheating
   - Ensure adequate cooling for the ODrive
   - Check if the heatsink is hot to the touch

## Step 6: Try a Different Motor

If available, connect a different motor to the same ODrive axis:
- If the new motor works, the original motor may be damaged
- If the new motor also fails, the issue is likely with the ODrive or power supply

## Step 7: Firmware Version Check

Some older ODrive firmware versions have known issues:
```python
import odrive
odrv = odrive.find_any()
print(f"Firmware version: {odrv.fw_version_major}.{odrv.fw_version_minor}.{odrv.fw_version_revision}")
```

If you're not on the latest version, consider updating.

## Advanced Troubleshooting

1. **Minimal Configuration Test**:
   Try with absolute minimum configuration:
   ```python
   import odrive
   odrv = odrive.find_any()
   odrv.axis0.motor.config.motor_type = 0  # HIGH_CURRENT
   odrv.axis0.motor.config.current_lim = 10
   odrv.axis0.motor.config.calibration_current = 5
   odrv.save_configuration()
   # After reboot
   odrv = odrive.find_any()
   odrv.axis0.requested_state = 4  # MOTOR_CALIBRATION
   ```

2. **Logging Motor Data During Calibration Attempt**:
   ```python
   import odrive
   import time
   
   odrv = odrive.find_any()
   print(f"Starting calibration: Error={odrv.axis0.error}")
   odrv.axis0.requested_state = 4  # MOTOR_CALIBRATION
   
   # Monitor during calibration
   for i in range(20):
       try:
           cal_current = odrv.axis0.motor.current_control.Iq_measured
           state = odrv.axis0.current_state
           error = odrv.axis0.error
           motor_error = odrv.axis0.motor.error
           print(f"State={state}, Current={cal_current}, Error=0x{error:X}, Motor Error=0x{motor_error:X}")
       except:
           print("Communication lost")
       time.sleep(0.2)
   ```

## Specific Error 0x801 Information

The error 0x801 (AXIS_ERROR_MOTOR_FAILED) specifically indicates:

1. **Phase Resistance Check Failed**: Motor phase resistance was outside expected range
2. **Current Measurement Failure**: Failed to reach target current during calibration
3. **Hardware Protection**: ODrive's protection circuits activated
4. **Motor Type Mismatch**: The specified motor type may not match your actual motor

## Additional Hardware Resources

- [ODrive Hardware Reference](https://docs.odriverobotics.com/v/latest/hardware-reference.html)
- [Brushless Motor Basics](https://docs.odriverobotics.com/v/latest/guides/getting-started.html#choosing-a-motor)
- [Community Forum Discussions](https://discourse.odriverobotics.com/search?q=AXIS_ERROR_MOTOR_FAILED%20order%3Alatest)
