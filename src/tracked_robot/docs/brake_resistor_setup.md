# Brake Resistor Configuration for ODrive Controller

This document provides instructions on how to configure and connect the brake resistor for your ODrive controller used with 1kW BLDC motors in the tracked robot.

## What is a Brake Resistor?

A brake resistor is a power resistor used to dissipate energy during regenerative braking. When motors slow down or reverse direction, they can generate electricity that flows back into the controller. This can cause voltage spikes that might damage the ODrive. The brake resistor safely converts this excess energy into heat.

## Hardware Specifications

Based on the image, you have:
- A gold aluminum-housed power resistor (likely 50W or 100W)
- Black connecting wires attached to both ends
- Likely resistance value between 0.5-2.0 ohms

## Physical Connection

1. Connect the brake resistor to the ODrive's brake terminals:
   - Locate the "BR" and "B-" terminals on your ODrive
   - Connect the black wires from the resistor to these terminals
   - Ensure the connections are secure

2. Mount considerations:
   - The resistor will get hot during operation (can exceed 100°C)
   - Mount it to a heat sink or metal surface if possible
   - Keep it away from plastic components and wiring
   - Ensure adequate airflow around the resistor

## Software Configuration

The ODrive setup script (`odrive_setup.py`) has been updated to handle various ODrive firmware versions and their different configuration attributes. The script will try these configuration paths:

```python
if hasattr(odrv.config, 'brake_resistance'):
    odrv.config.brake_resistance = config['brake_resistance']
    
if hasattr(odrv.config, 'enable_brake_resistor'):
    odrv.config.enable_brake_resistor = True
elif hasattr(odrv, 'config_brake_resistance'):
    odrv.config_brake_resistance = config['brake_resistance']
elif hasattr(odrv.config, 'brake_resistor_enabled'):
    odrv.config.brake_resistor_enabled = True
```

## Configuring Your Specific Resistor

1. Check the markings on your resistor for its resistance value (in ohms)
2. Update the configuration value in `odrive_setup.py`:
   - Find the `default_config` dictionary
   - Update the `'brake_resistance'` value to match your resistor:
     ```python
     'brake_resistance': 0.5,  # Replace with your actual resistor value
     ```

## Testing the Configuration

After connecting the resistor and updating the configuration:

1. Run the ODrive setup script:
   ```bash
   cd /home/x4/isaac_xavier
   ./src/tracked_robot/scripts/odrive_setup.py
   ```

2. Verify the configuration was applied:
   ```bash
   python3 -c "import odrive; odrv = odrive.find_any(); print('Brake resistor enabled:', getattr(odrv.config, 'enable_brake_resistor', None))"
   ```

3. During testing with the robot:
   - Monitor the bus voltage during deceleration
   - Check if the resistor gets warm (indicates it's working)
   - If you see voltage spikes over 52V, the resistor might not be configured correctly

## Troubleshooting

If you encounter issues:

1. **Resistor not getting warm during braking**:
   - Verify the software configuration
   - Check connections to the ODrive
   - Ensure resistance value is correctly set

2. **Voltage spikes during deceleration**:
   - Resistor value might be too high
   - Connections might be loose
   - Brake resistor might not be enabled in software

3. **ODrive errors related to overvoltage**:
   - Check error codes using: `python3 -c "import odrive; odrv = odrive.find_any(); print(odrv.error)"`
   - Error 1 indicates DC bus overvoltage - brake resistor issue

## Safety Notes

- The brake resistor can get VERY hot during operation
- Ensure it's not touching any flammable materials
- Consider adding a thermal protection if operating the robot under heavy loads
