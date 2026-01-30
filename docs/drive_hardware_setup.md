# Hardware Setup Guide for Drive System

## Overview

This guide covers the setup and configuration of the drive system for both Jetson Orin Nano Super and Jetson Nano Dev Kits using BILDA 2x40A Motor Controllers and Saturn 5304 Series Planetary Gear Motors.

## Hardware Specifications

### Jetson Boards
- **Primary**: NVIDIA Jetson Orin Nano Super Dev Kit
- **Secondary**: NVIDIA Jetson Nano Dev Kit

### Motor Controllers
- **Type**: BILDA 2x40A Motor Controller (H-Bridge)
- **Quantity**: 2 (one for left side, one for right side)
- **Input**: PWM signals on Jetson pins
- **Output**: 2x 40A motor channels per controller

### Motors
- **Type**: Saturn 5304 Series Planetary Gear Motor
- **Quantity**: 6 total (3 per side)
  - Left Front (LF)
  - Left Middle (LM)
  - Left Back (LB)
  - Right Front (RF)
  - Right Middle (RM)
  - Right Back (RB)

### PWM Interface
- **Protocol**: Linux sysfs PWM (`/sys/class/pwm/`)
- **Frequency**: 50 kHz (configurable in drive_config.yaml)
- **Voltage**: 3.3V logic (compatible with Jetson output)

## PWM Pin Configuration

### Jetson Orin Nano
```
PWM Chip 0: J14 Header
  - Channel 0: Pin 32 (GPIO8 / PWM0)
  - Channel 1: Pin 33 (GPIO11 / PWM1)

PWM Chip 1: J14 Header  
  - Channel 0: Pin 12 (GPIO18 / PWM2)
  - Channel 1: Pin 13 (GPIO19 / PWM3)
```

### Jetson Nano
```
PWM Chip 0: J21 Header
  - Channel 0: Pin 32 (GPIO8 / PWM0)
  - Channel 1: Pin 33 (GPIO11 / PWM1)

Note: Jetson Nano may have limited PWM channels. Check available:
  ls /sys/class/pwm/
```

## Current Motor Configuration

### Motor Wiring (BILDA Controller Channels)

**Left Motor Controller (PWM Chip 0)**
- Channel A: Motors LF + LM (paired, controlled via PWM0)
- Channel B: Motor LB (single, controlled via PWM1)

**Right Motor Controller (PWM Chip 1)**
- Channel A: Motors RF + RM (paired, controlled via PWM2)
- Channel B: Motor RB (single, controlled via PWM3)

## Quick Start Checklist

Follow this **in order** for fastest setup:

- [ ] **Step 1: Physical Wiring**
  - [ ] Connect motors to BILDA controllers (see section below)
  - [ ] Connect power to controllers
  - [ ] Connect Jetson GND to controller GND

- [ ] **Step 2: Configure PWM in Device Tree**
  ```bash
  sudo /opt/nvidia/jetson-io.py
  # Select PWM for desired pins, apply, reboot
  ```

- [ ] **Step 3: Verify PWM Devices**
  ```bash
  ls /sys/class/pwm/
  cat /sys/class/pwm/pwmchip0/npwm  # Check channels
  cat /sys/class/pwm/pwmchip1/npwm
  ```

- [ ] **Step 4: Update Configuration**
  - Edit `config/drive/drive_config.yaml`
  - Set `chip:` and `channel:` based on output above

- [ ] **Step 5: Test with Simulation**
  ```bash
  ros2 run drive drive_controller_sim
  ros2 topic echo /drive/sim_status
  ```

- [ ] **Step 6: Test with Hardware**
  ```bash
  sudo ros2 run drive drive_controller_node
  ros2 topic pub /drive/cmd_vel geometry_msgs/Twist "{linear: {x: 0.5}}"
  ```

## Detailed Setup Sections

1. **Power Distribution**
   - Connect battery positive to BILDA controller VCC (positive terminals)
   - Connect battery negative to BILDA controller GND (negative terminals)
   - Share ground between Jetson and controllers

2. **Motor Connections (Left Controller)**
   ```
   LF + LM motors -> Channel A (positive/negative terminals)
   LB motor -> Channel B (positive/negative terminals)
   ```

3. **Motor Connections (Right Controller)**
   ```
   RF + RM motors -> Channel A (positive/negative terminals)
   RB motor -> Channel B (positive/negative terminals)
   ```

4. **PWM Control Connections (3.3V Logic)**
   ```
   Jetson Pin 32 (PWM0) -> Left Controller PWM A input
   Jetson Pin 33 (PWM1) -> Left Controller PWM B input
   Jetson Pin 12 (PWM2) -> Right Controller PWM A input
   Jetson Pin 13 (PWM3) -> Right Controller PWM B input
   Jetson GND (Pin 6, 9, 14, 20, 25, 30, 34, 39) -> Controller GND
   ```

### 2. Enable PWM in Device Tree (if needed)

For Jetson Nano and Orin Nano, PWM pins must be configured in the device tree before they appear in sysfs.

**Step-by-step**:

1. **Run Jetson Pin Configuration Tool**
   ```bash
   sudo /opt/nvidia/jetson-io/jetson-io.py
   ```

2. **Configure PWM Pins**
   - Navigate to "Configure pins for a specific use case"
   - Select "PWM" for the pins you want to use (e.g., pins 32, 33, 12, 13)
   - Apply the configuration
   - The tool will show which PWM chips will be created

3. **Reboot for Changes to Take Effect**
   ```bash
   sudo reboot
   ```

4. **Verify PWM Devices Now Exist**
   ```bash
   ls /sys/class/pwm/
   # Output should include: pwmchip0, pwmchip1, etc.
   ```

5. **Check Number of Channels per Chip**
   ```bash
   cat /sys/class/pwm/pwmchip0/npwm
   cat /sys/class/pwm/pwmchip1/npwm
   # Should show: 1, 2, or 4 depending on chip
   ```

6. **Update drive_config.yaml with Correct Chip/Channel Numbers**
   - The "chip" number matches the pwmchipX number
   - The "channel" is 0 to (npwm-1)
   - Example: if pwmchip0 has 2 channels, use chip: 0, channel: 0 or 1

**Important**: The chip numbers in `drive_config.yaml` are the logical PWM device numbers from sysfs, NOT the physical pin numbers.

### 3. Software Configuration

1. **Update drive_config.yaml**
   ```yaml
   drive:
     pwm_channels:
       motor_left_front_middle:
         chip: 0
         channel: 0
       motor_left_back:
         chip: 0
         channel: 1
       motor_right_front_middle:
         chip: 1
         channel: 0
       motor_right_back:
         chip: 1
         channel: 1
   ```

2. **Verify PWM Paths**
   ```bash
   # SSH into Jetson and check PWM sysfs
   ls -la /sys/class/pwm/pwmchip0/
   ls -la /sys/class/pwm/pwmchip1/
   ```

## Testing

### 1. Hardware Check (without ROS2)

```bash
# SSH into Jetson
ssh ubuntu@jetson-hostname

# Check available PWM devices
ls /sys/class/pwm/

# Check PWM chip details
cat /sys/class/pwm/pwmchip0/npwm  # Should show number of PWM channels
```

### 2. Simulation Testing (no hardware needed)

```bash
# Build the project
cd ~/2026-Rover-Code/ros2_ws
colcon build --symlink-install

# Source setup
source install/setup.bash

# Run simulation
ros2 run drive drive_controller_sim
```

Monitor output:
```bash
# In another terminal, watch the simulated motor states
ros2 topic echo /drive/sim_status
```

### 3. Hardware Testing (with actual motors)

```bash
# Run actual controller
sudo ros2 run drive drive_controller_node

# In another terminal, send test commands
ros2 topic pub /drive/cmd_vel geometry_msgs/Twist "{linear: {x: 0.5}}"
```

### 4. Test Node

```bash
# Run automated test sequence
ros2 run drive drive_test_node
```

## Troubleshooting

### PWM Paths Don't Exist

**Symptom**: "Could not export PWM channel" warnings

**Solutions**:
1. Verify PWM is enabled in device tree
2. Check actual chip numbers:
   ```bash
   ls /sys/class/pwm/
   ```
3. Update `drive_config.yaml` with correct chip numbers
4. For Nano, you might only have `pwmchip0`

### Motors Not Responding

**Check List**:
1. Verify power connections to BILDA controller
2. Test PWM output with oscilloscope (should see 50 kHz square wave)
3. Verify motor controller jumpers/switches are configured correctly
4. Check motor wiring to controller terminals
5. Test controllers manually with direct PWM signal

### Permission Denied on /sys/class/pwm/

**Solutions**:
```bash
# Option 1: Run with sudo
sudo ros2 run drive drive_controller_node

# Option 2: Configure udev rules (persistent solution)
sudo usermod -a -G gpio $(whoami)
```

### Asymmetric Motor Speeds

If left and right sides move at different speeds:
1. Use `front_middle_gain` and `back_gain` in config to balance
2. Check motor wiring polarity (if needed, swap motor leads)
3. Verify no mechanical obstructions

## Safety Considerations

1. **Always test with propeller guards** or in a contained area
2. **Verify motor direction** before running full speed
3. **Keep hands/objects away** from spinning motors
4. **Use emergency stop procedure** - command timeout is 1.0 second (check config)
5. **Test on simulation first** before hardware deployment

## Performance Tuning

### Control Parameters (in drive_config.yaml)

- **acceleration**: 0.1 (default) - increase for faster response, decrease for smoother acceleration
- **deadzone**: 0.05 (default) - increase if motors won't stop completely
- **front_middle_gain**: 1.0 (default) - adjust if LF/LM/RF/RM slip more than LB/RB
- **back_gain**: 1.0 (default) - adjust if LB/RB slip more than front/middle

### Motor Load Balancing

The dual-gain system allows independent control:
- If front/middle motors slip: increase `front_middle_gain`
- If back motor slips: increase `back_gain`
- Values > 1.0 increase speed, < 1.0 decrease speed

## References

- [NVIDIA Jetson Orin Nano Pinout](https://developer.nvidia.com/embedded/learn/get-started-jetson-orin-nano-devkit)
- [NVIDIA Jetson Nano Pinout](https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-devkit)
- [BILDA Motor Controller Documentation](https://www.bilda.world/)
- [Linux PWM Subsystem](https://www.kernel.org/doc/html/latest/userspace-api/pwm.html)
