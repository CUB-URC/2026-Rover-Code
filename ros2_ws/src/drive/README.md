# Drive Control System

Basic drive control system for the CUB-URC rover using Jetson PWM outputs.

## Hardware Support

- **Primary**: NVIDIA Jetson Orin Nano Super Dev Kit
- **Secondary**: NVIDIA Jetson Nano Dev Kit
- **Motor Controllers**: BILDA 2x40A Motor Controllers
- **Motors**: 6x Saturn 5304 Series Planetary Gear Motors (3 per side)
- **PWM Frequency**: 50 kHz

## Configuration

The system is configured via YAML in [config/drive_config.yaml](../../config/drive/drive_config.yaml).

### PWM Channels

Four PWM channels are configured:
- `motor_left_front_middle`: PWM chip 0, channel 0 (2 motors: LF + LM)
- `motor_left_back`: PWM chip 0, channel 1 (1 motor: LB)
- `motor_right_front_middle`: PWM chip 1, channel 0 (2 motors: RF + RM)
- `motor_right_back`: PWM chip 1, channel 1 (1 motor: RB)

**Hardware Note**: PWM chip and channel numbers differ between Jetson devices. If PWM initialization fails, check actual PWM devices:
```bash
ls /sys/class/pwm/
```
Then update `drive_config.yaml` with correct chip numbers.

### Motor Configuration

- **LF (Left Front)** + **LM (Left Middle)**: Paired on single PWM channel with independent speed control via `front_middle_gain`
- **LB (Left Back)**: Separate PWM channel with independent speed via `back_gain`
- **RF (Right Front)** + **RM (Right Middle)**: Paired on single PWM channel
- **RB (Right Back)**: Separate PWM channel

This allows torque balancing: reduce `back_gain` if back motors are slipping, increase `front_middle_gain` for front-heavy terrain.

### Control Parameters

- **max_speed**: Maximum speed as a percentage (0.0 to 1.0)
- **acceleration**: Speed change per control loop (prevents jerky movements)
- **deadzone**: Joystick input deadzone threshold
- **timeout**: Command timeout in seconds (automatically stops if no command received)

## Building

```bash
cd ros2_ws
colcon build --symlink-install
```

## Hardware Setup

**Important**: Before running on actual hardware, see [Hardware Setup Guide](../../docs/drive_hardware_setup.md)

This covers:
- PWM pin configuration for Jetson Orin Nano and Jetson Nano
- Motor and motor controller connections
- BILDA 2x40A Controller wiring
- Troubleshooting PWM issues
- Safety considerations

## Running

### Simulation Mode (Recommended First)

Test parameter loading and control logic without hardware:

```bash
ros2 run drive drive_controller_sim
```

Monitor motor states in another terminal:
```bash
ros2 topic echo /drive/sim_status
```

This will show simulated motor speeds updating based on control parameters.

### Hardware Mode

Start the actual drive controller (requires PWM hardware access):

```bash
# May need sudo for /sys/class/pwm/ access
sudo ros2 run drive drive_controller_node
```

The node will:
1. Load configuration from `drive_config.yaml`
2. Initialize PWM channels via sysfs (Linux `/sys/class/pwm/` interface)
3. Subscribe to `/drive/cmd_vel` topic for commands
4. Control motors at 50 Hz
5. Log warnings if PWM paths don't exist (hardware compatibility issue)

### Send Commands

Publish `geometry_msgs/Twist` messages to `/drive/cmd_vel`:

```bash
# Move forward at 50% speed
ros2 topic pub /drive/cmd_vel geometry_msgs/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Turn left at 50% angular velocity
ros2 topic pub /drive/cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"

# Move forward and turn (combine linear and angular)
ros2 topic pub /drive/cmd_vel geometry_msgs/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}"
```

### Run Test Node

A test node is provided to verify basic motor functionality:

```bash
ros2 run drive drive_test_node
```

This will run a sequence:
1. Move forward for 2 seconds
2. Stop for 1 second
3. Move backward for 2 seconds
4. Stop for 1 second
5. Turn left for 2 seconds
6. Stop for 1 second
7. Turn right for 2 seconds
8. Stop

## Testing Workflow

**Recommended testing order**:

1. **Verify parameters** (simulation, no hardware)
   ```bash
   ros2 run drive drive_controller_sim
   ```

2. **Test ROS2 communication** (with simulation)
   ```bash
   # Terminal 1: simulator
   ros2 run drive drive_controller_sim
   
   # Terminal 2: watch motor output
   ros2 topic echo /drive/sim_status
   
   # Terminal 3: send test commands
   ros2 topic pub /drive/cmd_vel geometry_msgs/Twist "{linear: {x: 0.5}}"
   ```

3. **Verify hardware setup** (if using real motors)
   - Check PWM paths exist: `ls /sys/class/pwm/`
   - Check motor connections and power
   - See [Hardware Setup Guide](../../docs/drive_hardware_setup.md)

4. **Run on hardware** (with motors connected)
   ```bash
   sudo ros2 run drive drive_controller_node
   ```

5. **Tune parameters** if needed
   - Adjust gains in `drive_config.yaml`
   - Modify acceleration or deadzone
   - Rebuild: `colcon build --symlink-install`

## Command Interface

### Subscription

- **Topic**: `/drive/cmd_vel`
- **Message Type**: `geometry_msgs/Twist`
- **Fields**:
  - `linear.x`: Forward/backward velocity (-1.0 to 1.0)
  - `angular.z`: Rotation velocity (-1.0 to 1.0)
  - Other fields are ignored

### Differential Drive Kinematics

```
left_motor_speed = linear_velocity - angular_velocity
right_motor_speed = linear_velocity + angular_velocity
```

## Technical Details

### PWM Interface

The system uses Linux sysfs PWM interface (`/sys/class/pwm/`):

1. Exports PWM channels
2. Sets frequency based on configuration
3. Enables PWM output
4. Writes duty cycle values (0-1.0)
5. Cleans up on shutdown

### Motor Control Logic

- **Forward**: Positive speed → forward PWM on, reverse PWM off
- **Backward**: Negative speed → forward PWM off, reverse PWM on
- **Stop**: Speed 0 → both PWMs off
- **Acceleration Limiting**: Smooth acceleration to prevent motor strain

## Troubleshooting

### PWM not working
- **Issue**: "Could not export PWM channel" or "Could not set PWM period"
- **Cause**: PWM chip/channel numbers don't match hardware
- **Solution**: 
  1. Run `ls /sys/class/pwm/` to see available PWM chips
  2. Check actual chip numbering (may be different on Nano vs Orin)
  3. Update `config/drive_config.yaml` with correct values
  4. For Jetson Nano, you might only have `pwmchip0`
  5. For Jetson Orin Nano, check if `pwmchip0` and `pwmchip1` exist

### PWM Permission Denied
- **Issue**: `Permission denied` when accessing `/sys/class/pwm/`
- **Solution**: Run with appropriate permissions
  ```bash
  sudo ros2 run drive drive_controller_node
  ```
  Or configure udev rules for unprivileged access

### Motors not responding
- Check BILDA motor controller power connections
- Verify PWM frequency matches controller specifications (should be 50 kHz)
- Check motor wiring to motor controller
- Verify no PWM initialization errors in node logs

### Nodes not starting
- Verify YAML config file exists at `share/drive/config/drive_config.yaml`
- Check ROS2 environment is properly sourced: `source install/setup.bash`
- Review node logs for specific errors

## Future Enhancements

- [ ] IMU-based heading control
- [ ] Odometry feedback from encoders
- [ ] Dynamic speed limiting based on terrain
- [ ] Autonomous navigation integration
- [ ] Motor current monitoring

## Coding Standards

This package adheres to the CUB-URC coding standards as defined in [docs/coding_standards.md](../../docs/coding_standards.md):

### Naming Conventions
- **Package**: snake_case (`drive`)
- **Nodes**: snake_case (`drive_controller_node`, `drive_test_node`)
- **Classes**: CamelCase (`DriveController`, `DriveTestNode`)
- **Methods/Functions**: snake_case (`set_pwm_speed()`, `control_loop()`)

### Configuration Management
- **No hardcoded values** in source files
- **All parameters** loaded from `config/drive_config.yaml`
- **YAML format** preferred for hardware configuration
- Configuration path: `share/drive/config/drive_config.yaml`

### Topic Naming
- **Standardized namespace**: `/drive/` prefix for all drive topics
- **Command velocity**: `/drive/cmd_vel` (geometry_msgs/Twist)
- **Motor status**: `/drive/motor_status` (published in future)
- Follows ROS2 naming conventions and best practices

### Code Organization
- PWM interface abstraction in private methods
- Clear separation of concerns (configuration, PWM I/O, control logic)
- Comprehensive error handling and logging
- Timer-based control loop at 50 Hz for consistent behavior
