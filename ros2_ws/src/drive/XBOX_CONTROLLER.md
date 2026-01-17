# Xbox Controller Input Guide

Use an Xbox controller connected via USB to drive the rover.

## Hardware Setup

1. **Connect Xbox controller** to your machine (or rover via USB hub)
2. **Verify it's detected**:
   ```bash
   ls /dev/input/js*
   ```
   You should see `js0`, `js1`, etc. Usually the first controller is `js0`.

3. **Check permissions** (if you get permission denied):
   ```bash
   sudo usermod -a -G input $USER
   # Log out and back in for changes to take effect
   ```

## Running the Xbox Controller Node

### In Development (with simulation)

Terminal 1 - Start the simulator:
```bash
cd ros2_ws
source install/setup.bash
ros2 run drive drive_controller_sim
```

Terminal 2 - Start the Xbox controller:
```bash
source install/setup.bash
ros2 run drive xbox_controller_node --ros-args -p device:=/dev/input/js0
```

Terminal 3 - Monitor the motor status:
```bash
source install/setup.bash
ros2 topic echo /drive/sim_status
```

Terminal 4 - Test with controller input:
```bash
# Just move the left stick up/down or left/right
# The motor status should change in Terminal 3
```

### On Hardware (with real motors)

Terminal 1 - Start the drive controller:
```bash
source install/setup.bash
sudo ros2 run drive drive_controller_node
```

Terminal 2 - Start the Xbox controller:
```bash
source install/setup.bash
ros2 run drive xbox_controller_node --ros-args -p device:=/dev/input/js0
```

## Control Mapping

| Input | Action |
|-------|--------|
| Left stick up/down | Forward/backward speed (linear.x) |
| Left stick left/right | Rotation (angular.z) |
| Deadzone | 0.1 (adjustable with `-p deadzone:=0.05`) |

## Parameters

Customize behavior with ROS2 parameters:

```bash
ros2 run drive xbox_controller_node \
  --ros-args \
  -p device:=/dev/input/js0 \
  -p max_linear_speed:=1.0 \
  -p max_angular_speed:=0.5 \
  -p deadzone:=0.1
```

| Parameter | Default | Description |
|-----------|---------|-------------|
| `device` | `/dev/input/js0` | Joystick device path |
| `max_linear_speed` | `1.0` | Max forward/backward speed (0.0-1.0) |
| `max_angular_speed` | `1.0` | Max rotation speed (0.0-1.0) |
| `deadzone` | `0.1` | Stick deadzone threshold |

## Button Mapping (for reference)

| Button | Function |
|--------|----------|
| A (0) | — |
| B (1) | — |
| X (2) | — |
| Y (3) | — |
| LB (4) | — |
| RB (5) | — |
| Back (6) | — |
| Start (7) | — |
| Left stick click (8) | — |
| Right stick click (9) | — |

*Buttons currently not used. Can be extended for additional functions (e.g., emergency stop, mode switching).*

## Troubleshooting

### Device not found
```bash
# List available joysticks
ls -la /dev/input/js*

# If empty, the controller isn't connected
# Try: sudo evtest  (to see all input devices)
```

### Permission denied
```bash
# Add user to input group
sudo usermod -a -G input $USER
sudo reboot
```

### Controller detected but no response
1. Check the device parameter matches: `ros2 run drive xbox_controller_node --ros-args -p device:=/dev/input/js0`
2. Verify commands are being published: `ros2 topic echo /drive/cmd_vel`
3. Check if the drive controller is listening: `ros2 topic list | grep cmd_vel`

### Erratic behavior
- Increase deadzone: `-p deadzone:=0.2`
- Reduce max speeds: `-p max_linear_speed:=0.5`
- Check for stick drift (move stick to center and hold, check echo output)

## Advanced: Custom Button Actions

To add custom button functionality (e.g., emergency stop), edit `xbox_controller_node.cpp`:

```cpp
else if (event.type & JS_EVENT_BUTTON) {
    if (event.number == 6 && event.value) {  // Back button
        // Publish emergency stop
        auto msg = geometry_msgs::msg::Twist();
        cmd_vel_pub_->publish(msg);
        RCLCPP_WARN(this->get_logger(), "Emergency stop!");
    }
}
```

Then rebuild: `colcon build --symlink-install`
