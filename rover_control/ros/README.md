# comms_bridge (temporary ROS 2 bridge)

Lets `rover-base` on the base-station laptop drive the real rover through the drive team's
`drive_controller`:

```
rover-base (gamepad) --UDP 5005--> comms_bridge_node.py --/drive/cmd_vel--> drive_controller --> motors
                     <--UDP 5006--  (telemetry)
```

Python, using the same C++ safety core (`RoverController`) as everything else. Tested on a
laptop against a fake ROS layer, **not yet on the Jetson**. Long term this becomes a C++ node
in the `comms` package.

## One-time setup on the Jetson

Works on the Jetson itself (ROS 2 Humble installed) or inside the team's `orin` Docker container
(drop `sudo` there). Use the system Python (the one ROS uses), not a venv.

```bash
sudo apt install -y build-essential cmake python3-dev python3-pip
cd rover_control
python3 -m pip install --upgrade pip
python3 -m pip install -e .
```

Give the Jetson's Ethernet port (to the rover Groove radio) the address **10.10.10.5/24**.

## Run

```bash
source /opt/ros/humble/setup.bash
cd rover_control
python3 ros/comms_bridge_node.py                                  # starts at 30% speed
python3 ros/comms_bridge_node.py --ros-args -p max_linear:=0.5 -p max_angular:=0.5
```

On the base-station laptop: `rover-base` (press **A** for Teleop).

**Only one thing may publish `/drive/cmd_vel`.** Don't run `xbox_controller_node` or
`ros2 topic pub /drive/cmd_vel` at the same time.

## Test order for the first session

1. **Without motors:** run only the bridge, then in another terminal
   `ros2 topic echo /drive/cmd_vel`. Drive with the gamepad: `linear.x` follows the left stick,
   `angular.z` goes **positive when you push the right stick left**. B (e-stop) → zeros.
2. **Watch status:** `ros2 topic echo /comms/stop_reason` and `/comms/link_ok`.
   Unplug the base radio's power → `LinkLost` and zeros within ~0.5 s.
3. **Motors, rover on blocks, hand on the physical e-stop:** start the drive team's
   `drive_controller`, then the bridge, then `rover-base`. Small stick movements first.

Rover-side e-stop (no release; release with START on the gamepad):
`ros2 service call /emergency_stop std_srvs/srv/Trigger`

## Topics, service, parameters

| Name | Type | Notes |
|---|---|---|
| `/drive/cmd_vel` (pub) | `geometry_msgs/Twist` | 50 Hz, `linear.x`, `angular.z` (+ = left), scaled by max_linear/max_angular, exactly 0 when stopped |
| `/comms/estop` (pub) | `std_msgs/Bool` | 5 Hz |
| `/comms/link_ok` (pub) | `std_msgs/Bool` | 5 Hz |
| `/comms/mode` (pub) | `std_msgs/String` | Idle / Teleop / Autonomous / Arrived |
| `/comms/stop_reason` (pub) | `std_msgs/String` | None / EStop / LinkLost / Idle / NoAutonomyCmd |
| `/comms/led` (pub) | `std_msgs/String` | Off / Red / Blue / FlashingGreen |
| `/autonomy/cmd_vel` (sub) | `geometry_msgs/Twist` | Used only in Autonomous mode |
| `/emergency_stop` (srv) | `std_srvs/Trigger` | Engages the e-stop |

Parameters: `port` 5005, `base_port` 5006, `rate_hz` 50, `link_timeout_s` 0.5,
`autonomy_timeout_s` 0.3, `max_linear` 0.3, `max_angular` 0.3.

## If the wheels don't move but /drive/cmd_vel looks right

The bridge is doing its job; check the drive side. Known issues in `drive_controller` (Nov 2026):
the goBILDA 2x40A expects 50 Hz servo pulses (1500 µs = stop, 1050–1950 µs = full reverse to
full forward, ~1 s at 1500 µs to arm), but the code outputs a 50 kHz duty cycle; reverse is
dropped by `abs(speed)`; and a clock-type mismatch may crash it on its first loop.
