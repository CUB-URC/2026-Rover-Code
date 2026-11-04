# Archive

Code from the first version of rover_control that is **not** part of the commands
side any more. Kept for reference; not built or tested.

- `drive_mixing/` — arcade (skid-steer) mixing, slew-rate limiter. The drive team's
  `drive_controller` in 2026-Rover-Code does this now.
- `motor_bridge/` — serial motor backend and reference Arduino sketch. The rover drives
  motors directly from the Jetson's PWM pins (drive team).
