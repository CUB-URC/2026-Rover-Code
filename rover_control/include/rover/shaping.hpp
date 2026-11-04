// Joystick input shaping, used on the base station before commands are sent.
// (Wheel mixing and acceleration limits are the drive team's job: see
//  ros2_ws/src/drive/drive_controller.cpp in 2026-Rover-Code.)
#pragma once

namespace rover {

float clamp(float x, float lo, float hi);

// Clamp to [-1, 1]; NaN becomes 0 so a corrupted value can never cause motion.
float sanitize(float x);

// Zero inside the deadband, rescaled so output still reaches +/-1 at full stick.
float apply_deadband(float x, float deadband);

// Blend of linear and cubic response: (1-e)*x + e*x^3. Finer control near center.
float apply_expo(float x, float expo);

// sanitize -> deadband -> expo -> scale. The usual pipeline for one stick axis.
float shape_axis(float x, float deadband, float expo, float scale = 1.0f);

}  // namespace rover
