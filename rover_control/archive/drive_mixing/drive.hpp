// Joystick shaping and skid-steer (differential) mixing.
#pragma once

namespace rover {

struct DriveConfig {
    float deadband = 0.08f;    // ignore stick noise below this
    float expo = 0.3f;         // 0 = linear, 1 = fully cubic (finer control near center)
    float max_speed = 1.0f;    // global speed cap, 0..1 (lower it for indoor testing!)
    float turn_scale = 0.7f;   // how aggressive steering is relative to throttle
    float max_accel = 2.0f;    // max change in wheel command per second (protects gearboxes)
};

struct WheelSpeeds {
    float left = 0.0f;   // -1..1
    float right = 0.0f;  // -1..1
};

float clamp(float x, float lo, float hi);

// Zero inside the deadband, rescaled so output still reaches +/-1 at full stick.
float apply_deadband(float x, float deadband);

// Blend of linear and cubic response: (1-e)*x + e*x^3.
float apply_expo(float x, float expo);

// Arcade-style mix: throttle + steer -> left/right wheel speeds, scaled to max_speed.
// Applies deadband and expo to both axes. Never exceeds +/-max_speed.
WheelSpeeds arcade_mix(float throttle, float steer, const DriveConfig& cfg);

// Limits how fast a value may change (rate in units/second).
class SlewLimiter {
public:
    explicit SlewLimiter(float rate_per_s) : rate_(rate_per_s) {}
    float step(float target, float dt);
    void reset(float value = 0.0f) { value_ = value; }
    float value() const { return value_; }

private:
    float rate_;
    float value_ = 0.0f;
};

}  // namespace rover
