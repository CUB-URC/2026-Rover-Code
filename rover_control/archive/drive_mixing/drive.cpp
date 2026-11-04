#include "rover/drive.hpp"

#include <algorithm>
#include <cmath>

namespace rover {

float clamp(float x, float lo, float hi) { return std::max(lo, std::min(hi, x)); }

float apply_deadband(float x, float deadband) {
    x = clamp(x, -1.0f, 1.0f);
    if (std::isnan(x)) return 0.0f;
    const float mag = std::fabs(x);
    if (mag <= deadband) return 0.0f;
    const float scaled = (mag - deadband) / (1.0f - deadband);
    return std::copysign(scaled, x);
}

float apply_expo(float x, float expo) {
    expo = clamp(expo, 0.0f, 1.0f);
    return (1.0f - expo) * x + expo * x * x * x;
}

WheelSpeeds arcade_mix(float throttle, float steer, const DriveConfig& cfg) {
    // NaN guard: a corrupted command must never turn into motion.
    if (std::isnan(throttle) || std::isnan(steer)) return {};

    const float t = apply_expo(apply_deadband(throttle, cfg.deadband), cfg.expo);
    const float s = apply_expo(apply_deadband(steer, cfg.deadband), cfg.expo) * cfg.turn_scale;

    float left = t + s;
    float right = t - s;

    // Normalize so the larger side is at most 1, preserving the turn ratio.
    const float biggest = std::max(std::fabs(left), std::fabs(right));
    if (biggest > 1.0f) {
        left /= biggest;
        right /= biggest;
    }
    const float cap = clamp(cfg.max_speed, 0.0f, 1.0f);
    return {left * cap, right * cap};
}

float SlewLimiter::step(float target, float dt) {
    if (dt <= 0.0f) return value_;
    const float max_delta = rate_ * dt;
    value_ += clamp(target - value_, -max_delta, max_delta);
    return value_;
}

}  // namespace rover
