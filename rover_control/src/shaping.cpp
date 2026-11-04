#include "rover/shaping.hpp"

#include <algorithm>
#include <cmath>

namespace rover {

float clamp(float x, float lo, float hi) { return std::max(lo, std::min(hi, x)); }

float sanitize(float x) {
    if (std::isnan(x)) return 0.0f;
    return clamp(x, -1.0f, 1.0f);
}

float apply_deadband(float x, float deadband) {
    x = sanitize(x);
    const float mag = std::fabs(x);
    if (mag <= deadband) return 0.0f;
    return std::copysign((mag - deadband) / (1.0f - deadband), x);
}

float apply_expo(float x, float expo) {
    expo = clamp(expo, 0.0f, 1.0f);
    return (1.0f - expo) * x + expo * x * x * x;
}

float shape_axis(float x, float deadband, float expo, float scale) {
    return sanitize(apply_expo(apply_deadband(x, deadband), expo) * scale);
}

}  // namespace rover
