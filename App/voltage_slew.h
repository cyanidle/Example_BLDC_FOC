#pragma once

#include <algorithm>
#include <cmath>

/*
 * Signed-voltage slew of the open-loop (`Q`) channel, as a pure function so the
 * host tests can cover it (tests/voltage_slew.cpp).
 *
 * The rate follows the voltage MAGNITUDE and whether it is being taken away or
 * driven up, never a plain value-versus-target comparison: with the latter, a
 * positive-to-negative move takes the "value is above the target" branch and the
 * applied voltage crosses the sign at the current-limiting rate, which is
 * measurably a bump rather than a ramp (bench: the same slider move from -q to
 * +q ramped smoothly, +q to -q kicked).
 *
 *   - |V| growing                        -> up_rate_v_s
 *   - |V| shrinking, sign unchanged      -> reduce_rate_v_s (the current limiter
 *                                           sheds a ceiling-forced drop fast)
 *   - |V| shrinking to zero, or through  -> shed_rate_v_s (the operator's stop
 *     zero to the other sign                rate: the wheel follows its own
 *                                           back-EMF down instead of the phases
 *                                           being shorted across it)
 *
 * A reversal is shed to 0 V first and only then raised with the other sign, so
 * the applied vector never steps through 180 degrees.
 */
namespace voltage_slew {

inline float slew(
    float applied_v,
    float target_v,
    float up_rate_v_s,
    float reduce_rate_v_s,
    float shed_rate_v_s,
    float dt_s
) {
    if (!(dt_s > 0.0f)) {
        return applied_v;
    }
    const float target_magnitude = std::abs(target_v);
    float magnitude = std::abs(applied_v);
    const bool crossing =
        applied_v != 0.0f && target_v != 0.0f && ((applied_v > 0.0f) != (target_v > 0.0f));
    if (crossing || target_magnitude == 0.0f) {
        magnitude = std::max(0.0f, magnitude - std::max(shed_rate_v_s, 0.0f) * dt_s);
        // Keep the sign being shed: the next passes raise the other one from 0.
        return magnitude == 0.0f ? 0.0f : std::copysign(magnitude, applied_v);
    }
    if (magnitude < target_magnitude) {
        magnitude = std::min(
            target_magnitude, magnitude + std::max(up_rate_v_s, 0.0f) * dt_s
        );
    }
    else {
        magnitude = std::max(
            target_magnitude, magnitude - std::max(reduce_rate_v_s, 0.0f) * dt_s
        );
    }
    return magnitude == 0.0f ? 0.0f : std::copysign(magnitude, target_v);
}

}
