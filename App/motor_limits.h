#pragma once

#include <algorithm>
#include <cmath>

/*
 * Voltage/speed limits of a FOC drive, as pure functions so the host tests can
 * cover them (tests/motor_limits.cpp). Both derive from the drive's own EMF
 * constant — DriveInfo::torque_const times the gearbox ratio is volts per wheel
 * rad/s — never from a bench-tuned voltage: the ceiling moves with the live
 * bus, so a limit baked against one supply reads wrong on the next one.
 */
namespace motor_limits {

enum class SupplyDerate {
    NONE,
    RAIL_HIGH,
    RAIL_LOW,
};

inline float directional_torque_limit(
    float demand_a,
    float wheel_rate_rad_s,
    float stopped_rate_rad_s,
    SupplyDerate derate,
    float scale
) {
    const bool moving = std::abs(wheel_rate_rad_s) > stopped_rate_rad_s;
    const float torque_motion = demand_a * wheel_rate_rad_s;
    const bool blocked =
        (derate == SupplyDerate::RAIL_HIGH && moving && torque_motion < 0.0f) ||
        (derate == SupplyDerate::RAIL_LOW && (!moving || torque_motion > 0.0f));
    return blocked ? demand_a * std::clamp(scale, 0.0f, 1.0f) : demand_a;
}

// q-voltage the current loops must be allowed to apply at `wheel_rate_rad_s`:
// the back-EMF plus the low-speed floor (phase IR and one-pass transients, see
// motor_tuning::simplefoc_current_loop_v_floor). A cap that trails the EMF by
// even a fraction of a volt per rad/s cannot hold the current at speed: the
// loop saturates, the applied vector turns into a speed-dependent brake and the
// velocity loop hunts around the speed where the cap crosses the EMF.
inline float current_loop_voltage_limit(
    float emf_per_wheel_rad_s,
    float wheel_rate_rad_s,
    float floor_v
) {
    return floor_v + emf_per_wheel_rad_s * std::abs(wheel_rate_rad_s);
}

// Largest wheel rate the modulation can still regulate with `floor_v` of
// headroom left to the current loop. Past it the speed loop commands voltage
// the bridge cannot apply: the current loops lose authority over the phase
// current, which is then set by the back-EMF against the saturated vector. On
// the bench a target past this ceiling sat there and the gate driver latched
// its bridge off (see simplefoc_modulation_headroom).
inline float reachable_wheel_rate(
    float modulation_limit_v,
    float emf_per_wheel_rad_s,
    float floor_v
) {
    if (!(emf_per_wheel_rad_s > 0.0f) || !(modulation_limit_v > floor_v)) {
        return 0.0f;
    }
    return (modulation_limit_v - floor_v) / emf_per_wheel_rad_s;
}

}
