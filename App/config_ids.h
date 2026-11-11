#pragma once

#include <cstdint>

/*
 * Runtime-config parameter ids — the single source shared by the CAN config
 * topic (app.cpp's ConfigSub), the serial bench channel (`C` writes, `G`
 * read-back, the `L`/`R` aliases) and the EEPROM wheel-config record.
 *
 * A value is applied as `value = numerator / denominator` (CAN) or a plain
 * float (serial) to the parameter selected by `id`. Keep this enum in sync
 * with mods/config_defs.lua in the repo root — the GUIs order their controls
 * by these ids.
 *
 * Units follow the backend's regulator: the velocity gains are per shaft
 * rad/s with the regulator's output unit (V on six-step, A on the FOC
 * builds). The SimpleFOC-only ids (12, 14, 15) are rejected by backends
 * without the corresponding knob.
 */
enum ConfigId: int32_t {
    CFG_WHEEL_DIAMETER = 0,  // m,             drive-wheel diameter (rad/s <-> m/s)
    CFG_PID_KP        = 1,   // U/(rad/s),     proportional gain (U = V on six-step, A on FOC)
    CFG_PID_KI        = 2,   // U/(rad/s*s),   integral gain
    CFG_PID_KD        = 3,   // U*s/(rad/s),   derivative gain
    CFG_PID_I_LIMIT   = 4,   // U,             integral anti-windup clamp
    CFG_OC_PEAK_A     = 5,   // A,             hard overcurrent trip level
    CFG_OC_PEAK_MS    = 6,   // ms,            hard overcurrent trip window
    CFG_OC_SUSTAIN_A  = 7,   // A,             sustained overcurrent trip level
    CFG_OC_SUSTAIN_MS = 8,   // ms,            sustained overcurrent trip window
    CFG_STALL_CURRENT = 9,   // A,             stall guard: current floor
    CFG_STALL_SPEED   = 10,  // rad/s,         stall guard: shaft-velocity ceiling
    CFG_STALL_MS      = 11,  // ms,            stall guard window (0 disables it)
    CFG_VEL_LPF       = 12,  // s,             velocity-feedback low-pass (SimpleFOC; 0 = off)
    CFG_SPEED_RAMP    = 13,  // m/s^2,         wheel speed set-point ramp
    CFG_TORQUE_RAMP   = 14,  // A/s,           q-current set-point slew (SimpleFOC soft start)
    CFG_BRAKE_CURRENT = 15,  // A,             braking-current ceiling against the motion (regen limit)
    CFG_ID_COUNT      = 16,  // first invalid id; also the EEPROM record width
};
