#pragma once

#include <cmath>

/*
 * When the EEPROM wheel-config record may be written.
 *
 * Writing the record stalls the app loop ~14 ms (measured on the bench: K5
 * persist=13771 us against a 200 Hz speed loop, so 2.8 missed periods), and the
 * bridge keeps its last commanded vector for that whole time. An operator
 * editing a tunable while the cart drives must not buy that stall, so the write
 * waits for a state where it costs nothing: no speed set-point, none applied,
 * and the wheel not turning. The measured speed covers the open-loop voltage
 * channel, and a latched speed set-point covers a `Q` command that followed a
 * `T` (set_voltage_point does not clear the target) — either one defers.
 */
namespace config_persist {

constexpr float idle_speed_m_s = 0.01f;

struct DriveState {
    float speed_target_m_s;    // commanded set-point
    float applied_target_m_s;  // ramp output actually being regulated
    float linear_velocity_m_s; // measured wheel speed
};

inline bool drive_idle(const DriveState& state) {
    return state.speed_target_m_s == 0.0f &&
           state.applied_target_m_s == 0.0f &&
           std::abs(state.linear_velocity_m_s) < idle_speed_m_s;
}

}
