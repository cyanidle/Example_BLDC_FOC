#pragma once

#include <cmath>

/*
 * When the EEPROM wheel-config record may be written.
 *
 * Writing the record stalls the app loop ~14 ms (measured on the bench: K5
 * persist=13771 us against a 200 Hz speed loop, so 2.8 missed periods), and the
 * bridge keeps its last commanded vector for that whole time. An operator
 * editing a tunable while the cart drives must not buy that stall, so the write
 * waits for a state where it costs nothing: no speed set-point, none applied, no
 * open-loop voltage, and the wheel not turning.
 *
 * The tolerances are deliberate. A source can hold a hair of command (a joystick
 * just above its deadzone, a tiny `T`), and that must not defer the record
 * forever; a latched speed target next to a `Q` command must not open the gate
 * either, which is why the applied open-loop voltage is part of the state.
 */
namespace config_persist {

constexpr float idle_speed_m_s = 0.01f;
constexpr float idle_voltage_v = 0.5f;

struct DriveState {
    float speed_target_m_s;
    float applied_target_m_s;
    float linear_velocity_m_s;
    float open_loop_voltage_v;
};

inline bool drive_idle(const DriveState& state) {
    return std::abs(state.speed_target_m_s) < idle_speed_m_s &&
           std::abs(state.applied_target_m_s) < idle_speed_m_s &&
           std::abs(state.linear_velocity_m_s) < idle_speed_m_s &&
           std::abs(state.open_loop_voltage_v) < idle_voltage_v;
}

}
