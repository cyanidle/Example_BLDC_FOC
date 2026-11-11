#pragma once

/*
 * Overcurrent / stall protection for the SimpleFOC backend, extracted into
 * pure logic so the host tests can drive it (tests/current_guard.cpp): the
 * caller samples the measurement and passes dt, nothing here touches hardware.
 */

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace current_guard {

enum class Fault: uint8_t {
    NONE,
    OVERCURRENT,
    STALLED,
};

struct Config {
    // Measured-current low-pass, s.
    float lpf_tf_s;
    // Phase-current magnitude thresholds and windows. The peak window is the
    // fast hard stop; the sustain window catches a load the soft limiter
    // cannot hold down. Both sit above the q-current ceiling the current
    // loops ramp to, so a legitimate current-limited acceleration must not
    // trip them.
    float peak_a;
    uint32_t peak_ms;
    float sustain_a;
    uint32_t sustain_ms;
    // A stall (locked rotor): a non-zero command is active, the shaft is not
    // turning and current is flowing. The current floor keeps a freewheeling
    // stop (zero current) from counting. A zero stall_ms disables the check
    // (the six-step backend does exactly that: its hall-progress watchdog
    // owns stalls and needs its full kick-recovery window). The overcurrent
    // windows have no disabled state — zero means "trip as soon as the level
    // is crossed".
    float stall_current_a;
    float stall_velocity_rad_s;
    uint32_t stall_ms;
};

// Largest phase current after removing the common mode. The Inverter's
// per-phase zero-current offsets drift together (shared front end), and a
// locked rotor parks its current on the d-axis where the q-current telemetry
// reads ~0 — only the phase magnitude sees either.
inline float phase_current_magnitude(float a, float b, float c) {
    const float mean = (a + b + c) / 3.0f;
    const float magnitude = std::max(
        { std::abs(a - mean), std::abs(b - mean), std::abs(c - mean) }
    );
    return std::isfinite(magnitude) ? magnitude : 0.0f;
}

class Guard {
public:
    explicit Guard(const Config& config): cfg(config) {}

    // Replace the parameters at run time (config topic / serial `C`); the
    // filter state and any latched fault are kept.
    void configure(const Config& config) { cfg = config; }

    void reset() {
        tripped = false;
        fault = Fault::NONE;
        peak_window_ms = 0.0f;
        sustain_window_ms = 0.0f;
        stall_window_ms = 0.0f;
    }

    // One control-loop pass over the measured phase-current magnitude.
    // command_active: a non-zero set-point is applied while the bridge is on.
    // The latch is sticky: after a trip, step() keeps reporting the fault
    // until reset().
    Fault step(float current_a, float velocity_rad_s, bool command_active, float dt_s) {
        if (tripped) {
            return fault;
        }
        if (dt_s > 0.0f && std::isfinite(current_a)) {
            const float alpha = std::clamp(dt_s / cfg.lpf_tf_s, 0.0f, 1.0f);
            filtered_a += (std::abs(current_a) - filtered_a) * alpha;
            const float dt_ms = dt_s * 1000.0f;

            peak_window_ms = filtered_a > cfg.peak_a ? peak_window_ms + dt_ms : 0.0f;
            sustain_window_ms = filtered_a > cfg.sustain_a ? sustain_window_ms + dt_ms : 0.0f;
            if (peak_window_ms >= (float)cfg.peak_ms ||
                sustain_window_ms >= (float)cfg.sustain_ms) {
                tripped = true;
                fault = Fault::OVERCURRENT;
                return fault;
            }

            const bool stalled_now = cfg.stall_ms > 0 && command_active &&
                filtered_a > cfg.stall_current_a &&
                std::abs(velocity_rad_s) < cfg.stall_velocity_rad_s;
            stall_window_ms = stalled_now ? stall_window_ms + dt_ms : 0.0f;
            if (cfg.stall_ms > 0 && stall_window_ms >= (float)cfg.stall_ms) {
                tripped = true;
                fault = Fault::STALLED;
            }
        }
        return fault;
    }

    bool is_tripped() const { return tripped; }
    float filtered_current() const { return filtered_a; }

private:
    Config cfg;
    float filtered_a = 0.0f;
    float peak_window_ms = 0.0f;
    float sustain_window_ms = 0.0f;
    float stall_window_ms = 0.0f;
    bool tripped = false;
    Fault fault = Fault::NONE;
};

}
