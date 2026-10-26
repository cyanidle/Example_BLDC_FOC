#pragma once

#include <cstdint>

#include <voltbro/math/regulators/pid.hpp>
#include <voltbro/utils.hpp>

/*
 * Bench-tunable constants of the compiled backend (VB_MOTOR_IMPL), sized for the
 * 57BL95S15-230 wheel module: 24 V, 150 W, 8.7 A, 0.5 N*m, 4 pole pairs. The
 * wheel radius and the velocity PID gains also arrive over the CAN config topic
 * and override the defaults below.
 */
namespace motor_tuning {

// The PID output unit follows the backend: the six-step and SimpleFOC loops
// produce a voltage set-point (V), the libvoltbro FOC loop a q-current (A).
inline PIDConfig default_velocity_pid_config() {
#if defined(VB_MOTOR_IMPL_VOLTBRO_FOC)
    return PIDConfig {
        .kp = 0.05f,              // A / (rad/s)
        .ki = 2.0f,               // A / (rad/s * s)
        .kd = 0.0f,
        .integral_error_lim = 10.0f,
        .max_output = 8.0f,       // A
        .min_output = -8.0f
    };
#elif defined(VB_MOTOR_IMPL_SIMPLEFOC)
    return PIDConfig {
        .kp = 0.05f,              // V / (rad/s)
        .ki = 2.0f,               // V / (rad/s * s)
        .kd = 0.0f,
        .integral_error_lim = 10.0f,
        .max_output = 17.0f,      // V
        .min_output = -17.0f
    };
#else
    return PIDConfig {
        .kp = 0.5f,               // V / (rad/s)
        .ki = 0.1f,               // V / (rad/s * s)
        .kd = 0.0f,
        .integral_error_lim = 10.0f,
        .max_output = 24.0f,      // V
        .min_output = -24.0f
    };
#endif
}

// d/q current regulators, V per A. Placeholders until bench-tuned: the reference
// sketch runs SimpleFOC in voltage torque mode and has no current loop at all.
constexpr float foc_current_kp = 0.5f;
constexpr float foc_current_ki = 50.0f;
constexpr float foc_current_integral_limit = 20.0f;  // A * s

constexpr micros_t foc_period_us = 200;
constexpr float foc_period_s = (float)foc_period_us * 1e-6f;
constexpr float foc_observer_g1 = 0.11891787927816833f;
constexpr float foc_observer_g2 = 29.719984189894298f;
constexpr float foc_observer_g3 = 2933.7234148496536f;
constexpr float foc_expected_acceleration = 0.0f;
constexpr float foc_current_lpf_coefficient = 0.1f;

inline PIDConfig foc_current_pid_config() {
    return PIDConfig {
        .kp = foc_current_kp,
        .ki = foc_current_ki,
        .kd = 0.0f,
        .integral_error_lim = foc_current_integral_limit,
    };
}

constexpr float simplefoc_velocity_lpf_tf = 0.1f;  // s
constexpr float simplefoc_pwm_frequency_hz = 25000.0f;
constexpr float simplefoc_driver_voltage_limit_v = 19.0f;
constexpr float simplefoc_motor_voltage_limit_v = 17.0f;
constexpr float simplefoc_current_limit_a = 8.0f;
constexpr float simplefoc_max_shaft_velocity_rad_s = 1000.0f;
constexpr uint32_t simplefoc_velocity_timeout_ms = 100;

}
