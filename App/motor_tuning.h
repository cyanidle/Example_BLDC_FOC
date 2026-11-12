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

// The PID output unit follows the backend: the six-step loop produces a
// voltage set-point (V), the SimpleFOC and libvoltbro FOC loops a q-current (A).
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
    // Bench-tuned on this module; also held the 18.5 V rail's full reachable
    // rate (~2.2 wheel rad/s) in both directions on 2026-11-12. The units are
    // wheel rad/s and amps:
    // the speed targets are wheel-side, the output is the q-current set-point.
    return PIDConfig {
        .kp = 2.1f,               // A / (rad/s)
        .ki = 1.11f,              // A / (rad/s * s)
        .kd = 0.0f,
        // Ignored by SimpleFOC's PID (it clamps the integral together with the
        // whole output by `limit`); kept for config-topic compatibility.
        .integral_error_lim = 10.0f,
        .max_output = 8.0f,       // A — the q-current set-point ceiling
        .min_output = -8.0f
    };
#else
    return PIDConfig {
        .kp = 0.5f,               // V / (rad/s)
        .ki = 0.1f,               // V / (rad/s * s)
        .kd = 0.0f,
        // Error-domain clamp: the volts the integral may add are ki * this, so
        // 100 * 0.1 = 10 V — the same value config_defs.lua shows as i_limit.
        .integral_error_lim = 100.0f,
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

constexpr float simplefoc_velocity_lpf_tf = 0.05f;  // s
// Rate estimate the Hall angle tracker free-runs on, s: the raw rate comes from
// a single 2 ms Hall interval, so it is smoothed before it is integrated.
constexpr float simplefoc_hall_rate_tau_s = 0.02f;
// Largest Hall rate a single sample may report before the rate filter will
// believe it, as a multiple of the speed the modulation ceiling can drive
// (simplefoc_motor_voltage_limit_v / DriveInfo::torque_const). The measurement
// is one edge, so it is coarse by nature (consecutive samples of the same
// motion differ by tens of percent); what this rejects is a jittering Hall
// line, which alternates sectors within a fraction of the edge interval and
// claims several times the top speed.
constexpr float simplefoc_hall_rate_plausible_multiple = 2.0f;
// Time constant of the Hall angle tracker, s: how fast the estimated angle is
// pulled towards the centre of the sector the count reports. It has to be
// shorter than the time the rotor spends in a sector (1.9 ms at 0.28 m/s) or
// the residual staircase is exactly the torque ripple the tracker exists to
// remove. Scaling it with the sector period was tried on the bench and changed
// nothing measurable: the half-sector clamp fires once per Hall edge either way
// (14606/14600 and 14306/14300), because a new sector centre is exactly half a
// sector ahead of the previous one — that snap is structural, not saturation.
constexpr float simplefoc_hall_angle_tau_s = 0.0008f;
constexpr float simplefoc_pwm_frequency_hz = 25000.0f;
// 60 V stays below the DRV8323's 65 V absolute-maximum PVDD rating. The 30 V
// phase cap leaves room for 1 m/s on this wheel (25.6 V at the measured EMF
// constant); below those ceilings the live bus remains the binding limit.
constexpr float simplefoc_driver_voltage_limit_v = 60.0f;
constexpr float simplefoc_motor_voltage_limit_v = 30.0f;
// Fraction of the half-bus the q-voltage may use. Centered SinePWM turns a
// q-voltage of half the bus into 100% duty on the peak phase, and the DRV8328
// only refreshes that phase's bootstrap capacitor while its high side is off:
// held at 100% the driver trips BST under-voltage and latches its bridge off
// (bench, T0.6: the speed loop sat at the voltage ceiling and the drive died).
// 0.9 caps the duty at 95%, leaving the refresh window at the cost of 10% of
// the top speed.
constexpr float simplefoc_modulation_headroom = 0.9f;
constexpr float simplefoc_current_limit_a = 8.0f;
// Low-speed clamp of the d/q current-loop output, V, and the adder above it:
// the limit is `floor_v + EMF-per-wheel-rad/s * |speed|` (motor_limits.h), the
// floor covering phase IR and one-pass transients, so the loops can always hold
// the current they are asked for. Below a few hundred rpm the back-EMF is
// small, so holding a current costs only a fraction of a volt; a bigger voltage
// there drives the real current to V/phase-R (20+ A) long before the loop can
// unwind, which is the audible hit heard when a spinning wheel is reversed or
// stopped (bench: a reversal from +0.3 m/s tripped the gate driver's own
// protection at the zero crossing). The clamp follows the back-EMF with speed,
// so it never limits normal operation. A fixed 3.2 V per wheel rad/s was tried
// instead of the EMF constant and trailed it (4.42 V per wheel rad/s on this
// drive), turning the limit into a speed-dependent brake past ~0.21 m/s — the
// speed loop then hunts wherever the limit crosses the EMF.
constexpr float simplefoc_current_loop_v_floor = 1.6f;
// Supply derate (see the derate block in the SimpleFOC backend). The torque
// demand is pulled towards zero while the rail is outside [low_v, over_v] and
// restored once it is back above full_v: under `low_v` the supply is being
// driven into current limit, over `over_v` the load is regenerating into a
// supply that cannot sink it. The nominal 18.5 V rail reads 18.4-18.6 V at
// rest and up to ~1.2 A of drive current; a stop from 0.3 m/s drove it to
// 22.1 V and the supply then cut out down to 4.1 V.
constexpr float simplefoc_supply_derate_low_v = 17.0f;
constexpr float simplefoc_supply_derate_full_v = 18.0f;
constexpr float simplefoc_supply_derate_over_v = 19.2f;
constexpr float simplefoc_supply_derate_rate_per_s = 4.0f;
// How long the raw bus sample must stay outside its window before the derate is
// allowed to act, s. The sample is one free-running ADC frame on a signal that
// aliases the switching ripple (measured: single frames 1.5 V off a steady
// 18.5 V rail), and a derate entry zeroes the whole torque demand, so acting on
// one frame is acting on noise — 51 entries in a 30 s steady hold.
constexpr float simplefoc_supply_derate_confirm_s = 0.0015f;
// Quiet time the gate-driver FAULT line must stay released before the loop
// re-arms itself after a transient trip.
constexpr float simplefoc_driver_fault_quiet_s = 0.1f;
constexpr float simplefoc_max_shaft_velocity_rad_s = 1000.0f;
constexpr uint32_t simplefoc_velocity_timeout_ms = 100;
// Loop-trace sampling period, us (App/loop_trace.h). 400 us gives a 2.5 kHz
// view of the loop over the 128-sample ring: ~51 ms of history, enough to hold
// the few periods before and after a stutter.
constexpr uint32_t simplefoc_trace_period_us = 400;
// Control period considered over budget, us (App/loop_trace.h `slow=`). The
// loop runs at ~1 kHz, so 2 ms is five nominal periods: while such a pass
// lasts, the bridge holds its last vector and the rotor turns through it —
// at 0.24 m/s (rotor 88 rad/s) 2 ms is 90 degrees electrical of rotation with
// no current-loop update, which shows up as a current spike and a stutter.
constexpr uint32_t simplefoc_loop_overrun_us = 2000;
// Time constant of the bus-voltage filter the voltage and speed budgets are
// derived from, s. The inverter's bus value is one free-running ADC frame, so
// it aliases the switching ripple and can read several volts low on a single
// pass; the budgets must follow the rail, not the frame (a per-pass ceiling
// turned a held command into a wandering target on the bench).
constexpr float simplefoc_bus_filter_tau_s = 0.05f;

// d/q current regulators (SimpleFOC PID_current_q / PID_current_d), V per A.
// Bench-tuned: `A` (characteriseMotor + tuneCurrentController) measured
// R = 0.159 ohm, Ld = 0.19 mH, Lq = 0.23 mH on this module and derived
// kp = 0.134 V/A, ki = 99 V/(A*s) for a 100 Hz current loop. The placeholders
// this replaced (kp 3, ki 300) oscillated: with the shunt front end's ripple
// the loop rang rail to rail, which is what made the speed loop buzz.
constexpr float simplefoc_current_kp = 0.134f;
constexpr float simplefoc_current_ki = 99.0f;
// Back-EMF that still counts as "nothing to hold", as a fraction of the d/q
// current loop's voltage floor. The regulators' integrals are most of the phase
// voltage at speed (they hold the back-EMF), so clearing them while the rotor
// turns drops the windings to ~0 V and the machine brakes on its own EMF —
// measured as a 9.5 A spike and a 20.7 V rail excursion when a wheel rolling at
// 0.3 m/s was released. Below this fraction of the floor the loop rebuilds the
// voltage in a pass or two, so a mode switch there is safe. The backend turns it
// into a wheel rate with the drive's EMF constant (torque_const * gear_ratio).
constexpr float simplefoc_regulator_reset_emf_fraction = 0.125f;
// Measured-current low-pass, s. The ADC is free-running (not synchronised to
// the PWM), so the shunt samples alias the switching ripple; this filter is
// what keeps that ripple out of the current regulators. TuneCurrentController
// derives its own value from the bandwidth (0.3 ms); 2 ms trades current-loop
// bandwidth for a quieter drive, which the hall quantisation does not need.
constexpr float simplefoc_current_lpf_tf = 0.002f;

// Soft start. Torque slew of the velocity loop's output (the q-current
// set-point), A/s: the PID cannot slam the bridge with a full-torque step, and
// 50 A/s reaches the 8 A limit in 160 ms. (8 A/s was tried on the bench and
// left the velocity loop limit-cycling at ~1 Hz: the slew's second of lag is
// slower than the loop's own correction, so the integral winds up and
// overshoots.) The speed set-point itself is additionally ramped by the
// MotorBackend façade. Runtime-tunable as config id 14.
constexpr float simplefoc_torque_ramp_a_s = 50.0f;
// Braking-current ceiling, A: the largest q-current the speed loop may demand
// against the measured rotation. That torque is regenerative and the bench
// supply cannot sink it, so the ceiling sits far under the 8 A drive limit;
// the supply-derate block in update() catches the rail rise a small brake
// still causes. 0 A makes a moving wheel coast. Runtime-tunable as config
// id 15.
constexpr float simplefoc_brake_current_a = 2.2f;
// Voltage slew of the open-loop channel (serial Q / Cyphal 4050), V/s. A zero
// command uses the gentler stop rate below; the asymmetric fast down-slew only
// exists to shed an over-limit current.
constexpr float simplefoc_voltage_ramp_v_s = 20.0f;
// ...and its downward slew (sign change, current limiting), V/s: fast, so the
// soft current ceiling is never the reason a large voltage sits on a stalled
// phase (phase R ~0.11 ohm: 2 V across it is ~18 A).
constexpr float simplefoc_voltage_down_ramp_v_s = 200.0f;
// Operator-stop slew, V/s. A zero command (serial `X`/`Q0`, a Cyphal 4050
// zero) must not short the phases against the back-EMF: at 0 V the windings
// brake as hard as the phase resistance allows (~36 A at 4 V of EMF), which is
// the audible "instant brake". Ramping the voltage down instead makes the
// wheel follow the back-EMF: the deficit stays around 0.02 V, so the braking
// current is a fraction of an amp and the stop takes ~1 s from a 4 V drive.
constexpr float simplefoc_voltage_stop_ramp_v_s = 5.0f;

// Overcurrent / stall protection over the common-mode-removed phase-current
// magnitude (App/current_guard.h). The thresholds sit above the 8 A q-current
// ceiling so a current-limited acceleration does not trip them; the windows are
// short enough that a locked rotor parks the bridge before it heats.
constexpr float simplefoc_guard_lpf_tf_s = 0.005f;
constexpr float simplefoc_oc_peak_a = 14.0f;
constexpr uint32_t simplefoc_oc_peak_ms = 25;
constexpr float simplefoc_oc_sustain_a = 10.0f;
constexpr uint32_t simplefoc_oc_sustain_ms = 250;
constexpr float simplefoc_stall_current_a = 2.0f;
constexpr float simplefoc_stall_velocity_rad_s = 0.11f;
constexpr uint32_t simplefoc_stall_ms = 600;
// How fast the open-loop channel's soft current ceiling falls while the
// filtered current is over the limit, V/s; under 80% of the limit it recovers
// at the normal voltage ramp rate.
constexpr float simplefoc_oc_backoff_v_s = 50.0f;

// Auto-calibration over the serial bench channel (`A`). The voltage driven
// into the phases while measuring R/L (small: it must stay far under the
// current limit with the rotor held by its d-axis), and the 3-phase
// series-parallel correction SimpleFOC's characteriseMotor applies to its
// readings. The current regulators are then re-tuned for this closed-loop
// bandwidth, Hz; the tuned gains live until reboot and are meant to be baked
// into simplefoc_current_kp/ki once a bench run looks good.
constexpr float simplefoc_characterise_voltage_v = 0.6f;
constexpr float simplefoc_characterise_correction = 1.5f;
constexpr float simplefoc_current_bandwidth_hz = 100.0f;

// Align-park voltage for the per-boot zero-electric-angle measurement
// (VB_SIMPLEFOC_ALIGN_ON_INIT=1). The rotor is pulled onto the q-axis of a
// fixed electrical angle and held there for ~0.7 s while the sensor is read;
// at the 0.11 ohm phase resistance 0.6 V settles under ~5.5 A, above the
// gearbox breakaway (bench: ~3.5 A starts the wheel) and well under the motor
// rating. The current guard is suspended while initFOC() runs, so this value
// is the only protection during the park.
constexpr float simplefoc_align_voltage_v = 0.6f;

}
