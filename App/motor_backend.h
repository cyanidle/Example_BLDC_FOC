#pragma once

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>

#include "main.h"

#include "current_guard.h"
#include "motor_tuning.h"

#include <voltbro/devices/inverter.hpp>
#include <voltbro/encoders/hall_sensor/hall_sensor.h>
#include <voltbro/math/regulators/pid.hpp>
#include <voltbro/motors/bldc/bldc.h>

/*
 * Backend-independent face of the wheel-drive control loop. Exactly one
 * implementation is compiled in, selected by VB_MOTOR_IMPL in CMakeLists.txt:
 *
 *   sixstep      (default) libvoltbro SixStepController + external velocity PID
 *   simplefoc    vendored Arduino-FOC (SimpleFOC) BLDCMotor over our bridge
 *   voltbro-foc  libvoltbro FOC (CORDIC, current control, internal velocity loop)
 *
 * Speed is always wheel linear velocity on the outside of this class: the
 * façade converts m/s to wheel angular velocity (rad/s) with the wheel radius,
 * so a backend and its regulator only ever work in wheel rad/s and the radius
 * never has to be re-derived per implementation. Travel follows the same rule:
 * accumulated wheel linear distance in m.
 */
enum class MotorFault: uint8_t {
    NONE,
    INVALID_HALL,
    INVALID_SUPPLY,
    INVALID_COMMAND,
    STALLED,
    OVERCURRENT,
    // The gate driver's own FAULT output is asserted: it has switched the
    // bridge off by itself. Latched until the driver is re-armed (`E1`).
    DRIVER_FAULT,
};

// Loop internals for the serial diagnostics line (`V1`): one snapshot per
// telemetry tick. Everything is in the loop's own units so the numbers can be
// compared against the set-points the operator sent.
struct MotorDiagnostics {
    float speed_target_m_s = 0.0f;     // ramped speed set-point, m/s
    float q_current_setpoint_a = 0.0f; // velocity PID output (q-current demand)
    float q_current_a = 0.0f;          // filtered q (torque) current
    float d_current_a = 0.0f;          // filtered d current (should stay ~0)
    // Filtered phase-current magnitude. This is the input of the overcurrent
    // and stall guards; it shows a current the d/q split hides, e.g. a locked
    // rotor parking its current on the d axis.
    float phase_current_a = 0.0f;
    float q_voltage_v = 0.0f;          // q voltage the current loop applies
    float d_voltage_v = 0.0f;
    float bus_voltage_v = 0.0f;        // inverter bus reading, V
    int32_t hall_counts = 0;           // absolute rotor count since boot
};

class MotorBackend {
public:
    explicit MotorBackend(float wheel_radius_m): wheel_radius_m(wheel_radius_m) {
        assert(wheel_radius_m > 0.0f);
    }
    virtual ~MotorBackend() = default;

    virtual const char* name() const = 0;

    virtual HAL_StatusTypeDef init() = 0;
    virtual HAL_StatusTypeDef start() = 0;
    virtual HAL_StatusTypeDef stop() = 0;
    virtual HAL_StatusTypeDef set_state(bool state) = 0;
    virtual bool is_on() const = 0;

    virtual void update() = 0;

    virtual void set_voltage_point(float volts) = 0;

    // Target wheel speed, m/s — the unit the backend works in. Command sources
    // (serial `T`, the cart runtime) send a NORMALIZED -1..1 set-point through
    // set_speed_target_normalized(); this form is for internal use (a stop, the
    // tests) and supersedes a normalized command, so a later full-scale change
    // cannot move a target that was given in m/s. The façade ramps the value it
    // actually applies towards this one (see apply_speed_ramp), so a step
    // command cannot ask for an acceleration the drive cannot deliver; the
    // six-step stall watchdog reads such a jump as a stalled start.
    void set_speed_target(float linear_m_s) {
        normalized_target = std::numeric_limits<float>::quiet_NaN();
        apply_target(linear_m_s);
    }

    // Normalized speed command, -1..1, scaled by the configured full-scale
    // wheel speed (config id 16). This is the shape every command source uses:
    // the module owns the m/s mapping (wheel radius, gearbox, EMF constant,
    // modulation ceiling), so a source cannot command a speed the drive has no
    // voltage to regulate. Out-of-range or non-finite commands are clamped.
    void set_speed_target_normalized(float command) {
        normalized_target = std::isfinite(command) ? std::clamp(command, -1.0f, 1.0f) : 0.0f;
        apply_target(normalized_target * max_speed_m_s);
    }

    // Full-scale wheel speed the -1..1 commands map onto, m/s (config id 16).
    // A normalized command is a fraction of this scale, so changing it while
    // the drive runs moves the latched target with it — otherwise the operator
    // lowers the full scale and nothing slows down until the source re-issues
    // its command.
    void set_max_speed(float m_s) {
        if (!(std::isfinite(m_s) && m_s > 0.0f)) return;
        max_speed_m_s = m_s;
        if (std::isfinite(normalized_target)) {
            apply_target(normalized_target * max_speed_m_s);
        }
    }

    float get_max_speed() const {
        return max_speed_m_s;
    }

    // Ramp rate of the applied set-point, m/s².
    void set_speed_ramp(float m_s2) {
        assert(m_s2 > 0.0f);
        speed_ramp_m_s2 = m_s2;
    }

    // Advance the applied set-point by at most `speed_ramp_m_s2 * dt_s` towards
    // the commanded one. Call once per speed-loop period, before run_speed_loop.
    //
    // The commanded value is clamped to the ceiling the backend reports
    // (get_speed_ceiling): past it the current loops have no voltage authority
    // and the phase current is set by the back-EMF instead, which is how a
    // high target used to end at the gate driver's own fault. A clamped target
    // is visible through is_target_clamped().
    void apply_speed_ramp(float dt_s) {
        if (!(dt_s > 0.0f)) return;
        // The commanded value is held at the smaller of the configured
        // full-scale speed and the live ceiling; the ceiling is what the drive
        // can actually regulate, the configuration is what the operator asked
        // for.
        const float ceiling = std::min(speed_ceiling_m_s, max_speed_m_s);
        const float target = std::clamp(speed_target_m_s, -ceiling, ceiling);
        target_clamped = std::abs(target - speed_target_m_s) > 1.0e-6f;
        const float step = speed_ramp_m_s2 * dt_s;
        float applied = speed_applied_m_s;
        if (applied < target) applied = std::min(target, applied + step);
        else if (applied > target) applied = std::max(target, applied - step);
        if (applied == speed_applied_m_s) return;
        speed_applied_m_s = applied;
        set_wheel_speed_target(applied / wheel_radius_m);
    }

    // Set-point the drive is actually regulating right now, m/s.
    float get_applied_speed_target() const {
        return speed_applied_m_s;
    }

    // Set-point the caller last commanded, m/s (before the ceiling clamp).
    float get_speed_target() const {
        return speed_target_m_s;
    }

    // Largest wheel speed the backend can still regulate, m/s. The backend
    // derives it from the live modulation ceiling and the drive's EMF constant
    // and reports it here (SimpleFOC does it on every pass); infinite on a
    // backend with no such model.
    float get_speed_ceiling() const {
        return speed_ceiling_m_s;
    }

    // True while the commanded target is being held at that ceiling.
    bool is_target_clamped() const {
        return target_clamped;
    }

    // Radius the façade converts with, m.
    float wheel_radius() const {
        return wheel_radius_m;
    }

    // The wheel diameter arrives over the config topic; every m/s value on this
    // interface is converted with this radius.
    void set_wheel_radius(float radius_m) {
        // Debug-only: the config path rejects a non-positive diameter.
        assert(radius_m > 0.0f);
        wheel_radius_m = radius_m;
    }

    virtual void run_speed_loop(float dt_s) = 0;
    virtual void set_velocity_pid(const PIDConfig& config) = 0;

    // Measured-velocity low-pass that the velocity PID closes over, s
    // (SimpleFOC's LPF_velocity). It is part of the loop: a time constant that
    // is long against the acceleration the drive delivers shows up as a
    // set-point error while ramping and as overshoot when the ramp ends. Bench
    // knob (serial `L`); false when the backend has no such filter.
    virtual bool set_velocity_lpf(float tf_s) {
        (void)tf_s;
        return false;
    }

    // Slew of the velocity loop's output (the q-current set-point), A/s: the
    // soft start that keeps a PID step from slamming the bridge. Config id 14;
    // false when the backend's speed loop has no output ramp.
    virtual bool set_torque_ramp(float a_s) {
        (void)a_s;
        return false;
    }

    // Braking-current ceiling, A: the largest torque the speed loop may demand
    // against the measured rotation. That torque is regenerative — it pumps
    // energy back into the rail — so on a supply that cannot sink it this
    // stays far under the drive's current limit, and 0 leaves a moving wheel
    // to coast. Config id 15; false when the backend does not clamp by motion.
    virtual bool set_brake_current(float a) {
        (void)a;
        return false;
    }

    // One-shot bench calibration over the serial `A` command: characterise the
    // motor's R/L from the live phase currents and re-tune the current
    // regulators from them (SimpleFOC's characteriseMotor/tuneCurrentController).
    // False when the backend cannot (no current sense) or will not (driver
    // disabled) run it; the measured values arrive as serial debug lines. It
    // blocks for seconds with the bridge energised — bench use, wheel lifted.
    virtual bool auto_calibrate() { return false; }

    // Wheel angular velocity, rad/s.
    virtual float get_velocity() const = 0;

    // Wheel (output shaft) angle, rad — accumulated since boot, signed.
    virtual float get_wheel_angle() const = 0;

    // Accumulated wheel travel, m (wheel angle * wheel radius), signed. A
    // consumer integrates differences of it, so a runtime wheel-radius change
    // rescales the whole history.
    float get_travel() const {
        return get_wheel_angle() * wheel_radius_m;
    }

    // Wheel linear velocity, m/s.
    float get_linear_velocity() const {
        return get_velocity() * wheel_radius_m;
    }

    // Radius the façade converts with, m — the config topic sets it and the
    // EEPROM keeps the last value, so it is also reported back.
    float get_wheel_radius() const {
        return wheel_radius_m;
    }

    // Open-loop voltage the backend is applying right now, V (magnitude; 0 on a
    // backend without the channel, and 0 while the speed loop owns the bridge).
    virtual float get_voltage_point() const { return 0.0f; }

    virtual MotorFault get_fault() const { return MotorFault::NONE; }
    virtual uint8_t get_recovery_attempts() const { return 0; }

    // Measured q-axis (torque) current, A, for bench telemetry. NAN when the
    // backend has no current measurement (six-step); the serial `S` line
    // appends an ` i=<A>` field only when this returns a finite value.
    // Non-const because a backend may sample hardware on the call; the
    // SimpleFOC implementation returns a value cached by update().
    virtual float get_torque_current() { return NAN; }

    // Bench diagnostics of the drive loop (serial `V1` mode). They separate a
    // velocity-loop limit cycle — set-point and measured q-current swinging
    // together — from a current-loop or angle fault, where the measured
    // current stops following its set-point or a large d-axis current appears.
    // False when the backend cannot provide them.
    virtual bool get_diagnostics(MotorDiagnostics& diagnostics) {
        (void)diagnostics;
        return false;
    }

    // Runtime overcurrent/stall protection parameters (CAN config topic /
    // serial `C`, EEPROM-backed). Backends feed the guard from their current
    // measurement in update(); a backend without one leaves it inert.
    virtual void set_current_guard(const current_guard::Config& config) {
        guard.configure(config);
    }

protected:
    // Overcurrent / stall protection, shared by the backends that measure
    // phase current (SimpleFOC's current sense, the six-step Inverter front
    // end). Defaults come from motor_tuning.h; set_current_guard() overrides.
    current_guard::Guard guard{
        current_guard::Config{
            .lpf_tf_s = motor_tuning::simplefoc_guard_lpf_tf_s,
            .peak_a = motor_tuning::simplefoc_oc_peak_a,
            .peak_ms = motor_tuning::simplefoc_oc_peak_ms,
            .sustain_a = motor_tuning::simplefoc_oc_sustain_a,
            .sustain_ms = motor_tuning::simplefoc_oc_sustain_ms,
            .stall_current_a = motor_tuning::simplefoc_stall_current_a,
            .stall_velocity_rad_s = motor_tuning::simplefoc_stall_velocity_rad_s,
            .stall_ms = motor_tuning::simplefoc_stall_ms,
        }
    };

    // Backends report the largest wheel speed they can regulate here (SimpleFOC:
    // motor_limits::reachable_wheel_rate of the live modulation ceiling). A
    // non-finite or non-positive report leaves the drive uncompelled to stop.
    void set_speed_ceiling(float m_s) {
        speed_ceiling_m_s = (std::isfinite(m_s) && m_s > 0.0f) ? m_s : 0.0f;
    }

    // Wheel angular velocity set-point, rad/s, produced by the speed ramp.
    virtual void set_wheel_speed_target(float wheel_rad_s) = 0;
    // Clear the regulator state of the running speed loop, if any.
    virtual void reset_speed_regulator() = 0;

private:
    // Single place both command forms go through, so the immediate-stop rule
    // cannot be bypassed by one of them.
    void apply_target(float linear_m_s) {
        speed_target_m_s = linear_m_s;
        if (linear_m_s == 0.0f) {
            // A stop must not wait for the ramp. The regulator is cleared too:
            // the loop is positional (its output is the voltage set-point), so
            // an integral wound up while driving would push the wheel forward
            // against the brake and the wheel would coast for seconds.
            speed_applied_m_s = 0.0f;
            set_wheel_speed_target(0.0f);
            reset_speed_regulator();
        }
    }

    float wheel_radius_m;
    float speed_target_m_s = 0.0f;   // commanded set-point, m/s
    // Last normalized command, -1..1; NaN when the last command was given in
    // m/s (then a full-scale change must not move it).
    float normalized_target = std::numeric_limits<float>::quiet_NaN();
    float speed_applied_m_s = 0.0f;  // ramped set-point handed to the backend
    // Ceiling the backend reports (see get_speed_ceiling); the routed target is
    // clamped to it. Infinite until a backend says otherwise.
    float speed_ceiling_m_s = std::numeric_limits<float>::infinity();
    // Full-scale speed a normalized -1..1 command maps onto, m/s (config id 16).
    // Starts from motor_tuning::default_max_speed_m_s, the same value app.cpp
    // boots with; a module replaces it with the speed its own rail can regulate
    // right after init, and the config topic/EEPROM can lower it further.
    float max_speed_m_s = motor_tuning::default_max_speed_m_s;
    bool target_clamped = false;
    // The slope is a compromise: the six-step watchdog needs Hall progress
    // within NO_PROGRESS_MS (500 ms) of a command and a friction-bound wheel
    // does not move at all from a 1 V step, while a steeper ramp lurches the
    // cart. 0.75 m/s² still asks for 0.375 m/s of set-point at that deadline,
    // which saturates the demand on a stalled wheel.
    float speed_ramp_m_s2 = 0.75f;
};

std::unique_ptr<MotorBackend> make_motor_backend(
    DriveRuntimeConfig& runtime_config,
    DriveInfo& drive_info,
    TIM_HandleTypeDef* htim,
    BaseInverter& inverter,
    HallSensor& hall,
    const PIDConfig& velocity_pid,
    float wheel_radius_m
);
