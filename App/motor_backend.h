#pragma once

#include <cassert>
#include <cstdint>
#include <memory>

#include "main.h"

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

    // Target wheel speed, m/s — the unit every command source uses (Cyphal,
    // serial `T`, the cart runtime). The façade ramps the value it actually
    // applies towards this one (see apply_speed_ramp), so a step command cannot
    // ask for an acceleration the drive cannot deliver; the six-step stall
    // watchdog reads such a jump as a stalled start.
    void set_speed_target(float linear_m_s) {
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

    // Ramp rate of the applied set-point, m/s².
    void set_speed_ramp(float m_s2) {
        assert(m_s2 > 0.0f);
        speed_ramp_m_s2 = m_s2;
    }

    // Advance the applied set-point by at most `speed_ramp_m_s2 * dt_s` towards
    // the commanded one. Call once per speed-loop period, before run_speed_loop.
    void apply_speed_ramp(float dt_s) {
        if (!(dt_s > 0.0f)) return;
        const float step = speed_ramp_m_s2 * dt_s;
        float applied = speed_applied_m_s;
        if (applied < speed_target_m_s) applied = std::min(speed_target_m_s, applied + step);
        else if (applied > speed_target_m_s) applied = std::max(speed_target_m_s, applied - step);
        if (applied == speed_applied_m_s) return;
        speed_applied_m_s = applied;
        set_wheel_speed_target(applied / wheel_radius_m);
    }

    // Set-point the drive is actually regulating right now, m/s.
    float get_applied_speed_target() const {
        return speed_applied_m_s;
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

    virtual MotorFault get_fault() const { return MotorFault::NONE; }
    virtual uint8_t get_recovery_attempts() const { return 0; }

protected:
    // Wheel angular velocity set-point, rad/s, produced by the speed ramp.
    virtual void set_wheel_speed_target(float wheel_rad_s) = 0;
    // Clear the regulator state of the running speed loop, if any.
    virtual void reset_speed_regulator() = 0;

private:
    float wheel_radius_m;
    float speed_target_m_s = 0.0f;   // commanded set-point, m/s
    float speed_applied_m_s = 0.0f;  // ramped set-point handed to the backend
    // 2 m/s² reaches the cart's top speed in about a second: the six-step
    // watchdog needs the shaft to move within NO_PROGRESS_MS of a command, and a
    // friction-bound wheel does not move at all from a 1 V step.
    float speed_ramp_m_s2 = 2.0f;
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
