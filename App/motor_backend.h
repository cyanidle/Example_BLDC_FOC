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
 * never has to be re-derived per implementation.
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
    // serial `T`, the cart runtime).
    void set_speed_target(float linear_m_s) {
        set_wheel_speed_target(linear_m_s / wheel_radius_m);
    }

    // Wheel diameter changes over the config topic; the radius converts every
    // following m/s command and every reported m/s value.
    void set_wheel_radius(float radius_m) {
        assert(radius_m > 0.0f);
        wheel_radius_m = radius_m;
    }

    virtual void run_speed_loop(float dt_s) = 0;
    virtual void set_velocity_pid(const PIDConfig& config) = 0;

    // Wheel angular velocity, rad/s.
    virtual float get_velocity() const = 0;

    // Wheel linear velocity, m/s (wheel rad/s * wheel radius).
    float get_linear_velocity() const {
        return get_velocity() * wheel_radius_m;
    }

    virtual MotorFault get_fault() const { return MotorFault::NONE; }
    virtual uint8_t get_recovery_attempts() const { return 0; }

protected:
    // Wheel angular velocity set-point, rad/s, produced by set_speed_target().
    virtual void set_wheel_speed_target(float wheel_rad_s) = 0;

private:
    float wheel_radius_m;
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
