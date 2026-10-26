#pragma once

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
 *   simplefoc    (default) vendored Arduino-FOC (SimpleFOC) BLDCMotor over our bridge
 *   sixstep      libvoltbro SixStepController + external velocity PID
 *   voltbro-foc  libvoltbro FOC (CORDIC, current control, internal velocity loop)
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
    virtual ~MotorBackend() = default;

    virtual const char* name() const = 0;

    virtual HAL_StatusTypeDef init() = 0;
    virtual HAL_StatusTypeDef start() = 0;
    virtual HAL_StatusTypeDef stop() = 0;
    virtual HAL_StatusTypeDef set_state(bool state) = 0;
    virtual bool is_on() const = 0;

    virtual void update() = 0;

    virtual void set_voltage_point(float volts) = 0;
    virtual void set_speed_target(float linear_m_s) = 0;
    virtual void run_speed_loop(float dt_s) = 0;
    virtual void set_wheel_radius(float radius_m) = 0;
    virtual void set_velocity_pid(const PIDConfig& config) = 0;

    virtual float get_velocity() const = 0;

    virtual MotorFault get_fault() const { return MotorFault::NONE; }
    virtual uint8_t get_recovery_attempts() const { return 0; }
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
