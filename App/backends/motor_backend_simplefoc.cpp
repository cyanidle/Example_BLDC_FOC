#include "motor_backend.h"

#include <algorithm>
#include <cmath>

#include "Arduino.h"
#include "motor_tuning.h"
#include "serial_line.h"
#include "BLDCMotor.h"
#include "common/base_classes/Sensor.h"
#include "drivers/BLDCDriver3PWM.h"

#ifndef VB_SIMPLEFOC_SENSOR_DIRECTION
#define VB_SIMPLEFOC_SENSOR_DIRECTION CW
#endif

// Bench-aligned with VB_SIMPLEFOC_ALIGN_ON_INIT=1: SimpleFOC reported
// zero_electric_angle = 1.0472 rad and sensor direction CW on this module
// (Hall angles are quantised to 60 degrees electrical, hence the round value).
#ifndef VB_SIMPLEFOC_ZERO_ELECTRIC_ANGLE
#define VB_SIMPLEFOC_ZERO_ELECTRIC_ANGLE 1.04719755f
#endif

#ifndef VB_SIMPLEFOC_ALIGN_ON_INIT
// 1 = find the zero electrical angle at boot (the motor turns and needs room).
#define VB_SIMPLEFOC_ALIGN_ON_INIT 0
#endif

namespace {

namespace tuning = motor_tuning;

enum : uint8_t {
    PIN_INH_A = 0,
    PIN_INH_B = 1,
    PIN_INH_C = 2,
    PIN_INL_A = 3,
    PIN_INL_B = 4,
    PIN_INL_C = 5,
    PIN_DRV_ENABLE = 6,
};

// SimpleFOC sensor over the board's Hall decoder: the EXTI handler in app.cpp
// keeps the count, this class only converts it into SimpleFOC's mechanical
// angle and the shaft rate that its LPF_velocity then filters.
class HallSensorAdapter final: public Sensor {
private:
    HallSensor& halls;
    int32_t last_counts = 0;
    uint32_t last_tick = 0;
    bool started = false;
    // Shaft rate is measured on the motor side of the gearbox; the rest of the
    // firmware (speed targets, reported velocity, odometry) speaks wheel rad/s.
    float velocity_scale = 1.0f;

public:
    explicit HallSensorAdapter(HallSensor& hall): halls(hall) {}

    void set_velocity_scale(float scale) {
        velocity_scale = scale;
    }

    using Sensor::init;

    float getSensorAngle() override {
        return ((float)halls.get_value() / (float)halls.CPR) * _2PI;
    }

    float getVelocity() override {
        int32_t counts;
        CRITICAL_SECTION(
            counts = halls.get_revolutions() * (int32_t)halls.CPR + (int32_t)halls.get_value();
        )

        const uint32_t now = HAL_GetTick();
        const float counts_to_rad = pi2 / (float)halls.CPR;
        if (!started) {
            last_counts = counts;
            last_tick = now;
            started = true;
            return velocity;
        }

        const int32_t delta = counts - last_counts;
        const uint32_t dt_ms = now - last_tick;
        if (delta != 0) {
            if (dt_ms == 0) {
                return velocity;
            }
            const float measured = ((float)delta * counts_to_rad) / ((float)dt_ms * 1e-3f) * velocity_scale;
            velocity = measured;
            last_counts = counts;
            last_tick = now;
        }
        else if (dt_ms >= tuning::simplefoc_velocity_timeout_ms) {
            velocity = 0.0f;
            last_tick = now;
        }
        return velocity;
    }
};

class SimpleFocBackend final: public MotorBackend {
private:
    HallSensorAdapter sensor;
    BaseInverter& inverter;
    BLDCDriver3PWM driver;
    BLDCMotor motor;
    float wheel_radius;
    PIDConfig velocity_pid;
    bool speed_mode = false;
    float voltage_target = 0.0f;
    float speed_target = 0.0f;
    MotorFault fault = MotorFault::NONE;

    void apply_velocity_pid() {
        motor.PID_velocity.P = velocity_pid.kp;
        motor.PID_velocity.I = velocity_pid.ki;
        motor.PID_velocity.D = velocity_pid.kd;
        motor.PID_velocity.output_ramp = 0.0f;
        motor.PID_velocity.limit = velocity_pid.max_output;
    }

    // Centered SinePWM swings each phase around driver->voltage_limit/2, so the
    // largest undistorted q-voltage is half the driver limit; anything above it
    // only flat-tops the sine.
    static float modulation_limit(float driver_voltage_limit) {
        return std::min(
            driver_voltage_limit * 0.5f,
            tuning::simplefoc_motor_voltage_limit_v
        );
    }

public:
    SimpleFocBackend(
        DriveRuntimeConfig&,
        DriveInfo& drive_info,
        TIM_HandleTypeDef*,
        BaseInverter& inverter_,
        HallSensor& hall,
        const PIDConfig& pid_config,
        float wheel_radius_m
    ):
        sensor(hall),
        inverter(inverter_),
        driver(PIN_INH_A, PIN_INH_B, PIN_INH_C, PIN_INL_A, PIN_INL_B, PIN_INL_C),
        motor(drive_info.common.ppairs),
        wheel_radius(wheel_radius_m),
        velocity_pid(pid_config)
    {
        sensor.set_velocity_scale(1.0f / (float)drive_info.common.gear_ratio);
        arduino_pwm_pin_register(PIN_INH_A, 1);
        arduino_pwm_pin_register(PIN_INH_B, 2);
        arduino_pwm_pin_register(PIN_INH_C, 3);
        arduino_pin_register(PIN_INL_A, INLA_GPIO_Port, INLA_Pin);
        arduino_pin_register(PIN_INL_B, INLB_GPIO_Port, INLB_Pin);
        arduino_pin_register(PIN_INL_C, INLC_GPIO_Port, INLC_Pin);
        arduino_pin_register(PIN_DRV_ENABLE, DRV_ENABLE_GPIO_Port, DRV_ENABLE_Pin);
    }

    const char* name() const override {
        return "simplefoc";
    }

    HAL_StatusTypeDef init() override {
        inverter.start();
        inverter.update();
        const float bus_voltage = inverter.get_busV();
        if (!std::isfinite(bus_voltage) || bus_voltage <= 0.0f) {
            return HAL_ERROR;
        }

        driver.voltage_power_supply = bus_voltage;
        driver.voltage_limit = std::min(bus_voltage, tuning::simplefoc_driver_voltage_limit_v);
        driver.pwm_frequency = tuning::simplefoc_pwm_frequency_hz;

        motor.linkDriver(&driver);
        motor.linkSensor(&sensor);
        motor.torque_controller = TorqueControlType::voltage;
        motor.controller = MotionControlType::torque;
        motor.foc_modulation = FOCModulationType::SinePWM;
        motor.voltage_limit = modulation_limit(driver.voltage_limit);
        motor.current_limit = tuning::simplefoc_current_limit_a;
        motor.velocity_limit = tuning::simplefoc_max_shaft_velocity_rad_s;
        motor.LPF_velocity.Tf = tuning::simplefoc_velocity_lpf_tf;
#if !VB_SIMPLEFOC_ALIGN_ON_INIT
        motor.sensor_direction = VB_SIMPLEFOC_SENSOR_DIRECTION;
        motor.zero_electric_angle = VB_SIMPLEFOC_ZERO_ELECTRIC_ANGLE;
#endif
        apply_velocity_pid();

        sensor.init();
        if (driver.init() != 1) {
            return HAL_ERROR;
        }
        if (motor.init() != 1) {
            return HAL_ERROR;
        }
#if VB_SIMPLEFOC_ALIGN_ON_INIT
        // Alignment turns the rotor, so the bridge has to be live while
        // initFOC() runs; it is parked again right after.
        driver.enable();
        HAL_GPIO_WritePin(DRV_ENABLE_GPIO_Port, DRV_ENABLE_Pin, GPIO_PIN_SET);
#endif
        if (motor.initFOC() != 1) {
            return HAL_ERROR;
        }
#if VB_SIMPLEFOC_ALIGN_ON_INIT
        driver.disable();
        HAL_GPIO_WritePin(DRV_ENABLE_GPIO_Port, DRV_ENABLE_Pin, GPIO_PIN_RESET);
#endif
        motor.disable();
        return HAL_OK;
    }

    HAL_StatusTypeDef start() override {
        if (motor.enabled) {
            return HAL_OK;
        }
        serial_line_debug("simplefoc start: zero_electric_angle", motor.zero_electric_angle);
        serial_line_debug("simplefoc start: sensor_direction", (float)(int)motor.sensor_direction);
        serial_line_debug("simplefoc start: voltage_limit", motor.voltage_limit);
        driver.setPwm(0.0f, 0.0f, 0.0f);
        for (int i = 0; i < 3; ++i) {
            HAL_GPIO_WritePin(DRV_ENABLE_GPIO_Port, DRV_ENABLE_Pin, GPIO_PIN_RESET);
            HAL_Delay(10);
            HAL_GPIO_WritePin(DRV_ENABLE_GPIO_Port, DRV_ENABLE_Pin, GPIO_PIN_SET);
            HAL_Delay(10);
        }
        HAL_GPIO_WritePin(DRV_ENABLE_GPIO_Port, DRV_ENABLE_Pin, GPIO_PIN_SET);
        motor.enable();
        return HAL_OK;
    }

    HAL_StatusTypeDef stop() override {
        motor.disable();
        HAL_GPIO_WritePin(DRV_ENABLE_GPIO_Port, DRV_ENABLE_Pin, GPIO_PIN_RESET);
        return HAL_OK;
    }

    HAL_StatusTypeDef set_state(bool state) override {
        return state ? start() : stop();
    }

    bool is_on() const override {
        return motor.enabled != 0;
    }

    void update() override {
        inverter.update();
        const float bus_voltage = inverter.get_busV();
        if (std::isfinite(bus_voltage) && bus_voltage > 0.0f) {
            driver.voltage_power_supply = bus_voltage;
            driver.voltage_limit = std::min(bus_voltage, tuning::simplefoc_driver_voltage_limit_v);
            motor.voltage_limit = modulation_limit(driver.voltage_limit);
            motor.PID_velocity.limit = std::min(velocity_pid.max_output, motor.voltage_limit);
            fault = MotorFault::NONE;
        }
        else {
            fault = MotorFault::INVALID_SUPPLY;
        }

        motor.loopFOC();
        if (!speed_mode) {
            motor.move(voltage_target);
        }
    }

    void set_voltage_point(float volts) override {
        speed_mode = false;
        motor.controller = MotionControlType::torque;
        voltage_target = volts;
    }

    void set_speed_target(float linear_m_s) override {
        if (!speed_mode) {
            speed_mode = true;
            motor.controller = MotionControlType::velocity;
            motor.PID_velocity.reset();
        }
        speed_target = linear_m_s / wheel_radius;
    }

    void run_speed_loop(float) override {
        if (speed_mode) {
            motor.move(speed_target);
        }
    }

    void set_wheel_radius(float radius_m) override {
        wheel_radius = radius_m;
    }

    void set_velocity_pid(const PIDConfig& config) override {
        velocity_pid = config;
        apply_velocity_pid();
    }

    float get_velocity() const override {
        return motor.shaft_velocity;
    }

    MotorFault get_fault() const override {
        return fault;
    }
};

}

std::unique_ptr<MotorBackend> make_motor_backend(
    DriveRuntimeConfig& runtime_config,
    DriveInfo& drive_info,
    TIM_HandleTypeDef* htim,
    BaseInverter& inverter,
    HallSensor& hall,
    const PIDConfig& velocity_pid,
    float wheel_radius_m
) {
    return std::make_unique<SimpleFocBackend>(
        runtime_config,
        drive_info,
        htim,
        inverter,
        hall,
        velocity_pid,
        wheel_radius_m
    );
}
