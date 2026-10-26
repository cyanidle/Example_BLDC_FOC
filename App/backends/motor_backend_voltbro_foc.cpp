#include "motor_backend.h"

#include <cmath>

#include "app.h"
#include "motor_tuning.h"

#include <voltbro/motors/bldc/foc/foc.hpp>

#include "cordic.h"

namespace {

namespace tuning = motor_tuning;

class VoltbroFocBackend final: public MotorBackend {
private:
    FOC motor;
    float wheel_radius;
    micros_t last_update = 0;
    bool update_scheduled = false;
    bool speed_mode = false;
    MotorFault fault = MotorFault::NONE;

public:
    VoltbroFocBackend(
        DriveRuntimeConfig& runtime_config,
        DriveInfo& drive_info,
        TIM_HandleTypeDef* htim,
        BaseInverter& inverter,
        HallSensor& hall,
        const PIDConfig& velocity_pid,
        float wheel_radius_m
    ):
        motor(
            tuning::foc_period_s,
            FiltersConfig {
                tuning::foc_expected_acceleration,
                tuning::foc_observer_g1,
                tuning::foc_observer_g2,
                tuning::foc_observer_g3,
                tuning::foc_current_lpf_coefficient
            },
            tuning::foc_current_pid_config(),
            tuning::foc_current_pid_config(),
            runtime_config,
            drive_info,
            htim,
            hall,
            inverter
        ),
        wheel_radius(wheel_radius_m)
    {
        motor.update_control_config(PIDConfig(velocity_pid));
    }

    const char* name() const override {
        return "voltbro-foc";
    }

    HAL_StatusTypeDef init() override {
        const CORDIC_ConfigTypeDef cordic_config = {
            .Function = CORDIC_FUNCTION_COSINE,
            .Scale = CORDIC_SCALE_0,
            .InSize = CORDIC_INSIZE_32BITS,
            .OutSize = CORDIC_OUTSIZE_32BITS,
            .NbWrite = CORDIC_NBWRITE_1,
            .NbRead = CORDIC_NBREAD_2,
            .Precision = CORDIC_PRECISION_10CYCLES,
        };
        const HAL_StatusTypeDef cordic_status = HAL_CORDIC_Configure(&hcordic, &cordic_config);
        if (cordic_status != HAL_OK) {
            return cordic_status;
        }
        return motor.init();
    }

    HAL_StatusTypeDef start() override {
        return motor.start();
    }
    HAL_StatusTypeDef stop() override {
        return motor.stop();
    }
    HAL_StatusTypeDef set_state(bool state) override {
        return motor.set_state(state);
    }
    bool is_on() const override {
        return motor.is_on();
    }

    void update() override {
        const micros_t now = micros_64();
        if (update_scheduled && subtract_64(now, last_update) < (int64_t)tuning::foc_period_us) {
            return;
        }
        last_update = now;
        update_scheduled = true;

        if (!motor.is_on()) {
            motor.update_sensors();
        }
        else {
            motor.update();
        }

        const float bus_voltage = motor.get_voltage();
        fault = (std::isfinite(bus_voltage) && bus_voltage > 0.0f) ?
            MotorFault::NONE : MotorFault::INVALID_SUPPLY;
    }

    void set_voltage_point(float volts) override {
        speed_mode = false;
        motor.set_voltage_point(volts);
    }

    void set_speed_target(float linear_m_s) override {
        if (!speed_mode) {
            motor.reset_control_regulator();
            speed_mode = true;
        }
        motor.set_velocity_point(linear_m_s / wheel_radius);
    }

    void run_speed_loop(float) override {
        // The FOC regulates velocity inside update(), at its own control rate.
    }

    void set_wheel_radius(float radius_m) override {
        wheel_radius = radius_m;
    }

    void set_velocity_pid(const PIDConfig& config) override {
        motor.update_control_config(PIDConfig(config));
    }

    float get_velocity() const override {
        return motor.get_velocity();
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
    return std::make_unique<VoltbroFocBackend>(
        runtime_config,
        drive_info,
        htim,
        inverter,
        hall,
        velocity_pid,
        wheel_radius_m
    );
}
