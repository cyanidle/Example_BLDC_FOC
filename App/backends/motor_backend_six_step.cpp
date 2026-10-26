#include "motor_backend.h"

#include <voltbro/motors/bldc/six_step/six_step_controller.h>
#include <voltbro/motors/bldc/six_step/six_step_speed_controller.h>

namespace {

class SixStepBackend final: public MotorBackend {
private:
    SixStepController motor;
    SixStepSpeedController speed;
    bool speed_mode = false;
    // The speed loop regulates rad/s (its gains are documented as V per rad/s),
    // so a linear target is converted here instead of scaling the feedback down
    // to m/s, which would shrink every gain by 1/wheel_radius.
    float wheel_radius;

    static MotorFault translate(SixStepController::Fault fault) {
        switch (fault) {
            case SixStepController::Fault::NONE:
                return MotorFault::NONE;
            case SixStepController::Fault::INVALID_HALL:
                return MotorFault::INVALID_HALL;
            case SixStepController::Fault::INVALID_SUPPLY:
                return MotorFault::INVALID_SUPPLY;
            case SixStepController::Fault::INVALID_COMMAND:
                return MotorFault::INVALID_COMMAND;
            case SixStepController::Fault::STALLED:
                return MotorFault::STALLED;
        }
        return MotorFault::NONE;
    }

public:
    SixStepBackend(
        DriveRuntimeConfig& runtime_config,
        DriveInfo& drive_info,
        TIM_HandleTypeDef* htim,
        BaseInverter& inverter,
        HallSensor& hall,
        const PIDConfig& velocity_pid,
        float wheel_radius_m
    ):
        motor(runtime_config, drive_info, htim, inverter, hall),
        speed(motor, PIDConfig(velocity_pid)),
        wheel_radius(wheel_radius_m)
    {
    }

    const char* name() const override {
        return "six-step";
    }

    HAL_StatusTypeDef init() override {
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
        motor.update();
    }

    void set_voltage_point(float volts) override {
        speed_mode = false;
        motor.set_voltage_point(volts);
    }

    void set_speed_target(float linear_m_s) override {
        if (!speed_mode) {
            speed.reset();
            speed_mode = true;
        }
        speed.set_target_velocity(linear_m_s / wheel_radius);
    }

    void run_speed_loop(float dt_s) override {
        if (speed_mode) {
            speed.update(dt_s);
        }
    }

    void set_wheel_radius(float radius_m) override {
        wheel_radius = radius_m;
    }

    void set_velocity_pid(const PIDConfig& config) override {
        speed.set_pid_config(PIDConfig(config));
    }

    float get_velocity() const override {
        return motor.get_velocity();
    }

    MotorFault get_fault() const override {
        return translate(motor.get_fault());
    }

    uint8_t get_recovery_attempts() const override {
        return motor.get_recovery_attempts();
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
    return std::make_unique<SixStepBackend>(
        runtime_config,
        drive_info,
        htim,
        inverter,
        hall,
        velocity_pid,
        wheel_radius_m
    );
}
