#include "motor_backend.h"

#include <voltbro/motors/bldc/six_step/six_step_controller.h>
#include <voltbro/motors/bldc/six_step/six_step_speed_controller.h>

#include "app.h"

namespace {

class SixStepBackend final: public MotorBackend {
private:
    SixStepController motor;
    SixStepSpeedController speed;
    // The guard reads the Inverter's phase currents directly: the controller
    // only refreshes them while it drives, and a parked bridge must be
    // watched too (the ADC+DMA keep running).
    BaseInverter& inverter;
    bool speed_mode = false;
    // Commands as the sources issued them (the controller only sees what the
    // guard gating passes on): the "send zero to rearm" contract checks
    // these, not the drive.
    float voltage_command = 0.0f;
    float speed_command = 0.0f;
    micros_t last_update_us = 0;

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
        MotorBackend(wheel_radius_m),
        motor(runtime_config, drive_info, htim, inverter, hall),
        speed(motor, PIDConfig(velocity_pid)),
        inverter(inverter)
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
        const micros_t now = micros_64();
        const int64_t elapsed_us = last_update_us ? subtract_64(now, last_update_us) : 0;
        last_update_us = now;
        // Same dt guard as the SimpleFOC backend: a wrap or a stall long
        // enough to make dt useless skips the step for this pass.
        const float dt_s =
            (elapsed_us > 0 && elapsed_us <= 500000) ? (float)elapsed_us * 1e-6f : 0.0f;

        // "Send zero to rearm", the same contract the controller's own stall
        // fault uses.
        if (guard.is_tripped() && voltage_command == 0.0f && speed_command == 0.0f) {
            guard.reset();
        }

        motor.update();
        // Refresh the measurement even while the controller coasts (it only
        // updates the Inverter on its driving path).
        inverter.update();

        // Current guard, overcurrent part only: the six-step bridge drives
        // voltage open-loop, so a locked rotor draws V/phase-R until something
        // parks it. Stall detection stays with the controller's hall-progress
        // watchdog — its kick recovery needs the window the guard's stall
        // timer would pre-empt.
        const bool command_active = motor.is_on() &&
            ((!speed_mode && voltage_command != 0.0f) ||
             (speed_mode && speed_command != 0.0f));
        (void)guard.step(
            current_guard::phase_current_magnitude(
                inverter.get_A(), inverter.get_B(), inverter.get_C()
            ),
            motor.get_velocity(),
            command_active,
            dt_s
        );
        if (guard.is_tripped()) {
            // set_voltage_point(0) coasts and resets the controller's own
            // recovery state; the command gating below keeps a repeated
            // non-zero command from re-driving until zero rearms the guard.
            motor.set_voltage_point(0.0f);
        }
    }

    void set_voltage_point(float volts) override {
        speed_mode = false;
        voltage_command = volts;
        motor.set_voltage_point(guard.is_tripped() ? 0.0f : volts);
    }

    void run_speed_loop(float dt_s) override {
        if (speed_mode) {
            if (guard.is_tripped()) {
                motor.set_voltage_point(0.0f);
            }
            else {
                speed.update(dt_s);
            }
        }
    }

    void set_velocity_pid(const PIDConfig& config) override {
        speed.set_pid_config(PIDConfig(config));
    }

    float get_velocity() const override {
        return motor.get_velocity();
    }

    float get_wheel_angle() const override {
        return motor.get_shaft_angle();
    }

    MotorFault get_fault() const override {
        if (guard.is_tripped()) {
            return MotorFault::OVERCURRENT;
        }
        return translate(motor.get_fault());
    }

    uint8_t get_recovery_attempts() const override {
        return motor.get_recovery_attempts();
    }

    void set_current_guard(const current_guard::Config& config) override {
        current_guard::Config adjusted = config;
        // The hall-progress watchdog owns stall detection here (see update()).
        adjusted.stall_ms = 0;
        guard.configure(adjusted);
    }

protected:
    void set_wheel_speed_target(float wheel_rad_s) override {
        if (!speed_mode) {
            speed.reset();
            speed_mode = true;
        }
        speed_command = wheel_rad_s;
        speed.set_target_velocity(wheel_rad_s);
    }

    void reset_speed_regulator() override {
        speed.reset();
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
