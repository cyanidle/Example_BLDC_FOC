#include "app.h"

#include <memory>

#include "tim.h"
#include "i2c.h"
#include "adc.h"
#include "fdcan.h"

#include <cyphal/node/node_info_handler.h>
#include <cyphal/node/registers_handler.hpp>
#include <cyphal/providers/G4CAN.h>

#include <uavcan/node/Mode_1_0.h>
#include <uavcan/si/unit/angle/Scalar_1_0.h>
#include <uavcan/primitive/scalar/Real32_1_0.h>
#include <uavcan/primitive/scalar/Natural32_1_0.h>
#include <uavcan/primitive/array/Integer32_1_0.h>

#include <voltbro/eeprom/eeprom.hpp>
#include <voltbro/encoders/hall_sensor/hall_sensor.h>
#include <voltbro/devices/inverter.hpp>
#include <voltbro/motors/bldc/six_step/six_step_controller.h>
#include <voltbro/motors/bldc/six_step/six_step_speed_controller.h>


EEPROM eeprom(&hi2c4);

// Hall channel order is (ENC_1, ENC_3, ENC_2): get_encoder_step() weights the
// three channels by the fixed `sequence` below, so this order is what maps the
// rotor position to the correct commutation step. Putting the channels in plain
// 1-2-3 order scrambles commutation (the motor won't drive) while leaving the
// velocity estimate intact — do not "tidy" it.
HallSensor hall_sensor(
    12,
    false,
    ENC_1_GPIO_Port,
    ENC_1_Pin,
    ENC_2_GPIO_Port,
    ENC_2_Pin,
    ENC_3_GPIO_Port,
    ENC_3_Pin,
    {
        HallPhase::PHASE_B,
        HallPhase::PHASE_C,
        HallPhase::PHASE_A
    }
);

// Current-sense / bus-voltage front end. The six-step controller reads bus
// voltage from here to scale its PWM, so it must be constructed and handed to
// the motor (motor->init() starts its ADC+DMA).
Inverter inverter(&hadc1);
DriveRuntimeConfig drive_runtime_config{};

DriveInfo drive_info {
    .torque_const = 0.069f,  // Nm / A == V / (rad/s)
    .max_current = 22,
    .max_torque = 1,
    .stall_current = 13,
    .stall_timeout = 3,
    .stall_tolerance = 0.2f,
    .calibration_voltage = 0.0f,
    .l_pins = std::array<GpioPin, 3>{
        GpioPin(INLA_GPIO_Port, INLA_Pin),
        GpioPin(INLB_GPIO_Port, INLB_Pin),
        GpioPin(INLC_GPIO_Port, INLC_Pin)
    },
    .en_pin = GpioPin(DRV_ENABLE_GPIO_Port, DRV_ENABLE_Pin),
    .common = {
        .ppairs = 2,
        .gear_ratio = 1
    }
};

std::unique_ptr<SixStepController> motor;
std::unique_ptr<SixStepSpeedController> speed_controller;

// Active control mode. DIRECT_VOLTAGE feeds open-loop voltage straight to the
// motor (bring-up / tuning); SPEED runs the velocity PID. A command on either
// topic switches the mode, so the two never fight over the voltage set-point.
// A single byte: reads/writes are inherently atomic on Cortex-M4.
enum class ControlMode : uint8_t { DIRECT_VOLTAGE, SPEED };
static volatile ControlMode control_mode = ControlMode::DIRECT_VOLTAGE;

/* ---- Runtime, CAN-tunable parameters (see ConfigSub / CONFIG_PORT below) ----
 * These have safe defaults so the node runs before any config message arrives;
 * the wheel master overwrites them at run time. */
// Naturally 4-byte aligned, so reads/writes are atomic on Cortex-M4.
static float wheel_radius = 0.05f;  // m (default: 0.1 m wheel diameter)

// Default velocity-PID gains. Output is a voltage set-point (V) per rad/s of error.
// Conservative placeholders: tune in the field via the config topic.
static PIDConfig make_default_pid_config() {
    return PIDConfig {
        .kp = 0.5f,
        .ki = 0.1f,
        .kd = 0.0f,
        .integral_error_lim = 10.0f,  // cap on accumulated error (rad/s * s)
        // Output is a voltage set-point: clamp it to the supply rail. Leaving
        // these at the +/-FLT_MAX default also disables the regulator's
        // saturation anti-windup (its "raw < max_output" test never trips).
        .max_output = 24.0f,          // V
        .min_output = -24.0f          // V
    };
}
static PIDConfig velocity_pid_config = make_default_pid_config();

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == ENC_1_Pin || GPIO_Pin == ENC_2_Pin || GPIO_Pin == ENC_3_Pin) {
        hall_sensor.handle_hall_channel(GPIO_Pin);
    }
}

static volatile encoder_data enc_val = 0;
static volatile int enc_rev = 0;
[[noreturn]] void app() {
    start_timers();
    start_cyphal();

    eeprom.wait_until_available();

    motor = std::make_unique<SixStepController>(
        drive_runtime_config,
        drive_info,
        &htim1,
        inverter,
        hall_sensor
    );

    HAL_IMPORTANT(motor->init())
    HAL_IMPORTANT(motor->start())

    speed_controller = std::make_unique<SixStepSpeedController>(
        *motor,
        PIDConfig(velocity_pid_config)
    );
    // Regulate linear wheel speed (m/s): scale the angular feedback by radius.
    speed_controller->set_feedback_scale(wheel_radius);

    set_cyphal_mode(uavcan_node_Mode_1_0_OPERATIONAL);

    // Velocity PID is run at a fixed cadence so its dt is constant and stable.
    constexpr micros PID_PERIOD_US = 5000;  // 200 Hz
    static micros pid_time = 0;
    while(true) {
        micros now = micros_64();

        EACH_N_MICROS(now, pid_time, PID_PERIOD_US, {
            if (control_mode == ControlMode::SPEED) {
                speed_controller->update((float)PID_PERIOD_US * 1e-6f);
            }
        })

        motor->update();

        cyphal_loop();
    }
}

TYPE_ALIAS(Natural32, uavcan_primitive_scalar_Natural32_1_0)
TYPE_ALIAS(Real32, uavcan_primitive_scalar_Real32_1_0)
TYPE_ALIAS(Int32Array, uavcan_primitive_array_Integer32_1_0)
static constexpr CanardPortID ENCODER_PORT = 7100;
static constexpr CanardPortID VELOCITY_PORT = 7200;   // shaft angular velocity, rad/s
static constexpr CanardPortID LINEAR_SPEED_PORT = 7300;  // wheel linear speed, m/s
static constexpr CanardPortID SPEED_CMD_PORT = 4000;  // target wheel speed command, m/s
static constexpr CanardPortID DIRECT_SPEED_CMD_PORT = 4050;  // open-loop voltage command, V
static constexpr CanardPortID CONFIG_PORT = 4100;     // runtime config (see ConfigSub)

/* ---- Config topic: uavcan.primitive.array.Integer32 on CONFIG_PORT ----
 * Exactly three elements: { id, numerator, denominator }. The node applies
 *     value = numerator / denominator
 * to the parameter selected by `id`. Sending one rational at a time lets any
 * float be set at any moment without a fixed-layout array.
 * Keep this enum in sync with config_defs.lua in the repo root. */
enum ConfigId : int32_t {
    CFG_WHEEL_DIAMETER = 0,  // m,            drive-wheel diameter (rad/s <-> m/s)
    CFG_PID_KP        = 1,   // V/(rad/s),    proportional gain
    CFG_PID_KI        = 2,   // V/(rad/s * s),integral gain
    CFG_PID_KD        = 3,   // V/(rad/s / s),derivative gain
    CFG_PID_I_LIMIT   = 4,   // V,            integral anti-windup clamp
};

// Apply a single decoded config value to the live parameters.
static void apply_config_value(int32_t id, float value) {
    if (id == CFG_WHEEL_DIAMETER) {
        if (value > 0.0f) {
            wheel_radius = value * 0.5f;
            // Keep the linear-speed PID feedback scale in sync with the radius.
            if (speed_controller) {
                speed_controller->set_feedback_scale(wheel_radius);
            }
        }
        return;
    }

    PIDConfig pid = velocity_pid_config;
    switch (id) {
        case CFG_PID_KP:        pid.kp = value; break;
        case CFG_PID_KI:        pid.ki = value; break;
        case CFG_PID_KD:        pid.kd = value; break;
        case CFG_PID_I_LIMIT:   pid.integral_error_lim = value; break;
        default: return;  // unknown id: ignore
    }
    velocity_pid_config = pid;
    if (speed_controller) {
        speed_controller->set_pid_config(PIDConfig(velocity_pid_config));
    }
}

void in_loop_reporting(millis current_t) {
    static millis report_time = 0;
    static const auto node_id = get_node_id();
    if ((current_t - report_time) >= (50)) {
        Natural32 ::Type enc_msg = {};
        enc_msg.value = hall_sensor.get_value();
        static CanardTransferID enc_transfer_id = 0;
        get_interface()->send_msg<Natural32>(&enc_msg, ENCODER_PORT + node_id, &enc_transfer_id);

        const float angular_velocity = motor->get_velocity();  // rad/s

        Real32::Type velocity_msg = {};
        velocity_msg.value = angular_velocity;
        static CanardTransferID velocity_transfer_id = 0;
        get_interface()->send_msg<Real32>(&velocity_msg, VELOCITY_PORT + node_id, &velocity_transfer_id);

        // Linear wheel speed from angular velocity and wheel radius: v = w * r.
        Real32::Type speed_msg = {};
        speed_msg.value = angular_velocity * wheel_radius;  // m/s
        static CanardTransferID speed_transfer_id = 0;
        get_interface()->send_msg<Real32>(&speed_msg, LINEAR_SPEED_PORT + node_id, &speed_transfer_id);

        report_time = current_t;
    }
}

// Target wheel speed command, m/s. Switches into closed-loop SPEED mode; the
// controller regulates linear wheel speed directly (feedback scaled by radius).
class SpeedCommandSub: public AbstractSubscription<Real32> {
public:
    SpeedCommandSub(InterfacePtr interface, CanardPortID port_id): AbstractSubscription<Real32>(interface, port_id) {};
    void handler(const Real32::Type& msg, CanardRxTransfer* _) override {
        if (!speed_controller) {
            return;
        }
        speed_controller->set_target_velocity(msg.value);  // m/s
        control_mode = ControlMode::SPEED;
    }
};

// Direct open-loop voltage command, V. Switches into DIRECT_VOLTAGE mode (the
// velocity PID stops) and feeds the voltage straight to the motor — for
// bring-up / tuning.
class DirectSpeedCommandSub: public AbstractSubscription<Real32> {
public:
    DirectSpeedCommandSub(InterfacePtr interface, CanardPortID port_id): AbstractSubscription<Real32>(interface, port_id) {};
    void handler(const Real32::Type& msg, CanardRxTransfer* _) override {
        control_mode = ControlMode::DIRECT_VOLTAGE;
        motor->set_voltage_point(msg.value);
    }
};

// Runtime configuration: one { id, numerator, denominator } triple per message.
class ConfigSub: public AbstractSubscription<Int32Array> {
public:
    ConfigSub(InterfacePtr interface, CanardPortID port_id): AbstractSubscription<Int32Array>(interface, port_id) {};
    void handler(const Int32Array::Type& msg, CanardRxTransfer* _) override {
        if (msg.value.count < 3) {
            return;
        }
        const int32_t id          = msg.value.elements[0];
        const int32_t numerator   = msg.value.elements[1];
        const int32_t denominator = msg.value.elements[2];
        if (denominator == 0) {
            return;
        }
        apply_config_value(id, (float)numerator / (float)denominator);
    }
};

ReservedObject<NodeInfoReader> node_info_reader;
ReservedObject<RegistersHandler<1>> registers_handler;
ReservedObject<SpeedCommandSub> speed_cmd_sub;
ReservedObject<DirectSpeedCommandSub> direct_speed_cmd_sub;
ReservedObject<ConfigSub> config_sub;

void setup_subscriptions() {
    HAL_FDCAN_ConfigGlobalFilter(
        &hfdcan1,
        FDCAN_REJECT,
        FDCAN_REJECT,
        FDCAN_REJECT_REMOTE,
        FDCAN_REJECT_REMOTE
    );

    auto cyphal_interface = get_interface();
    const auto node_id = get_node_id();

    speed_cmd_sub.create(cyphal_interface, SPEED_CMD_PORT + node_id);
    direct_speed_cmd_sub.create(cyphal_interface, DIRECT_SPEED_CMD_PORT + node_id);
    config_sub.create(cyphal_interface, CONFIG_PORT + node_id);

    node_info_reader.create(
        cyphal_interface,
        "org.voltbro.bldc_6step_example",
        uavcan_node_Version_1_0{1, 0},
        uavcan_node_Version_1_0{1, 0},
        uavcan_node_Version_1_0{1, 0},
        0
    );
    registers_handler.create(
        std::array<RegisterDefinition, 1>{{
            {
                "motor.is_on",
                [](
                    const uavcan_register_Value_1_0& v_in,
                    uavcan_register_Value_1_0& v_out,
                    RegisterAccessResponse::Type& response
                ){
                    // Only a write (bit value, tag 3) changes the motor state.
                    // A read carries an empty value; it must just report the
                    // current state — calling set_state() here would disable the
                    // motor on every register read/list and make it ignore all
                    // setpoint commands.
                    if (v_in._tag_ == 3) {
                        motor->set_state(v_in.bit.value.bitpacked[0] == 1);
                    }

                    response.persistent = true;
                    response._mutable = true;
                    v_out._tag_ = 3;
                    v_out.bit.value.bitpacked[0] = motor->is_on();
                    v_out.bit.value.count = 1;
                }
            }
        }},
        cyphal_interface
    );

    // One extended-ID acceptance filter per RX subscription. Cyphal uses
    // extended IDs and the global filter rejects everything unmatched, so a
    // missing slot = a silently dead topic. Publishers are TX-only and need no
    // filter. Keep this list <= hfdcan1.Init.ExtFiltersNbr (see fdcan.c; the
    // G4 hardware max is 8).
    const CanardFilter rx_filters[] = {
        node_info_reader->make_filter(node_id),
        registers_handler->make_filter(node_id),
        direct_speed_cmd_sub->make_filter(node_id),
        speed_cmd_sub->make_filter(node_id),
        config_sub->make_filter(node_id),
    };
    static_assert(
        sizeof(rx_filters) / sizeof(rx_filters[0]) <= 8,
        "more RX subscriptions than FDCAN extended filter slots (max 8); "
        "raise hfdcan1.Init.ExtFiltersNbr or drop a subscription"
    );

    static FDCAN_FilterTypeDef sFilterConfig;
    for (uint32_t i = 0; i < sizeof(rx_filters) / sizeof(rx_filters[0]); ++i) {
        HAL_IMPORTANT(apply_filter(i, &hfdcan1, &sFilterConfig, rx_filters[i]))
    }
}
