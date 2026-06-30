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
#include <uavcan/primitive/array/Real32_1_0.h>

#include <voltbro/eeprom/eeprom.hpp>
#include <voltbro/encoders/hall_sensor/hall_sensor.h>
#include <voltbro/motors/bldc/six_step/six_step_controller.h>
#include <voltbro/motors/bldc/six_step/six_step_speed_controller.h>


EEPROM eeprom(&hi2c4);

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

std::unique_ptr<SixStepController> motor;
std::unique_ptr<SixStepSpeedController> speed_controller;

/* ---- Runtime, CAN-tunable parameters (see ConfigSub / CONFIG_PORT below) ----
 * These have safe defaults so the node runs before any config message arrives;
 * the wheel master overwrites them at run time. */
// Naturally 4-byte aligned, so reads/writes are atomic on Cortex-M4.
static float wheel_radius = 0.05f;  // m (default: 0.1 m wheel diameter)

// Default velocity-PID gains. Output is a voltage set-point (V) per rad/s of error.
// Conservative placeholders: tune in the field via the config topic.
static PIDConfig make_default_pid_config() {
    return PIDConfig {
        .multiplier = 1.0f,
        .p_gain = 0.5f,
        .i_gain = 0.1f,
        .d_gain = 0.0f,
        .integral_error_lim = 10.0f,  // V
        .tolerance = 0.05f            // rad/s
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

    while (!eeprom.is_connected()) {
        eeprom.delay();
    }
    eeprom.delay();

    motor = std::make_unique<SixStepController>(
        0,
        DriveInfo {
            .torque_const = 0.069,  // Nm / A == V / (rad/s)
            .speed_const = 24.42,   // (rad/s) / V
            .max_current = 22,
            .max_torque = 1,
            .stall_current = 13,
            .stall_timeout = 3,
            .stall_tolerance = 0.2,
            .supply_voltage = 24,
            .l_pins = {
                GpioPin(INLA_GPIO_Port, INLA_Pin),
                GpioPin(INLB_GPIO_Port, INLB_Pin),
                GpioPin(INLC_GPIO_Port, INLC_Pin)
            },
            .en_pin = GpioPin(DRV_ENABLE_GPIO_Port, DRV_ENABLE_Pin),
            .common = {
                .ppairs = 2,
                .gear_ratio = 1
            }
        },
        &htim1,
        &hadc1,
        hall_sensor
    );

    HAL_IMPORTANT(motor->init())
    HAL_IMPORTANT(motor->start())

    speed_controller = std::make_unique<SixStepSpeedController>(
        *motor,
        PIDConfig(velocity_pid_config)
    );

    set_cyphal_mode(uavcan_node_Mode_1_0_OPERATIONAL);

    // Velocity PID is run at a fixed cadence so its dt is constant and stable.
    constexpr micros PID_PERIOD_US = 1000;  // 1 kHz
    static micros pid_time = 0;
    while(true) {
        micros now = micros_64();
        EACH_N_MICROS(now, pid_time, PID_PERIOD_US, {
            //speed_controller->update((float)PID_PERIOD_US * 1e-6f);
        })

        motor->update();

        cyphal_loop();
    }
}

TYPE_ALIAS(Natural32, uavcan_primitive_scalar_Natural32_1_0)
TYPE_ALIAS(Real32, uavcan_primitive_scalar_Real32_1_0)
TYPE_ALIAS(Real32Array, uavcan_primitive_array_Real32_1_0)
static constexpr CanardPortID ENCODER_PORT = 7100;
static constexpr CanardPortID VELOCITY_PORT = 7200;   // shaft angular velocity, rad/s
static constexpr CanardPortID LINEAR_SPEED_PORT = 7300;  // wheel linear speed, m/s
static constexpr CanardPortID SPEED_CMD_PORT = 4000;  // target wheel speed command, m/s
static constexpr CanardPortID DIRECT_SPEED_CMD_PORT = 4050;  // target shaft speed command, rad/s
static constexpr CanardPortID CONFIG_PORT = 4100;     // runtime config array (see below)

/* ---- Config topic layout: uavcan.primitive.array.Real32 on CONFIG_PORT ----
 * The wheel master publishes an array of floats; each node applies the values
 * below. Trailing entries may be omitted and keep their previous value, so the
 * array can grow without breaking older nodes.
 *
 *   index  name                 unit          meaning
 *   [0]    wheel_diameter       m             drive-wheel diameter (rad/s <-> m/s)
 *   [1]    velocity_pid_p       V/(rad/s)     proportional gain
 *   [2]    velocity_pid_i       V/(rad/s * s) integral gain
 *   [3]    velocity_pid_d       V/(rad/s / s) derivative gain
 *   [4]    velocity_pid_i_limit V             integral anti-windup clamp
 *   [5]    velocity_pid_tol     rad/s         error tolerance
 */
enum ConfigIndex : size_t {
    CFG_WHEEL_DIAMETER = 0,
    CFG_PID_P,
    CFG_PID_I,
    CFG_PID_D,
    CFG_PID_I_LIMIT,
    CFG_PID_TOLERANCE,
    CFG_COUNT
};

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

// Target wheel speed command, m/s. Converted to a shaft angular-velocity
// set-point for the velocity PID using the configured wheel radius.
class SpeedCommandSub: public AbstractSubscription<Real32> {
public:
    SpeedCommandSub(InterfacePtr interface, CanardPortID port_id): AbstractSubscription<Real32>(interface, port_id) {};
    void handler(const Real32::Type& msg, CanardRxTransfer* _) override {
        if (!speed_controller || wheel_radius <= 0.0f) {
            return;
        }
        const float target_omega = msg.value / wheel_radius;  // m/s -> rad/s
        speed_controller->set_target_velocity(target_omega);
    }
};

// Direct shaft speed command, rad/s. Fed straight to the velocity PID set-point,
// bypassing the wheel-radius conversion (handy for bring-up / tuning).
class DirectSpeedCommandSub: public AbstractSubscription<Real32> {
public:
    DirectSpeedCommandSub(InterfacePtr interface, CanardPortID port_id): AbstractSubscription<Real32>(interface, port_id) {};
    void handler(const Real32::Type& msg, CanardRxTransfer* _) override {
        motor->set_voltage_point(msg.value);
    }
};

// Runtime configuration: wheel diameter + velocity-PID gains (see layout above).
class ConfigSub: public AbstractSubscription<Real32Array> {
public:
    ConfigSub(InterfacePtr interface, CanardPortID port_id): AbstractSubscription<Real32Array>(interface, port_id) {};
    void handler(const Real32Array::Type& msg, CanardRxTransfer* _) override {
        const size_t n = msg.value.count;
        const float* v = msg.value.elements;

        if (n > CFG_WHEEL_DIAMETER && v[CFG_WHEEL_DIAMETER] > 0.0f) {
            wheel_radius = v[CFG_WHEEL_DIAMETER] * 0.5f;
        }

        // Start from the current gains so a short array only overrides what it carries.
        PIDConfig pid = velocity_pid_config;
        if (n > CFG_PID_P)         pid.p_gain = v[CFG_PID_P];
        if (n > CFG_PID_I)         pid.i_gain = v[CFG_PID_I];
        if (n > CFG_PID_D)         pid.d_gain = v[CFG_PID_D];
        if (n > CFG_PID_I_LIMIT)   pid.integral_error_lim = v[CFG_PID_I_LIMIT];
        if (n > CFG_PID_TOLERANCE) pid.tolerance = v[CFG_PID_TOLERANCE];
        velocity_pid_config = pid;

        if (speed_controller) {
            speed_controller->set_pid_config(PIDConfig(pid));
        }
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
                    static bool value = false;
                    if (v_in._tag_ == 3) {
                        value = v_in.bit.value.bitpacked[0] == 1;
                    }
                    else {
                        // TODO: report error
                    }

                    motor->set_state(value);

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

    static FDCAN_FilterTypeDef sFilterConfig;
    uint32_t filter_index = 0;
    HAL_IMPORTANT(apply_filter(
        filter_index,
        &hfdcan1,
        &sFilterConfig,
        node_info_reader->make_filter(node_id)
    ))

    filter_index += 1;
    HAL_IMPORTANT(apply_filter(
        filter_index,
        &hfdcan1,
        &sFilterConfig,
        registers_handler->make_filter(node_id)
    ))

    filter_index += 1;
    HAL_IMPORTANT(apply_filter(
        filter_index,
        &hfdcan1,
        &sFilterConfig,
        speed_cmd_sub->make_filter(node_id)
    ))

    filter_index += 1;
    HAL_IMPORTANT(apply_filter(
        filter_index,
        &hfdcan1,
        &sFilterConfig,
        direct_speed_cmd_sub->make_filter(node_id)
    ))

    filter_index += 1;
    HAL_IMPORTANT(apply_filter(
        filter_index,
        &hfdcan1,
        &sFilterConfig,
        config_sub->make_filter(node_id)
    ))
}
