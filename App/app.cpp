#include "app.h"

#include <memory>
#include <cstdio>

#include "tim.h"
#include "i2c.h"
#include "adc.h"
#include "fdcan.h"

#include <cyphal/node/node_info_handler.h>
#include <cyphal/node/registers_handler.hpp>
#include <cyphal/providers/G4CAN.h>

#include <uavcan/node/Mode_1_0.h>
#include <uavcan/node/Health_1_0.h>
#include <uavcan/si/unit/angle/Scalar_1_0.h>
#include <uavcan/primitive/scalar/Real32_1_0.hpp>
#include <uavcan/primitive/scalar/Natural32_1_0.hpp>
#include <uavcan/primitive/array/Integer32_1_0.hpp>
#include <uavcan/diagnostic/Record_1_1.hpp>

#include <voltbro/eeprom/eeprom.hpp>
#include <voltbro/encoders/hall_sensor/hall_sensor.h>
#include <voltbro/devices/inverter.hpp>

#include "motor_backend.h"
#include "motor_tuning.h"


EEPROM eeprom(&hi2c4);

// Hall channels are wired ENC_1 -> pin_1, ENC_2 -> pin_2, ENC_3 -> pin_3, with
// the phase weights (B, C, A) below mapping the 3-bit Hall state onto the
// six-step commutation table. This order is hardware-dependent: if a motor
// coggs or refuses to drive, verify its Hall harness permutation first.
// CPR is one count per Hall state: six per electrical turn, so 6 * pole pairs.
HallSensor hall_sensor(
    24,
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

// Current-sense / bus-voltage front end. Every backend reads bus voltage from
// here to scale its PWM, so it must be constructed and handed to the motor
// (the backend's init() starts its ADC+DMA).
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
        .ppairs = 4,
        .gear_ratio = 1
    }
};

std::unique_ptr<MotorBackend> motor;

/* ---- Runtime, CAN-tunable parameters (see ConfigSub / CONFIG_PORT below) ----
 * These have safe defaults so the node runs before any config message arrives;
 * the wheel master overwrites them at run time. */
// Naturally 4-byte aligned, so reads/writes are atomic on Cortex-M4.
static float wheel_radius = 0.05f;  // m (default: 0.1 m wheel diameter)

// Default velocity-PID gains (see motor_tuning.h for the per-implementation
// values). Overridden at run time by the config topic.
static PIDConfig make_default_pid_config() {
    return motor_tuning::default_velocity_pid_config();
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

    // The HallSensor ctor runs before GPIO clocks/inputs are initialized, so its
    // cached state is meaningless until we re-sample the pins here.
    hall_sensor.resync();

    // The EEPROM is only probed for presence; nothing is read from it later, so
    // a missing/stuck EEPROM must not block motor initialization forever.
    {
        const millis_t eeprom_wait_start = millis_32();
        while (!eeprom.is_connected() && (millis_32() - eeprom_wait_start) < 500) {
            HAL_Delay(2);
        }
    }

    motor = make_motor_backend(
        drive_runtime_config,
        drive_info,
        &htim1,
        inverter,
        hall_sensor,
        PIDConfig(velocity_pid_config),
        wheel_radius
    );

    HAL_IMPORTANT(motor->init())
    HAL_IMPORTANT(motor->start())


    set_cyphal_mode(uavcan_node_Mode_1_0_OPERATIONAL);

    // Velocity PID is run at a fixed cadence so its dt is constant and stable.
    constexpr micros_t PID_PERIOD_US = 5000;  // 200 Hz
    static micros_t pid_time = 0;
    while(true) {
        micros_t now = micros_64();

        EACH_N_MICROS(now, pid_time, PID_PERIOD_US, {
            motor->run_speed_loop((float)PID_PERIOD_US * 1e-6f);
        })

        motor->update();


        const millis_t now_ms = millis_32();

        static millis_t fault_check_time = 0;
        if ((now_ms - fault_check_time) >= 50) {
            fault_check_time = now_ms;
            const bool driver_faulted =
                HAL_GPIO_ReadPin(DRV_FAULT_GPIO_Port, DRV_FAULT_Pin) == GPIO_PIN_RESET;
            const auto fault = motor->get_fault();
            set_cyphal_health(
                driver_faulted || fault != MotorFault::NONE ?
                    uavcan_node_Health_1_0_WARNING : uavcan_node_Health_1_0_NOMINAL
            );

            static auto reported_fault = MotorFault::NONE;
            if (fault != reported_fault) {
                const char* reason = "cleared";
                switch (fault) {
                    case MotorFault::NONE: break;
                    case MotorFault::INVALID_HALL: reason = "invalid Hall inputs (000/111)"; break;
                    case MotorFault::INVALID_SUPPLY: reason = "invalid bus voltage"; break;
                    case MotorFault::INVALID_COMMAND: reason = "non-finite voltage command"; break;
                    case MotorFault::STALLED: reason = "stalled; send zero to rearm"; break;
                }
                uavcan_diagnostic_Record_1_1 msg = {};
                msg.severity.value = fault == MotorFault::NONE ?
                    uavcan_diagnostic_Severity_1_0_INFO : uavcan_diagnostic_Severity_1_0_WARNING;
                char* text = reinterpret_cast<char*>(msg.text.elements);
                const size_t text_size = sizeof(msg.text.elements);
                const unsigned kicks = motor->get_recovery_attempts();
                if (kicks == 0) {
                    msg.text.count = std::snprintf(
                        text, text_size, "%s: %s", motor->name(), reason
                    );
                }
                else {
                    msg.text.count = std::snprintf(
                        text, text_size, "%s: %s; kicks=%u", motor->name(), reason, kicks
                    );
                }
                static CanardTransferID fault_transfer_id = 0;
                get_interface()->send_msg<uavcan_diagnostic_Record_1_1>(
                    &msg, uavcan_diagnostic_Record_1_1_FIXED_PORT_ID_, &fault_transfer_id
                );
                reported_fault = fault;
            }
        }

        cyphal_loop();
    }
}

// libcxxcanard no longer provides TYPE_ALIAS: subscriptions take the raw DSDL
// types directly (AbstractSubscription<uavcan_primitive_scalar_Real32_1_0>).
using Natural32 = uavcan_primitive_scalar_Natural32_1_0;
using Real32 = uavcan_primitive_scalar_Real32_1_0;
using Int32Array = uavcan_primitive_array_Integer32_1_0;
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
            if (motor) {
                motor->set_wheel_radius(wheel_radius);
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
    if (motor) {
        motor->set_velocity_pid(velocity_pid_config);
    }
}

void in_loop_reporting(millis_t current_t) {
    static millis_t report_time = 0;
    static const auto node_id = get_node_id();
    if ((current_t - report_time) >= (50)) {
        Natural32 enc_msg = {};
        enc_msg.value = hall_sensor.get_value();
        static CanardTransferID enc_transfer_id = 0;
        get_interface()->send_msg<Natural32>(&enc_msg, ENCODER_PORT + node_id, &enc_transfer_id);

        const float angular_velocity = motor->get_velocity();  // rad/s

        Real32 velocity_msg = {};
        velocity_msg.value = angular_velocity;
        static CanardTransferID velocity_transfer_id = 0;
        get_interface()->send_msg<Real32>(&velocity_msg, VELOCITY_PORT + node_id, &velocity_transfer_id);

        // Linear wheel speed from angular velocity and wheel radius: v = w * r.
        Real32 speed_msg = {};
        speed_msg.value = angular_velocity * wheel_radius;  // m/s
        static CanardTransferID speed_transfer_id = 0;
        get_interface()->send_msg<Real32>(&speed_msg, LINEAR_SPEED_PORT + node_id, &speed_transfer_id);

        report_time = current_t;
    }
}

// Target wheel speed command, m/s. Enters closed-loop speed regulation; the
// backend resets its regulator when the mode changes so a stale integral from a
// previous set-point does not kick the motor.
class SpeedCommandSub: public AbstractSubscription<Real32> {
public:
    SpeedCommandSub(InterfacePtr interface, CanardPortID port_id): AbstractSubscription<Real32>(interface, port_id) {};
    void handler(const Real32& msg, CanardRxTransfer*) override {
        if (!motor) {
            return;
        }
        motor->set_speed_target(msg.value);  // m/s
    }
};

// Direct open-loop voltage command, V. Leaves speed regulation and feeds the
// voltage straight to the motor — for bring-up / tuning.
class DirectSpeedCommandSub: public AbstractSubscription<Real32> {
public:
    DirectSpeedCommandSub(InterfacePtr interface, CanardPortID port_id): AbstractSubscription<Real32>(interface, port_id) {};
    void handler(const Real32& msg, CanardRxTransfer*) override {
        if (!motor) {
            return;
        }
        motor->set_voltage_point(msg.value);
    }
};

// Runtime configuration: one { id, numerator, denominator } triple per message.
class ConfigSub: public AbstractSubscription<Int32Array> {
public:
    ConfigSub(InterfacePtr interface, CanardPortID port_id): AbstractSubscription<Int32Array>(interface, port_id) {};
    void handler(const Int32Array& msg, CanardRxTransfer*) override {
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
                    RegisterAccessResponse& response
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

    for (uint32_t i = 0; i < sizeof(rx_filters) / sizeof(rx_filters[0]); ++i) {
        HAL_IMPORTANT(apply_filter(i, &hfdcan1, rx_filters[i]))
    }
}
