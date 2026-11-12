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
#include <uavcan/si/unit/length/Scalar_1_0.hpp>
#include <uavcan/primitive/scalar/Real32_1_0.hpp>
#include <uavcan/primitive/array/Integer32_1_0.hpp>
#include <uavcan/diagnostic/Record_1_1.hpp>

#include <voltbro/eeprom/eeprom.hpp>
#include <voltbro/encoders/hall_sensor/hall_sensor.h>
#include <voltbro/devices/inverter.hpp>

#include "config_ids.h"
#include "heap_probe.h"
#include "loop_trace.h"
#include "motor_backend.h"
#include "motor_tuning.h"
#include "serial_line.h"


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
    // Nm / A == V / (rad/s), and the firmware uses it as the back-EMF constant:
    // every voltage and speed budget derives from it (App/motor_limits.h).
    // MEASURED, not nameplate: hold steady speeds and fit the applied q-voltage
    // against the hall-derived shaft rate (the halls count the rotor, so no
    // gearbox or nameplate enters the measurement). On this module 2026-11-12,
    // with debug/bench_speed_sweep.txt: |u| = 6.34..6.41 V at 2.09..2.12 wheel
    // rad/s forward and 6.38 V at 2.09 reverse -> 0.0472..0.0482 V per rotor
    // rad/s over the whole range, i.e. 3.06 V per WHEEL rad/s.
    // The nameplate 0.069 was 44 % high, and because the speed ceiling is
    // (modulation limit - floor) / EMF it capped the drive at 0.1943 m/s where
    // the same 18.5 V rail and headroom reach 0.2795.
    .torque_const = 0.0478f,
    .max_current = 8,
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
        .gear_ratio = 64
    }
};

std::unique_ptr<MotorBackend> motor;

/* ---- Runtime, CAN-tunable parameters (see ConfigSub / CONFIG_PORT below) ----
 * These have safe defaults so the node runs before any config message arrives;
 * the wheel master overwrites them at run time. */
// Boot-time wheel radius, m (0.32 m wheel diameter, the cart's wheel) until the
// config topic or a stored EEPROM value replaces it; the live radius lives in
// the motor façade. Keep it in sync with config_defs.lua's wheel_diameter.
constexpr float BOOT_WHEEL_RADIUS_M = 0.16f;

// Boot-time full-scale speed of a normalized (-1..1) command, m/s, until the
// backend reports what its rail can regulate and the EEPROM record overrides
// it. Mirrors mods/config_defs.lua's max_speed default.
constexpr float BOOT_MAX_WHEEL_SPEED_M_S = 0.5f;

// Default velocity-PID gains (see motor_tuning.h for the per-implementation
// values). Overridden at run time by the config topic.
static PIDConfig make_default_pid_config() {
    return motor_tuning::default_velocity_pid_config();
}
static PIDConfig velocity_pid_config = make_default_pid_config();

// Integral anti-windup, in the backend's output unit (V on six-step, A on the
// libvoltbro FOC; SimpleFOC's PID clamps the integral together with its whole
// output and ignores this): the largest contribution the integral term may
// add. The regulator clamps the accumulated error (rad/s * s) instead, so the
// operator value is divided by Ki on every gain change.
static float pid_i_limit_v = velocity_pid_config.ki * velocity_pid_config.integral_error_lim;

// Live overcurrent / stall protection parameters behind ConfigId 5..11; the
// compiled defaults come from motor_tuning.h, the config topic and the
// EEPROM override them.
static current_guard::Config guard_config{
    .lpf_tf_s = motor_tuning::simplefoc_guard_lpf_tf_s,
    .peak_a = motor_tuning::simplefoc_oc_peak_a,
    .peak_ms = motor_tuning::simplefoc_oc_peak_ms,
    .sustain_a = motor_tuning::simplefoc_oc_sustain_a,
    .sustain_ms = motor_tuning::simplefoc_oc_sustain_ms,
    .stall_current_a = motor_tuning::simplefoc_stall_current_a,
    .stall_velocity_rad_s = motor_tuning::simplefoc_stall_velocity_rad_s,
    .stall_ms = motor_tuning::simplefoc_stall_ms,
};

static float integral_error_limit(float limit_v, float ki) {
    return ki > 0.0f ? limit_v / ki : 0.0f;
}

/* ---- Live config registry ---------------------------------------------------
 * One float per ConfigId: what apply_config_value last accepted, what the
 * serial `G` read-back reports and what the EEPROM record stores. Filled from
 * the compiled defaults at boot; the config topic, the serial channel and a
 * stored EEPROM record override it. */
static float config_values[CFG_ID_COUNT];

static void init_config_defaults() {
    config_values[CFG_WHEEL_DIAMETER] = BOOT_WHEEL_RADIUS_M * 2.0f;
    config_values[CFG_PID_KP] = velocity_pid_config.kp;
    config_values[CFG_PID_KI] = velocity_pid_config.ki;
    config_values[CFG_PID_KD] = velocity_pid_config.kd;
    config_values[CFG_PID_I_LIMIT] = pid_i_limit_v;
    config_values[CFG_OC_PEAK_A] = guard_config.peak_a;
    config_values[CFG_OC_PEAK_MS] = (float)guard_config.peak_ms;
    config_values[CFG_OC_SUSTAIN_A] = guard_config.sustain_a;
    config_values[CFG_OC_SUSTAIN_MS] = (float)guard_config.sustain_ms;
    config_values[CFG_STALL_CURRENT] = guard_config.stall_current_a;
    config_values[CFG_STALL_SPEED] = guard_config.stall_velocity_rad_s;
    config_values[CFG_STALL_MS] = (float)guard_config.stall_ms;
    config_values[CFG_VEL_LPF] = motor_tuning::simplefoc_velocity_lpf_tf;
    // Mirrors the MotorBackend façade's compiled speed-ramp default.
    config_values[CFG_SPEED_RAMP] = 0.75f;
    config_values[CFG_TORQUE_RAMP] = motor_tuning::simplefoc_torque_ramp_a_s;
    config_values[CFG_BRAKE_CURRENT] = motor_tuning::simplefoc_brake_current_a;
    config_values[CFG_MAX_SPEED] = BOOT_MAX_WHEEL_SPEED_M_S;
}

/* ---- Wheel parameters persisted in the on-board EEPROM ---------------------
 * The modules kept their runtime config in RAM only, so a power cycle reverted a
 * wheel to the compiled-in defaults. Every applied config change is written back
 * after CONFIG_SAVE_DEBOUNCE_MS of quiet — a GUI session sends value after value,
 * and one EEPROM write per keystroke would wear the cell out. The magic and the
 * version reject an erased (0xFF) or foreign EEPROM.
 *
 * The record is the config-values registry below (one float per ConfigId), so
 * a new id only has to join the enum and the apply switch. The version rejects
 * a record written with a different CFG_ID_COUNT or layout: version 3 and
 * earlier kept named fields, and version 4 lacks the normalized full-scale
 * speed (id 16), so both are rejected (the module falls back to its compiled
 * defaults — the values are re-tunable over the config topic). The
 * value units follow the flashed backend (V on six-step, A on the FOC builds),
 * so a wheel moved between backends needs its gains re-tuned, not re-read. */
struct StoredWheelConfig {
    uint32_t magic;
    uint16_t version;
    uint16_t reserved;
    float values[CFG_ID_COUNT];
};
static constexpr uint32_t WHEEL_CONFIG_MAGIC = 0x314E4C47u;  // "GLN1"
static constexpr uint16_t WHEEL_CONFIG_VERSION = 5;
static constexpr uint16_t WHEEL_CONFIG_ADDRESS = 0;
static constexpr millis_t CONFIG_SAVE_DEBOUNCE_MS = 1000;

// 0 = nothing pending; otherwise millis_32() of the last change.
static millis_t config_changed_at = 0;

static void load_stored_wheel_config();
static void save_stored_wheel_config();

static void mark_config_changed() {
    const millis_t now = millis_32();
    config_changed_at = now ? now : 1;  // 0 is the "clean" sentinel
}

/// Write the pending config once the operator has stopped editing for a second.
static void service_config_persistence(millis_t now_ms) {
    if (!config_changed_at) return;
    if ((millis_t)(now_ms - config_changed_at) < CONFIG_SAVE_DEBOUNCE_MS) return;
    config_changed_at = 0;
    save_stored_wheel_config();
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == ENC_1_Pin || GPIO_Pin == ENC_2_Pin || GPIO_Pin == ENC_3_Pin) {
        hall_sensor.handle_hall_channel(GPIO_Pin);
    }
}

// Worst-case duration of each phase of the loop below, us (reported as `K5`).
// The drive's stutter is a periodic ~3 ms stall — measured at about one every
// 4..5 s — and knowing which phase absorbed it is the difference between a
// guess and a fix. Kept as a running maximum since boot: a stall is rare, so an
// average would hide it.
namespace {

enum LoopPhase: uint8_t { phase_pid, phase_update, phase_poll, phase_persist, phase_cyphal };

void loop_phase_timing(LoopPhase phase, micros_t& mark) {
    const micros_t now = micros_64();
    const int64_t elapsed = subtract_64(now, mark);
    mark = now;
    // A negative sample would pin the maximum forever, so drop it.
    if (elapsed < 0) {
        return;
    }
    const uint32_t elapsed_us = (uint32_t)elapsed;
    volatile loop_trace::Counters& counters = loop_trace::counters();
    volatile uint32_t* slot = nullptr;
    switch (phase) {
        case phase_pid: slot = &counters.phase_pid_max_us; break;
        case phase_update: slot = &counters.phase_update_max_us; break;
        case phase_poll: slot = &counters.phase_poll_max_us; break;
        case phase_persist: slot = &counters.phase_persist_max_us; break;
        case phase_cyphal: slot = &counters.phase_cyphal_max_us; break;
    }
    if (slot != nullptr && elapsed_us > *slot) {
        *slot = elapsed_us;
    }
}

}

[[noreturn]] void app() {
    start_timers();
    start_cyphal();

    // The config registry must hold the compiled defaults before any config
    // source (EEPROM record, CAN topic, serial `C`) overrides it.
    init_config_defaults();

    // The HallSensor ctor runs before GPIO clocks/inputs are initialized, so its
    // cached state is meaningless until we re-sample the pins here.
    hall_sensor.resync();

    // The EEPROM is probed for presence and, when it holds a stored wheel
    // config, that config is restored below; a missing or stuck EEPROM must not
    // block motor initialization forever.
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
        BOOT_WHEEL_RADIUS_M
    );

    serial_line_init();
    serial_line_debug("boot: motor init");
    HAL_IMPORTANT(motor->init())
    // The speed this rail can regulate is the boot default for everything that
    // speaks normalized commands; a stored record may ask for less, the live
    // ceiling still bounds it (see apply_speed_ramp).
    motor->set_max_speed(motor->get_speed_ceiling());
    config_values[CFG_MAX_SPEED] = motor->get_max_speed();
    // The bridge comes up enabled: `motor.start()` pulses the driver's enable
    // line and leaves it asserted. The debug serial channel owns that state
    // (`E0`/`E1`) — no Cyphal command, config write or register access changes
    // it. An enabled wheel does not move on its own: the set-point is zero at
    // boot and the six-step controller coasts while the target voltage is zero.
    serial_line_debug("boot: motor enabled");
    HAL_IMPORTANT(motor->start())

    // Last saved parameters win over the compiled-in defaults, so a wheel that
    // was tuned through the config GUI keeps its tuning across a power cycle.
    load_stored_wheel_config();

    serial_line_debug("boot: serial channel up");

    set_cyphal_mode(uavcan_node_Mode_1_0_OPERATIONAL);

    // Velocity PID is run at a fixed cadence so its dt is constant and stable.
    constexpr micros_t PID_PERIOD_US = 5000;  // 200 Hz
    static micros_t pid_time = 0;
    while(true) {
        // The control loop must not touch the free store: a newlib allocation
        // has no bounded cost, and losing a few milliseconds to it holds the
        // bridge on a stale voltage vector while the rotor turns through it.
        // Flagged here, counted by App/heap_probe.cpp (`K` reports it).
        heap_probe::hot(true);
        micros_t now = micros_64();
        micros_t phase_mark = now;

        EACH_N_MICROS(now, pid_time, PID_PERIOD_US, {
            const float dt = (float)PID_PERIOD_US * 1e-6f;
            motor->apply_speed_ramp(dt);
            motor->run_speed_loop(dt);
        })
        loop_phase_timing(phase_pid, phase_mark);

        motor->update();
        loop_phase_timing(phase_update, phase_mark);

        serial_line_poll(*motor);
        loop_phase_timing(phase_poll, phase_mark);

        const millis_t now_ms = millis_32();
        service_config_persistence(now_ms);
        loop_phase_timing(phase_persist, phase_mark);

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
                    case MotorFault::OVERCURRENT: reason = "overcurrent; send zero to rearm"; break;
                    case MotorFault::DRIVER_FAULT: reason = "gate driver fault; disable/enable to rearm"; break;
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
        loop_phase_timing(phase_cyphal, phase_mark);
        heap_probe::hot(false);
    }
}

// libcxxcanard no longer provides TYPE_ALIAS: subscriptions take the raw DSDL
// types directly (AbstractSubscription<uavcan_primitive_scalar_Real32_1_0>).
using Real32 = uavcan_primitive_scalar_Real32_1_0;
using Int32Array = uavcan_primitive_array_Integer32_1_0;
using Travel = uavcan_si_unit_length_Scalar_1_0;
static constexpr CanardPortID TRAVEL_PORT = 7100;     // accumulated wheel travel, m
static constexpr CanardPortID VELOCITY_PORT = 7200;   // shaft angular velocity, rad/s
static constexpr CanardPortID LINEAR_SPEED_PORT = 7300;  // wheel linear speed, m/s
static constexpr CanardPortID SPEED_CMD_PORT = 4000;  // normalized wheel speed command, -1..1
static constexpr CanardPortID DIRECT_SPEED_CMD_PORT = 4050;  // open-loop voltage command, V
static constexpr CanardPortID CONFIG_PORT = 4100;     // runtime config (see ConfigSub)

// Apply a single decoded config value to the live parameters. Returns whether
// the value was accepted; the serial channel answers ERR on a rejection, the
// CAN handler just ignores it.
static bool apply_config_value(int32_t id, float value) {
    if (id < 0 || id >= CFG_ID_COUNT || !std::isfinite(value)) {
        return false;
    }
    if (id == CFG_WHEEL_DIAMETER) {
        if (value <= 0.0f || !motor) {
            return false;
        }
        motor->set_wheel_radius(value * 0.5f);
        config_values[id] = value;
        mark_config_changed();
        return true;
    }

    if (id >= CFG_OC_PEAK_A && id <= CFG_STALL_MS) {
        // Protection parameters must be positive-domain and castable to their
        // fixed-point windows; a zero trip level would latch on the idle
        // offsets and a zero velocity ceiling would stall on any command, so
        // those are rejected as well.
        if (value < 0.0f || value > 1.0e6f) {
            return false;
        }
        current_guard::Config guard = guard_config;
        switch (id) {
            case CFG_OC_PEAK_A:     guard.peak_a = value; break;
            case CFG_OC_PEAK_MS:    guard.peak_ms = (uint32_t)value; break;
            case CFG_OC_SUSTAIN_A:  guard.sustain_a = value; break;
            case CFG_OC_SUSTAIN_MS: guard.sustain_ms = (uint32_t)value; break;
            case CFG_STALL_CURRENT: guard.stall_current_a = value; break;
            case CFG_STALL_SPEED:   guard.stall_velocity_rad_s = value; break;
            case CFG_STALL_MS:      guard.stall_ms = (uint32_t)value; break;
            default: return false;
        }
        if (guard.peak_a <= 0.0f || guard.sustain_a <= 0.0f ||
            guard.stall_current_a <= 0.0f || guard.stall_velocity_rad_s <= 0.0f) {
            return false;
        }
        guard_config = guard;
        if (motor) {
            motor->set_current_guard(guard);
        }
        config_values[id] = value;
        mark_config_changed();
        return true;
    }

    switch (id) {
        case CFG_PID_KP: {
            PIDConfig pid = velocity_pid_config;
            pid.kp = value;
            velocity_pid_config = pid;
            break;
        }
        case CFG_PID_KI: {
            PIDConfig pid = velocity_pid_config;
            pid.ki = value;
            velocity_pid_config = pid;
            break;
        }
        case CFG_PID_KD: {
            PIDConfig pid = velocity_pid_config;
            pid.kd = value;
            velocity_pid_config = pid;
            break;
        }
        case CFG_PID_I_LIMIT:
            if (value < 0.0f) return false;
            pid_i_limit_v = value;
            break;
        case CFG_VEL_LPF:
            if (!motor || !motor->set_velocity_lpf(value)) {
                return false;
            }
            break;
        case CFG_SPEED_RAMP:
            // The façade asserts a positive ramp, so the range is checked here.
            if (!(value > 0.0f) || value > 100.0f || !motor) {
                return false;
            }
            motor->set_speed_ramp(value);
            break;
        case CFG_TORQUE_RAMP:
            if (!motor || !motor->set_torque_ramp(value)) {
                return false;
            }
            break;
        case CFG_BRAKE_CURRENT:
            if (!motor || !motor->set_brake_current(value)) {
                return false;
            }
            break;
        case CFG_MAX_SPEED:
            // Full-scale speed of a normalized command: a non-positive value
            // would make every command a stop, and past a few m/s a cart wheel
            // is not a plausible configuration.
            if (!motor || !(value > 0.0f) || value > 10.0f) {
                return false;
            }
            motor->set_max_speed(value);
            break;
        default:
            return false;  // unknown id: ignore
    }
    if (id >= CFG_PID_KP && id <= CFG_PID_I_LIMIT) {
        // Ki may have just changed, so the error-domain clamp is re-derived
        // from the output-domain one every time.
        velocity_pid_config.integral_error_lim =
            integral_error_limit(pid_i_limit_v, velocity_pid_config.ki);
        if (motor) {
            motor->set_velocity_pid(velocity_pid_config);
        }
    }
    config_values[id] = value;
    mark_config_changed();
    return true;
}

/// Restore the parameters the wheel ran with before the last power cycle.
static void load_stored_wheel_config() {
    if (!eeprom.is_connected()) {
        serial_line_debug("eeprom: absent, wheel config stays at defaults");
        return;
    }
    StoredWheelConfig stored{};
    if (eeprom.read<StoredWheelConfig>(&stored, WHEEL_CONFIG_ADDRESS) != HAL_OK) {
        serial_line_debug("eeprom: read failed, wheel config stays at defaults");
        return;
    }
    if (stored.magic != WHEEL_CONFIG_MAGIC || stored.version != WHEEL_CONFIG_VERSION) {
        serial_line_debug("eeprom: nothing stored, wheel config stays at defaults");
        return;
    }
    for (int32_t id = 0; id < CFG_ID_COUNT; ++id) {
        // A backend without a knob rejects its value and keeps the default.
        (void)apply_config_value(id, stored.values[id]);
    }
    config_changed_at = 0;  // restored, not an edit that needs writing back
    serial_line_debug("eeprom: wheel config restored");
}

/// Persist the live wheel parameters. A failed write is logged, not fatal: the
/// drive keeps running on the values it already has.
static void save_stored_wheel_config() {
    if (!eeprom.is_connected()) {
        serial_line_debug("eeprom: absent, wheel config not saved");
        return;
    }
    StoredWheelConfig stored{};
    stored.magic = WHEEL_CONFIG_MAGIC;
    stored.version = WHEEL_CONFIG_VERSION;
    stored.reserved = 0;
    for (int32_t id = 0; id < CFG_ID_COUNT; ++id) {
        stored.values[id] = config_values[id];
    }
    if (eeprom.write<StoredWheelConfig>(&stored, WHEEL_CONFIG_ADDRESS) != HAL_OK) {
        serial_line_debug("eeprom: write failed");
        return;
    }
    serial_line_debug("eeprom: wheel config saved");
}

bool app_apply_config(int32_t id, float value) {
    return apply_config_value(id, value);
}

bool app_get_config(int32_t id, float& value) {
    if (id < 0 || id >= CFG_ID_COUNT) {
        return false;
    }
    value = config_values[id];
    return true;
}

void in_loop_reporting(millis_t current_t) {
    static millis_t report_time = 0;
    static const auto node_id = get_node_id();
    if ((current_t - report_time) >= (50)) {
        // Absolute wheel travel since boot, m — the odometry input on the Pi.
        // The raw Hall count wraps per revolution and is useless as a position.
        Travel travel_msg = {};
        travel_msg.meter = motor->get_travel();
        static CanardTransferID travel_transfer_id = 0;
        get_interface()->send_msg<Travel>(
            &travel_msg, TRAVEL_PORT + node_id, &travel_transfer_id
        );

        const float angular_velocity = motor->get_velocity();  // rad/s

        Real32 velocity_msg = {};
        velocity_msg.value = angular_velocity;
        static CanardTransferID velocity_transfer_id = 0;
        get_interface()->send_msg<Real32>(&velocity_msg, VELOCITY_PORT + node_id, &velocity_transfer_id);

        Real32 speed_msg = {};
        speed_msg.value = motor->get_linear_velocity();
        static CanardTransferID speed_transfer_id = 0;
        get_interface()->send_msg<Real32>(&speed_msg, LINEAR_SPEED_PORT + node_id, &speed_transfer_id);

        report_time = current_t;
    }
}

// Normalized target wheel speed command, -1..1 of the module's full-scale speed
// (config id 16). Enters closed-loop speed regulation; the backend resets its
// regulator when the mode changes so a stale integral from a previous set-point
// does not kick the motor. It does not enable: the enable state belongs to the
// debug serial channel.
class SpeedCommandSub: public AbstractSubscription<Real32> {
public:
    SpeedCommandSub(InterfacePtr interface, CanardPortID port_id): AbstractSubscription<Real32>(interface, port_id) {};
    void handler(const Real32& msg, CanardRxTransfer*) override {
        if (!motor) {
            return;
        }
        serial_line_debug("can speed_cmd", msg.value);
        motor->set_speed_target_normalized(msg.value);
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
        serial_line_debug("can direct_cmd", msg.value);
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
                    const uavcan_register_Value_1_0&,
                    uavcan_register_Value_1_0& v_out,
                    RegisterAccessResponse& response
                ){
                    // Read-only: the enable state is owned by the debug serial
                    // channel (`E0`/`E1`), so a write here is not applied and
                    // the register reports itself immutable. A read just
                    // reports the current state.
                    response.persistent = false;
                    response._mutable = false;
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
