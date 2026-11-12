#include <cmath>
#include <cstdlib>
#include <iostream>
#include <map>
#include <string>

#include "config_ids.h"
#include "loop_trace.h"
#include "motor_backend.h"
#include "serial_line.h"
#include "usart.h"

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart1;

millis_t millis_32() {
    return fake_tick;
}

// app.cpp in the firmware; the test records what the channel applies. The
// range mirrors for ids 12/13 model app.cpp's apply_config_value validation so
// the serial layer's ERR-on-reject contract is exercised end to end; everything
// else is accepted unless `config_apply_ok` is forced off.
std::map<int32_t, float> config_store;
bool config_apply_ok = true;

bool app_apply_config(int32_t id, float value) {
    if (id < 0 || id >= CFG_ID_COUNT) return false;
    if (id == CFG_VEL_LPF && !(value >= 0.0f && value <= 1.0f)) return false;
    if (id == CFG_SPEED_RAMP && !(value > 0.0f && value <= 100.0f)) return false;
    if (id == CFG_MAX_SPEED && !(value > 0.0f && value <= 10.0f)) return false;
    if (!config_apply_ok) return false;
    config_store[id] = value;
    return true;
}

bool app_get_config(int32_t id, float& value) {
    const auto entry = config_store.find(id);
    if (entry == config_store.end()) return false;
    value = entry->second;
    return true;
}

#define CHECK(condition) do { \
    if (!(condition)) { \
        std::cerr << __func__ << ':' << __LINE__ << ": " #condition "\n"; \
        std::exit(1); \
    } \
} while (false)

namespace {

// The fake records the wheel rad/s the façade derives from the m/s command.
constexpr float RADIUS_M = 0.05f;

float wheel_rad_s(float linear_m_s) {
    return linear_m_s / RADIUS_M;
}

struct FakeMotor : MotorBackend {
    explicit FakeMotor(float wheel_radius_m = RADIUS_M): MotorBackend(wheel_radius_m) {}

    float velocity = 0.0f;
    float angle = 0.0f;
    MotorFault fault = MotorFault::NONE;
    bool enabled = false;
    float target = 0.0f;
    float voltage = 0.0f;
    float current_a = NAN;
    int regulator_resets = 0;
    int calibrate_calls = 0;
    bool calibrated = false;

    const char* name() const override { return "fake"; }
    HAL_StatusTypeDef init() override { return HAL_OK; }
    HAL_StatusTypeDef start() override { return HAL_OK; }
    HAL_StatusTypeDef stop() override { return HAL_OK; }
    HAL_StatusTypeDef set_state(bool state) override {
        enabled = state;
        return HAL_OK;
    }
    bool is_on() const override { return enabled; }
    void update() override {}
    void set_voltage_point(float volts) override { voltage = volts; }
    void run_speed_loop(float) override {}
    void set_velocity_pid(const PIDConfig&) override {}
    float get_velocity() const override { return velocity; }
    float get_wheel_angle() const override { return angle; }
    MotorFault get_fault() const override { return fault; }
    float get_torque_current() override { return current_a; }
    bool auto_calibrate() override {
        calibrate_calls += 1;
        calibrated = enabled;
        return enabled;
    }

    // The ceiling is a protected façade hook the real backends drive from their
    // modulation limit; expose it so the test can exercise the clamp.
    void set_ceiling(float m_s) { set_speed_ceiling(m_s); }

protected:
    void set_wheel_speed_target(float target_rad_s) override { target = target_rad_s; }
    void reset_speed_regulator() override { regulator_resets += 1; }
};

std::string take_tx() {
    std::string out = fake_uart_tx;
    fake_uart_tx.clear();
    return out;
}

std::string take_mirror() {
    std::string out = fake_uart_mirror_tx;
    fake_uart_mirror_tx.clear();
    return out;
}

void pump(FakeMotor& motor) {
    // Each poll starts at most one TX chunk, so a reply longer than the chunk
    // size completes on the polls that follow.
    for (int i = 0; i < 4; ++i) {
        serial_line_poll(motor);
    }
}

void send(FakeMotor& motor, const std::string& line) {
    fake_uart_feed(&huart2, line.c_str());
    serial_line_poll(motor);
    pump(motor);
}

// The façade ramps the applied set-point towards the commanded one, so a step
// command reaches the backend only after enough speed-loop periods.
void settle(FakeMotor& motor) {
    motor.apply_speed_ramp(10.0f);
}

// Feeds in chunks with a poll in between, the way the real main loop drains the
// receive ring while a long line is still arriving.
void send_slow(FakeMotor& motor, const std::string& text, size_t chunk) {
    for (size_t offset = 0; offset < text.size(); offset += chunk) {
        fake_uart_feed(&huart2, text.substr(offset, chunk).c_str());
        serial_line_poll(motor);
    }
}

void test_setpoints(FakeMotor& motor) {
    // T is normalized: 0.5 is half of the module's full-scale wheel speed
    // (0.25 m/s by default, config id 16).
    send(motor, "T0.5\r\n");
    CHECK(take_tx() == "T=0.500\r\n");
    settle(motor);
    CHECK(motor.target == wheel_rad_s(0.125f));

    send(motor, "Q2.25\n");
    CHECK(take_tx() == "Q=2.250\r\n");
    CHECK(motor.voltage == 2.25f);

    send(motor, "T-1\n");
    CHECK(take_tx() == "T=-1.000\r\n");
    settle(motor);
    CHECK(motor.target == wheel_rad_s(-0.25f));

    // Out of range is clamped to full scale, not rejected: a slider at its end
    // must stop, not answer ERR in the middle of a drag.
    send(motor, "T12.3456\n");
    CHECK(take_tx() == "T=1.000\r\n");
    settle(motor);
    CHECK(motor.target == wheel_rad_s(0.25f));
    send(motor, "T-12.3456\n");
    CHECK(take_tx() == "T=-1.000\r\n");

    // The full-scale speed is a config value and is read back like any other.
    send(motor, "C16:0.4\n");
    CHECK(take_tx() == "C16=0.400\r\n");
    send(motor, "G16\n");
    CHECK(take_tx() == "C16=0.4000\r\n");
    send(motor, "C16\n");
    CHECK(take_tx() == "ERR C16\r\n");
    // A non-positive or silly full scale is rejected like any other bad value.
    send(motor, "C16:0\n");
    CHECK(take_tx() == "ERR C16:0\r\n");
}

void test_speed_target_uses_wheel_radius(FakeMotor& motor) {
    // Clear the applied set-point first: the ramp only re-derives the wheel
    // rad/s when it actually moves, so a radius change on its own keeps the
    // previous conversion (the radius is read at the ramp, not at the command).
    motor.set_speed_target(0.0f);
    motor.set_wheel_radius(0.1f);
    send(motor, "T1\n");  // full scale of the default 0.25 m/s
    CHECK(take_tx() == "T=1.000\r\n");
    settle(motor);
    CHECK(std::abs(motor.target - 2.5f) < 1e-4f);

    motor.velocity = 10.0f;
    CHECK(std::abs(motor.get_linear_velocity() - 1.0f) < 1e-4f);
    motor.velocity = 0.0f;

    motor.set_speed_target(0.0f);
    motor.set_wheel_radius(RADIUS_M);
    send(motor, "T0.5\n");
    CHECK(take_tx() == "T=0.500\r\n");
    settle(motor);
    CHECK(motor.target == wheel_rad_s(0.125f));
}

void test_enable(FakeMotor& motor) {
    send(motor, "E1\n");
    CHECK(take_tx() == "E=1\r\n");
    CHECK(motor.enabled);

    send(motor, "E0\n");
    CHECK(take_tx() == "E=0\r\n");
    CHECK(!motor.enabled);
}

void test_zero(FakeMotor& motor) {
    send(motor, "T0.4\n");
    send(motor, "Q3\n");
    take_tx();
    const int resets_before = motor.regulator_resets;
    send(motor, "X\n");
    CHECK(take_tx() == "OK\r\n");
    CHECK(motor.regulator_resets == resets_before + 1);
    CHECK(motor.target == 0.0f);
    CHECK(motor.voltage == 0.0f);
}

void test_status(FakeMotor& motor) {
    motor.velocity = 6.1734f;
    motor.fault = MotorFault::STALLED;
    send(motor, "T0.5\n");
    take_tx();
    send(motor, "S\n");
    CHECK(take_tx() == "S t=0.500 v=6.173 e=0 f=STALLED\r\n");
    motor.fault = MotorFault::NONE;
}

// Backends with a current measurement append ` i=<A>` to the status line; the
// legacy line (no current reported) stays byte-identical.
void test_status_current(FakeMotor& motor) {
    motor.velocity = 6.0f;
    motor.current_a = 2.5f;
    send(motor, "S\n");
    CHECK(take_tx() == "S t=0.500 v=6.000 e=0 f=NONE i=2.50\r\n");

    motor.current_a = -0.125f;
    send(motor, "S\n");
    CHECK(take_tx() == "S t=0.500 v=6.000 e=0 f=NONE i=-0.13\r\n");

    motor.current_a = NAN;
    motor.velocity = 0.0f;
    send(motor, "S\n");
    CHECK(take_tx() == "S t=0.500 v=0.000 e=0 f=NONE\r\n");
}

// `A` runs the backend's calibration and reports its verdict; the FakeMotor
// refuses while the driver is disabled, the way the SimpleFOC backend does.
void test_auto_calibrate(FakeMotor& motor) {
    send(motor, "A\n");
    CHECK(take_tx() == "A=0\r\n");
    CHECK(motor.calibrate_calls == 1);
    CHECK(!motor.calibrated);

    send(motor, "A1\n");
    CHECK(take_tx() == "ERR A1\r\n");

    send(motor, "E1\n");
    take_tx();
    send(motor, "A\n");
    CHECK(take_tx() == "A=1\r\n");
    CHECK(motor.calibrate_calls == 2);
    CHECK(motor.calibrated);

    send(motor, "E0\n");
    take_tx();
}

// '#' diagnostics are mirrored on USART1 (PC4): boot trace, echo and debug
// labels land there, protocol replies and telemetry do not.
void test_debug_mirror(FakeMotor& motor) {
    send(motor, "T0.5\n");
    CHECK(take_tx() == "T=0.500\r\n");
    CHECK(take_mirror().empty());

    send(motor, "D1\n");
    CHECK(take_tx() == "D=1\r\n");
    CHECK(take_mirror().empty());
    send(motor, "S\n");
    take_tx();
    CHECK(take_mirror() == "# rx S\r\n");
    send(motor, "D0\n");
    take_tx();
    CHECK(take_mirror() == "# rx D0\r\n");

    serial_line_debug("bench label", 1.5f);
    pump(motor);
    CHECK(take_mirror() == "# bench label 1.5000\r\n");
    CHECK(take_tx() == "# bench label 1.5000\r\n");

    serial_line_debug("plain label");
    pump(motor);
    CHECK(take_mirror() == "# plain label\r\n");
    CHECK(take_tx() == "# plain label\r\n");
}

void test_telemetry(FakeMotor& motor) {
    send(motor, "M0\n");
    CHECK(take_tx() == "M=0\r\n");
    fake_tick += 500;
    serial_line_poll(motor);
    CHECK(take_tx().empty());

    send(motor, "M10\n");
    CHECK(take_tx() == "M=10\r\n");
    fake_tick += 99;
    serial_line_poll(motor);
    CHECK(take_tx().empty());
    fake_tick += 1;
    serial_line_poll(motor);
    CHECK(take_tx().rfind("S t=", 0) == 0);
    send(motor, "M0\n");
    take_tx();
}

void test_errors(FakeMotor& motor) {
    const float target_before_errors = motor.target;
    send(motor, "Z9\n");
    CHECK(take_tx() == "ERR Z9\r\n");

    send(motor, "T\n");
    CHECK(take_tx() == "ERR T\r\n");

    send(motor, "E2\n");
    CHECK(take_tx() == "ERR E2\r\n");

    send(motor, "X1\n");
    CHECK(take_tx() == "ERR X1\r\n");

    send(motor, "T1.5x\n");
    CHECK(take_tx() == "ERR T1.5x\r\n");
    CHECK(motor.target == target_before_errors);
}

void test_info(FakeMotor& motor) {
    config_store[CFG_WHEEL_DIAMETER] = 0.32f;
    config_store[CFG_MAX_SPEED] = 0.35f;
    config_store[CFG_PID_KP] = 1.0f;
    config_store[CFG_PID_KI] = 5.0f;
    config_store[CFG_PID_KD] = 0.0f;
    send(motor, "I\n");
    CHECK(take_tx() ==
        "I commit=host-test motor=fake\r\n"
        "I cfg C0=0.3200 C16=0.3500 C1=1.0000 C2=5.0000 C3=0.0000\r\n");
    send(motor, "I1\n");
    CHECK(take_tx() == "ERR I1\r\n");
}

void test_help(FakeMotor& motor) {
    send(motor, "?\n");
    const std::string out = take_tx();
    CHECK(out.rfind("# T<float>", 0) == 0);
    CHECK(out.find("\r\n# I ") != std::string::npos);
    CHECK(out.find("\r\n# ?") != std::string::npos);
}

void test_overlong_line(FakeMotor& motor) {
    send_slow(motor, std::string(200, 'A') + "\n", 32);
    CHECK(take_tx().empty());
    send(motor, "S\n");
    CHECK(take_tx().rfind("S t=", 0) == 0);
}

void test_rx_overflow(FakeMotor& motor) {
    const float target = motor.target;

    // A burst that overruns the 128-byte receive ring: nothing that was buffered
    // can be trusted, so it is all dropped and the channel resynchronises on the
    // next newline instead of executing a truncated command.
    std::string burst;
    for (int i = 0; i < 40; ++i) {
        burst += "T9.5\n";
    }
    fake_uart_feed(&huart2, burst.c_str());
    serial_line_poll(motor);
    pump(motor);
    CHECK(take_tx().empty());
    CHECK(motor.target == target);

    send(motor, "\n");
    send(motor, "S\n");
    CHECK(take_tx().rfind("S t=", 0) == 0);

    send(motor, "T0.8\n");
    CHECK(take_tx() == "T=0.800\r\n");
    settle(motor);
    CHECK(motor.target == wheel_rad_s(0.2f));

    send(motor, "X\n");
    take_tx();
    CHECK(motor.target == 0.0f);
}

void test_blank_and_stray_terminators(FakeMotor& motor) {
    send(motor, "\r\n\r\n  \n");
    CHECK(take_tx().empty());
    send(motor, "  T0.25  \r\n");
    CHECK(take_tx() == "T=0.250\r\n");
}

void test_verbose_echo(FakeMotor& motor) {
    send(motor, "D1\n");
    CHECK(take_tx() == "D=1\r\n");

    send(motor, "S\n");
    const std::string out = take_tx();
    CHECK(out.rfind("# rx S\r\n", 0) == 0);
    CHECK(out.find("S t=") != std::string::npos);

    send(motor, "D0\n");
    CHECK(take_tx() == "# rx D0\r\nD=0\r\n");
}

void test_noise_discard(FakeMotor& motor) {
    const float target = motor.target;
    const std::string noisy = std::string("T") + "\x80" + "5\n";
    fake_uart_feed(&huart2, noisy.c_str());
    serial_line_poll(motor);
    CHECK(take_tx().empty());
    CHECK(motor.target == target);

    send(motor, "S\n");
    CHECK(take_tx().rfind("S t=", 0) == 0);
}

void test_rx_error_discards_line(FakeMotor& motor) {
    const float target = motor.target;
    fake_uart_feed(&huart2, "T9");
    fake_uart_error(&huart2, USART_ISR_ORE);
    fake_uart_feed(&huart2, "\n");
    serial_line_poll(motor);
    pump(motor);
    CHECK(take_tx().empty());
    CHECK(motor.target == target);

    // The resync newline was lost with the dropped bytes, so one line is spent
    // re-aligning the parser before the channel answers again.
    send(motor, "\n");
    send(motor, "S\n");
    CHECK(take_tx().rfind("S t=", 0) == 0);
}

void test_config_command(FakeMotor& motor) {
    send(motor, "C1:0.08\n");
    CHECK(take_tx() == "C1=0.080\r\n");
    CHECK(config_store.count(1) == 1);
    CHECK(std::abs(config_store[1] - 0.08f) < 1e-6f);

    send(motor, "C0:0.1\n");
    CHECK(take_tx() == "C0=0.100\r\n");
    CHECK(config_store.count(0) == 1);
    CHECK(std::abs(config_store[0] - 0.1f) < 1e-6f);

    send(motor, "C:\n");
    CHECK(take_tx() == "ERR C:\r\n");
    send(motor, "C1\n");
    CHECK(take_tx() == "ERR C1\r\n");
    send(motor, "Cx:1\n");
    CHECK(take_tx() == "ERR Cx:1\r\n");

    // A value the module rejects (unknown id) answers ERR, not an echo.
    send(motor, "C99:1\n");
    CHECK(take_tx() == "ERR C99:1\r\n");
    CHECK(config_store.count(99) == 0);
}

// `G` reports the live value of an id the way the GUI reads a slider back.
void test_config_readback(FakeMotor& motor) {
    config_store[CFG_PID_KP] = 1.0f;
    send(motor, "G1\n");
    CHECK(take_tx() == "C1=1.0000\r\n");

    config_store[CFG_VEL_LPF] = 0.02f;
    send(motor, "G12\n");
    CHECK(take_tx() == "C12=0.0200\r\n");

    send(motor, "G99\n");
    CHECK(take_tx() == "ERR G99\r\n");
    send(motor, "G\n");
    CHECK(take_tx() == "ERR G\r\n");
    send(motor, "Gx\n");
    CHECK(take_tx() == "ERR Gx\r\n");
}

// `L`/`R` are serial aliases of config ids 12/13: one storage, persisted, and
// the rejection path answers ERR.
void test_lpf_ramp_alias_config(FakeMotor& motor) {
    send(motor, "L0.02\n");
    CHECK(take_tx() == "L=0.0200\r\n");
    CHECK(config_store.count(CFG_VEL_LPF) == 1);
    CHECK(std::abs(config_store[CFG_VEL_LPF] - 0.02f) < 1e-6f);

    send(motor, "L2\n");
    CHECK(take_tx() == "ERR L2\r\n");
    send(motor, "L-0.1\n");
    CHECK(take_tx() == "ERR L-0.1\r\n");

    send(motor, "R0.5\n");
    CHECK(take_tx() == "R=0.500\r\n");
    CHECK(config_store.count(CFG_SPEED_RAMP) == 1);
    CHECK(std::abs(config_store[CFG_SPEED_RAMP] - 0.5f) < 1e-6f);

    send(motor, "R0\n");
    CHECK(take_tx() == "ERR R0\r\n");
    send(motor, "R200\n");
    CHECK(take_tx() == "ERR R200\r\n");

    // A forced rejection (backend without the knob) reaches the operator.
    config_apply_ok = false;
    send(motor, "L0.02\n");
    CHECK(take_tx() == "ERR L0.02\r\n");
    config_apply_ok = true;
    send(motor, "L0.02\n");
    CHECK(take_tx() == "L=0.0200\r\n");
}

void test_trace_counters_and_dump(FakeMotor& motor) {
    loop_trace::reset();
    send(motor, "K\n");
    CHECK(take_tx().rfind("# K1 hall bad=0 edges=0", 0) == 0);

    // Arming: the next sample carrying a suspicious event starts the dump.
    send(motor, "W1\n");
    CHECK(take_tx() == "W=1\r\n");
    CHECK(loop_trace::armed());

    loop_trace::Sample sample = {};
    sample.time_us = 123456;
    sample.velocity_rad_s = 0.9375f;
    sample.phase_current_a = 3.5f;
    sample.hall_raw = 5;
    sample.flags = loop_trace::HALL_ILLEGAL;
    loop_trace::push(sample);
    CHECK(!loop_trace::armed());

    pump(motor);
    std::string dump = take_tx();
    CHECK(dump.rfind("# trace: samples=1 depth=", 0) == 0);
    CHECK(dump.find("# T 123 e=") != std::string::npos);
    CHECK(dump.find(" im=3.50") != std::string::npos);
    CHECK(dump.find(" h=5") != std::string::npos);
    CHECK(dump.find(" ev=1") != std::string::npos);
    CHECK(dump.find("# trace end") != std::string::npos);
    // The dump unlocks the ring so the loop can fill it again.
    CHECK(!loop_trace::locked());

    send(motor, "W\n");
    std::string replay = take_tx();
    CHECK(replay.rfind("# trace: samples=1", 0) == 0);
    CHECK(replay.find("# trace end") != std::string::npos);

    // An unarmed event must not produce a dump on its own.
    loop_trace::push(sample);
    pump(motor);
    CHECK(take_tx().empty());
}

void test_speed_path_is_sign_symmetric(FakeMotor& motor) {
    // The open-loop voltage channel had a sign asymmetry: a reversal took the
    // current-limiting slew instead of the gentle one, so one direction ramped
    // and the other kicked. This test pins the speed path down against the same
    // class of bug: same command magnitude, same ramp, same clamp, mirrored.
    motor.set_ceiling(1.0f);
    motor.set_max_speed(1.0f);   // full scale, so the ceiling is what clamps
    motor.set_speed_ramp(2.0f);  // m/s^2, the façade default

    // Ramp transient: one 100 ms period at 2 m/s^2 = 0.2 m/s, both signs.
    for (float command : {0.6f, 0.2f}) {
        const float expected = std::min(command, 0.2f);
        motor.set_speed_target(0.0f);
        motor.set_speed_target(command);
        motor.apply_speed_ramp(0.1f);
        const float plus = motor.get_applied_speed_target();
        CHECK(std::abs(plus - expected) < 1e-6f);

        motor.set_speed_target(0.0f);
        motor.set_speed_target(-command);
        motor.apply_speed_ramp(0.1f);
        const float minus = motor.get_applied_speed_target();
        CHECK(std::abs(minus + expected) < 1e-6f);
        CHECK(std::abs(plus + minus) < 1e-6f);
    }

    // Clamp: symmetric about zero and flagged in both directions.
    motor.set_speed_target(1.5f);
    motor.apply_speed_ramp(10.0f);
    CHECK(std::abs(motor.get_applied_speed_target() - 1.0f) < 1e-6f);
    CHECK(motor.is_target_clamped());
    CHECK(std::abs(motor.target - wheel_rad_s(1.0f)) < 1e-4f);

    motor.set_speed_target(-1.5f);
    motor.apply_speed_ramp(10.0f);
    CHECK(std::abs(motor.get_applied_speed_target() + 1.0f) < 1e-6f);
    CHECK(motor.is_target_clamped());
    CHECK(std::abs(motor.target + wheel_rad_s(1.0f)) < 1e-4f);

    // An in-range command clears the clamp, again in both directions.
    for (float command : {0.5f, -0.5f}) {
        motor.set_speed_target(command);
        motor.apply_speed_ramp(10.0f);
        CHECK(!motor.is_target_clamped());
        CHECK(std::abs(motor.get_applied_speed_target() - command) < 1e-6f);
    }

    motor.set_speed_target(0.0f);
    motor.set_ceiling(1000.0f);
    motor.set_max_speed(0.25f);
    motor.set_speed_ramp(2.0f);
}

}

int main() {
    static UART_TypeDef uart_registers;
    huart2.Instance = &uart_registers;
    static UART_TypeDef mirror_registers;
    huart1.Instance = &mirror_registers;
    huart1.tx_capture = &fake_uart_mirror_tx;

    FakeMotor motor;
    serial_line_init();

    serial_line_poll(motor);
    pump(motor);
    const std::string boot = take_tx();
    CHECK(boot.rfind("# serial_line ready:", 0) == 0);
    CHECK(take_mirror() == boot);

    send(motor, "D0\n");
    CHECK(take_tx() == "# rx D0\r\nD=0\r\n");
    CHECK(take_mirror() == "# rx D0\r\n");
    send(motor, "M0\n");
    CHECK(take_tx() == "M=0\r\n");

    test_setpoints(motor);
    test_speed_target_uses_wheel_radius(motor);
    test_enable(motor);
    test_zero(motor);
    test_status(motor);
    test_status_current(motor);
    test_auto_calibrate(motor);
    test_debug_mirror(motor);
    test_telemetry(motor);
    test_errors(motor);
    test_help(motor);
    test_overlong_line(motor);
    test_rx_overflow(motor);
    test_blank_and_stray_terminators(motor);
    test_verbose_echo(motor);
    test_noise_discard(motor);
    test_rx_error_discards_line(motor);
    test_config_command(motor);
    test_config_readback(motor);
    test_info(motor);
    test_lpf_ramp_alias_config(motor);
    test_trace_counters_and_dump(motor);
    test_speed_path_is_sign_symmetric(motor);

    std::cout << "serial_line: all checks passed\n";
    return 0;
}
