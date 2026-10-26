#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

#include "motor_backend.h"
#include "serial_line.h"
#include "usart.h"

UART_HandleTypeDef huart2;

millis_t millis_32() {
    return fake_tick;
}

// app.cpp in the firmware; the test records what the channel applies.
int32_t applied_config_id = -1;
float applied_config_value = 0.0f;

void app_apply_config(int32_t id, float value) {
    applied_config_id = id;
    applied_config_value = value;
}

#define CHECK(condition) do { \
    if (!(condition)) { \
        std::cerr << __func__ << ':' << __LINE__ << ": " #condition "\n"; \
        std::exit(1); \
    } \
} while (false)

namespace {

struct FakeMotor : MotorBackend {
    float velocity = 0.0f;
    MotorFault fault = MotorFault::NONE;
    bool enabled = false;
    float target = 0.0f;
    float voltage = 0.0f;

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
    void set_speed_target(float linear_m_s) override { target = linear_m_s; }
    void run_speed_loop(float) override {}
    void set_wheel_radius(float) override {}
    void set_velocity_pid(const PIDConfig&) override {}
    float get_velocity() const override { return velocity; }
    MotorFault get_fault() const override { return fault; }
};

std::string take_tx() {
    std::string out = fake_uart_tx;
    fake_uart_tx.clear();
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

// Feeds in chunks with a poll in between, the way the real main loop drains the
// receive ring while a long line is still arriving.
void send_slow(FakeMotor& motor, const std::string& text, size_t chunk) {
    for (size_t offset = 0; offset < text.size(); offset += chunk) {
        fake_uart_feed(&huart2, text.substr(offset, chunk).c_str());
        serial_line_poll(motor);
    }
}

void test_setpoints(FakeMotor& motor) {
    send(motor, "T0.5\r\n");
    CHECK(take_tx() == "T=0.500\r\n");
    CHECK(motor.target == 0.5f);

    send(motor, "Q2.25\n");
    CHECK(take_tx() == "Q=2.250\r\n");
    CHECK(motor.voltage == 2.25f);

    send(motor, "T-1.25\n");
    CHECK(take_tx() == "T=-1.250\r\n");
    CHECK(motor.target == -1.25f);

    send(motor, "T12.3456\n");
    CHECK(take_tx() == "T=12.346\r\n");
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
    send(motor, "X\n");
    CHECK(take_tx() == "OK\r\n");
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

void test_help(FakeMotor& motor) {
    send(motor, "?\n");
    const std::string out = take_tx();
    CHECK(out.rfind("# T<float>", 0) == 0);
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

    send(motor, "T9.5\n");
    CHECK(take_tx() == "T=9.500\r\n");
    CHECK(motor.target == 9.5f);

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
    CHECK(applied_config_id == 1);
    CHECK(std::abs(applied_config_value - 0.08f) < 1e-6f);

    send(motor, "C0:0.1\n");
    CHECK(take_tx() == "C0=0.100\r\n");
    CHECK(applied_config_id == 0);
    CHECK(std::abs(applied_config_value - 0.1f) < 1e-6f);

    send(motor, "C:\n");
    CHECK(take_tx() == "ERR C:\r\n");
    send(motor, "C1\n");
    CHECK(take_tx() == "ERR C1\r\n");
    send(motor, "Cx:1\n");
    CHECK(take_tx() == "ERR Cx:1\r\n");
}

}

int main() {
    static UART_TypeDef uart_registers;
    huart2.Instance = &uart_registers;

    FakeMotor motor;
    serial_line_init();

    serial_line_poll(motor);
    pump(motor);
    CHECK(take_tx().rfind("# serial_line ready:", 0) == 0);

    send(motor, "D0\n");
    CHECK(take_tx() == "# rx D0\r\nD=0\r\n");
    send(motor, "M0\n");
    CHECK(take_tx() == "M=0\r\n");

    test_setpoints(motor);
    test_enable(motor);
    test_zero(motor);
    test_status(motor);
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

    std::cout << "serial_line: all checks passed\n";
    return 0;
}
