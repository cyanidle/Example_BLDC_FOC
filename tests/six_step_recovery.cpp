#include <cstdlib>
#include <iostream>
#include <limits>

#include "voltbro/motors/bldc/six_step/six_step_controller.h"

#define CHECK(condition) do { \
    if (!(condition)) { \
        std::cerr << __func__ << ':' << __LINE__ << ": " #condition "\n"; \
        std::exit(1); \
    } \
} while (false)

using Fault = SixStepController::Fault;

struct FakeInverter : BaseInverter {
    FakeInverter() { busV = 24.0f; }
    void start() override { is_started = true; }
    void update() override {}
    void voltage(float value) { busV = value; }
};

struct Rig {
    GPIO_TypeDef hall_pins, bridge_pins;
    TIM_TypeDef timer;
    TIM_HandleTypeDef htim{&timer};
    FakeInverter inverter;
    DriveRuntimeConfig config;
    DriveInfo info{
        .torque_const = 0.069f,
        .max_current = 22, .max_torque = 1,
        .stall_current = 13, .stall_timeout = 3, .stall_tolerance = 0.2f,
        .calibration_voltage = 0,
        .l_pins = std::array<GpioPin, 3>{
            GpioPin(&bridge_pins, 1), GpioPin(&bridge_pins, 2), GpioPin(&bridge_pins, 4)
        },
        .en_pin = GpioPin(&bridge_pins, 8),
        .common = {.ppairs = 2, .gear_ratio = 1}
    };
    HallSensor hall;
    SixStepController motor;
    uint32_t epoch;

    explicit Rig(bool inverted = false, uint8_t raw = 1, bool trusted_exti = false) :
        hall(12, inverted, &hall_pins, 1, &hall_pins, 2, &hall_pins, 4,
             {HallPhase::PHASE_B, HallPhase::PHASE_C, HallPhase::PHASE_A}, trusted_exti),
        motor(config, info, &htim, inverter, hall) {
        fake_tick = 0;
        fake_primask = 0;
        set_hall(raw);
        hall.resync();
        CHECK(motor.init() == HAL_OK);
        CHECK(motor.start() == HAL_OK);
        epoch = fake_tick;
    }

    void set_hall(uint8_t raw) {
        hall_pins.IDR = ((raw & 2) ? 1 : 0) | ((raw & 4) ? 2 : 0) | ((raw & 1) ? 4 : 0);
    }
    void at(uint32_t elapsed) {
        fake_tick = epoch + elapsed;
        motor.update();
        CHECK(fake_primask == 0);
    }
    void coasted() const {
        CHECK(timer.CCR1 == 0 && timer.CCR2 == 0 && timer.CCR3 == 0);
        CHECK((bridge_pins.ODR & 7) == 0);
    }
    void driving(unsigned source, unsigned sink) const {
        const uint32_t duty[] = {timer.CCR1, timer.CCR2, timer.CCR3};
        for (unsigned i = 0; i < 3; ++i) CHECK(duty[i] == (i == source ? 333U : 0U));
        CHECK((bridge_pins.ODR & 7) == ((1U << source) | (1U << sink)));
    }
};

void hall_polling_and_resync() {
    for (bool trusted : {false, true}) {
        Rig r(false, 1, trusted);
        CHECK(r.hall.get_value() == 0);
        r.set_hall(3);
        r.hall.handle_hall_channel(1);
        CHECK(r.hall.get_value() == 1); // resync must seed the first transition
        r.hall.update_value();
        CHECK(r.hall.get_value() == 1);
        r.set_hall(2);
        r.at(1); // no EXTI: polling must observe the missed edge
        CHECK(r.hall.get_value() == 2);
        r.hall.handle_hall_channel(4); // delayed/duplicate EXTI must not double count
        CHECK(r.hall.get_value() == 2);
        CHECK(r.hall.get_step() == EncoderStep::BC);
        fake_primask = 1;
        r.hall.update_value();
        CHECK(fake_primask == 1);
        fake_primask = 0;
    }
}

void bounded_retries_and_rearm() {
    Rig r;
    r.motor.set_voltage_point(4);
    for (uint32_t t = 0; t <= 920; ++t) {
        if (t % 100 == 0) r.motor.set_voltage_point(4); // test.lua streamer
        r.at(t);
        if (t == 199) {
            CHECK(!r.motor.is_recovering());
            r.driving(0, 1);
        }
        if (t == 200 || t == 439) {
            CHECK(r.motor.get_recovery_attempts() == 1);
        }
        if ((t >= 200 && t < 240) || (t >= 440 && t < 480) || (t >= 680 && t < 720)) {
            CHECK(r.motor.is_recovering());
            r.driving(0, 2); // +60 degrees, same 4 V duty
        }
    }
    CHECK(r.motor.get_fault() == Fault::STALLED);
    CHECK(r.motor.get_recovery_attempts() == 3);
    r.coasted();
    r.set_hall(3); // even externally moving a faulted motor must not restart it
    r.motor.set_voltage_point(-4);
    r.at(1000);
    CHECK(r.motor.get_fault() == Fault::STALLED);
    r.coasted();
    r.motor.set_voltage_point(0);
    CHECK(r.motor.get_fault() == Fault::NONE);
    CHECK(r.motor.get_recovery_attempts() == 0);
    r.coasted();
    r.motor.set_voltage_point(4);
    r.at(1001); // zero and nonzero can arrive before the next update
    r.driving(0, 2);
    r.motor.stop();
    r.coasted();
    r.motor.start();
    r.motor.set_voltage_point(4);
    r.at(1100);
    CHECK(!r.motor.is_recovering());
}

void successful_kick_and_later_stall() {
    Rig r;
    r.motor.set_voltage_point(4);
    r.at(0);
    r.at(200);
    CHECK(r.motor.is_recovering());
    const uint8_t cycle[] = {3, 2, 6, 4, 5, 1};
    for (unsigned i = 0; i < 6; ++i) {
        r.set_hall(cycle[i]);
        r.at(210 + i * 10);
        CHECK(!r.motor.is_recovering());
        CHECK(r.motor.get_fault() == Fault::NONE);
        if (i < 5) CHECK(r.motor.get_recovery_attempts() == 1);
    }
    CHECK(r.motor.get_recovery_attempts() == 0);
    r.at(459);
    CHECK(!r.motor.is_recovering());
    r.at(460);
    CHECK(r.motor.is_recovering());
}

void both_directions_and_all_sectors() {
    const uint8_t cycle[] = {1, 3, 2, 6, 4, 5};
    const unsigned source[] = {0, 0, 1, 1, 2, 2};
    const unsigned sink[] = {1, 2, 2, 0, 0, 1};
    for (bool inverted : {false, true}) {
        for (int sign : {-1, 1}) {
            for (unsigned i = 0; i < 6; ++i) {
                Rig r(inverted, cycle[i]);
                const bool reverse = (sign < 0) != inverted;
                r.motor.set_voltage_point(sign * 4.0f);
                r.at(0);
                r.driving(reverse ? sink[i] : source[i], reverse ? source[i] : sink[i]);
                const unsigned next = (i + (reverse ? 5 : 1)) % 6;
                r.at(200);
                r.driving(reverse ? sink[next] : source[next], reverse ? source[next] : sink[next]);
                r.set_hall(cycle[next]);
                r.at(210);
                CHECK(!r.motor.is_recovering());
                CHECK(r.motor.get_fault() == Fault::NONE);
            }
        }
    }
}

void chatter_does_not_replenish_retries() {
    Rig r;
    r.motor.set_voltage_point(4);
    r.at(0);
    for (uint32_t t = 1; t <= 1200; ++t) {
        r.set_hall((t / 10) % 2 ? 3 : 1); // rock across one boundary
        r.at(t);
    }
    CHECK(r.motor.get_fault() == Fault::STALLED);
    r.coasted();
}

void invalid_inputs_and_zero_abort() {
    Rig r;
    r.motor.set_voltage_point(4);
    r.set_hall(7);
    r.at(0);
    CHECK(r.motor.get_fault() == Fault::INVALID_HALL);
    r.coasted();
    r.at(1000);
    CHECK(r.motor.get_recovery_attempts() == 0);
    r.set_hall(0);
    r.at(1001);
    CHECK(r.motor.get_fault() == Fault::INVALID_HALL);
    r.coasted();
    r.set_hall(1); // recover valid physical inputs without an EXTI
    r.at(1002);
    CHECK(r.motor.get_fault() == Fault::NONE);
    r.driving(0, 1);
    for (float volts : {0.0f, -1.0f, std::numeric_limits<float>::quiet_NaN()}) {
        r.inverter.voltage(volts);
        r.at(1003);
        CHECK(r.motor.get_fault() == Fault::INVALID_SUPPLY);
        r.coasted();
    }
    r.inverter.voltage(24);
    r.motor.set_voltage_point(std::numeric_limits<float>::infinity());
    r.at(1004);
    CHECK(r.motor.get_fault() == Fault::INVALID_COMMAND);
    r.coasted();
    r.motor.set_voltage_point(0);
    r.at(1005);
    r.motor.set_voltage_point(4);
    r.at(1006);
    r.at(1206);
    CHECK(r.motor.is_recovering());
    r.motor.set_voltage_point(0);
    r.at(1207);
    CHECK(!r.motor.is_recovering());
    CHECK(r.motor.get_fault() == Fault::NONE);
    r.coasted();
}

void timer_wrap() {
    Rig r;
    r.epoch = UINT32_MAX - 100;
    r.motor.set_voltage_point(4);
    for (uint32_t t = 0; t <= 920; ++t) r.at(t);
    CHECK(r.motor.get_fault() == Fault::STALLED);
    r.coasted();
}

void test_lua_pulses() {
    Rig r;
    const float targets[] = {0, 4, 0, -4, 0};
    for (uint32_t t = 0; t < 5000; ++t) {
        const float voltage = targets[t / 1000];
        if (t % 100 == 0) r.motor.set_voltage_point(voltage);
        r.at(t);
        if (voltage == 0) {
            CHECK(r.motor.get_fault() == Fault::NONE);
            CHECK(r.motor.get_recovery_attempts() == 0);
            r.coasted();
        } else if (t % 1000 >= 920) {
            CHECK(r.motor.get_fault() == Fault::STALLED);
            r.coasted();
        } else {
            CHECK(r.motor.get_fault() == Fault::NONE);
        }
    }
}

int main() {
    hall_polling_and_resync();
    bounded_retries_and_rearm();
    successful_kick_and_later_stall();
    both_directions_and_all_sectors();
    chatter_does_not_replenish_retries();
    invalid_inputs_and_zero_abort();
    timer_wrap();
    test_lua_pulses();
    std::cout << "six-step recovery: all checks passed\n";
}
