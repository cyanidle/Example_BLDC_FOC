#include <cstdlib>
#include <iostream>

#include "loop_trace.h"

#define CHECK(condition) do { \
    if (!(condition)) { \
        std::cerr << __func__ << ':' << __LINE__ << ": " #condition "\n"; \
        std::exit(1); \
    } \
} while (false)

namespace {

loop_trace::Sample sample_at(uint32_t time_us, uint16_t flags = 0) {
    loop_trace::Sample sample = {};
    sample.time_us = time_us;
    sample.flags = flags;
    return sample;
}

void test_ring_keeps_newest_and_reads_oldest_first() {
    loop_trace::reset();
    for (uint32_t i = 0; i < 10; ++i) {
        loop_trace::push(sample_at(i));
    }
    CHECK(loop_trace::count() == 10);
    CHECK(loop_trace::at(0).time_us == 0);
    CHECK(loop_trace::at(9).time_us == 9);

    // Fill past the depth: the oldest samples fall out, the ordering holds.
    const uint32_t total = loop_trace::DEPTH + 13;
    for (uint32_t i = 10; i < total; ++i) {
        loop_trace::push(sample_at(i));
    }
    CHECK(loop_trace::count() == loop_trace::DEPTH);
    CHECK(loop_trace::at(0).time_us == total - loop_trace::DEPTH);
    CHECK(loop_trace::at(loop_trace::DEPTH - 1).time_us == total - 1);
}

void test_trigger_is_one_shot_and_needs_arming() {
    loop_trace::reset();
    // Unarmed: an event must not latch a trigger.
    loop_trace::push(sample_at(0, loop_trace::HALL_ILLEGAL));
    CHECK(!loop_trace::take_trigger());

    loop_trace::arm(true);
    CHECK(loop_trace::armed());
    loop_trace::push(sample_at(1, loop_trace::HALL_ILLEGAL));
    CHECK(!loop_trace::armed());
    CHECK(loop_trace::take_trigger());
    // Reported exactly once, and a quiet sample never triggers.
    CHECK(!loop_trace::take_trigger());

    loop_trace::arm(true);
    loop_trace::push(sample_at(2, 0));
    CHECK(!loop_trace::take_trigger());

    // Every triggering flag counts as an event.
    for (uint16_t bit = 0; bit < 15; ++bit) {
        const uint16_t flag = (uint16_t)(1u << bit);
        loop_trace::arm(true);
        loop_trace::push(sample_at(3, flag));
        const bool expected = (loop_trace::TRIGGER_FLAGS & flag) != 0;
        CHECK(loop_trace::take_trigger() == expected);
    }
}

void test_locked_ring_drops_samples() {
    loop_trace::reset();
    for (uint32_t i = 0; i < 5; ++i) {
        loop_trace::push(sample_at(i));
    }
    loop_trace::lock(true);
    CHECK(loop_trace::locked());
    // The history being read out must not be overwritten.
    for (uint32_t i = 5; i < 20; ++i) {
        loop_trace::push(sample_at(i));
    }
    CHECK(loop_trace::count() == 5);
    CHECK(loop_trace::at(0).time_us == 0);
    CHECK(loop_trace::at(4).time_us == 4);
    loop_trace::lock(false);
    CHECK(!loop_trace::locked());
    loop_trace::push(sample_at(20));
    CHECK(loop_trace::count() == 6);
    CHECK(loop_trace::at(5).time_us == 20);
}

void test_counters_start_clear() {
    loop_trace::reset();
    const volatile loop_trace::Counters& counters = loop_trace::counters();
    CHECK(counters.hall_illegal == 0);
    CHECK(counters.guard_trips == 0);
    CHECK(counters.driver_faults == 0);
    // The blame counters: one per mechanism that can stop the drive.
    CHECK(counters.guard_overcurrent == 0);
    CHECK(counters.guard_stalled == 0);
    CHECK(counters.derate_high == 0);
    CHECK(counters.derate_low == 0);
    CHECK(counters.angle_snaps == 0);
    CHECK(counters.angle_wraps == 0);
    CHECK(counters.velocity_timeouts == 0);
    CHECK(counters.current_limits == 0);
    CHECK(counters.voltage_limits == 0);
    CHECK(counters.brake_clamps == 0);
    CHECK(counters.coast_entries == 0);
    CHECK(counters.regulator_resets == 0);
    CHECK(counters.slow_loops == 0);
    CHECK(counters.loop_max_us == 0);
    CHECK(counters.heap_allocations == 0);
    CHECK(counters.heap_hot_allocations == 0);
    CHECK(counters.heap_max_us == 0);
    CHECK(counters.heap_hot_max_us == 0);
}

void test_every_event_fits_its_flag_and_trigger_set() {
    // A sample carries 16 bits of flags; every defined flag has to sit inside
    // them and the trigger set has to be a subset of the defined flags.
    const uint16_t all = loop_trace::HALL_ILLEGAL | loop_trace::RATE_IMPLAUSIBLE |
        loop_trace::GUARD_TRIP | loop_trace::SUPPLY_DERATE | loop_trace::DRIVER_FAULT |
        loop_trace::TARGET_CLAMPED | loop_trace::ANGLE_SNAP | loop_trace::ANGLE_WRAP |
        loop_trace::SLOW_LOOP | loop_trace::CURRENT_LIMIT | loop_trace::VOLTAGE_LIMIT |
        loop_trace::BRAKE_CLAMP | loop_trace::COAST | loop_trace::REGULATOR_RESET |
        loop_trace::VELOCITY_TIMEOUT;
    CHECK((loop_trace::TRIGGER_FLAGS & ~all) == 0);
    loop_trace::Sample sample = sample_at(0, all);
    CHECK(sample.flags == all);
}

}

int main() {
    test_ring_keeps_newest_and_reads_oldest_first();
    test_trigger_is_one_shot_and_needs_arming();
    test_locked_ring_drops_samples();
    test_counters_start_clear();
    test_every_event_fits_its_flag_and_trigger_set();
    std::cout << "loop_trace: ok\n";
    return 0;
}
