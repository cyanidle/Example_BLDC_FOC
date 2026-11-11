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

loop_trace::Sample sample_at(uint32_t time_us, uint8_t flags = 0) {
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
    for (uint8_t bit = 0; bit < 7; ++bit) {
        const uint8_t flag = (uint8_t)(1u << bit);
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
    CHECK(counters.angle_snap == 0);
    CHECK(counters.guard_trips == 0);
    CHECK(counters.driver_faults == 0);
}

}

int main() {
    test_ring_keeps_newest_and_reads_oldest_first();
    test_trigger_is_one_shot_and_needs_arming();
    test_locked_ring_drops_samples();
    test_counters_start_clear();
    std::cout << "loop_trace: ok\n";
    return 0;
}
