#include <cmath>
#include <cstdlib>
#include <iostream>

#include "voltage_slew.h"

#define CHECK(condition) do { \
    if (!(condition)) { \
        std::cerr << __func__ << ':' << __LINE__ << ": " #condition "\n"; \
        std::exit(1); \
    } \
} while (false)

// The compiled rates (motor_tuning.h): up 20 V/s, current-limited drop 200 V/s,
// operator stop / sign change 5 V/s.
static constexpr float UP = 20.0f;
static constexpr float REDUCE = 200.0f;
static constexpr float SHED = 5.0f;

static float step1(float applied, float target, float dt = 0.001f) {
    return voltage_slew::slew(applied, target, UP, REDUCE, SHED, dt);
}

static void test_growing_magnitude_uses_up_rate() {
    // 1 ms at 20 V/s = 0.02 V, in both directions.
    CHECK(std::abs(step1(0.0f, 2.0f) - 0.02f) < 1e-6f);
    CHECK(std::abs(step1(0.0f, -2.0f) + 0.02f) < 1e-6f);
}

static void test_shrinking_magnitude_uses_reduce_rate() {
    // A ceiling-forced drop and an operator move with the sign unchanged both
    // take the fast (current-limiting) rate: 200 V/s -> 0.2 V per ms.
    CHECK(std::abs(step1(2.0f, 1.0f) - 1.8f) < 1e-5f);
    CHECK(std::abs(step1(-2.0f, -1.0f) + 1.8f) < 1e-5f);
}

static void test_stop_sheds_at_the_gentle_rate() {
    // A zero command must not short the windings across their back-EMF: the
    // stop rate (5 V/s) is what lets the wheel follow the voltage down.
    CHECK(std::abs(step1(2.0f, 0.0f) - 1.995f) < 1e-5f);
    CHECK(std::abs(step1(-2.0f, 0.0f) + 1.995f) < 1e-5f);
    // The stop rate is 40x slower than the current-limiting rate.
    CHECK(step1(2.0f, 0.0f) > step1(2.0f, 1.0f));
}

static void test_reversal_passes_through_zero_at_the_gentle_rate() {
    // Regression: with a value-vs-target slew the +2 -> -2 move took the
    // "value is above the target" branch, i.e. 200 V/s, and the vector stepped
    // through 180 degrees in ~20 ms — the reported bump. Now the sign may only
    // change once the magnitude is zero, and the shed takes |V|/5 s.
    float applied = 2.0f;
    const float dt = 0.001f;
    int steps = 0;
    while (applied > 0.0f && steps < 5000) {
        applied = step1(applied, -2.0f, dt);
        ++steps;
    }
    // 2 V at 5 V/s is 400 ms, and no step may skip the crossing.
    CHECK(steps >= 399 && steps <= 401);
    CHECK(applied == 0.0f);
    // ...and only then does it grow with the other sign, at the up rate.
    const float after = step1(applied, -2.0f, dt);
    CHECK(after < 0.0f && std::abs(after + 0.02f) < 1e-6f);
}

static void test_saturates_at_the_target() {
    // A step larger than the target magnitude lands on it exactly, in both
    // directions, and never overshoots.
    CHECK(step1(0.0f, 2.0f, 1.0f) == 2.0f);
    CHECK(step1(0.0f, -2.0f, 1.0f) == -2.0f);
    CHECK(step1(2.0f, 0.5f, 1.0f) == 0.5f);
    CHECK(step1(-2.0f, -0.5f, 1.0f) == -0.5f);
    // Its magnitude never exceeds the target's on the grow path.
    CHECK(std::abs(step1(1.9f, 2.0f, 1.0f)) == 2.0f);
}

static void test_degenerate_inputs_hold_the_voltage() {
    // A non-positive dt is a skipped pass, not a step.
    CHECK(voltage_slew::slew(1.5f, -2.0f, UP, REDUCE, SHED, 0.0f) == 1.5f);
    CHECK(voltage_slew::slew(1.5f, -2.0f, UP, REDUCE, SHED, -1.0f) == 1.5f);
    // A shed rate of zero pins the magnitude (and keeps the sign).
    CHECK(voltage_slew::slew(1.5f, -2.0f, UP, REDUCE, 0.0f, 0.01f) == 1.5f);
}

int main() {
    test_growing_magnitude_uses_up_rate();
    test_shrinking_magnitude_uses_reduce_rate();
    test_stop_sheds_at_the_gentle_rate();
    test_reversal_passes_through_zero_at_the_gentle_rate();
    test_saturates_at_the_target();
    test_degenerate_inputs_hold_the_voltage();
    std::cout << "voltage_slew: ok\n";
    return 0;
}
