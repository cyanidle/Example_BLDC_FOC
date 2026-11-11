#include <cmath>
#include <cstdlib>
#include <iostream>

#include "motor_limits.h"

#define CHECK(condition) do { \
    if (!(condition)) { \
        std::cerr << __func__ << ':' << __LINE__ << ": " #condition "\n"; \
        std::exit(1); \
    } \
} while (false)

// The drive these numbers come from: 57BL95S15-230 with the 64:1 gearbox,
// torque_const 0.069 V per rotor rad/s, an 18.5 V rail. Its modulation ceiling
// is min(bus, 19) * 0.5 * 0.9 = 8.325 V and the EMF per wheel rad/s is 0.069 *
// 64 = 4.416 V.
static constexpr float EMF_PER_WHEEL_RAD_S = 0.069f * 64.0f;
static constexpr float CEILING_V = 18.5f * 0.5f * 0.9f;
static constexpr float FLOOR_V = 1.6f;

// The current-loop budget must sit above the back-EMF at every speed, or the
// loop saturates and the vector it holds brakes the wheel: a 3.2 V/(rad/s) cap
// against a 4.416 V/(rad/s) EMF lost authority above 0.21 m/s.
static void test_current_loop_budget_clears_the_emf() {
    for (float rate = 0.0f; rate <= 5.0f; rate += 0.05f) {
        const float limit = motor_limits::current_loop_voltage_limit(
            EMF_PER_WHEEL_RAD_S, rate, FLOOR_V
        );
        CHECK(limit > EMF_PER_WHEEL_RAD_S * rate);
        CHECK(std::abs(
            limit - (FLOOR_V + EMF_PER_WHEEL_RAD_S * rate)) < 1e-4f);
    }
    // Direction must not matter: the budget follows |speed|.
    CHECK(std::abs(
        motor_limits::current_loop_voltage_limit(EMF_PER_WHEEL_RAD_S, -2.0f, FLOOR_V) -
        motor_limits::current_loop_voltage_limit(EMF_PER_WHEEL_RAD_S, 2.0f, FLOOR_V)
    ) < 1e-6f);
}

// The reachable speed is where the EMF plus the loop's floor meets the
// modulation ceiling: (8.325 - 1.6) / 4.416 = 1.523 wheel rad/s = 0.2437 m/s on
// a 0.32 m wheel — below the 0.3 m/s a bench run used to ask for, which is why
// that target sat at the ceiling and the drive faulted.
static void test_reachable_rate() {
    const float rate = motor_limits::reachable_wheel_rate(CEILING_V, EMF_PER_WHEEL_RAD_S, FLOOR_V);
    CHECK(rate > 1.50f && rate < 1.55f);
    CHECK(std::abs(rate - (CEILING_V - FLOOR_V) / EMF_PER_WHEEL_RAD_S) < 1e-6f);

    // The relationship is linear in the rail, which is what lets one
    // derivation serve every supply.
    const float high = motor_limits::reachable_wheel_rate(CEILING_V + 2.0f, EMF_PER_WHEEL_RAD_S, FLOOR_V);
    const float low = motor_limits::reachable_wheel_rate(CEILING_V - 2.0f, EMF_PER_WHEEL_RAD_S, FLOOR_V);
    CHECK(high > rate && low < rate);
    CHECK(std::abs((high - rate) - 2.0f / EMF_PER_WHEEL_RAD_S) < 1e-4f);
    CHECK(std::abs((rate - low) - 2.0f / EMF_PER_WHEEL_RAD_S) < 1e-4f);
}

// Degenerate inputs must report "no regulatable speed" instead of a negative
// or infinite one: the caller then clamps the target to zero.
static void test_degenerate_inputs() {
    CHECK(motor_limits::reachable_wheel_rate(CEILING_V, 0.0f, FLOOR_V) == 0.0f);
    CHECK(motor_limits::reachable_wheel_rate(0.0f, EMF_PER_WHEEL_RAD_S, FLOOR_V) == 0.0f);
    // A ceiling under the floor leaves no headroom at all.
    CHECK(motor_limits::reachable_wheel_rate(FLOOR_V, EMF_PER_WHEEL_RAD_S, FLOOR_V) == 0.0f);
    CHECK(motor_limits::reachable_wheel_rate(FLOOR_V * 0.5f, EMF_PER_WHEEL_RAD_S, FLOOR_V) == 0.0f);
}

int main() {
    test_current_loop_budget_clears_the_emf();
    test_reachable_rate();
    test_degenerate_inputs();
    std::cout << "motor_limits: ok\n";
    return 0;
}
