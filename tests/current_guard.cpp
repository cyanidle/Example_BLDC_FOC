#include <cmath>
#include <cstdlib>
#include <iostream>

#include "current_guard.h"
#include "motor_tuning.h"

#define CHECK(condition) do { \
    if (!(condition)) { \
        std::cerr << __func__ << ':' << __LINE__ << ": " #condition "\n"; \
        std::exit(1); \
    } \
} while (false)

using current_guard::Fault;
using current_guard::Guard;

// Drives the guard at 1 kHz (a realistic control-loop period) and returns the
// elapsed simulated time when the predicate holds, or -1 after `limit_ms`.
static float run_until(
    Guard& guard,
    float current_a,
    float velocity_rad_s,
    bool command_active,
    float limit_ms,
    bool (Guard::*predicate)() const
) {
    float t_ms = 0.0f;
    while (t_ms < limit_ms) {
        (void)guard.step(current_a, velocity_rad_s, command_active, 0.001f);
        t_ms += 1.0f;
        if ((guard.*predicate)()) {
            return t_ms;
        }
    }
    return -1.0f;
}

static current_guard::Config bench_config();

static Guard bench_guard() {
    return Guard(bench_config());
}

static current_guard::Config bench_config() {
    return current_guard::Config{
        .lpf_tf_s = motor_tuning::simplefoc_guard_lpf_tf_s,
        .peak_a = motor_tuning::simplefoc_oc_peak_a,
        .peak_ms = motor_tuning::simplefoc_oc_peak_ms,
        .sustain_a = motor_tuning::simplefoc_oc_sustain_a,
        .sustain_ms = motor_tuning::simplefoc_oc_sustain_ms,
        .stall_current_a = motor_tuning::simplefoc_stall_current_a,
        .stall_velocity_rad_s = motor_tuning::simplefoc_stall_velocity_rad_s,
        .stall_ms = motor_tuning::simplefoc_stall_ms,
    };
}

static void common_mode_is_removed() {
    // Idle offsets drift together on the shared front end (~0.6 A was
    // observed on the bench); the magnitude must not see them.
    CHECK(current_guard::phase_current_magnitude(1.0f, 1.0f, 1.0f) < 1e-4f);
    CHECK(current_guard::phase_current_magnitude(0.58f, 0.57f, 0.59f) < 0.05f);
    // The reverse lock dump: I_B=+15.77, I_C=-13.44, I_A=0.79 (common-mode
    // 1.04 A removed by the mean).
    CHECK(std::abs(current_guard::phase_current_magnitude(0.79f, 15.77f, -13.44f) - 14.73f) < 0.2f);
    CHECK(current_guard::phase_current_magnitude(-8.0f, 4.0f, 4.0f) > 5.9f);
    // A non-finite sample must not poison the result.
    CHECK(current_guard::phase_current_magnitude(NAN, 1.0f, 1.0f) == 0.0f);
}

static void thresholds_keep_headroom_over_the_current_limit() {
    // Retuning the current limit or the guard must keep the protection above
    // the q-current ceiling the regulators ramp to, or every current-limited
    // acceleration would fault.
    CHECK(motor_tuning::simplefoc_oc_sustain_a > motor_tuning::simplefoc_current_limit_a);
    CHECK(motor_tuning::simplefoc_oc_peak_a > motor_tuning::simplefoc_oc_sustain_a);
    CHECK(motor_tuning::simplefoc_stall_current_a < motor_tuning::simplefoc_current_limit_a);
}

static void current_limited_acceleration_does_not_trip() {
    Guard guard = bench_guard();
    const float tripped_at = run_until(guard, 8.0f, 20.0f, true, 5000.0f, &Guard::is_tripped);
    CHECK(tripped_at < 0.0f);
    CHECK(guard.step(8.0f, 20.0f, true, 0.001f) == Fault::NONE);
}

static void peak_overcurrent_trips_fast() {
    Guard guard = bench_guard();
    // 15 A crosses the 14 A peak threshold through the 5 ms filter after
    // ~14 ms, so the fault lands well inside 50 ms — and not before the
    // window has even started.
    const float tripped_at = run_until(guard, 15.0f, 0.0f, true, 50.0f, &Guard::is_tripped);
    CHECK(tripped_at > 15.0f);
    CHECK(tripped_at < 50.0f);
    CHECK(guard.step(0.0f, 0.0f, false, 0.001f) == Fault::OVERCURRENT);
}

static void sustained_overcurrent_trips() {
    Guard guard = bench_guard();
    // 11 A is over the 10 A sustain threshold but under the 14 A peak: only
    // the 250 ms window can fire, and it must not fire early.
    const float tripped_at = run_until(guard, 11.0f, 0.0f, true, 400.0f, &Guard::is_tripped);
    CHECK(tripped_at > 200.0f);
    CHECK(tripped_at < 300.0f);
    CHECK(guard.step(0.0f, 0.0f, false, 0.001f) == Fault::OVERCURRENT);
}

static void chatter_below_the_window_does_not_trip() {
    Guard guard = bench_guard();
    for (int i = 0; i < 6; ++i) {
        // 100 ms over, 100 ms under: the sustain window resets on every dip.
        run_until(guard, 11.0f, 0.0f, true, 100.0f, &Guard::is_tripped);
        run_until(guard, 0.0f, 0.0f, true, 100.0f, &Guard::is_tripped);
        CHECK(!guard.is_tripped());
    }
}

static void locked_rotor_stalls() {
    Guard guard = bench_guard();
    // The reverse lock: 3 A flowing (soft-limited), shaft frozen, command on.
    const float tripped_at = run_until(guard, 3.0f, 0.0f, true, 700.0f, &Guard::is_tripped);
    CHECK(tripped_at > 500.0f);
    CHECK(tripped_at < 650.0f);
    CHECK(guard.step(0.0f, 0.0f, false, 0.001f) == Fault::STALLED);
}

static void moving_or_idle_shaft_does_not_stall() {
    {
        Guard guard = bench_guard();
        const float tripped_at = run_until(guard, 3.0f, 1.0f, true, 2000.0f, &Guard::is_tripped);
        CHECK(tripped_at < 0.0f);
    }
    {
        // Parked bridge: current flows (offsets) but no command is active.
        Guard guard = bench_guard();
        const float tripped_at = run_until(guard, 3.0f, 0.0f, false, 2000.0f, &Guard::is_tripped);
        CHECK(tripped_at < 0.0f);
    }
    {
        // Freewheeling to a stop: no current, no stall.
        Guard guard = bench_guard();
        const float tripped_at = run_until(guard, 0.0f, 0.0f, true, 2000.0f, &Guard::is_tripped);
        CHECK(tripped_at < 0.0f);
    }
}

static void latch_is_sticky_and_rearms() {
    Guard guard = bench_guard();
    run_until(guard, 15.0f, 0.0f, true, 50.0f, &Guard::is_tripped);
    CHECK(guard.is_tripped());
    for (int i = 0; i < 100; ++i) {
        CHECK(guard.step(0.0f, 0.0f, false, 0.001f) == Fault::OVERCURRENT);
    }
    guard.reset();
    CHECK(!guard.is_tripped());
    CHECK(guard.step(0.0f, 0.0f, false, 0.001f) == Fault::NONE);
}

static void zero_dt_does_not_accumulate() {
    Guard guard = bench_guard();
    for (int i = 0; i < 10000; ++i) {
        (void)guard.step(100.0f, 0.0f, true, 0.0f);
    }
    CHECK(!guard.is_tripped());
}

static void zero_stall_window_disables_stall() {
    // The six-step configuration: its hall-progress watchdog owns stalls, so
    // the guard runs with the stall timer off and must never latch one.
    current_guard::Config config = bench_config();
    config.stall_ms = 0;
    Guard guard(config);
    const float tripped_at = run_until(guard, 3.0f, 0.0f, true, 5000.0f, &Guard::is_tripped);
    CHECK(tripped_at < 0.0f);
}

static void zero_peak_window_trips_as_soon_as_the_level_is_crossed() {
    current_guard::Config config = bench_config();
    config.peak_ms = 0;
    Guard guard(config);
    const float tripped_at = run_until(guard, 15.0f, 0.0f, true, 50.0f, &Guard::is_tripped);
    CHECK(tripped_at > 0.0f);
    CHECK(tripped_at < 30.0f);
    CHECK(guard.step(0.0f, 0.0f, false, 0.001f) == Fault::OVERCURRENT);
}

static void configure_replaces_parameters() {
    Guard guard = bench_guard();
    current_guard::Config config = bench_config();
    config.peak_a = 5.0f;
    config.peak_ms = 10;
    guard.configure(config);
    // 6 A now crosses the lowered peak level well inside the window.
    const float tripped_at = run_until(guard, 6.0f, 0.0f, true, 50.0f, &Guard::is_tripped);
    CHECK(tripped_at > 0.0f);
    CHECK(tripped_at < 40.0f);
    // The latch survives a configure(): only reset() clears it.
    current_guard::Config same = bench_config();
    guard.configure(same);
    CHECK(guard.is_tripped());
}

int main() {
    common_mode_is_removed();
    thresholds_keep_headroom_over_the_current_limit();
    current_limited_acceleration_does_not_trip();
    peak_overcurrent_trips_fast();
    sustained_overcurrent_trips();
    chatter_below_the_window_does_not_trip();
    locked_rotor_stalls();
    moving_or_idle_shaft_does_not_stall();
    latch_is_sticky_and_rearms();
    zero_dt_does_not_accumulate();
    zero_stall_window_disables_stall();
    zero_peak_window_trips_as_soon_as_the_level_is_crossed();
    configure_replaces_parameters();
    return 0;
}
