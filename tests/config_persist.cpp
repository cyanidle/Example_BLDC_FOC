#include <cstdlib>
#include <iostream>

#include "config_persist.h"

#define CHECK(condition) do { \
    if (!(condition)) { \
        std::cerr << __func__ << ':' << __LINE__ << ": " #condition "\n"; \
        std::exit(1); \
    } \
} while (false)

using config_persist::DriveState;
using config_persist::drive_idle;

// An EEPROM record write stalls the app loop ~14 ms, so it may only run where
// that costs nothing: no set-point, none applied, wheel not turning.
static void test_stopped_drive_is_idle() {
    CHECK(drive_idle(DriveState{0.0f, 0.0f, 0.0f}));
    // A wheel that has just stopped but still reports a trickle of speed is not
    // idle yet: the stall would land during the tail of a deceleration.
    CHECK(drive_idle(DriveState{0.0f, 0.0f, -0.009f}));
    CHECK(!drive_idle(DriveState{0.0f, 0.0f, 0.05f}));
    CHECK(!drive_idle(DriveState{0.0f, 0.0f, -0.05f}));
}

static void test_running_drive_is_never_idle() {
    // Any commanded or applied set-point defers the write, both directions.
    CHECK(!drive_idle(DriveState{0.2f, 0.0f, 0.0f}));
    CHECK(!drive_idle(DriveState{0.0f, 0.2f, 0.0f}));
    CHECK(!drive_idle(DriveState{-0.2f, -0.2f, -0.2f}));
    // Ramping down towards zero is still running.
    CHECK(!drive_idle(DriveState{0.0f, 0.01f, 0.01f}));
    // The open-loop voltage channel has zero set-points while the wheel spins,
    // so the measured speed is what defers the write there.
    CHECK(!drive_idle(DriveState{0.0f, 0.0f, 2.8f}));
}

int main() {
    test_stopped_drive_is_idle();
    test_running_drive_is_never_idle();
    std::cout << "config_persist: ok\n";
    return 0;
}
