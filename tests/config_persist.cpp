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
    CHECK(drive_idle(DriveState{0.0f, 0.0f, 0.0f, 0.0f}));
    // A wheel that has just stopped but still reports a trickle of speed is not
    // idle yet: the stall would land during the tail of a deceleration.
    CHECK(drive_idle(DriveState{0.0f, 0.0f, -0.009f, 0.0f}));
    CHECK(!drive_idle(DriveState{0.0f, 0.0f, 0.05f, 0.0f}));
    CHECK(!drive_idle(DriveState{0.0f, 0.0f, -0.05f, 0.0f}));
    // A hair of command (a joystick just above its deadzone, a tiny `T`) must
    // not defer the record forever.
    CHECK(drive_idle(DriveState{0.004f, 0.002f, 0.001f, 0.0f}));
}

static void test_running_drive_is_never_idle() {
    // Any commanded or applied set-point defers the write, both directions.
    CHECK(!drive_idle(DriveState{0.2f, 0.0f, 0.0f, 0.0f}));
    CHECK(!drive_idle(DriveState{0.0f, 0.2f, 0.0f, 0.0f}));
    CHECK(!drive_idle(DriveState{-0.2f, -0.2f, -0.2f, 0.0f}));
    // Ramping down towards zero is still running.
    CHECK(!drive_idle(DriveState{0.0f, 0.01f, 0.01f, 0.0f}));
    // The open-loop channel: zero set-points and a wheel that has not started
    // turning yet are not enough, because the bridge is applying voltage.
    CHECK(!drive_idle(DriveState{0.0f, 0.0f, 0.0f, 2.8f}));
    CHECK(!drive_idle(DriveState{0.0f, 0.0f, 0.0f, -0.9f}));
    // A latched speed target next to a `Q` command: both keep the gate shut.
    CHECK(!drive_idle(DriveState{0.3f, 0.0f, 2.8f, 4.0f}));
}

int main() {
    test_stopped_drive_is_idle();
    test_running_drive_is_never_idle();
    std::cout << "config_persist: ok\n";
    return 0;
}
