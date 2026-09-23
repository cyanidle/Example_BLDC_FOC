# BLDC motor: *6 step commutation*

## Firmware example

This example uses common functions and motor "API" classes from [libvoltbro](https://github.com/VBCores/libvoltbro) and [libcxxcanard](https://github.com/VBCores/libcxxcanard) for Cyphal integration. Project is generated with CubeMX with CMake toolchain. All code is in "[App](App)" directory, Inc and Src contain only startup and peripheral config code.

All main logic is in [App/app.cpp](App/app.cpp). There you can find a **6-step BLDC motor** and **encoder** classes instances, communication logic and some glue code. You can use this classes directly (but note that they may not implement full functionality of the driver), subclass them to customize or add some logic, or use their sources from [libvoltbro](https://github.com/VBCores/libvoltbro) as an example of how to configure this board.

Common functionality of this firmware, not directly related to motor control:

- Clock and common timers configuration (microsecond counter, LED heartbeat)
- Basic EEPROM storage class
- Configured FDCAN+Cyphal communication stack ([libcxxcanard](https://github.com/VBCores/libcxxcanard))

## Six-step startup/stall recovery

The controller polls the physical Hall inputs as well as handling EXTI edges.
Polling repairs a missed interrupt, and duplicate samples do not add encoder
counts. Invalid Hall states (`000`/`111`) always coast; recovery never guesses
a phase pair from invalid inputs.

With a nonzero voltage command and no forward Hall-count progress for 200 ms,
the controller advances the stator field by one 60-degree electrical sector in
the commanded direction. This kick lasts at most 40 ms, ending early on forward
Hall progress. It uses the **requested voltage**, with the existing 95% PWM cap;
there is no voltage boost. It then returns to ordinary Hall-based commutation.

There are at most three kicks before latching a stall fault and coasting. With
no movement, the sequence faults after approximately 920 ms. A full electrical
turn of forward progress since the last kick restores the retry budget; rocking
across one Hall boundary does not. Repeated nonzero commands do not clear a
latched stall. A zero command or explicit disable rearms recovery. Zero also
coasts immediately, including during a kick.

This fits the repository's `test.lua`: it streams direct voltage at 10 Hz and
alternates one-second `+4 V`, zero, `-4 V`, zero phases. Each zero phase rearms the
next attempt. The 200 ms threshold is a bring-up setting, not a tuned crawling
speed controller: legitimate Hall intervals longer than it can trigger kicks.
The constants are in `six_step_controller.h` in libvoltbro.

Invalid Hall inputs, invalid bus voltage/commands, and exhausted recovery are
reported through Cyphal heartbeat health (`WARNING`) and a diagnostic record
on subject 8184 when the fault changes. For example:
`six-step: stalled; send zero to rearm; kicks=3`.
The unmodified `test.lua` has no diagnostic subscription; use a Cyphal diagnostic
subscriber to see these records. Recovery state and attempt count are also
available through `get_fault()`, `is_recovering()`, and `get_recovery_attempts()`.

Recovery can escape an occasional equilibrium and bounds a failed attempt. It
does not calibrate the motor's Hall/phase relationship or prove that it is wired
correctly. A persistently wrong mapping still needs hardware verification.

### Hardware-free verification

The host tests compile the actual Hall decoder and six-step controller against
a small fake HAL, exercising all sectors/directions, missed/duplicate edges,
bounded kicks, successful recovery, Hall chatter, invalid inputs, zero/disable
rearming, timer wrap, and the `test.lua` pulse sequence. They do not model motor
torque, gate-driver timing, or prove that a physical motor will start.

Run from `SixStep/`:

```sh
cmake -S tests -B tests/build -G Ninja
cmake --build tests/build
ctest --test-dir tests/build --output-on-failure
ninja -C build
```
