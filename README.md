# BLDC wheel module: 6-step / FOC firmware

## Motor control implementations (compile-time switch)

`VB_MOTOR_IMPL` selects the implementation at configure time:

| value | backend | control |
|-------|---------|---------|
| `simplefoc` (default) | vendored Arduino-FOC v2.4.0 (`Drivers/Arduino-FOC`) | 3-PWM sine FOC, d/q current loops over the board's shunts, SimpleFOC velocity PID in q-current |
| `sixstep` | libvoltbro `SixStepController` + external velocity PID | Hall-only six-step commutation, run-time stall recovery |
| `voltbro-foc` | libvoltbro `FOC` | CORDIC field-oriented control with d/q current loops at 5 kHz, velocity loop inside the library |

All three sit behind one interface, `App/motor_backend.h` (`MotorBackend`), and
`App/app.cpp` is implementation-agnostic: it forwards the two CAN command topics
to `set_voltage_point()` / `set_speed_target_normalized()` (speed commands are
-1..1 of the module's full-scale wheel speed, config id 16), calls `update()`
every loop iteration, runs the speed loop from the fixed 200 Hz slot, and
reports velocity / accumulated wheel travel / faults identically. `make_motor_backend()`
is defined by the single backend translation unit CMake compiles in.

Speed, travel and every command source speak wheel units (m/s, m); the m/s <-> wheel rad/s
conversion and the wheel radius itself live in the façade — the unit contract is at the
top of `App/motor_backend.h`.

```sh
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Debug
ninja -C build

cmake -S . -B build-sixstep -G Ninja -DCMAKE_BUILD_TYPE=Debug -DVB_MOTOR_IMPL=sixstep
ninja -C build-sixstep

cmake -S . -B build-foc -G Ninja -DCMAKE_BUILD_TYPE=Debug -DVB_MOTOR_IMPL=voltbro-foc
ninja -C build-foc
```

Use one build directory per implementation (the object files differ), or
reconfigure an existing one with the other value.

`App/motor_tuning.h` holds the bench-tunable constants of the compiled backend
(FOC current regulators, speed observer, SimpleFOC filters and voltage caps).
The wheel radius and the velocity-PID gains also arrive at run time on the CAN
config topic and win over those defaults.

## Hardware

Wheel module of the 57BL95S15-230 class (24 V, 150 W, 8.7 A, 0.5 N*m,
**4 pole pairs**):

- `drive_info.common.ppairs = 4`; the Hall decoder counts one state per
  commutation step, so `CPR = 6 * pole pairs = 24`.
- TIM1 CH1/CH2/CH3 (PA8/PA9/PA10) drive the gate driver's high-side inputs
  `INH_A/B/C`. `INL_A/B/C` (PB13/14/15) are plain GPIOs held high for the whole
  run — the driver derives its own dead time from the high-side pulses.
- `DRV_ENABLE` (PB3) enables the bridge (low = blocked). `nFAULT` (PB5) is
  active low and has the MCU's internal pull-up; every backend pulses
  `DRV_ENABLE` low for a moment on start.
  `nFAULT` is only reported through the Cyphal heartbeat, it does not inhibit
  the PWM by itself.
- Hall inputs `ENC_1/2/3` = PB6/PC7/PC8, EXTI on both edges. The `HallSensor`
  constructor maps the harness as (B, C, A); if a motor cogs or refuses to
  drive, verify the per-module Hall permutation first.
- Bus voltage and phase currents come from ADC1 through libvoltbro `Inverter`,
  which each backend's `init()` starts (it also records the current offsets).

## Firmware example

This example uses common functions and motor "API" classes from
[libvoltbro](https://github.com/VBCores/libvoltbro) and
[libcxxcanard](https://github.com/VBCores/libcxxcanard) for Cyphal integration.
Project is generated with CubeMX with CMake toolchain. All code is in
"[App](App)" directory, Inc and Src contain only startup and peripheral config
code.

All main logic is in [App/app.cpp](App/app.cpp). There you can find a motor
backend instance, the Hall sensor, communication logic and some glue code. You
can use these classes directly (but note that they may not implement full
functionality of the driver), subclass them to customize or add some logic, or
use their sources from [libvoltbro](https://github.com/VBCores/libvoltbro) as an
example of how to configure this board.

Common functionality of this firmware, not directly related to motor control:

- Clock and common timers configuration (microsecond counter, LED heartbeat)
- Basic EEPROM storage class
- Configured FDCAN+Cyphal communication stack ([libcxxcanard](https://github.com/VBCores/libcxxcanard))

## Backend notes

### six-step

Hall-only commutation with a voltage set-point from an external PID. See the
stall-recovery section below.

### voltbro-foc

Field-oriented control with the library's own d/q current regulators and a
velocity regulator (`control_reg`, target in rad/s, output in A).

- `update()` self-throttles to `motor_tuning::foc_period_us` (200 µs, 5 kHz),
  because the speed observer assumes a fixed period; its gains in
  `motor_tuning.h` were computed for exactly that period with
  `voltbro/motors/bldc/foc/g_calculator.py`.
- The FOC reads sin/cos from the CORDIC peripheral through the LL API.
  CubeMX only configures the instance, so the backend programs the unit itself
  in `init()` (cosine, Q1.31, two results, 10 cycles).
- There is no electrical calibration: `encoder.electric_offset` stays 0 and the
  Hall-derived electrical angle is quantized to 60 degrees. Expect to bench-tune
  the current PI, the observer bandwidth and the offset before the drive is
  competitive with the six-step one.

### simplefoc

Arduino-FOC v2.4.0 (`Drivers/Arduino-FOC`, provenance and the list of local
adaptations in `Drivers/Arduino-FOC/PROVENANCE.md`) running without an Arduino
framework: `Drivers/Arduino-FOC/compat` provides the Arduino core surface
(pins, time base) and `Drivers/Arduino-FOC/board` replaces the library's weak
PWM hooks with the TIM1/INH channels of this board.

- The Hall sensor is a `Sensor` adapter over libvoltbro's decoder (the EXTI
  handler in `app.cpp` owns the counting), so no Arduino interrupt or pin code
  is used for it.
- No `initFOC()` alignment by default — the motor does not move at boot. Set
  `VB_SIMPLEFOC_ALIGN_ON_INIT=1` (backend defines) to find the zero electrical
  angle at start-up, or set `VB_SIMPLEFOC_SENSOR_DIRECTION` /
  `VB_SIMPLEFOC_ZERO_ELECTRIC_ANGLE` to known values.
- PWM 25 kHz, a 19 V driver cap and 8 A current limit, velocity PID 0.05 / 2.0
  and `LPF_velocity.Tf = 0.1` follow the reference bring-up sketch for this wheel
  module; they live in `App/motor_tuning.h`.
- `motor.voltage_limit` is additionally clamped to `driver.voltage_limit / 2`:
  centered SinePWM swings every phase around half the driver limit, so a larger
  q-voltage set-point would only flat-top the sine. The 17 V entry in
  `motor_tuning.h` is the upper bound, not the effective limit on a 24 V bus.

#### Current control

Torque control runs in `TorqueControlType::foc_current`: the velocity PID's
output is a q-current set-point (A, clamped to the 8 A current limit) and
SimpleFOC's `PID_current_q` / `PID_current_d` close the loop over the measured
phase currents. The currents come from an `InverterCurrentSense` adapter over
the same libvoltbro `Inverter` every backend uses for bus voltage — it owns
the ADC+DMA and the zero-current offsets (sampled with the bridge disabled),
the adapter only converts its amp-valued phases into SimpleFOC's
`PhaseCurrent_s`. Because the ADC free-runs with 16x oversampling, the
measurement is not PWM-synchronised; the current LPF
(`simplefoc_current_lpf_tf`) removes the residual ripple.

`initFOC()`'s current-sense alignment is skipped (`skip_align`): it probes the
phases with live voltage while this firmware parks the bridge during init.
The phase mapping and polarities are compile-time bench knowledge instead:
`VB_SIMPLEFOC_CURRENT_SIGN_A/B/C` (backend defines, default `+1`). Bench
procedure for a new module: lift the wheel, `E1`, apply a small direct voltage
(`Q0.5`) and read `S ... i=` — the reported q-current must follow the sign of
the applied voltage and settle near zero at zero volts. A wrong sign makes the
wheel hum against itself (the current PID saturates at the voltage limit);
flip that phase's macro and re-verify the drive direction afterwards, because
the hardcoded zero electrical angle is only valid for one polarity set.

The open-loop voltage channel (serial `Q` / Cyphal 4050) deliberately leaves
current mode — its number stays volts. The status line reports the measured
q-current as an extra ` i=<A>` field (`S t=... v=... e=... f=... i=...`),
which is how the sign check above is observed.

#### Soft start

Three layers, so no command source can step the bridge:

- speed commands are ramped by the `MotorBackend` façade
  (`apply_speed_ramp`, 2 m/s²) before they reach the velocity PID;
- the velocity PID's output (the q-current set-point) is slewed by
  `PID_velocity.output_ramp` (`simplefoc_torque_ramp_a_s`, 8 A/s), so the
  torque builds over ~1 s to the current limit and a PID reset (enable, mode
  switch) restarts the ramp from zero;
- the open-loop voltage channel slews its applied voltage
  (`simplefoc_voltage_ramp_v_s`, 20 V/s) in `update()`; a zero command
  bypasses the ramp and parks the bridge immediately;
- the current guard (`App/current_guard.h`) watches the common-mode-removed
  phase-current magnitude: while it is over the current limit the open-loop
  channel's voltage ceiling integrates down (current-limited acceleration),
  14 A for 25 ms or 10 A for 250 ms parks the bridge with an OVERCURRENT
  fault, and a shaft stalled under command for 600 ms parks it with STALLED.
  Both latch until a zero command rearms them; the parameters are runtime
  config (ids 5-11).

## CAN config coverage

`CFG_WHEEL_DIAMETER` (id 0) and the velocity gains `CFG_PID_KP/KI/KD` (1-3) reach
all three backends; the gain units follow the backend's regulator output
(V per rad/s for six-step, A per rad/s for the FOC backends — a wheel tuned on
one backend needs its gains re-scaled when reflashed to another).
`CFG_PID_I_LIMIT` (4) only applies to the six-step and libvoltbro FOC backends —
SimpleFOC's `PIDController` has no integral clamp of its own (it clamps the whole
output by `limit`), so the value is accepted and ignored when
`VB_MOTOR_IMPL=simplefoc`.

The current guard ids `CFG_OC_*` (5-8) and `CFG_STALL_*` (9-11) carry the
overcurrent/stall protection parameters of `App/current_guard.h`: the
overcurrent levels and windows apply to every backend that measures phase
current (six-step and SimpleFOC both read the module's 3-shunt front end), the
stall part only to SimpleFOC — the six-step hall-progress watchdog owns stalls
and would be pre-empted by the guard's shorter window. A stall window of 0
disables the stall check. All of them are EEPROM-persisted (config record
version 3; older records are rejected and fall back to the compiled defaults).

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

The backend additionally runs the current guard's overcurrent part over the
Inverter's phase currents (the controller itself has no current limit): 14 A
for 25 ms or 10 A for 250 ms parks the bridge with an OVERCURRENT fault,
rearmed by a zero command like the stall fault. The guard's stall timer is
forced off here — the kick recovery above needs its full window.

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

The FOC backends report only `INVALID_SUPPLY` (a non-finite or non-positive bus
voltage); the diagnostic text is prefixed with the backend name.

### Hardware-free verification

The host tests compile the actual Hall decoder and six-step controller against
a small fake HAL, exercising all sectors/directions, missed/duplicate edges,
bounded kicks, successful recovery, Hall chatter, invalid inputs, zero/disable
rearming, timer wrap, and the `test.lua` pulse sequence. The serial-line test
also covers the `S` status line, including the optional ` i=<A>` current field
backends with current measurement append. They do not model motor torque,
gate-driver timing, or prove that a physical motor will start. The
`voltbro-foc` and `simplefoc` backends have no host test of their own; they are
verified by building all three configurations.

```sh
cmake -S tests -B tests/build -G Ninja
cmake --build tests/build
ctest --test-dir tests/build --output-on-failure
ninja -C build
```
