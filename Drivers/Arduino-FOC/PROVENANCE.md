# Arduino-FOC (SimpleFOC) — vendored copy

Upstream: <https://github.com/simplefoc/Arduino-FOC> tag `v2.4.0`
(release archive `Arduino-FOC-2.4.0.zip`,
sha256 `1de6d703dc067ab1fcbaadf2724867caa040718a8d231017847b81f0ae1f9416`),
MIT licensed, see `LICENSE`.

Vendored contents: `src/`, `LICENSE`, `README.md`, `library.json`,
`library.properties`. The release archive's `examples/` (7.8 MB), `.github/`,
`Doxyfile`, `logo.png`, `CNAME`, `CITATION.cff`, `CODE_OF_CONDUCT.md` and
`keywords.txt` are **not** vendored: they are not needed to build the library and
would dominate the repository size. Re-download the release archive if you need
them.

## Local adaptations

`src/` is upstream apart from one cosmetic edit: the unused parameter of
`FOCMotor::estimateBEMF()` is left unnamed in `FOCMotor.h` so it does not trip
`-Wunused-parameter`. Everything this firmware needs on top of the library lives
in two directories that are ours:

- `compat/` — a minimal Arduino core for the bare-metal CubeMX/HAL build. There
  is no Arduino framework here, so `Arduino.h`, the pin API, `millis()`/`micros()`
  and a `Print` stub are provided by the firmware. `Arduino.h` also supplies
  `min`/`max` as typed overloads instead of Arduino's macros, so it does not
  break `std::` calls in the rest of the firmware; `abs` comes from `<cmath>`.
- `board/` — the STM32G474 bridge glue. The library's hardware layer
  (`src/drivers/hardware_specific/generic_mcu.cpp`, and every other
  `hardware_specific` backend) is *not* compiled; `board/vbcore_g4_pwm.cpp`
  provides the strong `_configure3PWM()` / `_writeDutyCycle3PWM()` definitions that
  the library declares, driving the board's TIM1 high-side channels and INL
  per-phase enable pins.

Only the translation units listed in `CMakeLists.txt` are compiled — the rest of
`src/` (steppers, current sensing, other MCU backends, the Commander, the hall /
magnetic sensor drivers) is unused here and would not build against this shim.

## Updating

Replace `src/`, `LICENSE`, `README.md`, `library.json` and `library.properties`
with the new release, keep `compat/` and `board/`, then rebuild. Start with
`-DVB_MOTOR_IMPL=simplefoc` and fix whatever the new release broke in
`compat/Arduino.h`; check the compiled-file list for removed/renamed sources.
