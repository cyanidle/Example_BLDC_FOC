# BLDC motor: *6 step commutation*

## Firmware example

This example uses common functions and motor "API" classes from [libvoltbro](https://github.com/VBCores/libvoltbro) and [libcxxcanard](https://github.com/VBCores/libcxxcanard) for Cyphal integration. Project is generated with CubeMX with CMake toolchain. All code is in "[App](App)" directory, Inc and Src contain only startup and peripheral config code.

All main logic is in [App/app.cpp](App/app.cpp). There you can find a **6-step BLDC motor** and **encoder** classes instances, communication logic and some glue code. You can use this classes directly (but note that they may not implement full functionality of the driver), subclass them to customize or add some logic, or use their sources from [libvoltbro](https://github.com/VBCores/libvoltbro) as an example of how to configure this board.

Common functionality of this firmware, not directly related to motor control:

- Clock and common timers configuration (microsecond counter, LED heartbeat)
- Basic EEPROM storage class
- Configured FDCAN+Cyphal communication stack ([libcxxcanard](https://github.com/VBCores/libcxxcanard))
