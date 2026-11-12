#pragma once

#ifdef __cplusplus
class MotorBackend;
#endif

/*
 * SimpleFOC-style serial line channel on USART2 (PA2 TX, PA3 RX, 115200 8N1).
 * It runs beside Cyphal: the cart keeps driving over CAN, this is a separate
 * bench/terminal channel that can command the same motor backend.
 *
 * One ASCII request per line, terminated by '\n' (a preceding '\r' is ignored,
 * blank lines are skipped). Replies end with "\r\n".
 *
 *   T<float>   target wheel speed, -1..1 of the module's full-scale wheel
 *              speed (config id 16, `G16`)  -> T=<value>
 *              (the module owns the physical mapping — wheel radius, gearbox,
 *               back-EMF constant, modulation ceiling — so a bench command
 *               cannot ask for a speed the drive has no voltage to regulate.
 *               Values outside -1..1 are clamped.)
 *   Q<float>   open-loop voltage, V          -> Q=<value>
 *   E<0|1>     driver enable                 -> E=<0|1>
 *              (the only way to change it: a module boots enabled and no
 *               Cyphal command, config write or register access enables or
 *               disables the bridge. `E1` also replays the gate-driver enable
 *               pulse, which is what clears a latched DRIVER_FAULT.)
 *   X          zero both set-points          -> OK
 *   M<rate>    telemetry rate in Hz, 0 = off -> M=<rate>
 *              (off until asked; max 100 Hz)
 *   D<0|1>     echo received lines as "# rx -> D=<0|1>"
 *              <line>" (on by default)
 *   V<0|1>     loop diagnostics: the telemetry tick sends a "D" line instead
 *              of the status line -> V=<0|1>
 *              D t=<m/s> v=<rad/s> sp=<A> iq=<A> id=<A> im=<A> uq=<V> ud=<V>
 *              b=<V> n=<counts> f=<fault> e=<0|1>
 *              (`t` applied speed set-point, `sp` velocity-PID output =
 *               q-current demand, `iq`/`id` the filtered d/q currents the
 *               regulators close over, `im` the unfiltered phase-current
 *               magnitude the guards see, `uq`/`ud` applied d/q voltage, `b`
 *               the bus voltage, `n` absolute rotor count, `f` the MotorFault
 *               enum value (0 NONE, 4 STALLED, 5 OVERCURRENT, 6 DRIVER_FAULT)
 *               and `e` the bridge enable state. SimpleFOC only; it is the
 *               tool that tells a velocity-loop limit cycle from a
 *               current-loop or angle fault. While on, the telemetry rate is
 *               capped at 71 Hz to fit one line into the 115200 baud link.)
 *   L<float>   velocity feedback low-pass, s (0 = off; SimpleFOC only)
 *                                            -> L=<value>
 *              alias of C12 (config id 12): persisted in the module EEPROM
 *              and read back with G12
 *   R<float>   wheel speed set-point ramp, m/s^2 -> R=<value>
 *              alias of C13 (config id 13): persisted, read back with G13
 *   C<id>:<v>  set a runtime config value, same ids as the CAN config topic
 *              (see App/config_ids.h / mods/config_defs.lua: 0 wheel diameter
 *              m, 1..3 velocity Kp/Ki/Kd, 4 integrator limit, 5..8
 *              overcurrent guard, 9..11 stall guard, 12 velocity LPF s,
 *              13 speed ramp m/s^2, 14 torque ramp A/s, 15 brake current A)
 *              -> C<id>=<value>; ERR when the id is unknown or the value is
 *              rejected (wrong range, or a knob the compiled backend lacks)
 *   G<id>      read a runtime config value back -> C<id>=<value> (the live
 *              value: compiled default, EEPROM record or last accepted write)
 *   S          one status line now           -> S t=<-1..1> v=<rad/s> e=<0|1> f=<FAULT> [i=<A>]
 *              (`t` is the commanded normalized set-point — the module's
 *               commanded m/s over its full-scale speed, whatever source issued
 *               it; `v` is the wheel angular velocity in rad/s; `i` is the
 *               measured q-axis current and appears only on backends with a
 *               current measurement, e.g. SimpleFOC's d/q loop)
 *   A          calibrate the motor and re-tune the
 *              current regulators (SimpleFOC only;
 *              driver must be enabled, wheel lifted) -> A=<0|1>
 *              (blocks for a few seconds with the phases energised: R/L are
 *               measured and the current PID gains re-derived from them; the
 *               values arrive as "# simplefoc calibrate: ..." debug lines and
 *               the gains live until reboot. Zero the set-points first.)
 *   K          stop/limit counters, five lines -> "# K1 hall bad=<n> edges=<n>
 *              calls=<n> rate_bad=<n> snap=<n> wrap=<n>"
 *              "# K2 stop guard_oc=<n> stall=<n> derate_hi=<n>
 *              derate_lo=<n> driver=<n> coast=<n>"
 *              "# K3 limit clamp=<n> cur=<n> volt=<n> brake=<n> reset=<n>"
 *              "# K4 loop slow=<n> max_us=<n> vel_to=<n> heap=<hot>/<all>
 *              heap_us=<hot_max>/<all_max>"
 *              "# K5 ph pid=<n> update=<n> poll=<n> persist=<n> cyphal=<n>"
 *              (One counter per mechanism that can stop the drive or take
 *               authority from a regulator, absolute totals since boot: read it
 *               before and after an event and the counter that moved names the
 *               mechanism. Hall: transitions no single step explains, counted
 *               edges, rate samples the drive could not have produced, angle
 *               snaps onto a sector edge, 0/2pi wraps of the angle estimate,
 *               rate timeouts. Stop: current-guard trips split by overcurrent
 *               and stall, supply-derate entries split by rail high (regen) and
 *               low (droop), gate-driver faults, coasts entered. Limit: target
 *               clamped to the ceiling, velocity loop at the current limit,
 *               current loop at its voltage limit, regen brake clamp, torque
 *               regulator resets. Loop: control periods over budget and the
 *               worst one since boot, plus free-store allocations made while
 *               the loop was running and the worst allocator call — the loop is
 *               required to make none, see App/heap_probe.h — and the worst
 *               duration of each phase of the app loop (pid slot, update, serial
 *               poll, config persistence, cyphal), which is what attributes a
 *               stall to a phase instead of leaving it a total.)
 *   W          dump the loop trace now       -> "# trace: samples=<n> depth=128"
 *              then one "# T <ms> e=<deg> a=<deg> v=<rad/s> iq=<A> id=<A>
 *              im=<A> uq=<V> ud=<V> b=<V> h=<0..7> ev=<bits>" line per sample
 *              (oldest first) and "# trace end"
 *   W<0|1>     arm (1) / disarm (0) the automatic dump -> W=<0|1>
 *              (while armed, the first sample that carries a Hall anomaly, an
 *               angle snap, a guard trip or a driver fault starts the dump;
 *               App/loop_trace.h holds ~51 ms at 2.5 kHz, which is the history
 *               a 100 Hz D line cannot show. The dump takes over the link for
 *               ~1 s and the telemetry resumes afterwards.)
 *   ?          command list                  -> one "# ..." line per command
 *   anything else                            -> ERR <line>
 *
 * Lines starting with '#' are diagnostics, not replies: the boot trace
 * ("# serial_line ready: motor=... enabled=... fault=... target=... max_m_s=...
 * voltage_v=...") and the per-command echo.
 *
 * `t` in the `D` diagnostics line is the applied physical set-point in m/s
 * (the ramp's output); the normalized command in the `S` line is the value the
 * sources send. Both are reported, so a clamped or ramping drive is visible
 * either way.
 */

#ifdef __cplusplus
void serial_line_init();
void serial_line_poll(MotorBackend& motor);

// Diagnostic line ("# <label> <value>") for the rest of the application, so
// startup and CAN activity show up on the same terminal as the protocol.
void serial_line_debug(const char* label);
void serial_line_debug(const char* label, float value);
#endif

// Byte-level receive handler, called from USART2_IRQHandler before the HAL
// handler. C linkage so the CubeMX interrupt file can call it.
#ifdef __cplusplus
extern "C" {
#endif
void serial_line_rx_irq(void);
#ifdef __cplusplus
}
#endif
