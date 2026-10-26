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
 *   T<float>   target wheel speed, m/s       -> T=<value>
 *   Q<float>   open-loop voltage, V          -> Q=<value>
 *   E<0|1>     driver enable                 -> E=<0|1>
 *   X          zero both set-points          -> OK
 *   M<rate>    telemetry rate in Hz, 0 = off -> M=<rate>
 *              (off until asked; max 100 Hz)
 *   D<0|1>     echo received lines as "# rx -> D=<0|1>"
 *              <line>" (on by default)
 *   C<id>:<v>  runtime config, same ids as the CAN config topic
 *              (0 wheel diameter m, 1..3 velocity Kp/Ki/Kd,
 *              4 integrator limit) -> C<id>=<value>
 *   S          one status line now           -> S t=<m/s> v=<rad/s> e=<0|1> f=<FAULT>
 *   ?          command list                  -> one "# ..." line per command
 *   anything else                            -> ERR <line>
 *
 * Lines starting with '#' are diagnostics, not replies: the boot trace
 * ("# serial_line ready: motor=... enabled=... fault=... target_m_s=...
 * voltage_v=...") and the per-command echo.
 *
 * `t` in the status line is the set-point last issued over this channel: a
 * set-point sent over Cyphal is not mirrored there. `v` is the shaft angular
 * velocity in rad/s, as reported by the backend.
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
