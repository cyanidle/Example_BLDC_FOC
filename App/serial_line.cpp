#include "serial_line.h"

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>

#include <voltbro/utils.hpp>

#include "config_ids.h"
#include "loop_trace.h"
#include "motor_backend.h"
#include "usart.h"

millis_t millis_32();
// app.cpp; declared here so this unit stays free of the Cyphal headers.
bool app_apply_config(int32_t id, float value);
bool app_get_config(int32_t id, float& value);

namespace {

constexpr size_t RX_RING_SIZE = 128;
constexpr size_t TX_RING_SIZE = 1024;
constexpr size_t TX_CHUNK_SIZE = 64;
constexpr size_t LINE_SIZE = 64;
constexpr unsigned TELEMETRY_MAX_HZ = 100;
constexpr unsigned REPLY_CAPACITY = 96;
// With `V1` the diagnostics line replaces the status line (it carries the
// set-point and the velocity too), so the rate is capped to keep one 96-byte
// line inside the 115200 baud link budget: 12 ms is ~7.4 kB/s of the 11.5 kB/s.
constexpr millis_t DIAGNOSTICS_MIN_PERIOD_MS = 12;

uint8_t rx_ring[RX_RING_SIZE];
volatile size_t rx_head = 0;
volatile size_t rx_tail = 0;

uint8_t tx_ring[TX_RING_SIZE];
volatile size_t tx_head = 0;
volatile size_t tx_tail = 0;
uint8_t tx_chunk[TX_CHUNK_SIZE];

char line_buffer[LINE_SIZE];
size_t line_length = 0;
bool line_overlong = false;
bool line_noise = false;
volatile bool rx_overflow = false;

struct SetPoints {
    float voltage_v = 0.0f;
    unsigned telemetry_hz = 0;
    millis_t telemetry_time = 0;
    bool verbose = true;
    // `V1`: append a `D ...` loop-diagnostics line to every telemetry tick.
    bool diagnostics = false;
    bool booted = false;
};

SetPoints set_points;

struct Line {
    char text[REPLY_CAPACITY] = {};
    size_t length = 0;
};

void line_text(Line& line, const char* text) {
    const size_t available = REPLY_CAPACITY - line.length;
    const size_t count = std::strlen(text) < available ? std::strlen(text) : available;
    std::memcpy(line.text + line.length, text, count);
    line.length += count;
}

void line_unsigned(Line& line, unsigned long value) {
    char digits[12];
    size_t count = 0;
    do {
        digits[count++] = static_cast<char>('0' + value % 10);
        value /= 10;
    } while (value != 0 && count < sizeof(digits));
    while (count > 0) {
        const char digit[2] = {digits[--count], '\0'};
        line_text(line, digit);
    }
}

void line_signed(Line& line, long value) {
    if (value < 0) {
        line_text(line, "-");
        line_unsigned(line, 0UL - static_cast<unsigned long>(value));
        return;
    }
    line_unsigned(line, static_cast<unsigned long>(value));
}

void tx_write(const char* text, size_t length);
void mirror_write(const char* text, size_t length);

// newlib-nano has no %f in printf unless the link pulls in _printf_float, so
// every value this channel reports is scaled to an integer and printed by hand.
void line_fixed(Line& line, float value, unsigned decimals) {
    if (std::isnan(value)) {
        line_text(line, "nan");
        return;
    }
    if (std::isinf(value)) {
        line_text(line, value < 0 ? "-inf" : "inf");
        return;
    }

    // Room for a gain as small as 0.0002: four decimals is not enough to tell
    // it from zero on a read-back.
    static const long scale_by_decimals[] = {1, 10, 100, 1000, 10000, 100000, 1000000};
    if (decimals >= sizeof(scale_by_decimals) / sizeof(scale_by_decimals[0])) {
        decimals = sizeof(scale_by_decimals) / sizeof(scale_by_decimals[0]) - 1;
    }
    const long scale = scale_by_decimals[decimals];

    const float scaled = value * static_cast<float>(scale);
    if (scaled > 2.0e9f || scaled < -2.0e9f) {
        line_text(line, "ovf");
        return;
    }
    long rounded = static_cast<long>(scaled + (scaled < 0 ? -0.5f : 0.5f));
    if (rounded < 0) {
        line_text(line, "-");
        rounded = -rounded;
    }
    line_unsigned(line, static_cast<unsigned long>(rounded / scale));
    if (decimals == 0) {
        return;
    }
    line_text(line, ".");
    long remainder = rounded % scale;
    for (long divisor = scale / 10; divisor > 0; divisor /= 10) {
        const char digit[2] = {static_cast<char>('0' + (remainder / divisor) % 10), '\0'};
        line_text(line, digit);
    }
}

void line_finish(Line& line) {
    // '#' marks diagnostics, and diagnostics are what the USART1 mirror
    // carries; protocol replies stay on USART2 alone.
    const bool diagnostics = line.text[0] == '#';
    line_text(line, "\r\n");
    tx_write(line.text, line.length);
    if (diagnostics) {
        mirror_write(line.text, line.length);
    }
}

void tx_kick() {
    // Only ever called from thread context (serial_line_poll / tx_write): the
    // HAL keeps gState BUSY_TX until its own IRQ path finished, so starting the
    // next chunk from the TX-complete callback would get HAL_BUSY.
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (huart2.gState == HAL_UART_STATE_READY) {
        const size_t available = (tx_head + TX_RING_SIZE - tx_tail) % TX_RING_SIZE;
        if (available > 0) {
            const size_t count = available < TX_CHUNK_SIZE ? available : TX_CHUNK_SIZE;
            for (size_t i = 0; i < count; ++i) {
                tx_chunk[i] = tx_ring[(tx_tail + i) % TX_RING_SIZE];
            }
            if (HAL_UART_Transmit_IT(&huart2, tx_chunk, static_cast<uint16_t>(count)) == HAL_OK) {
                tx_tail = (tx_tail + count) % TX_RING_SIZE;
            }
        }
    }
    __set_PRIMASK(primask);
}

// Room left in the transmit ring, bytes. The trace dump paces itself by this:
// several lines per poll pass outrun a 115200 baud link (the poll runs orders of
// magnitude faster), and tx_write drops a whole message it cannot fit — which is
// how the first dump lost 9 of every 10 samples.
size_t tx_free_space() {
    return TX_RING_SIZE - 1 - (tx_head + TX_RING_SIZE - tx_tail) % TX_RING_SIZE;
}

void tx_write(const char* text, size_t length) {
    if (length >= TX_RING_SIZE) {
        return;
    }
    if ((tx_head + TX_RING_SIZE - tx_tail) % TX_RING_SIZE + length >= TX_RING_SIZE) {
        // Dropping a whole message keeps the stream line-aligned; a partial one
        // would glue two replies into a single unparsable line.
        return;
    }
    for (size_t i = 0; i < length; ++i) {
        tx_ring[tx_head] = static_cast<uint8_t>(text[i]);
        tx_head = (tx_head + 1) % TX_RING_SIZE;
    }
    tx_kick();
}

// USART1 (PC4) mirror of the diagnostics: every '#' line this channel produces
// also goes out on the second UART, so a bench can watch the boot trace, echo
// and debug labels on a separate dongle while USART2 carries the protocol.
// Same ring-and-kick shape (and thread-context-kick-only rule) as above.
constexpr size_t MIRROR_RING_SIZE = 512;
uint8_t mirror_ring[MIRROR_RING_SIZE];
volatile size_t mirror_head = 0;
volatile size_t mirror_tail = 0;
uint8_t mirror_chunk[TX_CHUNK_SIZE];

void mirror_kick() {
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (huart1.gState == HAL_UART_STATE_READY) {
        const size_t available = (mirror_head + MIRROR_RING_SIZE - mirror_tail) % MIRROR_RING_SIZE;
        if (available > 0) {
            const size_t count = available < TX_CHUNK_SIZE ? available : TX_CHUNK_SIZE;
            for (size_t i = 0; i < count; ++i) {
                mirror_chunk[i] = mirror_ring[(mirror_tail + i) % MIRROR_RING_SIZE];
            }
            if (HAL_UART_Transmit_IT(&huart1, mirror_chunk, static_cast<uint16_t>(count)) == HAL_OK) {
                mirror_tail = (mirror_tail + count) % MIRROR_RING_SIZE;
            }
        }
    }
    __set_PRIMASK(primask);
}

void mirror_write(const char* text, size_t length) {
    if (length >= MIRROR_RING_SIZE) {
        return;
    }
    if ((mirror_head + MIRROR_RING_SIZE - mirror_tail) % MIRROR_RING_SIZE + length >= MIRROR_RING_SIZE) {
        return;
    }
    for (size_t i = 0; i < length; ++i) {
        mirror_ring[mirror_head] = static_cast<uint8_t>(text[i]);
        mirror_head = (mirror_head + 1) % MIRROR_RING_SIZE;
    }
    mirror_kick();
}

const char* fault_name(MotorFault fault) {
    switch (fault) {
        case MotorFault::NONE: return "NONE";
        case MotorFault::INVALID_HALL: return "INVALID_HALL";
        case MotorFault::INVALID_SUPPLY: return "INVALID_SUPPLY";
        case MotorFault::INVALID_COMMAND: return "INVALID_COMMAND";
        case MotorFault::STALLED: return "STALLED";
        case MotorFault::OVERCURRENT: return "OVERCURRENT";
        case MotorFault::DRIVER_FAULT: return "DRIVER_FAULT";
    }
    return "UNKNOWN";
}

void send_boot_trace(MotorBackend& motor) {
    Line line;
    line_text(line, "# serial_line ready: motor=");
    line_text(line, motor.name());
    line_text(line, " enabled=");
    line_unsigned(line, motor.is_on() ? 1 : 0);
    line_text(line, " fault=");
    line_text(line, fault_name(motor.get_fault()));
    line_text(line, " target=");
    line_fixed(line, motor.get_max_speed() > 0.0f ?
        motor.get_speed_target() / motor.get_max_speed() : 0.0f, 3);
    line_text(line, " max_m_s=");
    line_fixed(line, motor.get_max_speed(), 3);
    line_text(line, " voltage_v=");
    line_fixed(line, set_points.voltage_v, 3);
    line_finish(line);
}

void send_echo(const char* command) {
    Line line;
    line_text(line, "# rx ");
    line_text(line, command);
    line_finish(line);
}

void send_status(MotorBackend& motor) {    Line line;
    line_text(line, "S t=");
    // Normalized command (the value sources send): the commanded m/s over the
    // module's full-scale speed, whatever source issued it.
    const float full_scale = motor.get_max_speed();
    line_fixed(line, full_scale > 0.0f ? motor.get_speed_target() / full_scale : 0.0f, 3);
    line_text(line, " v=");
    line_fixed(line, motor.get_velocity(), 3);
    line_text(line, " e=");
    line_unsigned(line, motor.is_on() ? 1 : 0);
    line_text(line, " f=");
    line_text(line, fault_name(motor.get_fault()));
    // Only backends with a current measurement report it (SimpleFOC's d/q
    // loop); six-step keeps the byte-identical legacy line.
    const float current_a = motor.get_torque_current();
    if (std::isfinite(current_a)) {
        line_text(line, " i=");
        line_fixed(line, current_a, 2);
    }
    line_finish(line);
}

void send_diagnostics(MotorBackend& motor) {
    MotorDiagnostics diagnostics;
    if (!motor.get_diagnostics(diagnostics)) {
        return;
    }
    Line line;
    line_text(line, "D t=");
    line_fixed(line, diagnostics.speed_target_m_s, 3);
    line_text(line, " v=");
    line_fixed(line, motor.get_velocity(), 3);
    line_text(line, " sp=");
    line_fixed(line, diagnostics.q_current_setpoint_a, 2);
    line_text(line, " iq=");
    line_fixed(line, diagnostics.q_current_a, 2);
    line_text(line, " id=");
    line_fixed(line, diagnostics.d_current_a, 2);
    line_text(line, " im=");
    line_fixed(line, diagnostics.phase_current_a, 2);
    line_text(line, " uq=");
    line_fixed(line, diagnostics.q_voltage_v, 2);
    line_text(line, " ud=");
    line_fixed(line, diagnostics.d_voltage_v, 2);
    line_text(line, " b=");
    line_fixed(line, diagnostics.bus_voltage_v, 1);
    line_text(line, " n=");
    line_signed(line, (long)diagnostics.hall_counts);
    line_text(line, " f=");
    line_unsigned(line, (unsigned long)motor.get_fault());
    line_text(line, " e=");
    line_unsigned(line, motor.is_on() ? 1u : 0u);
    line_finish(line);
}

void send_error(const char* command) {
    Line line;
    line_text(line, "ERR ");
    line_text(line, command);
    line_finish(line);
}

/* ---- Loop trace dump (`W`, `K`) ---------------------------------------------
 * The ring buffer itself is App/loop_trace.*; this side owns the formatting and
 * the pacing. A full dump is ~12 kB over a 115200 baud link into a 1 kB transmit
 * ring, so it is emitted a few lines per poll pass (tx_write drops a message it
 * cannot fit whole) and the telemetry is suspended until it is through. */
struct TraceDump {
    bool active = false;
    size_t total = 0;
    size_t next = 0;
};
TraceDump trace_dump;

// One counter per mechanism that can stop the drive or take authority from a
// regulator (see loop_trace::Counters). Grouped into three replies because the
// serial reply buffer is 96 bytes; read `K` before and after an event and the
// counter that moved names the mechanism. Absolute totals since boot.
void send_counters() {
    const volatile loop_trace::Counters& counters = loop_trace::counters();

    {
        Line line;
        line_text(line, "# K1 hall bad=");
        line_unsigned(line, counters.hall_illegal);
        line_text(line, " edges=");
        line_unsigned(line, counters.hall_counted);
        line_text(line, " calls=");
        line_unsigned(line, counters.angle_calls);
        line_text(line, " rate_bad=");
        line_unsigned(line, counters.rate_implausible);
        line_text(line, " snap=");
        line_unsigned(line, counters.angle_snaps);
        line_text(line, " wrap=");
        line_unsigned(line, counters.angle_wraps);
        line_finish(line);
    }

    {
        Line line;
        line_text(line, "# K2 stop guard_oc=");
        line_unsigned(line, counters.guard_overcurrent);
        line_text(line, " stall=");
        line_unsigned(line, counters.guard_stalled);
        line_text(line, " derate_hi=");
        line_unsigned(line, counters.derate_high);
        line_text(line, " derate_lo=");
        line_unsigned(line, counters.derate_low);
        line_text(line, " driver=");
        line_unsigned(line, counters.driver_faults);
        line_text(line, " coast=");
        line_unsigned(line, counters.coast_entries);
        line_finish(line);
    }

    {
        Line line;
        line_text(line, "# K3 limit clamp=");
        line_unsigned(line, counters.target_clamps);
        line_text(line, " cur=");
        line_unsigned(line, counters.current_limits);
        line_text(line, " volt=");
        line_unsigned(line, counters.voltage_limits);
        line_text(line, " brake=");
        line_unsigned(line, counters.brake_clamps);
        line_text(line, " reset=");
        line_unsigned(line, counters.regulator_resets);
        line_finish(line);
    }

    {
        Line line;
        line_text(line, "# K4 loop slow=");
        line_unsigned(line, counters.slow_loops);
        line_text(line, " max_us=");
        line_unsigned(line, counters.loop_max_us);
        line_text(line, " vel_to=");
        line_unsigned(line, counters.velocity_timeouts);
        line_text(line, " heap=");
        line_unsigned(line, counters.heap_hot_allocations);
        line_text(line, "/");
        line_unsigned(line, counters.heap_allocations);
        line_text(line, " heap_us=");
        line_unsigned(line, counters.heap_hot_max_us);
        line_text(line, "/");
        line_unsigned(line, counters.heap_max_us);
        line_finish(line);
    }

    {
        Line line;
        line_text(line, "# K5 ph pid=");
        line_unsigned(line, counters.phase_pid_max_us);
        line_text(line, " update=");
        line_unsigned(line, counters.phase_update_max_us);
        line_text(line, " poll=");
        line_unsigned(line, counters.phase_poll_max_us);
        line_text(line, " persist=");
        line_unsigned(line, counters.phase_persist_max_us);
        line_text(line, " cyphal=");
        line_unsigned(line, counters.phase_cyphal_max_us);
        line_finish(line);
    }
}

void trace_begin_dump() {
    // Lock first: the ring must stop advancing before its length is trusted.
    loop_trace::lock(true);
    trace_dump.total = loop_trace::count();
    trace_dump.next = 0;
    trace_dump.active = true;
    Line line;
    line_text(line, "# trace: samples=");
    line_unsigned(line, (unsigned long)trace_dump.total);
    line_text(line, " depth=");
    line_unsigned(line, (unsigned long)loop_trace::DEPTH);
    line_finish(line);
}

void trace_service() {
    if (!trace_dump.active) {
        return;
    }
    // One line per call, and only while the transmit ring has room for a whole
    // one: the poll runs far faster than the link drains, so emitting a fixed
    // number of lines per pass simply overruns the ring (tx_write then drops
    // them whole, which is what lost most of the first bench dump).
    constexpr size_t TRACE_LINE_ROOM = 160;
    if (tx_free_space() < TRACE_LINE_ROOM) {
        return;
    }
    if (trace_dump.next < trace_dump.total) {
        const loop_trace::Sample& sample = loop_trace::at(trace_dump.next++);
        Line line;
        line_text(line, "# T ");
        line_unsigned(line, (unsigned long)(sample.time_us / 1000u));
        line_text(line, " e=");
        line_fixed(line, sample.angle_error_rad * (180.0f / (pi2 * 0.5f)), 1);
        line_text(line, " a=");
        line_fixed(line, sample.angle_deg, 1);
        line_text(line, " v=");
        line_fixed(line, sample.velocity_rad_s, 3);
        line_text(line, " iq=");
        line_fixed(line, sample.q_current_a, 2);
        line_text(line, " id=");
        line_fixed(line, sample.d_current_a, 2);
        line_text(line, " im=");
        line_fixed(line, sample.phase_current_a, 2);
        line_text(line, " uq=");
        line_fixed(line, sample.q_voltage_v, 2);
        line_text(line, " ud=");
        line_fixed(line, sample.d_voltage_v, 2);
        line_text(line, " b=");
        line_fixed(line, sample.bus_voltage_v, 1);
        line_text(line, " h=");
        line_unsigned(line, sample.hall_raw);
        line_text(line, " ev=");
        line_unsigned(line, sample.flags);
        line_finish(line);
        return;
    }
    trace_dump.active = false;
    loop_trace::lock(false);
    Line line;
    line_text(line, "# trace end");
    line_finish(line);
}

void send_help() {
    static const char* const help[] = {
        "# T<float>  target wheel speed, -1..1 of the full-scale speed",
        "# Q<float>  open-loop voltage, V",
        "# E<0|1>    driver enable",
        "# X         zero both set-points",
        "# M<rate>   telemetry rate in Hz, 0 = off",
        "# S         status line",
        "# V<0|1>    loop diagnostics (D line) with every status line",
        "# L<float>  velocity feedback low-pass, s (config id 12, persisted)",
        "# R<float>  wheel speed set-point ramp, m/s^2 (config id 13, persisted)",
        "# C<id>:<v> set a runtime config value (ids: see config_defs.lua)",
        "# G<id>     read a runtime config value back (16 = full-scale m/s)",
        "# K         stop/limit counters, one per mechanism (`# K1..K5 ...`)",
        "# W         dump the loop trace now (`# T ...` lines)",
        "# W<0|1>    arm the automatic trace dump (0/1)",
        "# A         calibrate motor R/L, re-tune current loop",
        "# ?         this list",
    };
    for (const char* entry : help) {
        Line line;
        line_text(line, entry);
        line_finish(line);
    }
}

const char* trim(const char* text) {
    while (*text == ' ' || *text == '\t') {
        ++text;
    }
    return text;
}

bool parse_float(const char* text, float& value) {
    char* end = nullptr;
    value = std::strtof(text, &end);
    if (end == text) {
        return false;
    }
    if (*trim(end) != '\0') {
        return false;
    }
    return std::isfinite(value);
}

bool parse_unsigned(const char* text, unsigned long& value) {
    char* end = nullptr;
    value = std::strtoul(text, &end, 10);
    if (end == text) {
        return false;
    }
    return *trim(end) == '\0';
}

void handle_command(const char* command, MotorBackend& motor) {
    Line line;
    const char* argument = trim(command + 1);

    switch (command[0]) {
        case 'T': {
            // Normalized -1..1 of the module's full-scale speed (config id 16):
            // the module owns the m/s mapping, so a bench command cannot ask
            // for a speed the drive has no voltage to regulate. Out-of-range
            // values are clamped, not rejected (a slider at its end stops).
            float value = 0.0f;
            if (!parse_float(argument, value)) {
                send_error(command);
                return;
            }
            if (value > 1.0f) value = 1.0f;
            else if (value < -1.0f) value = -1.0f;
            motor.set_speed_target_normalized(value);
            line_text(line, "T=");
            line_fixed(line, value, 3);
            break;
        }
        case 'Q': {
            float value = 0.0f;
            if (!parse_float(argument, value)) {
                send_error(command);
                return;
            }
            set_points.voltage_v = value;
            motor.set_voltage_point(value);
            line_text(line, "Q=");
            line_fixed(line, value, 3);
            break;
        }
        case 'E': {
            unsigned long value = 0;
            if (!parse_unsigned(argument, value) || value > 1) {
                send_error(command);
                return;
            }
            motor.set_state(value == 1);
            line_text(line, "E=");
            line_unsigned(line, motor.is_on() ? 1 : 0);
            break;
        }
        case 'X':
            if (*argument != '\0') {
                send_error(command);
                return;
            }
            set_points.voltage_v = 0.0f;
            motor.set_speed_target_normalized(0.0f);
            motor.set_voltage_point(0.0f);
            line_text(line, "OK");
            break;
        case 'M': {
            unsigned long value = 0;
            if (!parse_unsigned(argument, value)) {
                send_error(command);
                return;
            }
            if (value > TELEMETRY_MAX_HZ) {
                value = TELEMETRY_MAX_HZ;
            }
            set_points.telemetry_hz = static_cast<unsigned>(value);
            set_points.telemetry_time = millis_32();
            line_text(line, "M=");
            line_unsigned(line, value);
            break;
        }
        case 'D': {
            unsigned long value = 0;
            if (!parse_unsigned(argument, value) || value > 1) {
                send_error(command);
                return;
            }
            set_points.verbose = value == 1;
            line_text(line, "D=");
            line_unsigned(line, value);
            break;
        }
        case 'V': {
            unsigned long value = 0;
            if (!parse_unsigned(argument, value) || value > 1) {
                send_error(command);
                return;
            }
            set_points.diagnostics = value == 1;
            line_text(line, "V=");
            line_unsigned(line, value);
            break;
        }
        case 'C': {
            const char* colon = std::strchr(argument, ':');
            if (colon == nullptr) {
                send_error(command);
                return;
            }
            char id_text[8] = {};
            const size_t id_length = static_cast<size_t>(colon - argument);
            if (id_length == 0 || id_length >= sizeof(id_text)) {
                send_error(command);
                return;
            }
            std::memcpy(id_text, argument, id_length);
            unsigned long id = 0;
            float value = 0.0f;
            if (!parse_unsigned(id_text, id) || !parse_float(colon + 1, value)) {
                send_error(command);
                return;
            }
            if (!app_apply_config(static_cast<int32_t>(id), value)) {
                // An unknown id or a value the module rejected (wrong range,
                // backend without the knob): say so instead of echoing it.
                send_error(command);
                return;
            }
            line_text(line, "C");
            line_unsigned(line, id);
            line_text(line, "=");
            line_fixed(line, value, 3);
            break;
        }
        case 'G': {
            unsigned long id = 0;
            if (!parse_unsigned(argument, id)) {
                send_error(command);
                return;
            }
            float value = 0.0f;
            if (!app_get_config(static_cast<int32_t>(id), value)) {
                send_error(command);
                return;
            }
            line_text(line, "C");
            line_unsigned(line, id);
            line_text(line, "=");
            line_fixed(line, value, 4);
            break;
        }
        case 'K':
            if (*argument != '\0') {
                send_error(command);
                return;
            }
            send_counters();
            return;
        case 'W': {
            if (*argument == '\0') {
                // Dump the last DEPTH samples: what the loop saw before, and
                // at, the moment this was asked for.
                trace_begin_dump();
                return;
            }
            unsigned long value = 0;
            if (!parse_unsigned(argument, value) || value > 1) {
                send_error(command);
                return;
            }
            loop_trace::arm(value == 1);
            line_text(line, "W=");
            line_unsigned(line, value);
            break;
        }
        case 'L': {
            float value = 0.0f;
            if (!parse_float(argument, value)) {
                send_error(command);
                return;
            }
            // Config id 12 (persisted, readable with `G12`): the serial alias
            // exists because the bench scripts predate the config ids.
            if (!app_apply_config(CFG_VEL_LPF, value)) {
                send_error(command);
                return;
            }
            line_text(line, "L=");
            line_fixed(line, value, 4);
            break;
        }
        case 'R': {
            float value = 0.0f;
            if (!parse_float(argument, value)) {
                send_error(command);
                return;
            }
            // Config id 13 (persisted, readable with `G13`), same alias story
            // as `L`.
            if (!app_apply_config(CFG_SPEED_RAMP, value)) {
                send_error(command);
                return;
            }
            line_text(line, "R=");
            line_fixed(line, value, 3);
            break;
        }
        case 'S':
            if (*argument != '\0') {
                send_error(command);
                return;
            }
            send_status(motor);
            return;
        case 'A':
            if (*argument != '\0') {
                send_error(command);
                return;
            }
            line_text(line, motor.auto_calibrate() ? "A=1" : "A=0");
            break;
        case '?':
            send_help();
            return;
        default:
            send_error(command);
            return;
    }
    line_finish(line);
}

void drain_receive(MotorBackend& motor) {
    // Snapshot-and-clear before draining: an overflow that happens while we are
    // draining is then reported by the next poll instead of being lost.
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const bool overflowed = rx_overflow;
    rx_overflow = false;
    __set_PRIMASK(primask);

    if (overflowed) {
        // Bytes were lost, so nothing buffered can be trusted: drop it all and
        // swallow input up to the next newline instead of running a command the
        // sender never wrote. The host is expected to retry.
        rx_tail = rx_head;
        line_length = 0;
        line_overlong = true;
    }

    while (rx_tail != rx_head) {
        const char symbol = static_cast<char>(rx_ring[rx_tail]);
        rx_tail = (rx_tail + 1) % RX_RING_SIZE;

        if (symbol == '\r') {
            continue;
        }
        if (symbol != '\n') {
            if (line_overlong) {
                continue;
            }
            if (symbol != '\t' && (symbol < 0x20 || symbol > 0x7e)) {
                // Noisy or electrically glitched line (the receiver sees this
                // whenever a host opens the port): never let it reach the parser.
                line_noise = true;
                continue;
            }
            if (line_length + 1 < LINE_SIZE) {
                line_buffer[line_length++] = symbol;
            }
            else {
                // Overlong line: discard it up to the next newline so a stuck
                // sender cannot desynchronise the commands that follow.
                line_overlong = true;
                line_length = 0;
            }
            continue;
        }

        line_overlong = false;
        line_buffer[line_length] = '\0';
        line_length = 0;
        if (line_noise) {
            line_noise = false;
            continue;
        }
        const char* command = trim(line_buffer);
        if (*command == '\0') {
            continue;
        }
        if (set_points.verbose) {
            send_echo(command);
        }
        handle_command(command, motor);
    }
}

}

void serial_line_init() {
    rx_head = 0;
    rx_tail = 0;
    tx_head = 0;
    tx_tail = 0;
    mirror_head = 0;
    mirror_tail = 0;
    line_length = 0;
    line_overlong = false;
    line_noise = false;
    rx_overflow = false;
    // RXNE and the error interrupts are handled directly in serial_line_rx_irq:
    // HAL's one-shot receive cannot be re-armed from its own completion
    // callback (HAL still owns RxState there), which would silently drop bytes.
    huart2.Instance->CR1 |= USART_CR1_RXNEIE;
    huart2.Instance->CR3 |= USART_CR3_EIE;
}

void serial_line_poll(MotorBackend& motor) {
    drain_receive(motor);
    tx_kick();
    mirror_kick();

    if (!set_points.booted) {
        set_points.booted = true;
        send_boot_trace(motor);
    }

    // A trigger latched by the loop (a Hall event, an angle snap, a guard trip,
    // a driver fault) turns into a dump here; the telemetry waits until the
    // dump is through so the link carries the history in one piece.
    if (loop_trace::take_trigger()) {
        trace_begin_dump();
    }
    if (trace_dump.active) {
        trace_service();
        return;
    }

    if (set_points.telemetry_hz == 0) {
        return;
    }
    const millis_t now = millis_32();
    millis_t period_ms = 1000u / set_points.telemetry_hz;
    if (set_points.diagnostics) {
        // The `D` line carries the set-point and the velocity as well, so with
        // diagnostics on it replaces the `S` line: one 96-character line per
        // tick is what 115200 baud can carry at the shorter period below.
        if (period_ms < DIAGNOSTICS_MIN_PERIOD_MS) {
            period_ms = DIAGNOSTICS_MIN_PERIOD_MS;
        }
    }
    if ((now - set_points.telemetry_time) >= period_ms) {
        set_points.telemetry_time = now;
        if (set_points.diagnostics) {
            send_diagnostics(motor);
        }
        else {
            send_status(motor);
        }
    }
}

void serial_line_debug(const char* label) {
    Line line;
    line_text(line, "# ");
    line_text(line, label);
    line_finish(line);
}

void serial_line_debug(const char* label, float value) {
    Line line;
    line_text(line, "# ");
    line_text(line, label);
    line_text(line, " ");
    line_fixed(line, value, 4);
    line_finish(line);
}

extern "C" void serial_line_rx_irq(void) {
    const uint32_t status = huart2.Instance->ISR;

    if ((status & (USART_ISR_PE | USART_ISR_FE | USART_ISR_NE | USART_ISR_ORE)) != 0U) {
        huart2.Instance->ICR = USART_ICR_PECF | USART_ICR_FECF | USART_ICR_NECF | USART_ICR_ORECF;
        // A framing/noise/overrun hit means the bytes of the line in flight are
        // not trustworthy: mark the stream so the parser discards that line.
        rx_overflow = true;
    }

    if ((status & USART_ISR_RXNE) != 0U) {
        const uint8_t byte = static_cast<uint8_t>(huart2.Instance->RDR & 0xFFu);
        const size_t next = (rx_head + 1) % RX_RING_SIZE;
        if (next != rx_tail) {
            rx_ring[rx_head] = byte;
            rx_head = next;
        }
        else {
            rx_overflow = true;
        }
    }
}
