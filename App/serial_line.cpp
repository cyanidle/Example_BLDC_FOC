#include "serial_line.h"

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>

#include <voltbro/utils.hpp>

#include "motor_backend.h"
#include "usart.h"

millis_t millis_32();
// app.cpp; declared here so this unit stays free of the Cyphal headers.
void app_apply_config(int32_t id, float value);

millis_t millis_32();

namespace {

constexpr size_t RX_RING_SIZE = 128;
constexpr size_t TX_RING_SIZE = 1024;
constexpr size_t TX_CHUNK_SIZE = 64;
constexpr size_t LINE_SIZE = 64;
constexpr unsigned TELEMETRY_MAX_HZ = 100;
constexpr unsigned REPLY_CAPACITY = 96;

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
    float target_m_s = 0.0f;
    float voltage_v = 0.0f;
    unsigned telemetry_hz = 0;
    millis_t telemetry_time = 0;
    bool verbose = true;
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

void tx_write(const char* text, size_t length);

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

    static const long scale_by_decimals[] = {1, 10, 100, 1000};
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
    line_text(line, "\r\n");
    tx_write(line.text, line.length);
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

const char* fault_name(MotorFault fault) {
    switch (fault) {
        case MotorFault::NONE: return "NONE";
        case MotorFault::INVALID_HALL: return "INVALID_HALL";
        case MotorFault::INVALID_SUPPLY: return "INVALID_SUPPLY";
        case MotorFault::INVALID_COMMAND: return "INVALID_COMMAND";
        case MotorFault::STALLED: return "STALLED";
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
    line_text(line, " target_m_s=");
    line_fixed(line, set_points.target_m_s, 3);
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
    line_fixed(line, set_points.target_m_s, 3);
    line_text(line, " v=");
    line_fixed(line, motor.get_velocity(), 3);
    line_text(line, " e=");
    line_unsigned(line, motor.is_on() ? 1 : 0);
    line_text(line, " f=");
    line_text(line, fault_name(motor.get_fault()));
    line_finish(line);
}

void send_error(const char* command) {
    Line line;
    line_text(line, "ERR ");
    line_text(line, command);
    line_finish(line);
}

void send_help() {
    static const char* const help[] = {
        "# T<float>  target wheel speed, m/s",
        "# Q<float>  open-loop voltage, V",
        "# E<0|1>    driver enable",
        "# X         zero both set-points",
        "# M<rate>   telemetry rate in Hz, 0 = off",
        "# S         status line",
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
            float value = 0.0f;
            if (!parse_float(argument, value)) {
                send_error(command);
                return;
            }
            set_points.target_m_s = value;
            motor.set_speed_target(value);
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
            set_points.target_m_s = 0.0f;
            set_points.voltage_v = 0.0f;
            motor.set_speed_target(0.0f);
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
            app_apply_config(static_cast<int32_t>(id), value);
            line_text(line, "C");
            line_unsigned(line, id);
            line_text(line, "=");
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

    if (!set_points.booted) {
        set_points.booted = true;
        send_boot_trace(motor);
    }

    if (set_points.telemetry_hz == 0) {
        return;
    }
    const millis_t now = millis_32();
    const millis_t period_ms = 1000u / set_points.telemetry_hz;
    if ((now - set_points.telemetry_time) >= period_ms) {
        set_points.telemetry_time = now;
        send_status(motor);
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
    line_fixed(line, value, 3);
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
