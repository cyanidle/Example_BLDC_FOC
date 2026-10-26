#include "Arduino.h"

#include <voltbro/utils.hpp>

millis_t millis_32();
micros_t micros_64();

namespace {

constexpr uint8_t PIN_COUNT = 16;

enum class PinKind : uint8_t { UNREGISTERED, GPIO, PWM };

struct PinEntry {
    PinKind kind = PinKind::UNREGISTERED;
    GPIO_TypeDef* port = nullptr;
    uint16_t mask = 0;
    uint8_t timer_channel = 0;
};

PinEntry pins[PIN_COUNT];

bool is_free(uint8_t id) {
    return id < PIN_COUNT && pins[id].kind == PinKind::UNREGISTERED;
}

const PinEntry& lookup(uint8_t id) {
    if (id >= PIN_COUNT || pins[id].kind == PinKind::UNREGISTERED) {
        Error_Handler();
    }
    return pins[id];
}

}

void arduino_pin_register(uint8_t id, GPIO_TypeDef* port, uint16_t mask) {
    if (!is_free(id) || port == nullptr || mask == 0) {
        Error_Handler();
    }
    pins[id] = PinEntry {PinKind::GPIO, port, mask, 0};
}

void arduino_pwm_pin_register(uint8_t id, uint8_t timer_channel) {
    if (!is_free(id) || timer_channel == 0 || timer_channel > 3) {
        Error_Handler();
    }
    pins[id] = PinEntry {PinKind::PWM, nullptr, 0, timer_channel};
}

uint8_t arduino_pwm_pin_channel(uint8_t id) {
    if (id >= PIN_COUNT) {
        return 0;
    }
    return pins[id].kind == PinKind::PWM ? pins[id].timer_channel : 0;
}

void pinMode(uint8_t id, uint8_t mode) {
    const PinEntry& entry = lookup(id);
    if (entry.kind == PinKind::PWM) {
        // Timer pins are owned by the PWM backend (already in alternate function
        // mode); the library only ever asks for OUTPUT before configuring them.
        if (mode != OUTPUT) {
            Error_Handler();
        }
        return;
    }

    GPIO_InitTypeDef init = {};
    init.Pin = entry.mask;
    init.Speed = GPIO_SPEED_FREQ_LOW;
    switch (mode) {
        case OUTPUT:
            init.Mode = GPIO_MODE_OUTPUT_PP;
            init.Pull = GPIO_NOPULL;
            break;
        case INPUT:
            init.Mode = GPIO_MODE_INPUT;
            init.Pull = GPIO_NOPULL;
            break;
        case INPUT_PULLUP:
            init.Mode = GPIO_MODE_INPUT;
            init.Pull = GPIO_PULLUP;
            break;
        default:
            Error_Handler();
    }
    HAL_GPIO_Init(entry.port, &init);
}

void digitalWrite(uint8_t id, uint8_t value) {
    const PinEntry& entry = lookup(id);
    if (entry.kind != PinKind::GPIO) {
        Error_Handler();
    }
    HAL_GPIO_WritePin(entry.port, entry.mask, value ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

int digitalRead(uint8_t id) {
    const PinEntry& entry = lookup(id);
    if (entry.kind != PinKind::GPIO) {
        Error_Handler();
    }
    return HAL_GPIO_ReadPin(entry.port, entry.mask) == GPIO_PIN_SET ? HIGH : LOW;
}

void delay(unsigned long ms) {
    HAL_Delay(static_cast<uint32_t>(ms));
}

void delayMicroseconds(unsigned int us) {
    const micros_t deadline = micros_64() + us;
    while (micros_64() < deadline) {}
}

unsigned long millis() {
    return static_cast<unsigned long>(millis_32());
}

unsigned long micros() {
    return static_cast<unsigned long>(micros_64());
}
