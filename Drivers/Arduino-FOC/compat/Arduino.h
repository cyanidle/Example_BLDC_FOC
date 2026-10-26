#pragma once

#include <stddef.h>
#include <stdint.h>

#include <cmath>

#include "main.h"

#define HIGH 1
#define LOW 0

#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2

#ifndef PI
#define PI 3.1415926535897932384626433832795
#endif
#define F(x) (x)

typedef bool boolean;
typedef uint8_t byte;

class __FlashStringHelper;
class StringSumHelper;

template <class T>
static inline T min(T a, T b) {
    return a < b ? a : b;
}

template <class T>
static inline T max(T a, T b) {
    return a > b ? a : b;
}

class Print {
public:
    virtual ~Print() = default;
    virtual size_t print(const char* text) = 0;
    virtual size_t print(char value) = 0;
    virtual size_t print(int value) = 0;
    virtual size_t print(float value, int decimals) = 0;
    virtual size_t println(const char* text) = 0;
    virtual size_t println(char value) = 0;
    virtual size_t println(int value) = 0;
    virtual size_t println(float value, int decimals) = 0;
    virtual size_t println() = 0;
};

// The stock Arduino time API, backed by the firmware's TIM7 microsecond base.
// libvoltbro names its time types millis_t/micros_t, so the Arduino names are free.
unsigned long millis();
unsigned long micros();

void arduino_pin_register(uint8_t id, GPIO_TypeDef* port, uint16_t mask);
void arduino_pwm_pin_register(uint8_t id, uint8_t timer_channel);
uint8_t arduino_pwm_pin_channel(uint8_t id);

void pinMode(uint8_t id, uint8_t mode);
void digitalWrite(uint8_t id, uint8_t value);
int digitalRead(uint8_t id);

void delay(unsigned long ms);
void delayMicroseconds(unsigned int us);
