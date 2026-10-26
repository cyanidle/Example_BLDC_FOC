#include "Arduino.h"

#include "drivers/hardware_api.h"
#include "main.h"
#include "tim.h"

namespace {

struct VbDriverParams {
    uint8_t channels[3] = {0, 0, 0};
};

VbDriverParams driver_params;

uint32_t channel_to_hal(uint8_t channel) {
    switch (channel) {
        case 1: return TIM_CHANNEL_1;
        case 2: return TIM_CHANNEL_2;
        case 3: return TIM_CHANNEL_3;
    }
    Error_Handler();
    return TIM_CHANNEL_1;
}

void write_compare(uint8_t channel, uint32_t value) {
    switch (channel) {
        case 1: htim1.Instance->CCR1 = value; break;
        case 2: htim1.Instance->CCR2 = value; break;
        case 3: htim1.Instance->CCR3 = value; break;
        default: Error_Handler();
    }
}

uint32_t timer_clock_hz() {
    const uint32_t pclk2 = HAL_RCC_GetPCLK2Freq();
    const uint32_t ppre2 = (RCC->CFGR & RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos;
    return (ppre2 & 0x4U) ? (pclk2 * 2U) : pclk2;
}

bool apply_frequency(long pwm_frequency) {
    const uint32_t ticks = timer_clock_hz() / static_cast<uint32_t>(pwm_frequency);
    if (ticks < 2U) {
        return false;
    }
    const uint32_t prescaler = ticks / 65536U + 1U;
    const uint32_t period = ticks / prescaler;
    if (period < 2U) {
        return false;
    }
    __HAL_TIM_SET_PRESCALER(&htim1, prescaler - 1U);
    __HAL_TIM_SET_AUTORELOAD(&htim1, period - 1U);
    __HAL_TIM_SET_COUNTER(&htim1, 0U);
    htim1.Instance->EGR = TIM_EGR_UG;
    __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
    return true;
}

}

void* _configure3PWM(long pwm_frequency, const int pinA, const int pinB, const int pinC) {
    const int pin_ids[3] = {pinA, pinB, pinC};
    for (uint8_t i = 0; i < 3; ++i) {
        if (pin_ids[i] < 0 || pin_ids[i] > 0xFF) {
            return SIMPLEFOC_DRIVER_INIT_FAILED;
        }
        const uint8_t channel = arduino_pwm_pin_channel(static_cast<uint8_t>(pin_ids[i]));
        if (channel == 0) {
            return SIMPLEFOC_DRIVER_INIT_FAILED;
        }
        driver_params.channels[i] = channel;
    }

    // A negative frequency is the library's NOT_SET: keep the CubeMX timer setup.
    if (pwm_frequency > 0 && !apply_frequency(pwm_frequency)) {
        return SIMPLEFOC_DRIVER_INIT_FAILED;
    }

    for (uint8_t i = 0; i < 3; ++i) {
        if (HAL_TIM_PWM_Start(&htim1, channel_to_hal(driver_params.channels[i])) != HAL_OK) {
            return SIMPLEFOC_DRIVER_INIT_FAILED;
        }
    }
    return &driver_params;
}

void _writeDutyCycle3PWM(float dc_a, float dc_b, float dc_c, void* params) {
    const VbDriverParams* driver = static_cast<VbDriverParams*>(params);
    if (driver == nullptr) {
        Error_Handler();
    }
    const float full_scale = static_cast<float>(htim1.Instance->ARR + 1U);
    const float duty[3] = {dc_a, dc_b, dc_c};
    for (uint8_t i = 0; i < 3; ++i) {
        const float clamped = (duty[i] < 0.0f) ? 0.0f : ((duty[i] > 1.0f) ? 1.0f : duty[i]);
        write_compare(driver->channels[i], static_cast<uint32_t>(lrintf(clamped * full_scale)));
    }
}
