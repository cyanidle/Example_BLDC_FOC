#include "app.h"

#include "tim.h"

static uint32_t millis_k __attribute__ ((__aligned__(4))) = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) {
        HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
    }
    else if (htim->Instance == TIM7) {
        millis_k += 1;
    }
    else if (htim->Instance == TIM1) {
        // TODO
    }
}

// TIM7 runs at 1 MHz with period 999: counter is microseconds within the
// current millisecond, millis_k counts whole milliseconds. Compute in 64 bits:
// the old `millis_k * 1000u + counter` wrapped in 32-bit arithmetic after
// ~71.6 minutes, freezing every micros_64()-based scheduler (e.g. the PID).
micros micros_64() {
    uint32_t ms, us;
    CRITICAL_SECTION(
        ms = millis_k;
        us = __HAL_TIM_GetCounter(&htim7);
    )
    return (micros)ms * 1000u + us;
}

millis millis_32() {
    return millis_k;
}

void start_timers() {
    HAL_TIM_Base_Start_IT(&htim2);
    HAL_TIM_Base_Start_IT(&htim7);
}
