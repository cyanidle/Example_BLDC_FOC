#include "app.h"

#include "tim.h"

static uint32_t millis_k __attribute__ ((__aligned__(4))) = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) {
        HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
    }
    else if (htim->Instance == TIM7) {
        millis_k += 1;
    }
    else if (htim->Instance == TIM1) {
        // TODO
    }
}

micros micros_64() {
    return (micros)(millis_k * 1000u + __HAL_TIM_GetCounter(&htim7));
}

millis millis_32() {
    return millis_k;
}

void start_timers() {
    HAL_TIM_Base_Start_IT(&htim2);
    HAL_TIM_Base_Start_IT(&htim7);
}
