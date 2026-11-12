#include "app.h"

#include "tim.h"

#include "cycle_counter.h"

static cycle_counter::Extension dwt_cycles;
static bool timebase_started = false;

static void start_timebase() {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    dwt_cycles = {};
    timebase_started = true;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) {
        HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
    }
    else if (htim->Instance == TIM1) {
        // TODO
    }
}

// DWT->CYCCNT is independent of interrupt latency. Extend its 32-bit wrap in
// the same critical section as the read so every caller sees monotonic time.
micros_t micros_64() {
    uint64_t cycles;
    CRITICAL_SECTION(
        if (!timebase_started) {
            start_timebase();
        }
        cycles = cycle_counter::extend(dwt_cycles, DWT->CYCCNT);
    )
    return cycles / (SystemCoreClock / 1000000U);
}

millis_t millis_32() {
    return (millis_t)(micros_64() / 1000U);
}

void start_timers() {
    HAL_TIM_Base_Start_IT(&htim2);
    CRITICAL_SECTION(
        if (!timebase_started) {
            start_timebase();
        }
    )
}
