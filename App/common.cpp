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
micros_t micros_64() {
    uint32_t ms, us;
    CRITICAL_SECTION(
        ms = millis_k;
        us = __HAL_TIM_GetCounter(&htim7);
    )
    return (micros_t)ms * 1000u + us;
}
// KNOWN DEFECT of the software millisecond counter: millis_k is incremented by
// the TIM7 update interrupt, which runs some time after the counter wraps, so a
// read that straddles the wrap while that interrupt is delayed returns a time up
// to ~1 ms in the PAST — the clock steps backwards. Measured while timing the
// loop phases: -996 us between two reads taken microseconds apart. Every dt in
// the drive comes from here, so the symptom is a pass with dt = 0 (a skipped
// ramp/slew step) roughly once per millisecond boundary that a long critical
// section happens to straddle. Folding the pending update flag in over-corrects
// by +1 ms (the flag is visible both before and after the callback), which is
// worse, so this is left as is until the timebase is moved to hardware —
// DWT->CYCCNT at the core clock (wraps every 25 s at 170 MHz, extended in
// software) has no such window at all. Do not paper over it per caller.

millis_t millis_32() {
    return millis_k;
}

void start_timers() {
    HAL_TIM_Base_Start_IT(&htim2);
    HAL_TIM_Base_Start_IT(&htim7);
}
