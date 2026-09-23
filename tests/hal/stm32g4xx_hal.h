#pragma once

#include <cmath>
#include <cstdint>

// Host-only register/time model. Tests compile the production controller and
// Hall decoder; only their hardware boundary is substituted.
struct GPIO_TypeDef { uint16_t IDR = 0; uint16_t ODR = 0; };
enum GPIO_PinState { GPIO_PIN_RESET, GPIO_PIN_SET };
enum HAL_StatusTypeDef { HAL_OK };
struct TIM_TypeDef { uint32_t ARR = 1999; uint32_t CCR1 = 0, CCR2 = 0, CCR3 = 0; };
struct TIM_HandleTypeDef { TIM_TypeDef* Instance; };
struct ADC_HandleTypeDef {};

inline uint32_t fake_tick = 0;
inline uint32_t fake_primask = 0;
inline uint32_t __get_PRIMASK() { return fake_primask; }
inline void __disable_irq() { fake_primask = 1; }
inline void __set_PRIMASK(uint32_t value) { fake_primask = value; }
inline uint32_t HAL_GetTick() { return fake_tick; }
inline void HAL_Delay(uint32_t ms) { fake_tick += ms; }

inline GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef* port, uint16_t pin) {
    return (port->IDR & pin) ? GPIO_PIN_SET : GPIO_PIN_RESET;
}
inline void HAL_GPIO_WritePin(GPIO_TypeDef* port, uint16_t pin, GPIO_PinState state) {
    if (state == GPIO_PIN_SET) port->ODR |= pin;
    else port->ODR &= ~pin;
}
inline void HAL_GPIO_TogglePin(GPIO_TypeDef* port, uint16_t pin) { port->ODR ^= pin; }

constexpr uint32_t TIM_CHANNEL_1 = 0, TIM_CHANNEL_2 = 1, TIM_CHANNEL_3 = 2;
constexpr uint32_t ADC_SINGLE_ENDED = 0;
inline HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef*, uint32_t) { return HAL_OK; }
inline HAL_StatusTypeDef HAL_ADCEx_Calibration_Start(ADC_HandleTypeDef*, uint32_t) { return HAL_OK; }
inline HAL_StatusTypeDef HAL_ADC_Start_DMA(ADC_HandleTypeDef*, uint32_t*, uint32_t) { return HAL_OK; }
inline void arm_abs_f32(const float* in, float* out, uint32_t count) {
    for (uint32_t i = 0; i < count; ++i) out[i] = std::fabs(in[i]);
}
