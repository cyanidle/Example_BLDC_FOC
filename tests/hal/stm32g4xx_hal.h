#pragma once

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <string>

// Host-only register/time model. Tests compile the production controller and
// Hall decoder; only their hardware boundary is substituted.
struct GPIO_TypeDef { uint16_t IDR = 0; uint16_t ODR = 0; };
enum GPIO_PinState { GPIO_PIN_RESET, GPIO_PIN_SET };
enum HAL_StatusTypeDef { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT };
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

// ---- UART ------------------------------------------------------------------
// Register-level model for App/serial_line.cpp: TX chunks are captured as they
// are handed to HAL_UART_Transmit_IT, RX bytes are injected through the mock
// USART registers and the firmware's own interrupt handler.
struct UART_TypeDef {
    uint32_t ISR = 0;
    uint32_t RDR = 0;
    uint32_t CR1 = 0;
    uint32_t CR3 = 0;
    uint32_t ICR = 0;
};

constexpr uint32_t USART_ISR_PE = 1u << 0;
constexpr uint32_t USART_ISR_FE = 1u << 1;
constexpr uint32_t USART_ISR_NE = 1u << 2;
constexpr uint32_t USART_ISR_ORE = 1u << 3;
constexpr uint32_t USART_ISR_RXNE = 1u << 5;
constexpr uint32_t USART_ICR_PECF = 1u << 0;
constexpr uint32_t USART_ICR_FECF = 1u << 1;
constexpr uint32_t USART_ICR_NECF = 1u << 2;
constexpr uint32_t USART_ICR_ORECF = 1u << 3;
constexpr uint32_t USART_CR1_RXNEIE = 1u << 5;
constexpr uint32_t USART_CR3_EIE = 1u << 0;

constexpr uint32_t HAL_UART_STATE_READY = 0;

struct UART_HandleTypeDef {
    UART_TypeDef* Instance = nullptr;
    uint32_t gState = HAL_UART_STATE_READY;
    uint32_t RxState = HAL_UART_STATE_READY;
};

inline std::string fake_uart_tx;

extern "C" void serial_line_rx_irq(void);

inline HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef* huart, const uint8_t* data, uint16_t size) {
    if (huart->gState != HAL_UART_STATE_READY) {
        return HAL_BUSY;
    }
    fake_uart_tx.append(reinterpret_cast<const char*>(data), size);
    return HAL_OK;
}

inline void fake_uart_feed(UART_HandleTypeDef* huart, const char* text) {
    for (size_t i = 0; text[i] != '\0'; ++i) {
        huart->Instance->RDR = static_cast<uint8_t>(text[i]);
        huart->Instance->ISR |= USART_ISR_RXNE;
        serial_line_rx_irq();
        huart->Instance->ISR &= ~USART_ISR_RXNE;
    }
}

// Drives the error path of the receive interrupt (framing/noise/overrun).
inline void fake_uart_error(UART_HandleTypeDef* huart, uint32_t flags) {
    huart->Instance->ISR |= flags;
    serial_line_rx_irq();
    huart->Instance->ISR = 0;
}

inline void Error_Handler(void) { std::abort(); }
