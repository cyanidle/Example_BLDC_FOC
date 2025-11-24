/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void app();
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BAT_DIV_Pin GPIO_PIN_0
#define BAT_DIV_GPIO_Port GPIOC
#define I_A_Pin GPIO_PIN_1
#define I_A_GPIO_Port GPIOC
#define I_B_Pin GPIO_PIN_2
#define I_B_GPIO_Port GPIOC
#define I_C_Pin GPIO_PIN_3
#define I_C_GPIO_Port GPIOC
#define DIP_SW8_Pin GPIO_PIN_0
#define DIP_SW8_GPIO_Port GPIOA
#define DIP_SW7_Pin GPIO_PIN_1
#define DIP_SW7_GPIO_Port GPIOA
#define LED2_Pin GPIO_PIN_5
#define LED2_GPIO_Port GPIOA
#define DIP_SW1_Pin GPIO_PIN_2
#define DIP_SW1_GPIO_Port GPIOB
#define DIP_SW2_Pin GPIO_PIN_10
#define DIP_SW2_GPIO_Port GPIOB
#define DIP_SW3_Pin GPIO_PIN_11
#define DIP_SW3_GPIO_Port GPIOB
#define DIP_SW4_Pin GPIO_PIN_12
#define DIP_SW4_GPIO_Port GPIOB
#define INLA_Pin GPIO_PIN_13
#define INLA_GPIO_Port GPIOB
#define INLB_Pin GPIO_PIN_14
#define INLB_GPIO_Port GPIOB
#define INLC_Pin GPIO_PIN_15
#define INLC_GPIO_Port GPIOB
#define ENC_2_Pin GPIO_PIN_7
#define ENC_2_GPIO_Port GPIOC
#define ENC_2_EXTI_IRQn EXTI9_5_IRQn
#define ENC_3_Pin GPIO_PIN_8
#define ENC_3_GPIO_Port GPIOC
#define ENC_3_EXTI_IRQn EXTI9_5_IRQn
#define INH_A_Pin GPIO_PIN_8
#define INH_A_GPIO_Port GPIOA
#define INH_B_Pin GPIO_PIN_9
#define INH_B_GPIO_Port GPIOA
#define INH_C_Pin GPIO_PIN_10
#define INH_C_GPIO_Port GPIOA
#define DIP_SW6_Pin GPIO_PIN_11
#define DIP_SW6_GPIO_Port GPIOA
#define DIP_SW5_Pin GPIO_PIN_12
#define DIP_SW5_GPIO_Port GPIOA
#define SPI3_CS0_Pin GPIO_PIN_15
#define SPI3_CS0_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_2
#define LED1_GPIO_Port GPIOD
#define DRV_ENABLE_Pin GPIO_PIN_3
#define DRV_ENABLE_GPIO_Port GPIOB
#define DRV_FAULT_Pin GPIO_PIN_5
#define DRV_FAULT_GPIO_Port GPIOB
#define ENC_1_Pin GPIO_PIN_6
#define ENC_1_GPIO_Port GPIOB
#define ENC_1_EXTI_IRQn EXTI9_5_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */