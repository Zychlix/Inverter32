/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f3xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define FAULT_SR_3V3_Pin GPIO_PIN_5
#define FAULT_SR_3V3_GPIO_Port GPIOE
#define FAULT_RST_Pin GPIO_PIN_6
#define FAULT_RST_GPIO_Port GPIOE
#define CURR_A__Pin GPIO_PIN_0
#define CURR_A__GPIO_Port GPIOC
#define CURR_A_C1_Pin GPIO_PIN_1
#define CURR_A_C1_GPIO_Port GPIOC
#define CURR_B__Pin GPIO_PIN_2
#define CURR_B__GPIO_Port GPIOC
#define CURR_B_C3_Pin GPIO_PIN_3
#define CURR_B_C3_GPIO_Port GPIOC
#define RD_Pin GPIO_PIN_0
#define RD_GPIO_Port GPIOA
#define SAMPLE_Pin GPIO_PIN_1
#define SAMPLE_GPIO_Port GPIOA
#define RDVEL_Pin GPIO_PIN_2
#define RDVEL_GPIO_Port GPIOA
#define CS_RES_Pin GPIO_PIN_4
#define CS_RES_GPIO_Port GPIOA
#define T3_Pin GPIO_PIN_7
#define T3_GPIO_Port GPIOA
#define RESET_RES_Pin GPIO_PIN_4
#define RESET_RES_GPIO_Port GPIOC
#define T2_Pin GPIO_PIN_5
#define T2_GPIO_Port GPIOC
#define T1_Pin GPIO_PIN_2
#define T1_GPIO_Port GPIOB
#define TH1_Pin GPIO_PIN_8
#define TH1_GPIO_Port GPIOE
#define TH2_Pin GPIO_PIN_14
#define TH2_GPIO_Port GPIOE
#define LOT_IN_Pin GPIO_PIN_12
#define LOT_IN_GPIO_Port GPIOB
#define DOS_IN_Pin GPIO_PIN_13
#define DOS_IN_GPIO_Port GPIOB
#define ADC_IN_B_Pin GPIO_PIN_14
#define ADC_IN_B_GPIO_Port GPIOB
#define ADC_IN_A_Pin GPIO_PIN_15
#define ADC_IN_A_GPIO_Port GPIOB
#define VBUS_Pin GPIO_PIN_8
#define VBUS_GPIO_Port GPIOD
#define ADC_12V_Pin GPIO_PIN_9
#define ADC_12V_GPIO_Port GPIOD
#define GPIO_D_Pin GPIO_PIN_9
#define GPIO_D_GPIO_Port GPIOC
#define GPIO_C_Pin GPIO_PIN_8
#define GPIO_C_GPIO_Port GPIOA
#define GPIO_B_Pin GPIO_PIN_9
#define GPIO_B_GPIO_Port GPIOA
#define GPIO_A_Pin GPIO_PIN_10
#define GPIO_A_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_6
#define LED_GPIO_Port GPIOF
#define X_OUT_Pin GPIO_PIN_15
#define X_OUT_GPIO_Port GPIOA
#define PRECHARGE_OUT_Pin GPIO_PIN_10
#define PRECHARGE_OUT_GPIO_Port GPIOC
#define FAN_OUT_Pin GPIO_PIN_11
#define FAN_OUT_GPIO_Port GPIOC
#define PUMP_OUT_Pin GPIO_PIN_12
#define PUMP_OUT_GPIO_Port GPIOC
#define MAIN_OUT_Pin GPIO_PIN_0
#define MAIN_OUT_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
