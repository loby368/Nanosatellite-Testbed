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
#include "motorcontrol.h"

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
#define M1_CURR_AMPL_W_Pin GPIO_PIN_0
#define M1_CURR_AMPL_W_GPIO_Port GPIOC
#define M1_CURR_AMPL_V_Pin GPIO_PIN_1
#define M1_CURR_AMPL_V_GPIO_Port GPIOC
#define M1_TEMPERATURE_Pin GPIO_PIN_2
#define M1_TEMPERATURE_GPIO_Port GPIOC
#define M1_CURR_AMPL_U_Pin GPIO_PIN_0
#define M1_CURR_AMPL_U_GPIO_Port GPIOA
#define M1_BUS_VOLTAGE_Pin GPIO_PIN_1
#define M1_BUS_VOLTAGE_GPIO_Port GPIOA
#define M1_OCP_Pin GPIO_PIN_6
#define M1_OCP_GPIO_Port GPIOA
#define LED2_Pin GPIO_PIN_13
#define LED2_GPIO_Port GPIOB
#define M1_PWM_UH_Pin GPIO_PIN_8
#define M1_PWM_UH_GPIO_Port GPIOA
#define M1_PWM_VH_Pin GPIO_PIN_9
#define M1_PWM_VH_GPIO_Port GPIOA
#define M1_PWM_WH_Pin GPIO_PIN_10
#define M1_PWM_WH_GPIO_Port GPIOA
#define M1_PWM_EN_U_Pin GPIO_PIN_10
#define M1_PWM_EN_U_GPIO_Port GPIOC
#define M1_PWM_EN_V_Pin GPIO_PIN_11
#define M1_PWM_EN_V_GPIO_Port GPIOC
#define M1_PWM_EN_W_Pin GPIO_PIN_12
#define M1_PWM_EN_W_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
