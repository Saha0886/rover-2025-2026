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
#include "stm32f1xx_hal.h"

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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define M2_FI_Pin GPIO_PIN_6
#define M2_FI_GPIO_Port GPIOA
#define M2_BI_Pin GPIO_PIN_7
#define M2_BI_GPIO_Port GPIOA
#define M1_BI_Pin GPIO_PIN_0
#define M1_BI_GPIO_Port GPIOB
#define M1_FI_Pin GPIO_PIN_1
#define M1_FI_GPIO_Port GPIOB
#define ADDR_X1_Pin GPIO_PIN_12
#define ADDR_X1_GPIO_Port GPIOB
#define ADDR_X0_Pin GPIO_PIN_13
#define ADDR_X0_GPIO_Port GPIOB
#define M1_ENC_B_Pin GPIO_PIN_8
#define M1_ENC_B_GPIO_Port GPIOA
#define M1_ENC_A_Pin GPIO_PIN_9
#define M1_ENC_A_GPIO_Port GPIOA
#define M2_ENC_B_Pin GPIO_PIN_6
#define M2_ENC_B_GPIO_Port GPIOB
#define M2_ENC_A_Pin GPIO_PIN_7
#define M2_ENC_A_GPIO_Port GPIOB

#define M1_CMD_PWM 0x0A
#define M2_CMD_PWM 0x0B

#define M1_CMD_ENC 0x1A
#define M2_CMD_ENC 0x1B

#define PWM_STD 65535
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
