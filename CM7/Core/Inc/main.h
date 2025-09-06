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
#include "stm32h7xx_hal.h"

#include "stm32h7xx_nucleo.h"
#include <stdio.h>

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
#define Mot1_PWM_Pin GPIO_PIN_9
#define Mot1_PWM_GPIO_Port GPIOE
#define Mot2_PWM_Pin GPIO_PIN_11
#define Mot2_PWM_GPIO_Port GPIOE
#define Mot3_PWM_Pin GPIO_PIN_13
#define Mot3_PWM_GPIO_Port GPIOE
#define Mot4_PWM_Pin GPIO_PIN_14
#define Mot4_PWM_GPIO_Port GPIOE
#define USB_OTG_FS_PWR_EN_Pin GPIO_PIN_10
#define USB_OTG_FS_PWR_EN_GPIO_Port GPIOD
#define USB_OTG_FS_OVCR_Pin GPIO_PIN_7
#define USB_OTG_FS_OVCR_GPIO_Port GPIOG
#define Mot4_Encoder_A_Pin GPIO_PIN_6
#define Mot4_Encoder_A_GPIO_Port GPIOC
#define Mot4_Encoder_B_Pin GPIO_PIN_7
#define Mot4_Encoder_B_GPIO_Port GPIOC
#define Mot1_Encoder_A_Pin GPIO_PIN_15
#define Mot1_Encoder_A_GPIO_Port GPIOA
#define Mot1_Dir_A_Pin GPIO_PIN_0
#define Mot1_Dir_A_GPIO_Port GPIOD
#define Mot1_Dir_B_Pin GPIO_PIN_1
#define Mot1_Dir_B_GPIO_Port GPIOD
#define Mot2_Dir_A_Pin GPIO_PIN_2
#define Mot2_Dir_A_GPIO_Port GPIOD
#define Mot2_Dir_B_Pin GPIO_PIN_3
#define Mot2_Dir_B_GPIO_Port GPIOD
#define Mot3_Dir_A_Pin GPIO_PIN_4
#define Mot3_Dir_A_GPIO_Port GPIOD
#define Mot3_Dir_B_Pin GPIO_PIN_5
#define Mot3_Dir_B_GPIO_Port GPIOD
#define Mot4_Dir_A_Pin GPIO_PIN_6
#define Mot4_Dir_A_GPIO_Port GPIOD
#define Mot4_Dir_B_Pin GPIO_PIN_7
#define Mot4_Dir_B_GPIO_Port GPIOD
#define Mot1_Encoder_B_Pin GPIO_PIN_3
#define Mot1_Encoder_B_GPIO_Port GPIOB
#define Mot2_Encoder_A_Pin GPIO_PIN_4
#define Mot2_Encoder_A_GPIO_Port GPIOB
#define Mot2_Encoder_B_Pin GPIO_PIN_5
#define Mot2_Encoder_B_GPIO_Port GPIOB
#define Mot3_Encoder_A_Pin GPIO_PIN_6
#define Mot3_Encoder_A_GPIO_Port GPIOB
#define Mot3_Encoder_B_Pin GPIO_PIN_7
#define Mot3_Encoder_B_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
