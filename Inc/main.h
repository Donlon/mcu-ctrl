/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#define BTN1_Pin               GPIO_PIN_13
#define BTN1_GPIO_Port         GPIOC

#define LED3_Pin               GPIO_PIN_0
#define LED3_GPIO_Port         GPIOC
#define LED2_Pin               GPIO_PIN_1
#define LED2_GPIO_Port         GPIOC
#define LED1_Pin               GPIO_PIN_2
#define LED1_GPIO_Port         GPIOC
#define LED0_Pin               GPIO_PIN_3
#define LED0_GPIO_Port         GPIOC

#define DOT_LAT_Pin            GPIO_PIN_4
#define DOT_LAT_GPIO_Port      GPIOA
#define DOT_SHIFT_Pin          GPIO_PIN_6
#define DOT_SHIFT_GPIO_Port    GPIOA
#define DOT_EN_Pin             GPIO_PIN_4
#define DOT_EN_GPIO_Port       GPIOC

#define MT2_B_Pin              GPIO_PIN_5
#define MT2_B_GPIO_Port        GPIOC
#define MT2_A_Pin              GPIO_PIN_0
#define MT2_A_GPIO_Port        GPIOB
#define MT1_B_Pin              GPIO_PIN_1
#define MT1_B_GPIO_Port        GPIOB
#define MT1_A_Pin              GPIO_PIN_2
#define MT1_A_GPIO_Port        GPIOB

#define BTN0_Pin               GPIO_PIN_10
#define BTN0_GPIO_Port         GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
