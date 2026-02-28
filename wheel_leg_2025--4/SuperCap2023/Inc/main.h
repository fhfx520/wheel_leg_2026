/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
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
#define ccram_enable (1)
#if ccram_enable==1
#define CCMRAM __attribute__((section("ccmram")))
#endif
#if ccram_enable==0
#define CCMRAM /**/
#endif
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define charge_pump_Pin GPIO_PIN_13
#define charge_pump_GPIO_Port GPIOC
#define V_chassis_Pin GPIO_PIN_0
#define V_chassis_GPIO_Port GPIOA
#define V_cap_Pin GPIO_PIN_1
#define V_cap_GPIO_Port GPIOA
#define V_bat_Pin GPIO_PIN_2
#define V_bat_GPIO_Port GPIOA
#define T1_Pin GPIO_PIN_4
#define T1_GPIO_Port GPIOA
#define T2_Pin GPIO_PIN_5
#define T2_GPIO_Port GPIOA
#define T3_Pin GPIO_PIN_6
#define T3_GPIO_Port GPIOA
#define stander_3_3v_Pin GPIO_PIN_7
#define stander_3_3v_GPIO_Port GPIOA
#define I_chassis_Pin GPIO_PIN_1
#define I_chassis_GPIO_Port GPIOB
#define I_bat_Pin GPIO_PIN_14
#define I_bat_GPIO_Port GPIOB
#define DEBUG_TX_Pin GPIO_PIN_6
#define DEBUG_TX_GPIO_Port GPIOB
#define DEBUG_RX_Pin GPIO_PIN_7
#define DEBUG_RX_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
