/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32l4xx_hal.h"

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
#define motASX2_Pin GPIO_PIN_0
#define motASX2_GPIO_Port GPIOA
#define motASX1_Pin GPIO_PIN_1
#define motASX1_GPIO_Port GPIOA
#define motPDX1_Pin GPIO_PIN_4
#define motPDX1_GPIO_Port GPIOA
#define motPDX2_Pin GPIO_PIN_5
#define motPDX2_GPIO_Port GPIOA
#define motADX1_Pin GPIO_PIN_6
#define motADX1_GPIO_Port GPIOA
#define TRIG_Pin GPIO_PIN_7
#define TRIG_GPIO_Port GPIOA
#define servo_Pin GPIO_PIN_1
#define servo_GPIO_Port GPIOB
#define motADX2_Pin GPIO_PIN_2
#define motADX2_GPIO_Port GPIOB
#define motPSX1_Pin GPIO_PIN_14
#define motPSX1_GPIO_Port GPIOD
#define ECHO_Pin GPIO_PIN_8
#define ECHO_GPIO_Port GPIOB
#define motPSX2_Pin GPIO_PIN_9
#define motPSX2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define SERVO_posSinistra 165
#define SERVO_posDestra 30
#define SERVO_posDefault 105
#define MOT_VEL_MAX 20000
#define MOT_VEL_MIN 10000
#define MOT_VEL_ZERO 0
#define MOT_DIREZIONE_DX 1
#define MOT_DIREZIONE_SX 0
#define SERVO_MIN_TICK   25U   // 1.0 ms
#define SERVO_MAX_TICK  120U   // 2.0 ms
#define RIT_SPIN 200
#define RIT_SPIN_PATHFINDING 600
#define TEMP_STOP 3000
#define REVERSE_TIME 500
#define startup_delay 5000
#define startup_spin 2000
#define N_RICERCHE 50



/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
