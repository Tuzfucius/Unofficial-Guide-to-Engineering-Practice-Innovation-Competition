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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
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
#define SERVO_DIR_PIN_Pin GPIO_PIN_15
#define SERVO_DIR_PIN_GPIO_Port GPIOB
#define MOTOR_DIR_Pin GPIO_PIN_8
#define MOTOR_DIR_GPIO_Port GPIOC
#define MOTOR_EN_Pin GPIO_PIN_9
#define MOTOR_EN_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */
void set_openmv_light(uint8_t brightness);
void HAL_Delay_us(uint32_t nus);
void toggle_openmv_mode(void);
void Servo_SendPositionCommand(uint8_t id, uint16_t position);
void Servo_SendPositionAccelSpeedCommand(UART_HandleTypeDef *huart_handle, uint8_t id, uint16_t acceleration, int position, uint16_t speed);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
