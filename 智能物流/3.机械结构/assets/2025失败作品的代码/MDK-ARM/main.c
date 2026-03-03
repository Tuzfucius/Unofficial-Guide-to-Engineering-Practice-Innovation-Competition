/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "usart.h"
#include "Motor.h"
#include "encoder.h"
#include "Motor_PID.h"
#include "angle_pid.h"
#include "fire_debugger.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include "hwt101.h"
#include <string.h>
#include "servo.h"
#include "user_uart.h"
#include "fashion_star_uart_servo.h"
#include "ring_buffer.h"
#include "qr.h"
#include "openmv_uart.h"
#include "All.h"
#include "motor_test.h"

/* USER CODE END Includes */

/* Private function prototypes -----------------------------------------------*/
void Car_Competition_Sequence(void);
void Task2(void);

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
// main.c

/* USER CODE BEGIN PV */
static const unsigned char tongse[] = {
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf8, 0x3f, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf8, 0x07, 0xff, 0xff, 0xff, 0xff,
    0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xff, 0xff, 0xf8, 0x03, 0xff, 0xff, 0xff, 0xff,
    0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xff, 0xff, 0xf0, 0x07, 0xff, 0xff, 0xff, 0xff,
    0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xff, 0xff, 0xe0, 0x0f, 0xff, 0xff, 0xff, 0xff,
    0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xff, 0xff, 0xe0, 0x1f, 0xff, 0xff, 0xff, 0xff,
    0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xff, 0xff, 0xc0, 0x3f, 0xff, 0xff, 0xff, 0xff,
    0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xff, 0xff, 0x80, 0x00, 0x00, 0x00, 0xff, 0xff,
    0xf8, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xf0, 0x1f, 0xff, 0xff, 0x80, 0x00, 0x00, 0x00, 0xff, 0xff,
    0xf8, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xf0, 0x1f, 0xff, 0xff, 0x00, 0x00, 0x00, 0x01, 0xff, 0xff,
    0xf8, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xf0, 0x1f, 0xff, 0xfe, 0x00, 0x00, 0x00, 0x03, 0xff, 0xff,
    0xf8, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xf0, 0x1f, 0xff, 0xfc, 0x00, 0x00, 0x00, 0x07, 0xff, 0xff,
    0xf8, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xf0, 0x1f, 0xff, 0xf8, 0x03, 0xff, 0xf8, 0x07, 0xff, 0xff,
    0xf8, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xf0, 0x1f, 0xff, 0xf0, 0x07, 0xff, 0xf0, 0x0f, 0xff, 0xff,
    0xf8, 0x0e, 0x00, 0x00, 0x00, 0x00, 0x70, 0x1f, 0xff, 0xe0, 0x0f, 0xff, 0xf0, 0x1f, 0xff, 0xff,
    0xf8, 0x0e, 0x00, 0x00, 0x00, 0x00, 0x70, 0x1f, 0xff, 0xc0, 0x1f, 0xff, 0xe0, 0x1f, 0xff, 0xff,
    0xf8, 0x0e, 0x00, 0x00, 0x00, 0x00, 0x70, 0x1f, 0xff, 0x00, 0x3f, 0xff, 0xc0, 0x3f, 0xff, 0xff,
    0xf8, 0x0e, 0x00, 0x00, 0x00, 0x00, 0x70, 0x1f, 0xfe, 0x00, 0x7f, 0xff, 0xc0, 0x7f, 0xff, 0xff,
    0xf8, 0x0e, 0x00, 0x00, 0x00, 0x00, 0x70, 0x1f, 0xf8, 0x00, 0xff, 0xff, 0x80, 0x7f, 0xff, 0xff,
    0xf8, 0x0e, 0x1f, 0xff, 0xff, 0xf8, 0x70, 0x1f, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xff,
    0xf8, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xf0, 0x1f, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xff,
    0xf8, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xf0, 0x1f, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xff,
    0xf8, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xf0, 0x1f, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xff,
    0xf8, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xf0, 0x1f, 0xfc, 0x30, 0x00, 0x00, 0x00, 0x00, 0x07, 0xff,
    0xf8, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xf0, 0x1f, 0xfe, 0x70, 0x3f, 0xf8, 0x0f, 0xfe, 0x07, 0xff,
    0xf8, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xf0, 0x1f, 0xff, 0xf0, 0x3f, 0xf8, 0x0f, 0xfe, 0x07, 0xff,
    0xf8, 0x0f, 0xe0, 0x00, 0x00, 0x07, 0xf0, 0x1f, 0xff, 0xf0, 0x3f, 0xf8, 0x0f, 0xfe, 0x07, 0xff,
    0xf8, 0x0f, 0xf0, 0x00, 0x00, 0x0f, 0xf0, 0x1f, 0xff, 0xf0, 0x3f, 0xf8, 0x0f, 0xfe, 0x07, 0xff,
    0xf8, 0x0f, 0xf0, 0x00, 0x00, 0x0f, 0xf0, 0x1f, 0xff, 0xf0, 0x3f, 0xf8, 0x0f, 0xfe, 0x07, 0xff,
    0xf8, 0x0f, 0xf0, 0x00, 0x00, 0x0f, 0xf0, 0x1f, 0xff, 0xf0, 0x3f, 0xf8, 0x0f, 0xfe, 0x07, 0xff,
    0xf8, 0x0f, 0xf0, 0x00, 0x00, 0x0f, 0xf0, 0x1f, 0xff, 0xf0, 0x3f, 0xf8, 0x0f, 0xfe, 0x07, 0xff,
    0xf8, 0x0f, 0xf0, 0x3f, 0xfc, 0x0f, 0xf0, 0x1f, 0xff, 0xf0, 0x3f, 0xf8, 0x0f, 0xfe, 0x07, 0xff,
    0xf8, 0x0f, 0xf0, 0x3f, 0xfc, 0x0f, 0xf0, 0x1f, 0xff, 0xf0, 0x3f, 0xf8, 0x0f, 0xfe, 0x07, 0xff,
    0xf8, 0x0f, 0xf0, 0x3f, 0xfc, 0x0f, 0xf0, 0x1f, 0xff, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x07, 0xff,
    0xf8, 0x0f, 0xf0, 0x3f, 0xfc, 0x0f, 0xf0, 0x1f, 0xff, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x07, 0xff,
    0xf8, 0x0f, 0xf0, 0x3f, 0xfc, 0x0f, 0xf0, 0x1f, 0xff, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x07, 0xff,
    0xf8, 0x0f, 0xf0, 0x3f, 0xfc, 0x0f, 0xf0, 0x1f, 0xff, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x07, 0xff,
    0xf8, 0x0f, 0xf0, 0x3f, 0xfc, 0x0f, 0xf0, 0x1f, 0xff, 0xf0, 0x3f, 0xff, 0xff, 0xfe, 0x03, 0xff,
    0xf8, 0x0f, 0xf0, 0x3f, 0xfc, 0x0f, 0xf0, 0x1f, 0xff, 0xf0, 0x3f, 0xff, 0xff, 0xfe, 0x03, 0xff,
    0xf8, 0x0f, 0xf0, 0x3f, 0xfc, 0x0f, 0xf0, 0x1f, 0xff, 0xf0, 0x3f, 0xff, 0xff, 0xfe, 0x03, 0xff,
    0xf8, 0x0f, 0xf0, 0x00, 0x00, 0x0f, 0xf0, 0x1f, 0xff, 0xf0, 0x3f, 0xff, 0xff, 0xfe, 0x03, 0xff,
    0xf8, 0x0f, 0xf0, 0x00, 0x00, 0x0f, 0xf0, 0x1f, 0xff, 0xf0, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xf8, 0x0f, 0xf0, 0x00, 0x00, 0x0f, 0xf0, 0x1f, 0xff, 0xf0, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xf8, 0x0f, 0xf0, 0x00, 0x00, 0x0f, 0xf0, 0x1f, 0xff, 0xf0, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xf8, 0x0f, 0xf0, 0x00, 0x00, 0x0f, 0xf0, 0x1f, 0xff, 0xf0, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xf8, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xf0, 0x1f, 0xff, 0xf0, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xf8, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xf0, 0x1f, 0xff, 0xf0, 0x3f, 0xff, 0xff, 0xff, 0xf8, 0xff,
    0xf8, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xf0, 0x1f, 0xff, 0xf0, 0x3f, 0xff, 0xff, 0xff, 0xf8, 0x3f,
    0xf8, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xf0, 0x1f, 0xff, 0xf0, 0x3f, 0xff, 0xff, 0xff, 0xf8, 0x0f,
    0xf8, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xf0, 0x1f, 0xff, 0xf0, 0x1f, 0xff, 0xff, 0xff, 0xf8, 0x0f,
    0xf8, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xf0, 0x1f, 0xff, 0xf0, 0x0f, 0xff, 0xff, 0xff, 0xf0, 0x1f,
    0xf8, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xe0, 0x1f, 0xff, 0xf0, 0x03, 0xff, 0xff, 0xff, 0xc0, 0x1f,
    0xf8, 0x0f, 0xff, 0xff, 0xff, 0xf0, 0x00, 0x1f, 0xff, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f,
    0xf8, 0x0f, 0xff, 0xff, 0xff, 0xf8, 0x00, 0x3f, 0xff, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f,
    0xf8, 0x0f, 0xff, 0xff, 0xff, 0xf8, 0x00, 0x3f, 0xff, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7f,
    0xf8, 0x0f, 0xff, 0xff, 0xff, 0xfc, 0x00, 0x3f, 0xff, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x01, 0xff,
    0xf8, 0x0f, 0xff, 0xff, 0xff, 0xfc, 0x00, 0x7f, 0xff, 0xff, 0x80, 0x00, 0x00, 0x00, 0x0f, 0xff,
    0xf8, 0x0f, 0xff, 0xff, 0xff, 0xfc, 0x01, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

static const unsigned char yise[] = {
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf8, 0x3f, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xff, 0xff, 0xff, 0xf8, 0x07, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xff, 0xff, 0xff, 0xf8, 0x03, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xff, 0xff, 0xff, 0xf0, 0x07, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xff, 0xff, 0xff, 0xe0, 0x0f, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xff, 0xff, 0xff, 0xe0, 0x1f, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xe0, 0x7f, 0xff, 0xff, 0xf8, 0x0f, 0xff, 0xff, 0xff, 0xc0, 0x3f, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xe0, 0x7f, 0xff, 0xff, 0xf8, 0x0f, 0xff, 0xff, 0xff, 0x80, 0x00, 0x00, 0x00, 0xff, 0xff,
    0xff, 0xe0, 0x7f, 0xff, 0xff, 0xf8, 0x0f, 0xff, 0xff, 0xff, 0x80, 0x00, 0x00, 0x00, 0xff, 0xff,
    0xff, 0xe0, 0x7f, 0xff, 0xff, 0xf8, 0x0f, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x01, 0xff, 0xff,
    0xff, 0xe0, 0x7f, 0xff, 0xff, 0xf8, 0x0f, 0xff, 0xff, 0xfe, 0x00, 0x00, 0x00, 0x03, 0xff, 0xff,
    0xff, 0xe0, 0x7f, 0xff, 0xff, 0xf8, 0x0f, 0xff, 0xff, 0xfc, 0x00, 0x00, 0x00, 0x07, 0xff, 0xff,
    0xff, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xff, 0xff, 0xf8, 0x03, 0xff, 0xf8, 0x07, 0xff, 0xff,
    0xff, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xff, 0xff, 0xf0, 0x07, 0xff, 0xf0, 0x0f, 0xff, 0xff,
    0xff, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xff, 0xff, 0xe0, 0x0f, 0xff, 0xf0, 0x1f, 0xff, 0xff,
    0xff, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xff, 0xff, 0xc0, 0x1f, 0xff, 0xe0, 0x1f, 0xff, 0xff,
    0xff, 0xe0, 0x7f, 0xfc, 0x00, 0x00, 0x0f, 0xff, 0xff, 0x00, 0x3f, 0xff, 0xc0, 0x3f, 0xff, 0xff,
    0xff, 0xe0, 0x7f, 0xff, 0xff, 0xff, 0xf3, 0xff, 0xfe, 0x00, 0x7f, 0xff, 0xc0, 0x7f, 0xff, 0xff,
    0xff, 0xe0, 0x7f, 0xff, 0xff, 0xff, 0xf0, 0xff, 0xf8, 0x00, 0xff, 0xff, 0x80, 0x7f, 0xff, 0xff,
    0xff, 0xe0, 0x7f, 0xff, 0xff, 0xff, 0xf0, 0x3f, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xff,
    0xff, 0xe0, 0x7f, 0xff, 0xff, 0xff, 0xf0, 0x1f, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xff,
    0xff, 0xe0, 0x7f, 0xff, 0xff, 0xff, 0xf0, 0x1f, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xff,
    0xff, 0xe0, 0x3f, 0xff, 0xff, 0xff, 0xe0, 0x3f, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xff,
    0xff, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xfc, 0x30, 0x00, 0x00, 0x00, 0x00, 0x07, 0xff,
    0xff, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0xfe, 0x70, 0x3f, 0xf8, 0x0f, 0xfe, 0x07, 0xff,
    0xff, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0xff, 0xf0, 0x3f, 0xf8, 0x0f, 0xfe, 0x07, 0xff,
    0xff, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xf0, 0x3f, 0xf8, 0x0f, 0xfe, 0x07, 0xff,
    0xff, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x01, 0xff, 0xff, 0xf0, 0x3f, 0xf8, 0x0f, 0xfe, 0x07, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf0, 0x3f, 0xf8, 0x0f, 0xfe, 0x07, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf0, 0x3f, 0xf8, 0x0f, 0xfe, 0x07, 0xff,
    0xff, 0xff, 0xe0, 0xff, 0xff, 0x01, 0xff, 0xff, 0xff, 0xf0, 0x3f, 0xf8, 0x0f, 0xfe, 0x07, 0xff,
    0xff, 0xff, 0xf0, 0x1f, 0xff, 0x01, 0xff, 0xff, 0xff, 0xf0, 0x3f, 0xf8, 0x0f, 0xfe, 0x07, 0xff,
    0xff, 0xff, 0xf0, 0x1f, 0xff, 0x01, 0xff, 0xff, 0xff, 0xf0, 0x3f, 0xf8, 0x0f, 0xfe, 0x07, 0xff,
    0xff, 0xff, 0xf0, 0x1f, 0xff, 0x01, 0xff, 0xff, 0xff, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x07, 0xff,
    0xff, 0xff, 0xf0, 0x3f, 0xff, 0x01, 0xff, 0xff, 0xff, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x07, 0xff,
    0xff, 0xff, 0xf0, 0x3f, 0xff, 0x01, 0xff, 0xff, 0xff, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x07, 0xff,
    0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xff, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x07, 0xff,
    0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xff, 0xf0, 0x3f, 0xff, 0xff, 0xfe, 0x03, 0xff,
    0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xff, 0xf0, 0x3f, 0xff, 0xff, 0xfe, 0x03, 0xff,
    0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xff, 0xf0, 0x3f, 0xff, 0xff, 0xfe, 0x03, 0xff,
    0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xff, 0xf0, 0x3f, 0xff, 0xff, 0xfe, 0x03, 0xff,
    0xff, 0xff, 0xe0, 0x7f, 0xff, 0x01, 0xff, 0xff, 0xff, 0xf0, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xe0, 0x7f, 0xff, 0x01, 0xff, 0xff, 0xff, 0xf0, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xc0, 0x7f, 0xff, 0x01, 0xff, 0xff, 0xff, 0xf0, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xc0, 0x7f, 0xff, 0x01, 0xff, 0xff, 0xff, 0xf0, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0x80, 0xff, 0xff, 0x01, 0xff, 0xff, 0xff, 0xf0, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0x00, 0xff, 0xff, 0x01, 0xff, 0xff, 0xff, 0xf0, 0x3f, 0xff, 0xff, 0xff, 0xf8, 0xff,
    0xff, 0xff, 0x01, 0xff, 0xff, 0x01, 0xff, 0xff, 0xff, 0xf0, 0x3f, 0xff, 0xff, 0xff, 0xf8, 0x3f,
    0xff, 0xfc, 0x01, 0xff, 0xff, 0x01, 0xff, 0xff, 0xff, 0xf0, 0x3f, 0xff, 0xff, 0xff, 0xf8, 0x0f,
    0xff, 0xf8, 0x03, 0xff, 0xff, 0x01, 0xff, 0xff, 0xff, 0xf0, 0x1f, 0xff, 0xff, 0xff, 0xf8, 0x0f,
    0xff, 0xe0, 0x07, 0xff, 0xff, 0x01, 0xff, 0xff, 0xff, 0xf0, 0x0f, 0xff, 0xff, 0xff, 0xf0, 0x1f,
    0xff, 0x00, 0x0f, 0xff, 0xff, 0x01, 0xff, 0xff, 0xff, 0xf0, 0x03, 0xff, 0xff, 0xff, 0xc0, 0x1f,
    0xf0, 0x00, 0x1f, 0xff, 0xff, 0x01, 0xff, 0xff, 0xff, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f,
    0xf0, 0x00, 0x3f, 0xff, 0xff, 0x01, 0xff, 0xff, 0xff, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f,
    0xf8, 0x00, 0x7f, 0xff, 0xff, 0x01, 0xff, 0xff, 0xff, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7f,
    0xfc, 0x01, 0xff, 0xff, 0xff, 0x01, 0xff, 0xff, 0xff, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x01, 0xff,
    0xfe, 0x07, 0xff, 0xff, 0xff, 0x01, 0xff, 0xff, 0xff, 0xff, 0x80, 0x00, 0x00, 0x00, 0x0f, 0xff,
    0xfe, 0x1f, 0xff, 0xff, 0xff, 0x01, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_UART4_Init(void);
static void MX_UART5_Init(void);
static void MX_USART6_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t usart1_ReadBuffer[8];
uint8_t rx_byte;
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern Usart_DataTypeDef FSUS_Usart;
void User_Uart_Init(UART_HandleTypeDef *huartx);

// 张大头用
#define RX_BUFFER_SIZE 64          // 定义接收缓冲区的最大长度
uint8_t rx_buffer[RX_BUFFER_SIZE]; // DMA接收缓冲区

// --- 陀螺仪接收变量 --- hwt
#define GYRO_RX_BUFFER_SIZE 33 // 建议为11的倍数，可以缓存3帧数据
uint8_t gyro_rx_buffer[GYRO_RX_BUFFER_SIZE];
volatile uint8_t gyro_rx_len = 0;
volatile bool gyro_rx_flag   = false;

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_USART1_UART_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_TIM5_Init();
    MX_TIM9_Init();
    MX_TIM10_Init();
    MX_TIM11_Init();
    MX_TIM6_Init();
    MX_USART2_UART_Init();
    MX_UART4_Init();
    MX_UART5_Init();
    MX_USART6_UART_Init();
    /* USER CODE BEGIN 2 */
    Motor_Init();
    Encoders_All_Init();
    // ========== 优化的阻塞式初始化序列 ==========
    extern const SSD1306_Font_t Font_7x10;

    // 步骤1: 优先初始化OLED显示系统
    __HAL_RCC_GPIOD_CLK_ENABLE();
    HAL_Delay(200);

    ssd1306_Init();
    HAL_Delay(300);

    // 显示启动信息
    ssd1306_Fill(0);
    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString("STM32F407", Font_7x10, 1);
    ssd1306_SetCursor(0, 15);
    ssd1306_WriteString("Starting...", Font_7x10, 1);
    ssd1306_UpdateScreen();
    HAL_Delay(800);

    // 步骤2: 初始化调试系统
    ssd1306_Fill(0);
    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString("STM32F407", Font_7x10, 1);
    ssd1306_SetCursor(0, 15);
    ssd1306_WriteString("Init Debug...", Font_7x10, 1);
    ssd1306_UpdateScreen();

    Debugger_Init(&huart1);
    extern uint8_t usart1_rx_byte;
    HAL_UART_Receive_IT(&huart1, &usart1_rx_byte, 1);
    HAL_Delay(500);

    // 步骤3: 初始化控制系统
    ssd1306_Fill(0);
    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString("STM32F407", Font_7x10, 1);
    ssd1306_SetCursor(0, 15);
    ssd1306_WriteString("Init Control", Font_7x10, 1);
    ssd1306_UpdateScreen();

    Motor_PID_Init_All();
    Yaw_PID_Init();
    HAL_TIM_Base_Start_IT(&htim6);
    HAL_Delay(500);

    // 步骤4: 谨慎初始化传感器系统
    ssd1306_Fill(0);
    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString("STM32F407", Font_7x10, 1);
    ssd1306_SetCursor(0, 15);
    ssd1306_WriteString("Init Sensors", Font_7x10, 1);
    ssd1306_UpdateScreen();

    // 等待系统完全稳定
    HAL_Delay(1000);

    // 初始化HWT101（短时间阻塞）
    HWT101_Init(&huart4);
    HAL_Delay(200);

    // 步骤5: 初始化其他模块
    ssd1306_Fill(0);
    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString("STM32F407", Font_7x10, 1);
    ssd1306_SetCursor(0, 15);
    ssd1306_WriteString("Final Setup", Font_7x10, 1);
    ssd1306_UpdateScreen();

    qr_init();
    User_Uart_Init(&huart5);

    // 初始化OpenMV RPC通信
    omv_rpc_init();
    HAL_Delay(300);

    HAL_Delay(500);

    // 系统就绪显示
    ssd1306_Fill(0);
    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString("STM32F407", Font_7x10, 1);
    ssd1306_SetCursor(0, 15);
    ssd1306_WriteString("System Ready", Font_7x10, 1);
    ssd1306_SetCursor(0, 30);
    ssd1306_WriteString("All OK!", Font_7x10, 1);
    ssd1306_UpdateScreen();
    HAL_Delay(1000);

    // PID初始化已确认没问题
    Yaw_PID_Init(); // 初始化转向环PID控制器
    printf("Angle PID initialized!\r\n");

    // FSUS_SetServoAngle(&FSUS_Usart, 3, 20.0f, 2000, 0);

    // MPU6050_GetData(&hi2c2,&MPU6050_Data);

    // HAL_Delay(2000);
    // Car_Reset_And_Sync_State();
    // Car_Back_Pulses_PID(2000);
    // HAL_Delay(4000);
    // Car_Stop_PID();

    // Servo_UpDown(&FSUS_Usart, 600, 1500, 500);

    // Servo_catch(&FSUS_Usart, 1000);
    // Usart_DataTypeDef *usart = &FSUS_Usart;
    // uint16_t time_ms = 1000;

    // float current_angle = 0.0f;
    // float target_angle = 0.0f;

    // // 步骤 1: id6-800，下降到获取外部物料位置
    // FSUS_QueryServoAngleMTurn(usart, 6, &current_angle);
    // _angle = current_angle - ARM_DESCEND_FOR_CATCH;
    // Servo_UpDown(usart, target_angle, time_ms * 2, 0);
    // HAL_Delay(time_ms * 2 + 500);

    // Servo_TurntableRotate(&FSUS_Usart, 0, 1500, 500,500);

    // FSUS_SetServoAngleMTurn(&FSUS_Usart, 6, -50, 1000, 0);

    // float current_angle_id2 = 0.0f;
    // FSUS_QueryServoAngleMTurn(&FSUS_Usart, 2, &current_angle_id2);
    // ssd1306_Fill(0);
    // ssd1306_SetCursor(0, 0);
    // char buffer[20];
    // sprintf(buffer, "%.2f", current_angle_id2);
    // ssd1306_WriteString(buffer, Font_7x10, 1);

    // ssd1306_UpdateScreen();

    // Servo_catch(&FSUS_Usart, 1000);

    // test(&FSUS_Usart, 1000);

    // ssd1306_Fill(0);
    // ssd1306_UpdateScreen();
    // HAL_Delay(500);
    // // 3. 调用绘图函数，将图像数据加载到缓冲区
    // ssd1306_DrawBitmap(tongse, 0, 0, 128, 64);
    // // 4. 刷新屏幕，将缓冲区的内容显示出来 (关键一步!)
    // ssd1306_UpdateScreen();

    // Car_Go_Pulses_PID(2650);
    // HAL_Delay(1000);
    // Car_Stop_PID();

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */

    // servo_1(&FSUS_Usart, 3, 2500);
    // Car_Debug_Kp_Batch_Test();
    // Car_Debug_Yaw_PID_Test();
    // Car_Debug_Simple_Turn_Test();
    // Car_Debug_Yaw_PID_Test();
    //  Car_Debug_Left_Strafe_Test();

    // OpenMV连接测试
    omv_test_uart_basic();
    HAL_Delay(1000);
    omv_test_connection();
    HAL_Delay(2000);

    Car_Competition_Sequence();
    Task2();
    // uint32_t last_oled_refresh = 0;
		Motor_Speed_Test_Start_All();
    while (1) {
				Motor_Speed_Test_Update_Display();
				HAL_Delay(100);
    }

    // MPU6050_GetData(&hi2c2,&MPU6050_Data);
    // printf("MPU6050: ax=%.2f, ay=%.2f, az=%.2f, gx=%.2f, gy=%.2f, gz=%.2f\r\n", MPU6050_Data.Ax, MPU6050_Data.Ay, MPU6050_Data.Az, MPU6050_Data.Gx, MPU6050_Data.Gy, MPU6050_Data.Gz);
    // HAL_Delay(1000);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
     */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM       = 8;
    RCC_OscInitStruct.PLL.PLLN       = 336;
    RCC_OscInitStruct.PLL.PLLP       = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ       = 4;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
        Error_Handler();
    }
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void)
{

    /* USER CODE BEGIN TIM1_Init 0 */

    /* USER CODE END TIM1_Init 0 */

    TIM_MasterConfigTypeDef sMasterConfig               = {0};
    TIM_OC_InitTypeDef sConfigOC                        = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

    /* USER CODE BEGIN TIM1_Init 1 */

    /* USER CODE END TIM1_Init 1 */
    htim1.Instance               = TIM1;
    htim1.Init.Prescaler         = 83;
    htim1.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim1.Init.Period            = 999;
    htim1.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
    sConfigOC.OCMode       = TIM_OCMODE_PWM1;
    sConfigOC.Pulse        = 0;
    sConfigOC.OCPolarity   = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode   = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState  = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) {
        Error_Handler();
    }
    sBreakDeadTimeConfig.OffStateRunMode  = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel        = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime         = 0;
    sBreakDeadTimeConfig.BreakState       = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity    = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.AutomaticOutput  = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM1_Init 2 */

    /* USER CODE END TIM1_Init 2 */
    HAL_TIM_MspPostInit(&htim1);
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void)
{

    /* USER CODE BEGIN TIM2_Init 0 */

    /* USER CODE END TIM2_Init 0 */

    TIM_Encoder_InitTypeDef sConfig       = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    /* USER CODE BEGIN TIM2_Init 1 */

    /* USER CODE END TIM2_Init 1 */
    htim2.Instance               = TIM2;
    htim2.Init.Prescaler         = 0;
    htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim2.Init.Period            = 4294967295;
    htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    sConfig.EncoderMode          = TIM_ENCODERMODE_TI1;
    sConfig.IC1Polarity          = TIM_ICPOLARITY_RISING;
    sConfig.IC1Selection         = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler         = TIM_ICPSC_DIV1;
    sConfig.IC1Filter            = 0;
    sConfig.IC2Polarity          = TIM_ICPOLARITY_RISING;
    sConfig.IC2Selection         = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler         = TIM_ICPSC_DIV1;
    sConfig.IC2Filter            = 0;
    if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM2_Init 2 */

    /* USER CODE END TIM2_Init 2 */
}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void)
{

    /* USER CODE BEGIN TIM3_Init 0 */

    /* USER CODE END TIM3_Init 0 */

    TIM_Encoder_InitTypeDef sConfig       = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    /* USER CODE BEGIN TIM3_Init 1 */

    /* USER CODE END TIM3_Init 1 */
    htim3.Instance               = TIM3;
    htim3.Init.Prescaler         = 0;
    htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim3.Init.Period            = 65535;
    htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    sConfig.EncoderMode          = TIM_ENCODERMODE_TI1;
    sConfig.IC1Polarity          = TIM_ICPOLARITY_RISING;
    sConfig.IC1Selection         = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler         = TIM_ICPSC_DIV1;
    sConfig.IC1Filter            = 0;
    sConfig.IC2Polarity          = TIM_ICPOLARITY_RISING;
    sConfig.IC2Selection         = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler         = TIM_ICPSC_DIV1;
    sConfig.IC2Filter            = 0;
    if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM3_Init 2 */

    /* USER CODE END TIM3_Init 2 */
}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void)
{

    /* USER CODE BEGIN TIM4_Init 0 */

    /* USER CODE END TIM4_Init 0 */

    TIM_Encoder_InitTypeDef sConfig       = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    /* USER CODE BEGIN TIM4_Init 1 */

    /* USER CODE END TIM4_Init 1 */
    htim4.Instance               = TIM4;
    htim4.Init.Prescaler         = 0;
    htim4.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim4.Init.Period            = 65535;
    htim4.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    sConfig.EncoderMode          = TIM_ENCODERMODE_TI1;
    sConfig.IC1Polarity          = TIM_ICPOLARITY_RISING;
    sConfig.IC1Selection         = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler         = TIM_ICPSC_DIV1;
    sConfig.IC1Filter            = 0;
    sConfig.IC2Polarity          = TIM_ICPOLARITY_RISING;
    sConfig.IC2Selection         = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler         = TIM_ICPSC_DIV1;
    sConfig.IC2Filter            = 0;
    if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM4_Init 2 */

    /* USER CODE END TIM4_Init 2 */
}

/**
 * @brief TIM5 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM5_Init(void)
{

    /* USER CODE BEGIN TIM5_Init 0 */

    /* USER CODE END TIM5_Init 0 */

    TIM_Encoder_InitTypeDef sConfig       = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    /* USER CODE BEGIN TIM5_Init 1 */

    /* USER CODE END TIM5_Init 1 */
    htim5.Instance               = TIM5;
    htim5.Init.Prescaler         = 0;
    htim5.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim5.Init.Period            = 4294967295;
    htim5.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    sConfig.EncoderMode          = TIM_ENCODERMODE_TI1;
    sConfig.IC1Polarity          = TIM_ICPOLARITY_RISING;
    sConfig.IC1Selection         = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler         = TIM_ICPSC_DIV1;
    sConfig.IC1Filter            = 0;
    sConfig.IC2Polarity          = TIM_ICPOLARITY_RISING;
    sConfig.IC2Selection         = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler         = TIM_ICPSC_DIV1;
    sConfig.IC2Filter            = 0;
    if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM5_Init 2 */

    /* USER CODE END TIM5_Init 2 */
}

/**
 * @brief TIM6 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM6_Init(void)
{

    /* USER CODE BEGIN TIM6_Init 0 */

    /* USER CODE END TIM6_Init 0 */

    TIM_MasterConfigTypeDef sMasterConfig = {0};

    /* USER CODE BEGIN TIM6_Init 1 */

    /* USER CODE END TIM6_Init 1 */
    htim6.Instance               = TIM6;
    htim6.Init.Prescaler         = 8399;
    htim6.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim6.Init.Period            = 99;
    htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim6) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM6_Init 2 */

    /* USER CODE END TIM6_Init 2 */
}

/**
 * @brief TIM9 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM9_Init(void)
{

    /* USER CODE BEGIN TIM9_Init 0 */

    /* USER CODE END TIM9_Init 0 */

    TIM_OC_InitTypeDef sConfigOC = {0};

    /* USER CODE BEGIN TIM9_Init 1 */

    /* USER CODE END TIM9_Init 1 */
    htim9.Instance               = TIM9;
    htim9.Init.Prescaler         = 83;
    htim9.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim9.Init.Period            = 999;
    htim9.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&htim9) != HAL_OK) {
        Error_Handler();
    }
    sConfigOC.OCMode     = TIM_OCMODE_PWM1;
    sConfigOC.Pulse      = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM9_Init 2 */

    /* USER CODE END TIM9_Init 2 */
    HAL_TIM_MspPostInit(&htim9);
}

/**
 * @brief TIM10 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM10_Init(void)
{

    /* USER CODE BEGIN TIM10_Init 0 */

    /* USER CODE END TIM10_Init 0 */

    TIM_OC_InitTypeDef sConfigOC = {0};

    /* USER CODE BEGIN TIM10_Init 1 */

    /* USER CODE END TIM10_Init 1 */
    htim10.Instance               = TIM10;
    htim10.Init.Prescaler         = 83;
    htim10.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim10.Init.Period            = 999;
    htim10.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&htim10) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_Init(&htim10) != HAL_OK) {
        Error_Handler();
    }
    sConfigOC.OCMode     = TIM_OCMODE_PWM1;
    sConfigOC.Pulse      = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim10, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM10_Init 2 */

    /* USER CODE END TIM10_Init 2 */
    HAL_TIM_MspPostInit(&htim10);
}

/**
 * @brief TIM11 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM11_Init(void)
{

    /* USER CODE BEGIN TIM11_Init 0 */

    /* USER CODE END TIM11_Init 0 */

    TIM_OC_InitTypeDef sConfigOC = {0};

    /* USER CODE BEGIN TIM11_Init 1 */

    /* USER CODE END TIM11_Init 1 */
    htim11.Instance               = TIM11;
    htim11.Init.Prescaler         = 83;
    htim11.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim11.Init.Period            = 999;
    htim11.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&htim11) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_Init(&htim11) != HAL_OK) {
        Error_Handler();
    }
    sConfigOC.OCMode     = TIM_OCMODE_PWM1;
    sConfigOC.Pulse      = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim11, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM11_Init 2 */

    /* USER CODE END TIM11_Init 2 */
    HAL_TIM_MspPostInit(&htim11);
}

/**
 * @brief UART4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_UART4_Init(void)
{

    /* USER CODE BEGIN UART4_Init 0 */

    /* USER CODE END UART4_Init 0 */

    /* USER CODE BEGIN UART4_Init 1 */

    /* USER CODE END UART4_Init 1 */
    huart4.Instance          = UART4;
    huart4.Init.BaudRate     = 115200;
    huart4.Init.WordLength   = UART_WORDLENGTH_8B;
    huart4.Init.StopBits     = UART_STOPBITS_1;
    huart4.Init.Parity       = UART_PARITY_NONE;
    huart4.Init.Mode         = UART_MODE_TX_RX;
    huart4.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart4.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart4) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN UART4_Init 2 */

    /* USER CODE END UART4_Init 2 */
}

/**
 * @brief UART5 Initialization Function
 * @param None
 * @retval None
 */
static void MX_UART5_Init(void)
{

    /* USER CODE BEGIN UART5_Init 0 */

    /* USER CODE END UART5_Init 0 */

    /* USER CODE BEGIN UART5_Init 1 */

    /* USER CODE END UART5_Init 1 */
    huart5.Instance          = UART5;
    huart5.Init.BaudRate     = 115200;
    huart5.Init.WordLength   = UART_WORDLENGTH_8B;
    huart5.Init.StopBits     = UART_STOPBITS_1;
    huart5.Init.Parity       = UART_PARITY_NONE;
    huart5.Init.Mode         = UART_MODE_TX_RX;
    huart5.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart5.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart5) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN UART5_Init 2 */

    /* USER CODE END UART5_Init 2 */
}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void)
{

    /* USER CODE BEGIN USART1_Init 0 */

    /* USER CODE END USART1_Init 0 */

    /* USER CODE BEGIN USART1_Init 1 */

    /* USER CODE END USART1_Init 1 */
    huart1.Instance          = USART1;
    huart1.Init.BaudRate     = 115200;
    huart1.Init.WordLength   = UART_WORDLENGTH_8B;
    huart1.Init.StopBits     = UART_STOPBITS_1;
    huart1.Init.Parity       = UART_PARITY_NONE;
    huart1.Init.Mode         = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN USART1_Init 2 */

    /* USER CODE END USART1_Init 2 */
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{

    /* USER CODE BEGIN USART2_Init 0 */

    /* USER CODE END USART2_Init 0 */

    /* USER CODE BEGIN USART2_Init 1 */

    /* USER CODE END USART2_Init 1 */
    huart2.Instance          = USART2;
    huart2.Init.BaudRate     = 115200;
    huart2.Init.WordLength   = UART_WORDLENGTH_8B;
    huart2.Init.StopBits     = UART_STOPBITS_1;
    huart2.Init.Parity       = UART_PARITY_NONE;
    huart2.Init.Mode         = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN USART2_Init 2 */

    /* USER CODE END USART2_Init 2 */
}

/**
 * @brief USART6 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART6_UART_Init(void)
{

    /* USER CODE BEGIN USART6_Init 0 */

    /* USER CODE END USART6_Init 0 */

    /* USER CODE BEGIN USART6_Init 1 */

    /* USER CODE END USART6_Init 1 */
    huart6.Instance          = USART6;
    huart6.Init.BaudRate     = 115200;
    huart6.Init.WordLength   = UART_WORDLENGTH_8B;
    huart6.Init.StopBits     = UART_STOPBITS_1;
    huart6.Init.Parity       = UART_PARITY_NONE;
    huart6.Init.Mode         = UART_MODE_TX_RX;
    huart6.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart6.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart6) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN USART6_Init 2 */

    /* USER CODE END USART6_Init 2 */
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{

    /* DMA controller clock enable */
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* DMA interrupt init */
    /* DMA1_Stream2_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
    /* DMA1_Stream5_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    /* USER CODE BEGIN MX_GPIO_Init_1 */

    /* USER CODE END MX_GPIO_Init_1 */

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14, GPIO_PIN_RESET);

    /*Configure GPIO pins : PD11 PD12 PD13 */
    GPIO_InitStruct.Pin   = GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /*Configure GPIO pin : PD14 */
    GPIO_InitStruct.Pin   = GPIO_PIN_14;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* USER CODE BEGIN MX_GPIO_Init_2 */

    /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) {
        Motor_Speed_Test_Log_Data();
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) // 10ms 中断 (100Hz)
    {
        static uint8_t count = 0;

        // 1. 更新所有传感器和控制器
        Encoders_All_Update();
        Motor_PID_Update_All();
        Yaw_PID_Update(); // 更新转向环PID控制

        // count++; //禁用调试输出
        if (count >= 20) {
            // 2. 每5次中断 (即50ms)，请求主循环发送一次数据
            Debugger_Send_Wave(
                MotorD_PID.Speed_PID.Target,   // 通道1: 目标速度
                MotorD_PID.Speed_PID.Measured, // 通道2: 实际速度
                MotorD_PID.Speed_PID.Output,   // 通道3: PID输出PWM值
                0.0f                           // 通道4: 备用
            );
        }
    }
}

/**
 * @brief Rx Transfer completed callbacks
 * @param huart: uart handle
 * @param Size: Number of data received
 * @retval None
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART2) {
        qr_uart_rx_event_callback(huart, Size);
    } else if (huart->Instance == UART4) {
        // 调用解析函数处理接收到的数据
        HWT101_ParseData(rx_buffer, Size);

        // 清空缓冲区内容
        memset(rx_buffer, 0, Size);

        // 重新启动DMA+IDLE接收
        HAL_UARTEx_ReceiveToIdle_DMA(&huart4, rx_buffer, RX_BUFFER_SIZE);
    }
}

/**
 * @brief  串口空闲中断回调函数
 * @note   当总线上一段时间没有数据时（一帧数据接收结束），此中断被触发
 */
void USART_IdleCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART4) {
        HAL_UART_DMAStop(huart);
        gyro_rx_len  = GYRO_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_uart4_rx);
        gyro_rx_flag = true;
    }
}

// /**
//  * @brief  主循环中调用的接收数据检查与处理函数 张大头的
//  */
// void main_usart_rx_check(void)
// {
//   // 如果接收完成标志位为true
//   if (rx_flag)
//   {
//     printf("Received %d bytes: ", rx_len);
//     for (int i = 0; i < rx_len; i++)
//     {
//       printf("%02X ", rx_buffer[i]);
//     }
//     printf("\r\n");

//     // 清空缓冲区（为下次接收做准备）
//     // memset(rx_buffer, 0, rx_len); // 可以不清空，因为下次DMA会覆盖
//     rx_len = 0;

//     // 清除标志位
//     rx_flag = false;

//     // 重新开启DMA接收
//     HAL_UART_Receive_DMA(&huart2, rx_buffer, RX_BUFFER_SIZE);
//   }
// }

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {
    }
    /* USER CODE END Error_Handler_Debug */
}

/* USER CODE BEGIN 4 */
/**
 * @brief UART发送完成回调函数
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    // OpenMV UART6发送回调
    omv_uart_tx_callback(huart);

    // 其他UART回调处理...
}

/**
 * @brief UART错误回调函数
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    // OpenMV UART6错误回调
    omv_uart_error_callback(huart);

    // 其他UART错误回调处理...
}
/* USER CODE END 4 */ #ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
