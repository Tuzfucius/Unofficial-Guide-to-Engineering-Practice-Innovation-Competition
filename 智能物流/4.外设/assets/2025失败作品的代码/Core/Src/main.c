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
#include "visual_control.h"
#include "motor_test.h"
#include "datou_pwm.h"
#include "ft_servo.h"
#include "All.h"
#include "all2.h"

/* USER CODE END Includes */

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
TIM_HandleTypeDef htim12;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart3_tx;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart6_rx;

/* USER CODE BEGIN PV */
// main.c

/* USER CODE BEGIN PV */
const unsigned char tongse[] = {
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

const unsigned char yise[] = {
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

const unsigned char guanbi[] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x60, 0x00, 0x00, 0xe0, 0x00, 0x00, 0x00, 0x07, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x78, 0x00, 0x00, 0xf8, 0x00, 0x00, 0x00, 0x07, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x3e, 0x00, 0x00, 0xfc, 0x00, 0x00, 0x00, 0x03, 0xf0, 0x00, 0x00, 0x00, 0x03, 0x00,
    0x00, 0x00, 0x1f, 0x00, 0x01, 0xff, 0x00, 0x00, 0x00, 0x01, 0xf8, 0x00, 0x00, 0x00, 0x07, 0x80,
    0x00, 0x00, 0x1f, 0xc0, 0x01, 0xff, 0x00, 0x00, 0x00, 0x00, 0xfc, 0x00, 0x00, 0x00, 0x0f, 0xc0,
    0x00, 0x00, 0x0f, 0xe0, 0x01, 0xf8, 0x00, 0x00, 0x00, 0x00, 0xfe, 0x3f, 0xff, 0xff, 0xff, 0xe0,
    0x00, 0x00, 0x07, 0xf0, 0x03, 0xf0, 0x00, 0x00, 0x00, 0xc0, 0x7e, 0x3f, 0xff, 0xff, 0xff, 0xe0,
    0x00, 0x00, 0x07, 0xf0, 0x03, 0xf0, 0x00, 0x00, 0x00, 0xf0, 0x7e, 0x1f, 0x80, 0x00, 0x0f, 0xc0,
    0x00, 0x00, 0x03, 0xf8, 0x07, 0xe0, 0x00, 0x00, 0x00, 0xfc, 0x3e, 0x00, 0x00, 0x00, 0x0f, 0x80,
    0x00, 0x00, 0x03, 0xf8, 0x07, 0xc0, 0x00, 0x00, 0x00, 0xfe, 0x3e, 0x00, 0x00, 0x00, 0x0f, 0x80,
    0x00, 0x00, 0x01, 0xf8, 0x07, 0xc0, 0x00, 0x00, 0x00, 0xfe, 0x3c, 0x00, 0x38, 0x00, 0x0f, 0x80,
    0x00, 0x00, 0x01, 0xf0, 0x0f, 0x80, 0x00, 0x00, 0x00, 0xf8, 0x00, 0x00, 0x3e, 0x00, 0x0f, 0x80,
    0x00, 0x00, 0x00, 0xf0, 0x0f, 0x00, 0x0e, 0x00, 0x00, 0xf8, 0x00, 0x00, 0x3f, 0x80, 0x0f, 0x80,
    0x00, 0x00, 0x00, 0xe0, 0x1e, 0x00, 0x1f, 0x00, 0x00, 0xf8, 0x00, 0x00, 0x3f, 0x80, 0x0f, 0x80,
    0x00, 0x00, 0x00, 0x00, 0x1e, 0x00, 0x3f, 0x80, 0x00, 0xf8, 0x00, 0x00, 0x3f, 0x00, 0x0f, 0x80,
    0x03, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xc0, 0x00, 0xf8, 0x00, 0x00, 0x3e, 0x00, 0x0f, 0x80,
    0x01, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xe0, 0x00, 0xf8, 0x00, 0x00, 0x3e, 0x00, 0x0f, 0x80,
    0x00, 0xf8, 0x00, 0x07, 0xc0, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x00, 0x00, 0x3e, 0x00, 0x0f, 0x80,
    0x00, 0x00, 0x00, 0x07, 0xc0, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x00, 0x00, 0x3e, 0x03, 0x0f, 0x80,
    0x00, 0x00, 0x00, 0x07, 0xc0, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x00, 0x00, 0x3e, 0x07, 0x8f, 0x80,
    0x00, 0x00, 0x00, 0x07, 0xc0, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x00, 0x00, 0x3e, 0x07, 0xcf, 0x80,
    0x00, 0x00, 0x00, 0x07, 0xc0, 0x00, 0x00, 0x00, 0x00, 0xf9, 0xff, 0xff, 0xff, 0xff, 0xef, 0x80,
    0x00, 0x00, 0x00, 0x07, 0xc0, 0x00, 0x00, 0x00, 0x00, 0xf9, 0xff, 0xff, 0xff, 0xff, 0xff, 0x80,
    0x00, 0x00, 0x00, 0x07, 0xc0, 0x00, 0x00, 0x00, 0x00, 0xf8, 0xfc, 0x01, 0xfe, 0x00, 0x0f, 0x80,
    0x00, 0x00, 0x00, 0x07, 0xc0, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x00, 0x03, 0xfe, 0x00, 0x0f, 0x80,
    0x00, 0x00, 0x00, 0x07, 0xc0, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x00, 0x03, 0xfe, 0x00, 0x0f, 0x80,
    0x00, 0x00, 0x00, 0x07, 0xc0, 0x00, 0x01, 0x80, 0x00, 0xf8, 0x00, 0x07, 0xfe, 0x00, 0x0f, 0x80,
    0x00, 0x00, 0x00, 0x07, 0xc0, 0x00, 0x03, 0xc0, 0x00, 0xf8, 0x00, 0x07, 0xfe, 0x00, 0x0f, 0x80,
    0x00, 0x00, 0x00, 0x07, 0xc0, 0x00, 0x07, 0xe0, 0x00, 0xf8, 0x00, 0x0f, 0xfe, 0x00, 0x0f, 0x80,
    0x00, 0x00, 0x00, 0x07, 0xc0, 0x00, 0x0f, 0xf0, 0x00, 0xf8, 0x00, 0x1f, 0xbe, 0x00, 0x0f, 0x80,
    0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf8, 0x00, 0xf8, 0x00, 0x1f, 0x3e, 0x00, 0x0f, 0x80,
    0x1f, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc, 0x00, 0xf8, 0x00, 0x3f, 0x3e, 0x00, 0x0f, 0x80,
    0x0f, 0xc0, 0x00, 0x0f, 0xb8, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x00, 0x7e, 0x3e, 0x00, 0x0f, 0x80,
    0x00, 0x00, 0x00, 0x0f, 0xbc, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x00, 0x7c, 0x3e, 0x00, 0x0f, 0x80,
    0x00, 0x00, 0x00, 0x1f, 0x9c, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x00, 0xf8, 0x3e, 0x00, 0x0f, 0x80,
    0x00, 0x00, 0x00, 0x1f, 0x9e, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x01, 0xf0, 0x3e, 0x00, 0x0f, 0x80,
    0x00, 0x00, 0x00, 0x3f, 0x1e, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x03, 0xe0, 0x3e, 0x00, 0x0f, 0x80,
    0x00, 0x00, 0x00, 0x3f, 0x0f, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x07, 0xe0, 0x3e, 0x00, 0x0f, 0x80,
    0x00, 0x00, 0x00, 0x7e, 0x0f, 0x80, 0x00, 0x00, 0x00, 0xf8, 0x0f, 0xc0, 0x3e, 0x00, 0x0f, 0x80,
    0x00, 0x00, 0x00, 0x7e, 0x07, 0x80, 0x00, 0x00, 0x00, 0xf8, 0x1f, 0x00, 0x3e, 0x00, 0x0f, 0x80,
    0x00, 0x00, 0x00, 0xfc, 0x07, 0xc0, 0x00, 0x00, 0x00, 0xf8, 0x3e, 0x00, 0x3e, 0x00, 0x0f, 0x80,
    0x00, 0x00, 0x01, 0xfc, 0x07, 0xe0, 0x00, 0x00, 0x00, 0xf8, 0x7c, 0x00, 0x3e, 0x00, 0x0f, 0x80,
    0x00, 0x00, 0x01, 0xf8, 0x03, 0xf0, 0x00, 0x00, 0x00, 0xf8, 0xf8, 0x00, 0x3e, 0x00, 0x0f, 0x80,
    0x00, 0x00, 0x03, 0xf0, 0x01, 0xf8, 0x00, 0x00, 0x00, 0xf9, 0xe0, 0x00, 0x3e, 0x00, 0x0f, 0x80,
    0x00, 0x00, 0x07, 0xe0, 0x01, 0xfc, 0x00, 0x00, 0x00, 0xff, 0xc0, 0x00, 0x3e, 0x00, 0x0f, 0x80,
    0x00, 0x00, 0x0f, 0xe0, 0x00, 0xfe, 0x00, 0x00, 0x00, 0xff, 0x00, 0x00, 0x3e, 0x00, 0x0f, 0x80,
    0x00, 0x00, 0x1f, 0xc0, 0x00, 0x7f, 0x00, 0x00, 0x00, 0xf8, 0x00, 0x7f, 0xfe, 0x00, 0x0f, 0x80,
    0x00, 0x00, 0x3f, 0x80, 0x00, 0x3f, 0x80, 0x00, 0x00, 0xf8, 0x00, 0x3f, 0xfe, 0x00, 0x0f, 0x80,
    0x00, 0x00, 0x7e, 0x00, 0x00, 0x1f, 0xe0, 0x00, 0x00, 0xf8, 0x00, 0x0f, 0xfe, 0x00, 0x0f, 0x80,
    0x00, 0x00, 0xfc, 0x00, 0x00, 0x0f, 0xf8, 0x00, 0x00, 0xf8, 0x00, 0x03, 0xfc, 0x00, 0x0f, 0x80,
    0x00, 0x03, 0xf8, 0x00, 0x00, 0x07, 0xfe, 0x00, 0x00, 0xf8, 0x00, 0x01, 0xfc, 0x38, 0x0f, 0x80,
    0x00, 0x07, 0xe0, 0x00, 0x00, 0x03, 0xff, 0x80, 0x00, 0xf8, 0x00, 0x00, 0xf0, 0x3f, 0xff, 0x80,
    0x00, 0x1f, 0xc0, 0x00, 0x00, 0x01, 0xff, 0xf0, 0x00, 0xf8, 0x00, 0x00, 0x00, 0x0f, 0xff, 0x80,
    0x00, 0x7f, 0x00, 0x00, 0x00, 0x00, 0xff, 0xfc, 0x00, 0xf8, 0x00, 0x00, 0x00, 0x03, 0xff, 0x80,
    0x01, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xf8, 0x00, 0xf8, 0x00, 0x00, 0x00, 0x01, 0xff, 0x00,
    0x07, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xc0, 0x00, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x7e, 0x00,
    0x1f, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x07, 0x80, 0x00, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x7c, 0x00,
    0x1e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

// ssd1306_Fill(0);
// ssd1306_UpdateScreen();
// HAL_Delay(500);
// // 3. 调用绘图函数，将图像数据加载到缓冲区
// ssd1306_DrawBitmap(yise, 0, 0, 128, 64);
// // 4. 刷新屏幕，将缓冲区的内容显示出来 (关键一步!)
// ssd1306_UpdateScreen();

uint32_t fac_us;

void HAL_Delay_us_init(uint8_t SYSCLK)
{
  fac_us = SYSCLK;
}

void HAL_Delay_us(uint32_t nus)
{
  uint32_t ticks;
  uint32_t told, tnow, tcnt = 0;
  uint32_t reload = SysTick->LOAD;
  ticks = nus * fac_us;
  told = SysTick->VAL;
  while (1)
  {
    tnow = SysTick->VAL;
    if (tnow != told)
    {
      if (tnow < told)
        tcnt += told - tnow;
      else
        tcnt += reload - tnow + told;
      told = tnow;
      if (tcnt >= ticks)
        break;
    }
  }
}
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
static void MX_USART3_UART_Init(void);
static void MX_TIM12_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t usart1_ReadBuffer[8];
uint8_t rx_byte;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern Usart_DataTypeDef FSUS_Usart;
void User_Uart_Init(UART_HandleTypeDef *huartx);

extern UART_HandleTypeDef huart3; // 这原本是张大头的 2改3
extern DMA_HandleTypeDef hdma_usart3_tx;

// 张大头用
#define RX_BUFFER_SIZE 64                            // 定义接收缓冲区的最大长度
#define ZD_MOTOR_RX_BUFFER_SIZE 64                   // 定义接收缓冲区的最大长度
uint8_t zd_motor_rx_buffer[ZD_MOTOR_RX_BUFFER_SIZE]; // DMA接收缓冲区
uint8_t motor_addr = 1;                              // 定义电机地址
void main_usart_rx_check();                          // 声明检查函数

uint8_t rx_buffer[RX_BUFFER_SIZE]; // DMA接收缓冲区
volatile uint8_t rx_len = 0;       // volatile关键字防止编译器优化，确保中断和主程序能正确访问
volatile bool rx_flag = false;     // 接收完成标志

// --- 陀螺仪接收变量 --- hwt
#define GYRO_RX_BUFFER_SIZE 33 // 建议为11的倍数，可以缓存3帧数据
uint8_t gyro_rx_buffer[GYRO_RX_BUFFER_SIZE];
volatile uint8_t gyro_rx_len = 0;
volatile bool gyro_rx_flag = false;

// --- OpenMV Communication Variables ---
// 缓冲区大小
#define OPENMV_RX_BUFFER_SIZE 100
uint8_t openmv_dma_rx_buffer[OPENMV_RX_BUFFER_SIZE];
uint8_t openmv_process_buffer[OPENMV_RX_BUFFER_SIZE];
volatile bool openmv_data_ready = false;
volatile uint16_t openmv_data_length = 0;
bool color_matched = false;
// --- 【新增】全局变量，用于存储从OpenMV解析出的目标信息 ---
char target_color = ' ';
int target_x = -1;
int target_y = -1;
int target_angle = -1;
int center_x = 78;
int center_y = 30; // OpenMV图像中心点
// 左上角 0,0 ，右下角 128,64
// 对于openmv来说，检测点在目标点左侧，小车后退，检测点在目标点上方，小车左移

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
  MX_USART3_UART_Init();
  MX_TIM12_Init();
  /* USER CODE BEGIN 2 */
  void Servo_SendPositionCommand(uint8_t servo_id, uint16_t position);
  void Servo_SendPositionAccelSpeedCommand(UART_HandleTypeDef * huart_handle, uint8_t id, uint16_t acceleration, int position, uint16_t speed);
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
  HAL_Delay(500);

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
  ssd1306_Fill(0);
  ssd1306_SetCursor(0, 0);
  ssd1306_WriteString("5 ok", Font_7x10, 1);
  ssd1306_UpdateScreen();

  HAL_TIM_Base_Start_IT(&htim6);
  HAL_Delay(500);
  ssd1306_Fill(0);
  ssd1306_SetCursor(0, 0);
  ssd1306_WriteString("6 ok", Font_7x10, 1);
  ssd1306_UpdateScreen();

  Datou_PWM_Init();

  // 初始化HWT101（短时间阻塞）
  HWT101_Init(&huart4);
  HAL_Delay(200);

  qr_init();
  User_Uart_Init(&huart5);

  // 初始化OpenMV RPC通信
  HAL_UARTEx_ReceiveToIdle_DMA(&huart6, openmv_dma_rx_buffer, OPENMV_RX_BUFFER_SIZE);
  void parse_openmv_data(void);
  void set_openmv_light(uint8_t brightness);
  HAL_Delay(1200);
  HAL_Delay_us_init(fac_us);

  // PID初始化已确认没问题
  Yaw_PID_Init(); // 初始化转向环PID控制器

  // 系统就绪显示
  ssd1306_Fill(0);
  ssd1306_SetCursor(0, 0);
  ssd1306_WriteString("STM32F407", Font_7x10, 1);
  ssd1306_SetCursor(0, 15);
  ssd1306_WriteString("System Ready", Font_7x10, 1);
  ssd1306_SetCursor(0, 30);
  ssd1306_WriteString("All OK!", Font_7x10, 1);
  ssd1306_UpdateScreen();
  HAL_Delay(500);

  // 主任务开始 ---------------------------------------
  center_x = 78;
  center_y = 30;
  task1();
  HAL_Delay(20);
  color_matched = false;
  if (!Car_QR_Wait_And_Display(&color_matched, 8000U))
  {
    ssd1306_Fill(0);
    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString("QR Timeout", Font_7x10, 1);
    ssd1306_UpdateScreen();
  }
  servo_zero();
  HAL_Delay(50);
  set_openmv_light(1);
  HAL_Delay(50);
  task2(); // 第一级台阶抓取
  center_y = 34;
  center_x = 78;
  task3(); // 凸台抓取放置
  center_y = 37;
  center_x = 75;
  task4(); // 第一圈放置区
  // center_y = 16; //这是看高处的
  center_y = 32; // 这是看下面的
  task5(); //开始第二圈，抓取
  center_y = 32;
  // task6(); // 错误
  task7(); // 第二圈去凸台
  task8(); // 放置区
  task();

  // 测试----------------------------------
  // Car_Move_Forward_Auto(300);
  // HAL_Delay(1000);
  // Car_Move_Forward_Auto(1000);

  // --------------------------------------
  // task6();
  // center_x = 78;
  // center_y = 20;
  // near_place_all_low_2();

  // center_y = 44;
  // center_x = 75;
  // near_place_all_2();

  // Car_Turn_To_Angle_PID(90, 2190U);
  //  visual_align_blocking_V3_111();
  //  near_place_all();
  //  near_place_all_catchpart();

  // far_catch();

  // Car_Go_Straight_Pulses_PID(1400);
  // if (!Car_Wait_For_Position_Stop(5000U))
  // {
  //   Car_Stop_PID();
  // }

  // Car_Strafe_Left_Pulses_PID(200);
  // if (!Car_Wait_For_Position_Stop(1000U))
  // {
  //   Car_Stop_PID();
  // }
  // center_y = 44; // 低处看
  // visual_align_blocking_V3_111();
  // // near_place_catch_all();
  // near_catch();

  // const float yaw_demo_target = 90.0f;
  // float yaw_demo_start = g_hwt101_data.Angle_Z;

  // const float yaw_demo_target = 90.0f;
  // float yaw_demo_start = g_hwt101_data.Angle_Z;

  //  Motor_Test_Display_Yaw_Status(yaw_demo_start, yaw_demo_target);
  //  Car_Turn_To_Angle_PID(yaw_demo_target, 2000U);

  // 关闭 ----------------------
  ssd1306_Fill(0);
  ssd1306_UpdateScreen();
  HAL_Delay(500);
  ssd1306_DrawBitmap(guanbi, 0, 0, 128, 64);
  ssd1306_UpdateScreen();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    Car_Process_Stop_Request();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* 实时刷新偏航信息 */
    // Motor_Test_Display_Yaw_Status(g_hwt101_data.Angle_Z, 90.0f);
    // Motor_Speed_Test_Update_Display();
    // HAL_Delay(20);
  }
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
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 83;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
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
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 8399;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 99;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
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
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 83;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 999;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
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
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 83;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 999;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim10, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
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
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 83;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 999;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim11, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */
  HAL_TIM_MspPostInit(&htim11);
}

/**
 * @brief TIM12 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 83;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 999;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */
  HAL_TIM_MspPostInit(&htim12);
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
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
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
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
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
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
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
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */
}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */
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
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
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
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SERVO_DIR_PIN_GPIO_Port, SERVO_DIR_PIN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, MOTOR_DIR_Pin | MOTOR_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SERVO_DIR_PIN_Pin */
  GPIO_InitStruct.Pin = SERVO_DIR_PIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SERVO_DIR_PIN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PD11 PD12 PD13 */
  GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PD14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : MOTOR_DIR_Pin MOTOR_EN_Pin */
  GPIO_InitStruct.Pin = MOTOR_DIR_Pin | MOTOR_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
 * @brief  【已升级】解析来自OpenMV的数据缓冲区 (格式: "c,x,y,a\n").
 * @note   此函数会直接修改全局变量 target_color, target_x, target_y, target_angle.
 * @retval None
 */
void parse_openmv_data(void)
{
  // 【核心修正】更新sscanf的格式字符串为 "%c,%d,%d,%d" 以匹配4个字段
  // sscanf 会返回成功解析并赋值的变量数量。我们现在期望是4。
  int items_parsed = sscanf((const char *)openmv_process_buffer, "%c,%d,%d,%d",
                            &target_color, &target_x, &target_y, &target_angle);

  // 检查解析是否成功
  if (items_parsed == 4)
  {
    // 如果成功解析了4个项目，打印完整结果以供调试
    printf("Parse OK -> Color: %c, X: %d, Y: %d, Angle: %d\r\n",
           target_color, target_x, target_y, target_angle);
  }
  else
  {
    // 如果解析失败 (例如，只收到了3个字段或格式错误),
    // 重置所有全局变量为无效状态，确保系统安全。
    target_color = ' ';
    target_x = -1;
    target_y = -1;
    target_angle = -1;

    // 打印原始数据以帮助调试
    printf("Parse FAIL -> Raw data: %s\r\n", openmv_process_buffer);
  }

  // 清空处理缓冲区，为下一次接收做准备
  memset(openmv_process_buffer, 0, sizeof(openmv_process_buffer));
}

/**
 * @brief  【新增】发送指令到OpenMV以控制灯光亮度.
 * @param  brightness: 亮度值 (0-100).
 * @retval None
 */
void set_openmv_light(uint8_t brightness)
{
  // 1. 定义一个缓冲区来存储格式化后的命令字符串
  //    "L,100\n" 最多需要7个字节 (包括'\0')，我们给10个字节，足够安全
  char command_buffer[10];

  // 2. 使用 sprintf 将命令格式化到缓冲区中
  //    格式为 "L,亮度值\n"
  sprintf(command_buffer, "L,%d\n", brightness);

  // 3. 通过 UART6 发送命令
  //    strlen(command_buffer) 会计算字符串的实际长度
  HAL_UART_Transmit(&huart6, (uint8_t *)command_buffer, strlen(command_buffer), HAL_MAX_DELAY);

  // 4. (可选) 打印发送的命令以供调试
  printf("Sent to OpenMV: %s", command_buffer);
}
/**
 * @brief  向OpenMV发送模式切换指令 ('M\n').
 * @param  None
 * @retval None
 */
void toggle_openmv_mode(void)
{
  // 1. 定义一个常量字符串来存储命令
  //    命令是固定的 "M\n"
  const char *command = "M\n";

  // 2. 通过 UART6 发送命令
  //    strlen(command) 会计算字符串的实际长度 (2个字节: 'M' 和 '\n')
  HAL_UART_Transmit(&huart6, (uint8_t *)command, strlen(command), HAL_MAX_DELAY);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6) // 10ms 中断 (100Hz)
  {
    // static uint8_t count = 0;

    // 1. 更新所有传感器和控制器
    Encoders_All_Update();
    Motor_PID_Update_All();
    Yaw_PID_Update(); // 更新转向环PID控制

    // count++; //禁用调试输出
    // if (count >= 20)
    // {
    //   // 2. 每5次中断 (即50ms)，请求主循环发送一次数据
    //   Debugger_Send_Wave(
    //       MotorD_PID.Speed_PID.Target,   // 通道1: 目标速度
    //       MotorD_PID.Speed_PID.Measured, // 通道2: 实际速度
    //       MotorD_PID.Speed_PID.Output,   // 通道3: PID输出PWM值
    //       0.0f                           // 通道4: 备用
    //   );
    // }
    // 调用速度测试的日志/计时处理（在 TIM6 中断中处理测试超时与 settle 检测）
    // Motor_Speed_Test_Log_Data();
  }
  if (htim->Instance == TIM12)
  {
    HAL_TIM_PeriodElapsedCallback_datou(htim);
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
  if (huart->Instance == USART2)
  {
    qr_uart_rx_event_callback(huart, Size);
  }
  // qr_uart_rx_event_callback(huart, Size);
  if (huart->Instance == USART3) // 张大头的
  {

    // for (int i = 0; i < Size; i++)
    // {
    //   printf("0x%02X ", zd_motor_rx_buffer[i]);
    // }
    // printf("\r\n");
    // 重新启动DMA+IDLE接收，为下一次数据做准备
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, zd_motor_rx_buffer, ZD_MOTOR_RX_BUFFER_SIZE);
    __HAL_DMA_DISABLE_IT(&hdma_usart3_tx, DMA_IT_HT);
  }
  if (huart->Instance == UART4)
  {
    // 调用解析函数处理接收到的数据
    HWT101_ParseData(rx_buffer, Size);

    // 清空缓冲区内容
    memset(rx_buffer, 0, Size);

    // 重新启动DMA+IDLE接收
    HAL_UARTEx_ReceiveToIdle_DMA(&huart4, rx_buffer, RX_BUFFER_SIZE);
  }
  if (huart->Instance == USART6)
  {
    // 步骤1: 立即停止本次DMA传输，防止数据竞争
    HAL_UART_DMAStop(&huart6);

    // 步骤2: 将接收到的数据从DMA缓冲区安全地拷贝到应用缓冲区
    // Size 参数准确地告诉了我们接收了多少字节
    memcpy(openmv_process_buffer, openmv_dma_rx_buffer, Size);

    // 步骤3: 设置数据长度和完成标志位，通知主循环处理
    openmv_data_length = Size;
    openmv_data_ready = true;

    // 步骤4: 重启DMA接收，准备接收下一帧数据
    // 注意：这里使用的是DMA缓冲区，而不是应用缓冲区
    HAL_UARTEx_ReceiveToIdle_DMA(&huart6, openmv_dma_rx_buffer, OPENMV_RX_BUFFER_SIZE);
  }
  // qr_uart_rx_event_callback(huart, Size);
}

/**
 * @brief  串口空闲中断回调函数
 * @note   当总线上一段时间没有数据时（一帧数据接收结束），此中断被触发
 */
void USART_IdleCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == UART4)
  {
    HAL_UART_DMAStop(huart);
    gyro_rx_len = GYRO_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_uart4_rx);
    gyro_rx_flag = true;
  }
}

#define SERVO_UART_HANDLE huart5  // 使用的UART句柄是huart5
#define SERVO_DIR_PORT GPIOB      // 方向控制引脚的端口
#define SERVO_DIR_PIN GPIO_PIN_15 // 方向控制引脚的引脚号

// =============================================================================
//    核心功能函数：根据图片协议发送位置指令 (固定速度和时间)
// =============================================================================

/**
 * @brief 根据指定的协议图片格式，发送位置控制指令到舵机。
 * @note  此函数会使用固定的速度(2000)和时间(0)来构建指令。
 * @param id       舵机ID (0-253)
 * @param position 目标位置 (0-4095)
 */
void Servo_SendPositionCommand(uint8_t id, uint16_t position)
{
  // 1. 定义指令数组，长度为13字节 (2+1+1+1+1+2+2+2+1)
  uint8_t command[13];
  uint8_t checksum_sum = 0;

  // 2. 填充数据包
  // --- 固定部分 ---
  command[0] = 0xFF; // 包头1
  command[1] = 0xFF; // 包头2
  command[4] = 0x03; // 指令: 写指令 (WRITE)
  command[5] = 0x2A; // 写入首地址: 42 (目标位置低字节)

  // --- 根据图片格式，使用固定的值 ---
  command[3] = 0x09; // 指令包数据长度 (根据图片，固定为9)
  command[8] = 0x00; // 时间低字节 (固定为0)
  command[9] = 0x00; // 时间高字节 (固定为0)
  command[10] = 0xC4;
  command[11] = 0x09;

  // --- 从函数参数获取的可变部分 ---
  command[2] = id;                       // 舵机ID
  command[6] = (uint8_t)(position);      // 目标位置低字节
  command[7] = (uint8_t)(position >> 8); // 目标位置高字节

  // 3. 计算校验和
  // 从ID字节(index=2)加到最后一个数据字节(index=11)
  for (int i = 2; i < 12; i++)
  {
    checksum_sum += command[i];
  }
  command[12] = ~checksum_sum; // 按位取反

  HAL_UART_Transmit(&huart5, command, sizeof(command), HAL_MAX_DELAY);
}

/**
 * @brief  向STS/SMS系列舵机发送带有加速度和速度控制的位置模式指令.
 * @note   此函数严格按照 "位置模式控制转动指令(含加速度)" 协议构建.
 *         - 协议指令(Instruction): 0x03 (WRITE)
 *         - 写入首地址(Start Address): 0x29 (41)
 * @param  huart_handle   用于通信的UART句柄指针 (e.g., &huart5).
 * @param  id             舵机ID (范围: 0 ~ 253).
 * @param  acceleration   加速度 (范围: 0 ~ 255, 单位: 8.878 度/s^2).
 * @param  position       目标位置 (范围: 0 ~ 4095, 对应 0 ~ 360度).
 * @param  speed          目标速度 (范围: 0 ~ 3400, 单位: 0.732rpm).
 */
void Servo_SendPositionAccelSpeedCommand(UART_HandleTypeDef *huart_handle, uint8_t id, uint16_t acceleration, int position, uint16_t speed)
{
  // 1. 定义指令数组, 总长度为14字节
  //    包头(2) + ID(1) + 长度(1) + 指令(1) + 首地址(1) + 数据(7) + 校验和(1)
  uint8_t command[14];
  uint8_t checksum_sum = 0;

  // 2. 填充数据包的固定部分
  command[0] = 0xFF; // 包头1
  command[1] = 0xFF; // 包头2
  command[3] = 0x0A; // 指令包数据长度 = 1(指令) + 1(首地址) + 7(数据) + 1(校验和) = 10 (0x0A)
  command[4] = 0x03; // 指令: 写指令 (WRITE)
  command[5] = 0x29; // 写入首地址: 41 (0x29), 对应加速度寄存器

  // 3. 填充从函数参数获取的可变部分
  command[2] = id; // 舵机ID

  if (position < 0)
  {
    position = 32768 - position;
  }

  // --- 填充7个字节的数据 ---
  command[6] = acceleration;               // 数据1: 加速度 (1 byte)
  command[7] = (uint8_t)(position & 0xFF); // 数据2: 目标位置低字节
  command[8] = (uint8_t)(position >> 8);   // 数据3: 目标位置高字节
  command[9] = 0x00;                       // 数据4: 时间低字节 (根据协议, 此模式下无功能)
  command[10] = 0x00;                      // 数据5: 时间高字节 (根据协议, 此模式下无功能)
  command[11] = (uint8_t)(speed & 0xFF);   // 数据6: 目标速度低字节
  command[12] = (uint8_t)(speed >> 8);     // 数据7: 目标速度高字节

  // 4. 计算校验和
  //    校验和 = ~ (ID + Length + Instruction + StartAddr + Data1 + ... + Data7)
  //    也就是对 command 数组中从索引2到索引12的所有字节求和, 然后按位取反
  for (int i = 2; i < 13; i++)
  {
    checksum_sum += command[i];
  }
  command[13] = ~checksum_sum; // 按位取反得到最终的校验和

  // 5. 通过指定的UART发送指令
  HAL_UART_Transmit(huart_handle, command, sizeof(command), HAL_MAX_DELAY);
}

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
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
