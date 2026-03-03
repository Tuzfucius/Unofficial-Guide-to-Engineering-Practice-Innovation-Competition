#ifndef __ALL_H
#define __ALL_H

#ifdef __cplusplus
extern "C"
{
#endif

/* =========================== 标准库头文件 =========================== */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>

/* =========================== STM32 HAL库头文件 =========================== */
#include "main.h"
#include "stm32f4xx_hal.h"

/* =========================== 项目核心头文件 =========================== */
#include "usart.h"
#include "user_uart.h"

/* =========================== 电机控制头文件 =========================== */
#include "Motor.h"
#include "Motor_PID.h"
#include "encoder.h"

/* =========================== 舵机控制头文件 =========================== */
#include "servo.h"
#include "fashion_star_uart_servo.h"
#include "ring_buffer.h"

#include "datou_pwm.h"

/* =========================== 显示模块头文件 =========================== */
#include "ssd1306.h"
#include "ssd1306_fonts.h"

/* =========================== 传感器头文件 =========================== */
#include "hwt101.h"
// #include "mpu.h"

/* =========================== 二维码头文件 =========================== */
#include "qr.h"

/* =========================== 调试和辅助头文件 =========================== */
#include "fire_debugger.h"
#include "visual_control.h"

// #include "datou.h"

/* =========================== 自定义字体头文件 =========================== */
#include "custom_fonts.h"

  // /* =========================== 函数声明 =========================== */

  // // ========== 物料转换函数 ==========
  // void servo_1(Usart_DataTypeDef *usart, uint8_t item_count, uint32_t time_ms);
  // void Servo_TurntableSetInitPosition(Usart_DataTypeDef *usart, uint16_t time_ms);

  // // // ========== 4个舵机独立控制函数 ==========
  // // void Servo_TurntableRotate(Usart_DataTypeDef *usart, float angle, uint16_t time_ms, uint16_t acc, uint16_t dec);
  // // void Servo_MainRotate(Usart_DataTypeDef *usart, float angle, uint16_t time_ms, uint16_t acc, uint16_t dec);
  // // void Servo_ForwardBackward(Usart_DataTypeDef *usart, float angle, uint32_t time_ms, uint16_t power);
  // // void Servo_GripperOpen(Usart_DataTypeDef *usart, uint16_t time_ms, uint16_t acc, uint16_t dec);
  // // void Servo_GripperClose(Usart_DataTypeDef *usart, uint16_t time_ms, uint16_t acc, uint16_t dec);

  // // ========== 状态查询函数 ==========
  // void Servo_QueryStatus(Usart_DataTypeDef *usart);
  // FSUS_STATUS Servo_QuerySingleAngle(Usart_DataTypeDef *usart, uint8_t servo_id, float *angle);

  // // ========== 示例函数 ==========
  // void Task1(void);
  // void Task2(void);
  // void Task3(void);
  // void Task4(void);
  // void set0position(void);
  // void catch_green(void);
  // void catch_red(void);
  // void catch_blue(void);
  // void catchall(void);
  // void catch_middle_three(void);

  // void placeall_same(void);
  // void placeblue_same(void);
  // void placegreen_same(void);
  // void place_middle_three(void);
  // void placered_same(void);


  // void upcatchbule(void);
  // void upcatchgreen(void);
  // void upcatchred(void);
  // void catch_diff(void);
  // void upplace_same(void);

#ifdef __cplusplus
}
#endif

#endif /* __ALL_H */