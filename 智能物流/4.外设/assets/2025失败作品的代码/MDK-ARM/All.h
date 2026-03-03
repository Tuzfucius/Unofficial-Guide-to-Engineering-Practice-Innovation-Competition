#ifndef ALL_H
#define ALL_H

#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

// SSD1306 OLED显示屏相关结构体和函数声明
typedef struct {
    // 这里应该是字体结构体的定义
} SSD1306_Font_t;

extern const SSD1306_Font_t Font_7x10;

// 函数声明
void ssd1306_Fill(uint8_t color);
void ssd1306_SetCursor(uint8_t x, uint8_t y);
void ssd1306_WriteString(char* str, SSD1306_Font_t font, uint8_t color);
void ssd1306_UpdateScreen(void);

// 电机编码器结构体声明
typedef struct {
    int32_t Total_counter;
} Encoder_Struct_t;

extern Encoder_Struct_t MotorA_Encoder;
extern Encoder_Struct_t MotorB_Encoder;
extern Encoder_Struct_t MotorC_Encoder;
extern Encoder_Struct_t MotorD_Encoder;

// 小车控制函数声明
void Car_Reset_And_Sync_State(void);
void Car_FORWARD(float speed, uint32_t distance);
void Car_Strafe_Left_Smooth(float speed, uint32_t distance);
void Car_LEFT90(void);

#endif /* ALL_H */