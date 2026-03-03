// Core/Inc/ssd1306.h

#ifndef INC_SSD1306_H_
#define INC_SSD1306_H_

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include "ssd1306_fonts.h" 

// 屏幕尺寸
#define SSD1306_WIDTH  128
#define SSD1306_HEIGHT 64



// 函数原型
void ssd1306_Init(void);
void ssd1306_UpdateScreen(void);
void ssd1306_Fill(uint8_t color);
void ssd1306_DrawPixel(uint16_t x, uint16_t y, uint8_t color);

char ssd1306_WriteString(char* str, SSD1306_Font_t Font, uint8_t color);
void ssd1306_SetCursor(uint8_t x, uint8_t y);
char ssd1306_WriteChar(char ch, SSD1306_Font_t Font, uint8_t color);

/**
 * @brief  在指定位置绘制位图
 * @param  bitmap: 指向位图数据的指针
 * @param  x, y:   图像左上角的起始坐标
 * @param  width, height: 图像的宽度和高度
 * @retval None
 */
void ssd1306_DrawBitmap(const uint8_t* bitmap, uint16_t x, uint16_t y, uint16_t width, uint16_t height);
void Update_Display(const char* line1, const char* line2, const char* line3);
#endif // INC_SSD1306_H_