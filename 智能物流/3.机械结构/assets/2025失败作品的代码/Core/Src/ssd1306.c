
#include "ssd1306.h"
#include <string.h>
#include "ssd1306_fonts.h" 

// --- 1. 引脚宏定义 (核心配置区) ---
// 将OLED引脚与你在CubeIDE中配置的GPIO端口和引脚号对应起来
#define SSD1306_SCL_PORT GPIOD
#define SSD1306_SCL_PIN  GPIO_PIN_14  

#define SSD1306_SDA_PORT GPIOD
#define SSD1306_SDA_PIN  GPIO_PIN_13  

#define SSD1306_RES_PORT GPIOD
#define SSD1306_RES_PIN  GPIO_PIN_12  

#define SSD1306_DC_PORT  GPIOD
#define SSD1306_DC_PIN   GPIO_PIN_11  

// 屏幕缓冲区
static uint8_t SSD1306_Buffer[SSD1306_WIDTH * SSD1306_HEIGHT / 8];

// 函数声明
char ssd1306_WriteChar(char ch, SSD1306_Font_t Font, uint8_t color);

// 屏幕当前状态
static struct {
    uint16_t CurrentX;
    uint16_t CurrentY;
    bool Initialized;
} SSD1306;



// --- 2. 软件SPI底层函数 ---
static void ssd1306_Write_SPI(uint8_t byte) {
    for (uint8_t i = 0; i < 8; i++) {
        HAL_GPIO_WritePin(SSD1306_SCL_PORT, SSD1306_SCL_PIN, GPIO_PIN_RESET);
        if (byte & 0x80) { // MSB first
            HAL_GPIO_WritePin(SSD1306_SDA_PORT, SSD1306_SDA_PIN, GPIO_PIN_SET);
        } else {
            HAL_GPIO_WritePin(SSD1306_SDA_PORT, SSD1306_SDA_PIN, GPIO_PIN_RESET);
        }
        byte <<= 1;
        HAL_GPIO_WritePin(SSD1306_SCL_PORT, SSD1306_SCL_PIN, GPIO_PIN_SET);
    }
}

// --- 3. 写命令和数据函数 ---
static void ssd1306_WriteCommand(uint8_t command) {
    HAL_GPIO_WritePin(SSD1306_DC_PORT, SSD1306_DC_PIN, GPIO_PIN_RESET); // Command mode
    ssd1306_Write_SPI(command);
}

static void ssd1306_WriteData(uint8_t* data, uint16_t size) {
    HAL_GPIO_WritePin(SSD1306_DC_PORT, SSD1306_DC_PIN, GPIO_PIN_SET); // Data mode
    for (uint16_t i = 0; i < size; i++) {
        ssd1306_Write_SPI(data[i]);
    }
}

// --- 4. 屏幕初始化函数 ---
void ssd1306_Init(void) {
    // Reset the OLED
    HAL_GPIO_WritePin(SSD1306_RES_PORT, SSD1306_RES_PIN, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(SSD1306_RES_PORT, SSD1306_RES_PIN, GPIO_PIN_SET);
    HAL_Delay(10);

    // Init sequence
    ssd1306_WriteCommand(0xAE); // Display OFF
    ssd1306_WriteCommand(0x20); // Set Memory Addressing Mode
    ssd1306_WriteCommand(0x10); // 00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page Addressing Mode (RESET);11,Invalid
    ssd1306_WriteCommand(0xB0); // Set Page Start Address for Page Addressing Mode,0-7
    ssd1306_WriteCommand(0xC8); // Set COM Output Scan Direction
    ssd1306_WriteCommand(0x00); // Set low column address
    ssd1306_WriteCommand(0x10); // Set high column address
    ssd1306_WriteCommand(0x40); // Set start line address
    ssd1306_WriteCommand(0x81); // set contrast control register
    ssd1306_WriteCommand(0xFF);
    ssd1306_WriteCommand(0xA1); // Set segment re-map 0 to 127
    ssd1306_WriteCommand(0xA6); // Set normal display
    ssd1306_WriteCommand(0xA8); // Set multiplex ratio(1 to 64)
    ssd1306_WriteCommand(0x3F); //
    ssd1306_WriteCommand(0xA4); // 0xa4,Output follows RAM content;0xa5,Output ignores RAM content
    ssd1306_WriteCommand(0xD3); // Set display offset
    ssd1306_WriteCommand(0x00); // Not offset
    ssd1306_WriteCommand(0xD5); // Set display clock divide ratio/oscillator frequency
    ssd1306_WriteCommand(0xF0); // Set divide ratio
    ssd1306_WriteCommand(0xD9); // Set pre-charge period
    ssd1306_WriteCommand(0x22); //
    ssd1306_WriteCommand(0xDA); // Set com pins hardware configuration
    ssd1306_WriteCommand(0x12);
    ssd1306_WriteCommand(0xDB); // Set vcomh
    ssd1306_WriteCommand(0x20); // 0x20,0.77xVcc
    ssd1306_WriteCommand(0x8D); // Set DC-DC enable
    ssd1306_WriteCommand(0x14); //
    ssd1306_WriteCommand(0xAF); // Turn on SSD1306 panel

    // Clear screen
    ssd1306_Fill(0); // 0 for Black, 1 for White
    ssd1306_UpdateScreen();

    SSD1306.Initialized = true;
}

// --- 5. 绘图函数 ---
void ssd1306_Fill(uint8_t color) {
    memset(SSD1306_Buffer, (color == 0) ? 0x00 : 0xFF, sizeof(SSD1306_Buffer));
}

void ssd1306_UpdateScreen(void) {
    for (uint8_t i = 0; i < 8; i++) {
        ssd1306_WriteCommand(0xB0 + i);
        ssd1306_WriteCommand(0x00);
        ssd1306_WriteCommand(0x10);
        ssd1306_WriteData(&SSD1306_Buffer[SSD1306_WIDTH * i], SSD1306_WIDTH);
    }
}

void ssd1306_DrawPixel(uint16_t x, uint16_t y, uint8_t color) {
    if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT) {
        return;
    }
    if (color) {
        SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] |= 1 << (y % 8);
    } else {
        SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] &= ~(1 << (y % 8));
    }
}

void ssd1306_SetCursor(uint8_t x, uint8_t y) {
    SSD1306.CurrentX = x;
    SSD1306.CurrentY = y;
}

char ssd1306_WriteString(char* str, SSD1306_Font_t Font, uint8_t color) {
    while (*str) {
        if (ssd1306_WriteChar(*str, Font, color) != *str) {
            return *str;
        }
        str++;
    }
    return *str;
}

// You need to add ssd1306_WriteChar and font data for this to work
// The font data is provided in `ssd1306_fonts.c` (compiled separately).
// Do NOT include the .c file here — include the header only (already done at top).

char ssd1306_WriteChar(char ch, SSD1306_Font_t Font, uint8_t color) {
    uint32_t i, b, j;

    // 检查边界
    if (SSD1306_WIDTH <= (SSD1306.CurrentX + Font.width) ||
        SSD1306_HEIGHT <= (SSD1306.CurrentY + Font.height)) {
        return 0;
    }

    // 从字体数据中找到字符
    for (i = 0; i < Font.height; i++) {
        b = Font.data[(ch - 32) * Font.height + i];
        for (j = 0; j < Font.width; j++) {
            if ((b << j) & 0x8000) {
                ssd1306_DrawPixel(SSD1306.CurrentX + j, (SSD1306.CurrentY + i), (uint8_t) color);
            } else {
                ssd1306_DrawPixel(SSD1306.CurrentX + j, (SSD1306.CurrentY + i), (uint8_t)!color);
            }
        }
    }
    
    // 更新光标位置
    SSD1306.CurrentX += Font.width;
    return ch;
}

/**
 * @brief  在指定位置绘制位图 (通用版本)
 * @param  bitmap: 指向位图数据的指针 (水平扫描, MSB first)
 * @param  x, y:   图像左上角的起始坐标
 * @param  width, height: 图像的宽度和高度
 * @retval None
 */
void ssd1306_DrawBitmap(const uint8_t* bitmap, uint16_t x, uint16_t y, uint16_t width, uint16_t height) {
    int byte_width = (width + 7) / 8;
    uint8_t byte = 0;

    // 遍历图像的每一行
    for (uint16_t j = 0; j < height; j++, y++) {
        // 遍历图像的每一列
        for (uint16_t i = 0; i < width; i++) {
            // 每8个像素，从数组中读取下一个字节
            if (i % 8 == 0) {
                byte = bitmap[j * byte_width + i / 8];
            }
            
            // 检查当前像素位是否为1
            if (byte & 0x80) {
                // 如果是1，则在屏幕的 (x+i, y) 位置画一个点
                ssd1306_DrawPixel(x + i, y, 1);
            }
            
            // 将字节左移一位，准备检查下一个像素
            byte <<= 1;
        }
    }
}

// void Update_Display(const char* line1, const char* line2, const char* line3)
// {
//     char buffer[32]; // 用于格式化字符串的缓冲区

//     ssd1306_Fill(0); // 清屏 (黑色)

//     // 显示第一行 (测试标题)
//     ssd1306_SetCursor(0, 0);
//     ssd1306_WriteString((char*)line1, Font_7x10, 1);

//     // 显示第二行 (参数或状态)
//     if (line2) {
//         ssd1306_SetCursor(0, 12);
//         snprintf(buffer, sizeof(buffer), " %s", line2);
//         ssd1306_WriteString(buffer, Font_7x10, 1);
//     }
    
//     // 显示第三行 (结果)
//     if (line3) {
//         ssd1306_SetCursor(0, 24);
//         snprintf(buffer, sizeof(buffer), " %s", line3);
//         ssd1306_WriteString(buffer, Font_7x10, 1);
//     }
    
//     ssd1306_UpdateScreen(); // 将缓冲区内容刷新到屏幕
// }