#include "hwt101.h"
#include <string.h> // for memset
#include <stdio.h>
#include <math.h>


// 定义全局变量存储陀螺仪数据
HWT101_Data_t g_hwt101_data;

// 定义UART句柄指针，在Init函数中初始化
static UART_HandleTypeDef* hwt101_huart;

// 接收缓冲区
#define RX_BUFFER_SIZE 64 // 缓冲区大小，足以容纳多个数据包
static uint8_t rx_buffer[RX_BUFFER_SIZE];

static float g_angle_z_offset_deg = 0.0f;
static float g_latest_raw_angle_deg = 0.0f;

static float normalize_angle_deg(float angle)
{
    while (angle > 180.0f)
    {
        angle -= 360.0f;
    }
    while (angle < -180.0f)
    {
        angle += 360.0f;
    }
    return angle;
}

/**
 * @brief  初始化HWT101，开始DMA+IDLE接收
 * @param  huart: 指向UART句柄的指针
 * @retval None
 */
void HWT101_Init(UART_HandleTypeDef *huart)
{
    hwt101_huart = huart;

    // 1. 将本地数据结构清零
    memset(&g_hwt101_data, 0, sizeof(HWT101_Data_t));
    g_angle_z_offset_deg = 0.0f;
    g_latest_raw_angle_deg = 0.0f;

    // 2. 定义Z轴角度清零指令
    uint8_t reset_z_axis_cmd[] = {0xFF, 0xAA, 0x76, 0x00, 0x00};

    // 3. 发送指令给陀螺仪
    HAL_UART_Transmit(hwt101_huart, reset_z_axis_cmd, sizeof(reset_z_axis_cmd), 100); // 100ms超时

    // 4. 等待一小段时间，让传感器处理指令
    HAL_Delay(20);

    // 5. 使用HAL库提供的专用函数启动DMA+IDLE接收
    HAL_UARTEx_ReceiveToIdle_DMA(hwt101_huart, rx_buffer, RX_BUFFER_SIZE);
}

/**
 * @brief  解析接收到的HWT101数据
 * @param  pData: 指向接收数据缓冲区的指针
 * @param  Size: 数据长度
 * @retval None
 */
void HWT101_ParseData(const uint8_t* pData, uint16_t Size)
{
    
    for (uint16_t i = 0; i < Size; ++i)
    {
        // 1. 寻找包头 0x55
        if (pData[i] == 0x55)
        {
            // 确保这是一个完整包的开始
            if (i + 11 > Size)
            {
                continue; // 剩余数据不足11字节，不是完整包
            }

            // 2. 检查校验和
            uint8_t sum = 0;
            for (int j = 0; j < 10; ++j)
            {
                sum += pData[i + j];
            }

            if (sum == pData[i + 10]) // 校验通过
            {
                // 3. 判断数据包类型并解析
                int16_t data_raw;
                switch (pData[i + 1])
                {
                    case 0x52: // 角速度包
                        data_raw = (int16_t)((pData[i + 5] << 8) | pData[i + 4]);
                        g_hwt101_data.AngVel_Y = (float)data_raw / 32768.0f * 2000.0f;

                        data_raw = (int16_t)((pData[i + 7] << 8) | pData[i + 6]);
                        g_hwt101_data.AngVel_Z = (float)data_raw / 32768.0f * 2000.0f;
                        break;
                
                    case 0x53: // 角度包
                        data_raw = (int16_t)((pData[i + 7] << 8) | pData[i + 6]);
                        g_latest_raw_angle_deg = (float)data_raw / 32768.0f * 180.0f;
                        g_hwt101_data.Angle_Z = normalize_angle_deg(g_latest_raw_angle_deg - g_angle_z_offset_deg);
                        break;
                }
            }
        }
    }
}

void HWT101_ZeroYaw(void)
{
    g_angle_z_offset_deg = g_latest_raw_angle_deg;
    g_hwt101_data.Angle_Z = 0.0f;
}

void HWT101_SetYawOffset(float offset_deg)
{
    g_angle_z_offset_deg = offset_deg;
    g_hwt101_data.Angle_Z = normalize_angle_deg(g_latest_raw_angle_deg - g_angle_z_offset_deg);
}