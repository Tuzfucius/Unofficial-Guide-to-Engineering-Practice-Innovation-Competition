#ifndef __HWT101_H
#define __HWT101_H

#include "main.h"

// 定义陀螺仪数据结构体
typedef struct
{
    float AngVel_Y; // Y轴角速度
    float AngVel_Z; // Z轴角速度
    float Angle_Z;  // Z轴角度
} HWT101_Data_t;

// 外部声明一个数据变量，方便全局访问
extern HWT101_Data_t g_hwt101_data;

/**
 * @brief  初始化HWT101，开始DMA+IDLE接收
 * @param  huart: 指向UART句柄的指针
 * @retval None
 */
void HWT101_Init(UART_HandleTypeDef *huart);

/**
 * @brief  解析接收到的HWT101数据
 * @param  pData: 指向接收数据缓冲区的指针
 * @param  Size: 数据长度
 * @retval None
 */
void HWT101_ParseData(const uint8_t* pData, uint16_t Size);

/**
 * @brief  将当前陀螺仪Z轴角度作为零点，后续读数会基于此零点输出。
 */
void HWT101_ZeroYaw(void);

/**
 * @brief  设置陀螺仪Z轴角度的零点偏移（单位：度）。
 * @param  offset_deg: 需要减去的原始角度值
 */
void HWT101_SetYawOffset(float offset_deg);

#endif