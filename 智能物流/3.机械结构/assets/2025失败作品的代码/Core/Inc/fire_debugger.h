#ifndef __FIRE_DEBUGGER_H
#define __FIRE_DEBUGGER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h" // 根据您的MCU系列修改

/* Public function prototypes ------------------------------------------------*/

/**
 * @brief 初始化串口调试助手
 * @param huart 指向用于调试的UART句柄
 * @note  在主函数初始化部分调用一次
 */
void Debugger_Init(UART_HandleTypeDef *huart);

/**
 * @brief 串口调试助手的数据解析任务
 * @note  必须在主函数的 while(1) 循环中持续调用
 */
void Debugger_Task(void);

/**
 * @brief 通过调试助手向上位机发送波形数据
 * @param ch1 通道1的数据
 * @param ch2 通道2的数据
 * @param ch3 通道3的数据
 * @param ch4 通道4的数据
 */
void Debugger_Send_Wave(float ch1, float ch2, float ch3, float ch4);

/**
 * @brief UART接收中断回调函数中调用的数据处理函数
 * @param RxData 从串口接收到的单字节数据
 */
void Debugger_Receive_Handler(uint8_t RxData);

#ifdef __cplusplus
}
#endif

#endif /* __FIRE_DEBUGGER_H */