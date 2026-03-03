#include "stdio.h"
#include "stdint.h" // 添加此行以支持 uint8_t 类型
#include "main.h"
#include "usart.h"
#include "fire_debugger.h"
#include "user_uart.h" // 添加user_uart头文件
#include "qr.h"

// 重定义fputc函数
// 需要调用 stdio.h头文件，支持库中printf函数
extern UART_HandleTypeDef huart1;
uint8_t usart1_rx_byte;
int fputc(int ch, FILE *f)
{
	HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY); // 挂载到串口1
	return ch;
}
// 重定义fgetc函数
int fgetc(FILE *f)
{
	uint8_t ch;
	HAL_UART_Receive(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
	return ch;
}
// 串口中断回调 - 统一处理所有UART中断
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	// 处理 USART1 - 调试串口
	if (huart->Instance == USART1)
	{
		Debugger_Receive_Handler(usart1_rx_byte);
		// 再次启动任意长度的IT接收，这里我们只关心下一次中断的触发
		HAL_UART_Receive_IT(&huart1, &usart1_rx_byte, 1);
	}
	// 处理 UART5 - 舵机控制串口
	else if (huart->Instance == UART5)
	{
		extern uint8_t rc1;									 // 来自user_uart.c的接收缓存
		extern Usart_DataTypeDef FSUS_Usart; // 来自user_uart.c的串口结构体

		uint8_t ucTemp = rc1;
		RingBuffer_Push(FSUS_Usart.recvBuf, ucTemp); // 接收到数据放入缓冲区，不在中断具体处理数据
		HAL_UART_Receive_IT(FSUS_Usart.huartX, (uint8_t *)&rc1, 1);
	}
	// 处理 USART6 - OpenMV串口
	else if (huart->Instance == USART6)
	{
		extern void omv_uart_rx_callback(UART_HandleTypeDef *huart);
		omv_uart_rx_callback(huart);
	}
}
// 2. 发送完成回调 (必须添加)
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART6)
  {
    // 调用朋友库中的发送完成处理函数
    omv_uart_tx_callback(huart);
  }
}
// 3. 错误回调 (强烈建议添加)
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART6)
  {
    // 调用朋友库中的错误处理函数
    omv_uart_error_callback(huart);
  }
}