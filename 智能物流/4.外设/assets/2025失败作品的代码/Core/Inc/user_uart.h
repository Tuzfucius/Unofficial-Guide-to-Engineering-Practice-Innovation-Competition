#ifndef __USER_UART_H__
#define __USER_UART_H__

#include "ring_buffer.h"
#include "usart.h"

#define USART_RECV_BUF_SIZE 500
#define USART_SEND_BUF_SIZE 500

/* 可配置：FSUS (Fashion Star UART Servo) 接收缓冲区大小
 * 默认值 512 字节。若需要更大或更小，可在编译器命令行或在
 * 修改此宏为所需大小（例如 256/512/1024）。
 */
#ifndef FSUS_RX_BUF_SIZE
#define FSUS_RX_BUF_SIZE 512
#endif

/* 提示：在 `user_uart.c` 中请使用下列示例方式分配并初始化环形缓冲：
 *
 * static uint8_t FSUS_RxBufferStorage[FSUS_RX_BUF_SIZE];
 * static RingBufferTypeDef FSUS_RingBuffer;
 *
 * RingBuffer_Init(&FSUS_RingBuffer, FSUS_RxBufferStorage, FSUS_RX_BUF_SIZE);
 * FSUS_Usart.recvBuf = &FSUS_RingBuffer;
 */


typedef struct
{
    UART_HandleTypeDef *huartX;
    RingBufferTypeDef *sendBuf;
    RingBufferTypeDef *recvBuf;
} Usart_DataTypeDef;

extern Usart_DataTypeDef FSUS_Usart;

void User_Uart_Init(UART_HandleTypeDef *huartx);
void Usart_SendAll(Usart_DataTypeDef *usart);
#endif
