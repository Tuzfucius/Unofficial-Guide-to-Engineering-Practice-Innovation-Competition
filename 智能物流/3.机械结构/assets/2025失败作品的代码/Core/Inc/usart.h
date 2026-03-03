
#include "stdio.h"     //为了支持printf
#include "main.h"

#define USART1_LENGTH 8

extern uint8_t usart1_ReadBuffer[8];
//在mian.c中要定义这些外部变量

int fputc(int ch, FILE *f);
int fgetc(FILE *f);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

