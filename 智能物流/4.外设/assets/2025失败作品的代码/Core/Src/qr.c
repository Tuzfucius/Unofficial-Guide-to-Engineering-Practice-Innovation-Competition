#include "qr.h"
#include "usart.h" // CubeMX生成的usart头文件，包含huart2的定义
#include <string.h>

extern UART_HandleTypeDef huart2;
// DMA硬件直接写入的原始缓冲区
static uint8_t rx_dma_buffer[QR_RX_BUFFER_SIZE];
// 用于安全存放一帧完整数据的应用层缓冲区
static uint8_t qr_code_buffer[QR_RX_BUFFER_SIZE];
extern DMA_HandleTypeDef hdma_usart2_rx;

// volatile关键字是必须的，因为这些变量在中断和主循环中都会被访问
// volatile告诉编译器不要对这些变量的读写进行优化
static volatile uint16_t qr_data_length = 0;
static volatile uint8_t new_data_flag = 0;

/**
 * @brief  初始化QR码扫描模块并启动第一次DMA接收
 */
void qr_init(void)
{
    // 启动DMA接收，并使能IDLE中断来检测一帧数据的结束
    // 这行代码只需要在初始化时调用一次，之后会在回调函数中自动重新启动
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_dma_buffer, QR_RX_BUFFER_SIZE);

    __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT); // 建议保留
    __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_TC); // 建议保留
}

/**
 * @brief  检查是否有新的二维码数据
 */
uint8_t qr_has_new_data(void)
{
    return new_data_flag;
}

/**
 * @brief  获取二维码数据
 */
uint16_t qr_get_data(uint8_t *data_buf)
{
    // 检查标志位，确保有新数据
    if (new_data_flag)
    {
        // 将数据从我们的内部缓冲区拷贝到用户提供的缓冲区
        memcpy(data_buf, qr_code_buffer, qr_data_length);
        // 在数据末尾添加字符串结束符，方便按字符串处理
        data_buf[qr_data_length] = '\0';

        uint16_t len = qr_data_length;

        // 清除标志位和长度，为下一次接收做准备
        new_data_flag = 0;
        qr_data_length = 0;

        return len;
    }
    return 0;
}

/**
 * @brief  UART事件回调处理函数（核心逻辑）
 * 在HAL_UARTEx_RxEventCallback中调用此函数
 */
void qr_uart_rx_event_callback(UART_HandleTypeDef *huart, uint16_t size)
{
    // // 判断事件是否来自我们关心的USART2
    // printf("qr_uart_rx_event_callback\r\n");
    // // 确认没有上一次的数据还未被主循环处理
    // if (new_data_flag == 0)
    // {
    //     // 将从DMA缓冲区接收到的有效数据，拷贝到应用层缓冲区
    //     memcpy(qr_code_buffer, rx_dma_buffer, size);
    //     // 记录本帧数据的实际长度
    //     qr_data_length = size;
    //     // 设置新数据标志位，通知主循环前来处理
    //     new_data_flag = 1;
    // }

    // HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_dma_buffer, QR_RX_BUFFER_SIZE);

    // 确认没有上一次的数据还未被主循环处理
    if (new_data_flag == 0)
    {
        // 将从DMA缓冲区接收到的有效数据，拷贝到应用层缓冲区
        memcpy(qr_code_buffer, rx_dma_buffer, size);
        // 记录本帧数据的实际长度
        qr_data_length = size;
        // 设置新数据标志位，通知主循环前来处理
        new_data_flag = 1;
    }

    // 无论数据是否被处理，都必须重新启动DMA+IDLE接收
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_dma_buffer, QR_RX_BUFFER_SIZE);
}

// DMA_HandleTypeDef hdma_usart2_rx;

// if (qr_has_new_data())
// {
//     // 定义一个缓冲区来存放获取到的数据
//     uint8_t qr_data[256];
//     uint16_t len = qr_get_data(qr_data);

//     // 在这里处理你的数据
//     // 例如，通过另一个串口（如USART1）将其打印出来进行调试
//     if (len > 0)
//     {
//         // HAL_UART_Transmit(&huart1, (uint8_t*)"QR Code: ", 9, 0xFFFF);
//         // HAL_UART_Transmit(&huart1, qr_data, len, 0xFFFF);
//         // HAL_UART_Transmit(&huart1, (uint8_t*)"\r\n", 2, 0xFFFF);

//         // 或者如果您重定向了printf
//         printf("QR Code (%d bytes): %s\r\n", len, qr_data);
//     }
// }

// uint8_t qr_data[256];
// uint16_t len = qr_get_data(qr_data);

// if (len > 0)
// {
//     printf("QR Code (%d bytes): %s\r\n", len, qr_data);
// }