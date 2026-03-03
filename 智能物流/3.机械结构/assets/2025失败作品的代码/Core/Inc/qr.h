#ifndef __QR_H
#define __QR_H

#include "main.h"

// 定义DMA接收缓冲区的最大长度，可以根据二维码可能的最大长度调整
#define QR_RX_BUFFER_SIZE   256

/**
 * @brief  初始化QR码扫描模块的接收逻辑
 * @param  None
 * @retval None
 */
void qr_init(void);

/**
 * @brief  检查是否有新的、完整的二维码数据
 * @param  None
 * @retval 1: 有新数据, 0: 没有新数据
 */
uint8_t qr_has_new_data(void);

/**
 * @brief  获取接收到的二维码数据
 * @note   调用此函数后，新数据标志位会被清除。
 * @param  data_buf: 用于存储二维码数据的用户缓冲区指针
 * @retval 返回接收到的数据长度。如果无新数据，返回0。
 */
uint16_t qr_get_data(uint8_t *data_buf);

/**
 * @brief  内部使用的UART事件回调处理函数
 * @note   此函数应在HAL库的 `HAL_UARTEx_RxEventCallback` 中被调用
 * @param  huart: 触发回调的UART句柄
 * @param  size:  本次DMA接收到的数据字节数
 * @retval None
 */
void qr_uart_rx_event_callback(UART_HandleTypeDef *huart, uint16_t size);

#endif /* __QR_H */