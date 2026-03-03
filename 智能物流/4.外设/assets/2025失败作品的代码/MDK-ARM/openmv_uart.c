/*
 * openmv_uart.c
 *
 * Modified on: Oct 7, 2025
 * Author: OpenMV RPC Implementation
 *
 * Description: OpenMV RPC Master Interface Implementation for STM32
 * 描述: STM32端的OpenMV RPC主机接口实现
 * Based on official OpenMV RPC protocol: https://docs.openmv.io/library/omv.rpc.html
 *
 * 功能说明:
 * 1. 通过UART与OpenMV摄像头进行RPC通信
 * 2. 支持色块检测、边缘检测、危险区域检测等视觉功能
 * 3. 提供简单的API接口供上层应用调用
 * 4. 自动处理RPC协议的封装和解析
 */

#include "openmv_uart.h"
#include <string.h>
#include <stdlib.h>
#include <stdint.h> // 添加此头文件支持uint32_t等类型
#include <stdio.h>  // 添加sprintf支持
#include "ssd1306.h"
#include "ssd1306_fonts.h"

// --- 全局数据 ---
volatile omv_vision_data_t g_omv_vision_data; // 全局视觉数据结构，存储最新的检测结果

// --- 私有变量 ---

// RPC状态机枚举 - 用于管理通信状态
typedef enum {
    RPC_STATE_IDLE,             // 空闲状态
    RPC_STATE_WAITING_RESPONSE, // 等待响应状态
    RPC_STATE_RECEIVING_HEADER, // 接收头部数据状态
    RPC_STATE_RECEIVING_PAYLOAD // 接收负载数据状态
} rpc_state_t;

static volatile rpc_state_t rpc_state = RPC_STATE_IDLE;                          // 当前RPC状态
static volatile uint8_t rx_byte;                                                 // 接收缓冲字节
static uint8_t rx_buffer[OMV_RPC_MAX_PAYLOAD_SIZE + OMV_RPC_RESULT_HEADER_SIZE]; // 接收数据缓冲区
static volatile uint32_t rx_index       = 0;                                     // 接收缓冲区索引
static volatile uint32_t expected_bytes = 0;                                     // 期望接收的字节数
static volatile bool response_ready     = false;                                 // 响应就绪标志
static volatile bool response_error     = false;                                 // 响应错误标志

// 发送状态
static volatile bool tx_busy = false; // 发送忙碌标志

// --- 私有函数原型声明 ---
static bool rpc_send_packet(const uint8_t *data, uint32_t size); // 发送RPC数据包
static bool rpc_wait_response(uint32_t timeout_ms);              // 等待RPC响应
static void rpc_reset_receiver(void);                            // 重置接收器状态
static uint32_t get_tick_ms(void);                               // 获取系统毫秒时钟

// --- 公共函数实现 ---

/**
 * @brief 初始化OpenMV RPC通信接口
 * @note 必须在MX_USART6_UART_Init()之后调用
 */

void omv_rpc_init(void)
{
    // 初始化全局数据结构，清零所有字段
    memset((void *)&g_omv_vision_data, 0, sizeof(omv_vision_data_t));

    // 重置RPC状态机到初始状态
    rpc_reset_receiver();

    // 启动UART中断接收，开始监听OpenMV的数据
    HAL_UART_Receive_IT(OMV_UART_HANDLE, (uint8_t *)&rx_byte, 1);
}

/**
 * @brief 执行远程函数调用到OpenMV
 * @param function_name 要调用的函数名称
 * @param args 参数数据（可以为NULL）
 * @param args_size 参数数据大小（字节）
 * @param result 存储结果数据的缓冲区（可以为NULL）
 * @param result_size 结果缓冲区的最大大小
 * @param actual_size 实际接收到的结果数据大小
 * @return true 调用成功，false 调用失败
 */
bool omv_rpc_call(const char *function_name,
                  const uint8_t *args, uint32_t args_size,
                  uint8_t *result, uint32_t result_size,
                  uint32_t *actual_size)
{
    // 参数检查：函数名不能为空，且RPC必须处于空闲状态
    if (!function_name || rpc_state != RPC_STATE_IDLE) {
        return false;
    }

    // 计算数据包大小
    uint32_t name_len           = strlen(function_name); // 函数名长度
    uint32_t total_payload_size = OMV_RPC_COMMAND_HEADER_SIZE - OMV_RPC_HEADER_SIZE + name_len + args_size;

    // 检查数据包大小是否超出缓冲区限制
    if (total_payload_size > OMV_RPC_MAX_PAYLOAD_SIZE) {
        return false;
    }

    // 构建RPC命令数据包
    static uint8_t tx_buffer[OMV_RPC_MAX_PAYLOAD_SIZE + OMV_RPC_COMMAND_HEADER_SIZE];
    omv_rpc_command_t *cmd = (omv_rpc_command_t *)tx_buffer;

    // 填充RPC头部信息
    cmd->header.magic        = OMV_RPC_HEADER_MAGIC; // 魔数标识
    cmd->header.payload_size = total_payload_size;   // 负载大小
    cmd->command             = OMV_RPC_CMD_CALL;     // 命令类型：调用
    cmd->name_size           = name_len;             // 函数名长度
    cmd->args_size           = args_size;            // 参数长度

    // 复制函数名到数据包
    uint8_t *payload_ptr = tx_buffer + OMV_RPC_COMMAND_HEADER_SIZE;
    memcpy(payload_ptr, function_name, name_len);
    payload_ptr += name_len;

    // 复制参数到数据包（如果有参数）
    if (args && args_size > 0) {
        memcpy(payload_ptr, args, args_size);
    }

    // 发送数据包
    rpc_state      = RPC_STATE_WAITING_RESPONSE; // 切换到等待响应状态
    response_ready = false;                      // 清除响应就绪标志
    response_error = false;                      // 清除错误标志

    // 通过UART发送数据包
    if (!rpc_send_packet(tx_buffer, OMV_RPC_COMMAND_HEADER_SIZE + total_payload_size)) {
        rpc_state = RPC_STATE_IDLE;
        return false;
    }

    // 等待OpenMV响应
    if (!rpc_wait_response(OMV_RPC_RECV_TIMEOUT)) {
        rpc_state = RPC_STATE_IDLE;
        return false;
    }

    // 处理响应结果
    if (response_error) { // 检查是否有通信错误
        rpc_state = RPC_STATE_IDLE;
        return false;
    }

    // 解析响应数据包
    omv_rpc_result_t *response = (omv_rpc_result_t *)rx_buffer;
    if (response->header.magic != OMV_RPC_HEADER_MAGIC || // 验证魔数
        response->result != OMV_RPC_RESULT_OK) {          // 验证结果状态
        rpc_state = RPC_STATE_IDLE;
        return false;
    }

    // 复制结果数据到输出缓冲区
    if (result && result_size > 0 && response->data_size > 0) {
        uint32_t copy_size = (response->data_size < result_size) ? response->data_size : result_size;
        memcpy(result, rx_buffer + OMV_RPC_RESULT_HEADER_SIZE, copy_size);
        if (actual_size) {
            *actual_size = copy_size; // 返回实际复制的字节数
        }
    } else if (actual_size) {
        *actual_size = 0; // 没有数据时返回0
    }

    rpc_state = RPC_STATE_IDLE; // 恢复到空闲状态
    return true;                // 调用成功
}

/**
 * @brief 获取指定颜色的最大色块
 * @param color 颜色标识符（"red", "green", "blue"）
 * @param cx 存储色块中心X坐标的指针
 * @param cy 存储色块中心Y坐标的指针
 * @param area 存储色块面积的指针
 * @return true 找到色块，false 未找到
 */
bool omv_get_largest_blob(const char *color, float *cx, float *cy, float *area)
{
    // 参数有效性检查
    if (!color || !cx || !cy || !area) {
        return false;
    }

    // 准备调用参数：颜色字符串
    uint32_t actual_size = 0;
    uint8_t result_buffer[12]; // 3个float = 12字节

    // 调用OpenMV的色块检测函数
    bool success = omv_rpc_call("find_largest_blob_fast",
                                (uint8_t *)color, strlen(color),
                                result_buffer, sizeof(result_buffer),
                                &actual_size);

    // 解析返回结果
    if (success && actual_size >= 12) {
        // 从结果缓冲区解包float数据
        memcpy(cx, result_buffer, 4);       // 中心X坐标
        memcpy(cy, result_buffer + 4, 4);   // 中心Y坐标
        memcpy(area, result_buffer + 8, 4); // 面积

        // 更新全局数据结构
        g_omv_vision_data.blob_cx   = *cx;
        g_omv_vision_data.blob_cy   = *cy;
        g_omv_vision_data.blob_area = *area;
        g_omv_vision_data.new_data  = true; // 标记有新数据
        return true;
    }

    return false;
}

/**
 * @brief 获取对齐角度（边缘检测）
 * @param angle 存储角度值的指针（度）
 * @return true 检测到角度，false 未检测到
 */
bool omv_get_align_angle(float *angle)
{
    if (!angle) {
        return false;
    }

    uint32_t actual_size = 0;
    uint8_t result_buffer[4]; // 1个float = 4字节

    // 调用OpenMV的边缘角度检测函数（当前已禁用）
    bool success = omv_rpc_call("fast_edge_angle",
                                NULL, 0,
                                result_buffer, sizeof(result_buffer),
                                &actual_size);

    if (success && actual_size >= 4) {
        memcpy(angle, result_buffer, 4);
        g_omv_vision_data.align_angle = *angle;
        g_omv_vision_data.new_data    = true;
        return true;
    }

    return false;
}

/**
 * @brief 获取画面中心坐标
 * @param cx 存储中心X坐标的指针
 * @param cy 存储中心Y坐标的指针
 * @return true 获取成功，false 获取失败
 */
bool omv_get_frame_center(float *cx, float *cy)
{
    if (!cx || !cy) {
        return false;
    }

    uint32_t actual_size = 0;
    uint8_t result_buffer[8]; // 2个float = 8字节

    // 调用OpenMV的画面中心获取函数
    bool success = omv_rpc_call("get_frame_center",
                                NULL, 0,
                                result_buffer, sizeof(result_buffer),
                                &actual_size);

    if (success && actual_size >= 8) {
        memcpy(cx, result_buffer, 4);     // 中心X坐标
        memcpy(cy, result_buffer + 4, 4); // 中心Y坐标

        g_omv_vision_data.frame_cx = *cx;
        g_omv_vision_data.frame_cy = *cy;
        g_omv_vision_data.new_data = true;
        return true;
    }

    return false;
}

bool omv_check_danger(float *cx, float *cy, float *area)
{
    if (!cx || !cy || !area) {
        return false;
    }

    uint32_t actual_size = 0;
    uint8_t result_buffer[12]; // 3 floats = 12 bytes

    bool success = omv_rpc_call("detect_danger_yellow",
                                NULL, 0,
                                result_buffer, sizeof(result_buffer),
                                &actual_size);

    if (success && actual_size >= 12) {
        memcpy(cx, result_buffer, 4);
        memcpy(cy, result_buffer + 4, 4);
        memcpy(area, result_buffer + 8, 4);

        g_omv_vision_data.new_data = true;
        return true;
    }

    return false;
}

/**
 * @brief 清除新数据标志
 * @note 调用此函数后，new_data和error标志将被清零
 */
void omv_clear_data_flag(void)
{
    g_omv_vision_data.new_data = false;
    g_omv_vision_data.error    = false;
}

/**
 * @brief 向RPC解析器输入接收到的字节（用于中断处理）
 * @param byte 从UART接收到的字节
 * @note 此函数在UART接收中断中被调用，用于解析RPC协议
 */
void omv_rpc_feed_byte(uint8_t byte)
{
    // 如果RPC处于空闲状态，忽略接收到的数据
    if (rpc_state == RPC_STATE_IDLE) {
        return;
    }

    // 如果正在等待响应，开始接收头部数据
    if (rpc_state == RPC_STATE_WAITING_RESPONSE) {
        rpc_state      = RPC_STATE_RECEIVING_HEADER; // 切换到接收头部状态
        rx_index       = 0;                          // 重置接收索引
        expected_bytes = OMV_RPC_RESULT_HEADER_SIZE; // 设置期望的头部字节数
    }

    // 接收头部或负载数据
    if (rpc_state == RPC_STATE_RECEIVING_HEADER || rpc_state == RPC_STATE_RECEIVING_PAYLOAD) {
        rx_buffer[rx_index++] = byte; // 存储接收到的字节

        // 检查是否接收完当前阶段的数据
        if (rx_index >= expected_bytes) {
            if (rpc_state == RPC_STATE_RECEIVING_HEADER) {
                // 头部接收完成，检查是否需要接收负载数据
                omv_rpc_result_t *result = (omv_rpc_result_t *)rx_buffer;
                if (result->data_size > 0 && result->data_size <= OMV_RPC_MAX_PAYLOAD_SIZE) {
                    // 继续接收负载数据
                    rpc_state = RPC_STATE_RECEIVING_PAYLOAD;
                    expected_bytes += result->data_size;
                } else {
                    // 没有负载数据或数据大小无效
                    response_ready = true; // 标记响应已就绪
                }
            } else {
                // 负载数据接收完成
                response_ready = true; // 标记响应已就绪
            }
        }
    }
}

// --- Private Function Implementations ---

static bool rpc_send_packet(const uint8_t *data, uint32_t size)
{
    if (tx_busy || !data || size == 0) {
        return false;
    }

    tx_busy                  = true;
    HAL_StatusTypeDef status = HAL_UART_Transmit_IT(OMV_UART_HANDLE, (uint8_t *)data, size);

    if (status != HAL_OK) {
        tx_busy = false;
        return false;
    }

    // Wait for transmission to complete
    uint32_t start_time = get_tick_ms();
    while (tx_busy && (get_tick_ms() - start_time) < OMV_RPC_SEND_TIMEOUT) {
        // Wait
    }

    return !tx_busy;
}

static bool rpc_wait_response(uint32_t timeout_ms)
{
    uint32_t start_time = get_tick_ms();

    while (!response_ready && !response_error &&
           (get_tick_ms() - start_time) < timeout_ms) {
        // Wait for response
    }

    return response_ready;
}

static void rpc_reset_receiver(void)
{
    rpc_state      = RPC_STATE_IDLE;
    rx_index       = 0;
    expected_bytes = 0;
    response_ready = false;
    response_error = false;
}

static uint32_t get_tick_ms(void)
{
    return HAL_GetTick();
}

// --- HAL Callback Functions ---

// OpenMV UART Receive Handler (called from main UART callback)
void omv_uart_rx_callback(UART_HandleTypeDef *huart)
{
    if (huart == OMV_UART_HANDLE) {
        omv_rpc_feed_byte(rx_byte);
        // Re-arm reception
        HAL_UART_Receive_IT(OMV_UART_HANDLE, (uint8_t *)&rx_byte, 1);
    }
}

// OpenMV UART Transmit Complete Callback
void omv_uart_tx_callback(UART_HandleTypeDef *huart)
{
    if (huart == OMV_UART_HANDLE) {
        tx_busy = false;
    }
}

// OpenMV UART Error Callback
void omv_uart_error_callback(UART_HandleTypeDef *huart)
{
    if (huart == OMV_UART_HANDLE) {
        tx_busy        = false;
        response_error = true;
        rpc_reset_receiver();
        // Re-arm reception
        HAL_UART_Receive_IT(OMV_UART_HANDLE, (uint8_t *)&rx_byte, 1);
    }
}

// --- OpenMV Test Functions ---

/**
 * @brief 测试OpenMV连接状态
 * @note 通过发送简单的RPC调用来测试与OpenMV的通信是否正常
 * @return true 连接正常，false 连接失败
 */
bool omv_test_connection(void)
{
    extern const SSD1306_Font_t Font_7x10;
    uint32_t actual_size = 0;
    uint8_t test_result[4];
    char buffer[20];

    // 显示测试状态
    ssd1306_Fill(0);
    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString("OpenMV Test", Font_7x10, 1);
    ssd1306_SetCursor(0, 15);
    ssd1306_WriteString("Connection...", Font_7x10, 1);
    ssd1306_UpdateScreen();
    HAL_Delay(1000);

    // 先尝试简单的RPC调用
    bool success = omv_rpc_call("test_connection",
                                NULL, 0,
                                test_result, sizeof(test_result),
                                &actual_size);

    // 显示详细结果
    ssd1306_Fill(0);
    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString("OpenMV Connect", Font_7x10, 1);

    if (success) {
        ssd1306_SetCursor(0, 15);
        ssd1306_WriteString("Status: OK", Font_7x10, 1);
        ssd1306_SetCursor(0, 30);
        snprintf(buffer, sizeof(buffer), "Size: %u", actual_size);
        ssd1306_WriteString(buffer, Font_7x10, 1);
    } else {
        ssd1306_SetCursor(0, 15);
        ssd1306_WriteString("Status: FAIL", Font_7x10, 1);
        ssd1306_SetCursor(0, 30);
        snprintf(buffer, sizeof(buffer), "State: %d", (int)rpc_state);
        ssd1306_WriteString(buffer, Font_7x10, 1);
        ssd1306_SetCursor(0, 45);
        snprintf(buffer, sizeof(buffer), "Ready: %d", response_ready ? 1 : 0);
        ssd1306_WriteString(buffer, Font_7x10, 1);
    }
    ssd1306_UpdateScreen();
    HAL_Delay(3000);

    return success;
} /**
   * @brief 测试OpenMV色块检测功能
   * @note 测试红色色块检测，显示检测结果
   */
void omv_test_blob_detection(void)
{
    extern const SSD1306_Font_t Font_7x10;
    float cx, cy, area;
    char buffer[20];

    // 显示测试状态
    ssd1306_Fill(0);
    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString("OpenMV Test", Font_7x10, 1);
    ssd1306_SetCursor(0, 15);
    ssd1306_WriteString("Blob Detect...", Font_7x10, 1);
    ssd1306_UpdateScreen();

    if (omv_get_largest_blob("red", &cx, &cy, &area)) {
        ssd1306_SetCursor(0, 30);
        ssd1306_WriteString("Red Found!", Font_7x10, 1);
        ssd1306_SetCursor(0, 45);
        snprintf(buffer, sizeof(buffer), "X:%.0f Y:%.0f", cx, cy);
        ssd1306_WriteString(buffer, Font_7x10, 1);
        ssd1306_SetCursor(0, 60);
        snprintf(buffer, sizeof(buffer), "Area:%.0f", area);
        ssd1306_WriteString(buffer, Font_7x10, 1);
    } else {
        ssd1306_SetCursor(0, 30);
        ssd1306_WriteString("No Red Blob", Font_7x10, 1);
        ssd1306_SetCursor(0, 45);
        ssd1306_WriteString("or Comm Fail", Font_7x10, 1);
    }
    ssd1306_UpdateScreen();
    HAL_Delay(3000);
}

/**
 * @brief 测试OpenMV对齐角度检测功能
 * @note 测试边缘检测和角度计算
 */
void omv_test_align_detection(void)
{
    extern const SSD1306_Font_t Font_7x10;
    float angle;
    char buffer[20];

    // 显示测试状态
    ssd1306_Fill(0);
    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString("OpenMV Test", Font_7x10, 1);
    ssd1306_SetCursor(0, 15);
    ssd1306_WriteString("Align Detect...", Font_7x10, 1);
    ssd1306_UpdateScreen();

    if (omv_get_align_angle(&angle)) {
        ssd1306_SetCursor(0, 30);
        ssd1306_WriteString("Angle Found!", Font_7x10, 1);
        ssd1306_SetCursor(0, 45);
        snprintf(buffer, sizeof(buffer), "Angle: %.1f deg", angle);
        ssd1306_WriteString(buffer, Font_7x10, 1);
    } else {
        ssd1306_SetCursor(0, 30);
        ssd1306_WriteString("No Angle", Font_7x10, 1);
        ssd1306_SetCursor(0, 45);
        ssd1306_WriteString("or Comm Fail", Font_7x10, 1);
    }
    ssd1306_UpdateScreen();
    HAL_Delay(3000);
}

/**
 * @brief 综合测试OpenMV所有功能
 * @note 依次测试连接、色块检测、角度检测等功能
 */
void omv_test_all_functions(void)
{
    extern const SSD1306_Font_t Font_7x10;
    char buffer[20];

    // 显示测试开始
    ssd1306_Fill(0);
    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString("OpenMV Test", Font_7x10, 1);
    ssd1306_SetCursor(0, 15);
    ssd1306_WriteString("Starting...", Font_7x10, 1);
    ssd1306_UpdateScreen();
    HAL_Delay(1000);

    // 测试1: 连接测试
    if (omv_test_connection()) {
        // 连接成功，继续其他测试
    } else {
        // 连接失败就不继续测试了
        ssd1306_Fill(0);
        ssd1306_SetCursor(0, 0);
        ssd1306_WriteString("OpenMV Test", Font_7x10, 1);
        ssd1306_SetCursor(0, 15);
        ssd1306_WriteString("Connection", Font_7x10, 1);
        ssd1306_SetCursor(0, 30);
        ssd1306_WriteString("FAILED!", Font_7x10, 1);
        ssd1306_SetCursor(0, 45);
        ssd1306_WriteString("Test Stopped", Font_7x10, 1);
        ssd1306_UpdateScreen();
        HAL_Delay(3000);
        return;
    }

    // 测试2: 色块检测测试
    omv_test_blob_detection();

    // 测试3: 对齐角度检测测试
    omv_test_align_detection();

    // 测试4: 危险检测测试
    ssd1306_Fill(0);
    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString("OpenMV Test", Font_7x10, 1);
    ssd1306_SetCursor(0, 15);
    ssd1306_WriteString("Danger Test...", Font_7x10, 1);
    ssd1306_UpdateScreen();

    float danger_cx, danger_cy, danger_area;
    if (omv_check_danger(&danger_cx, &danger_cy, &danger_area)) {
        ssd1306_SetCursor(0, 30);
        ssd1306_WriteString("Danger Found!", Font_7x10, 1);
        ssd1306_SetCursor(0, 45);
        snprintf(buffer, sizeof(buffer), "X:%.0f Y:%.0f", danger_cx, danger_cy);
        ssd1306_WriteString(buffer, Font_7x10, 1);
    } else {
        ssd1306_SetCursor(0, 30);
        ssd1306_WriteString("No Danger", Font_7x10, 1);
    }
    ssd1306_UpdateScreen();
    HAL_Delay(3000);

    // 测试5: 画面中心测试
    ssd1306_Fill(0);
    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString("OpenMV Test", Font_7x10, 1);
    ssd1306_SetCursor(0, 15);
    ssd1306_WriteString("Frame Center...", Font_7x10, 1);
    ssd1306_UpdateScreen();

    float frame_cx, frame_cy;
    if (omv_get_frame_center(&frame_cx, &frame_cy)) {
        ssd1306_SetCursor(0, 30);
        ssd1306_WriteString("Center Found!", Font_7x10, 1);
        ssd1306_SetCursor(0, 45);
        snprintf(buffer, sizeof(buffer), "X:%.0f Y:%.0f", frame_cx, frame_cy);
        ssd1306_WriteString(buffer, Font_7x10, 1);
    } else {
        ssd1306_SetCursor(0, 30);
        ssd1306_WriteString("Center Fail", Font_7x10, 1);
    }
    ssd1306_UpdateScreen();
    HAL_Delay(3000);

    // 显示测试完成
    ssd1306_Fill(0);
    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString("OpenMV Test", Font_7x10, 1);
    ssd1306_SetCursor(0, 15);
    ssd1306_WriteString("All Tests", Font_7x10, 1);
    ssd1306_SetCursor(0, 30);
    ssd1306_WriteString("COMPLETE!", Font_7x10, 1);
    ssd1306_UpdateScreen();
    HAL_Delay(2000);
}

/**
 * @brief 简单的UART通信测试
 * @note 测试基本的UART发送和接收功能
 */
void omv_test_uart_basic(void)
{
    extern const SSD1306_Font_t Font_7x10;
    char buffer[20];

    ssd1306_Fill(0);
    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString("UART Test", Font_7x10, 1);
    ssd1306_SetCursor(0, 15);
    ssd1306_WriteString("Sending...", Font_7x10, 1);
    ssd1306_UpdateScreen();

    // 发送简单的测试字符串
    const char *test_msg     = "Hello OpenMV\r\n";
    HAL_StatusTypeDef status = HAL_UART_Transmit(OMV_UART_HANDLE,
                                                 (uint8_t *)test_msg,
                                                 strlen(test_msg),
                                                 1000);

    ssd1306_SetCursor(0, 30);
    if (status == HAL_OK) {
        ssd1306_WriteString("TX: OK", Font_7x10, 1);
    } else {
        snprintf(buffer, sizeof(buffer), "TX: ERR %d", status);
        ssd1306_WriteString(buffer, Font_7x10, 1);
    }

    ssd1306_SetCursor(0, 45);
    snprintf(buffer, sizeof(buffer), "RX Ready: %d", response_ready ? 1 : 0);
    ssd1306_WriteString(buffer, Font_7x10, 1);

    ssd1306_UpdateScreen();
    HAL_Delay(3000);
}

// --- Convenience Functions for Specific Colors ---

/**
 * @brief Find the largest red blob
 */
bool omv_find_red_blob(float *cx, float *cy, float *area)
{
    if (!cx || !cy || !area) return false;
    return omv_get_largest_blob("red", cx, cy, area);
}

/**
 * @brief Find the largest green blob
 */
bool omv_find_green_blob(float *cx, float *cy, float *area)
{
    if (!cx || !cy || !area) return false;
    return omv_get_largest_blob("green", cx, cy, area);
}

/**
 * @brief Find the largest blue blob
 */
bool omv_find_blue_blob(float *cx, float *cy, float *area)
{
    if (!cx || !cy || !area) return false;
    return omv_get_largest_blob("blue", cx, cy, area);
}

/**
 * @brief Detect yellow danger zones
 */
bool omv_detect_yellow_danger(float *cx, float *cy, float *area)
{
    if (!cx || !cy || !area) return false;
    return omv_check_danger(cx, cy, area);
}