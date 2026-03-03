/*
 * openmv_uart.h
 *
 * Modified on: Oct 7, 2025
 * Author: OpenMV RPC Implementation
 *
 * Description: OpenMV RPC Master Interface for STM32 using USART6
 * Based on official OpenMV RPC protocol: https://docs.openmv.io/library/omv.rpc.html
 */

#ifndef __OPENMV_UART_H
#define __OPENMV_UART_H

#include "stdint.h"
#include "stdbool.h"
#include "main.h"          // CubeMX generated main.h, includes HAL drivers
#include "stm32f4xx_hal.h" // HAL drivers
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

// --- OpenMV RPC Protocol Constants ---

// RPC packet structure based on OpenMV official implementation
#define OMV_RPC_HEADER_SIZE         8
#define OMV_RPC_HEADER_MAGIC        0x1209ABD7 // OpenMV RPC magic number
#define OMV_RPC_COMMAND_HEADER_SIZE 12
#define OMV_RPC_RESULT_HEADER_SIZE  8
#define OMV_RPC_MAX_PAYLOAD_SIZE    1024

// RPC Command Types
#define OMV_RPC_CMD_CALL   0x00
#define OMV_RPC_CMD_STREAM 0x01

// RPC Result Types
#define OMV_RPC_RESULT_OK    0x00
#define OMV_RPC_RESULT_ERROR 0x01

// Default timeouts (milliseconds)
#define OMV_RPC_SEND_TIMEOUT 1000
#define OMV_RPC_RECV_TIMEOUT 1000

// --- Data Structures ---

/**
 * @brief RPC packet header structure
 */
typedef struct __attribute__((packed)) {
    uint32_t magic;        // Magic number (OMV_RPC_HEADER_MAGIC)
    uint32_t payload_size; // Size of payload following this header
} omv_rpc_header_t;

/**
 * @brief RPC command packet structure
 */
typedef struct __attribute__((packed)) {
    omv_rpc_header_t header; // Common header
    uint32_t command;        // Command type
    uint32_t name_size;      // Size of function name
    uint32_t args_size;      // Size of arguments
    // Followed by: function_name + arguments
} omv_rpc_command_t;

/**
 * @brief RPC result packet structure
 */
typedef struct __attribute__((packed)) {
    omv_rpc_header_t header; // Common header
    uint32_t result;         // Result type (OK/ERROR)
    uint32_t data_size;      // Size of result data
    // Followed by: result_data
} omv_rpc_result_t;

/**
 * @brief Vision data structure for processed results
 */
typedef struct {
    float blob_cx;          // Blob center X coordinate
    float blob_cy;          // Blob center Y coordinate
    float blob_area;        // Blob area
    float align_angle;      // Alignment angle in degrees
    float frame_cx;         // Frame center X
    float frame_cy;         // Frame center Y
    volatile bool new_data; // New data available flag
    volatile bool error;    // Error flag
} omv_vision_data_t;
// --- Global Variables ---

// Global vision data instance
extern volatile omv_vision_data_t g_omv_vision_data;

// ---------------------------------------------------------------------------
// UART handle selection
// ---------------------------------------------------------------------------

// Declare UART handle symbols
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart6;

#ifndef OMV_UART_HANDLE
#define OMV_UART_HANDLE (&huart6)
#endif

// --- Public Function Prototypes ---

/**
 * @brief Initialize OpenMV RPC UART interface
 * @note Must be called after MX_USART6_UART_Init()
 */
void omv_rpc_init(void);

/**
 * @brief Execute remote function call on OpenMV
 * @param function_name Name of the function to call
 * @param args Arguments data (can be NULL)
 * @param args_size Size of arguments in bytes
 * @param result Buffer to store result data (can be NULL)
 * @param result_size Maximum size of result buffer
 * @param actual_size Actual size of received result data
 * @return true if call successful, false otherwise
 */
bool omv_rpc_call(const char *function_name,
                  const uint8_t *args, uint32_t args_size,
                  uint8_t *result, uint32_t result_size,
                  uint32_t *actual_size);

/**
 * @brief Get largest blob of specified color
 * @param color Color identifier ("red", "green", "blue")
 * @param cx Pointer to store center X coordinate
 * @param cy Pointer to store center Y coordinate
 * @param area Pointer to store blob area
 * @return true if blob found, false otherwise
 */
bool omv_get_largest_blob(const char *color, float *cx, float *cy, float *area);

/**
 * @brief Get alignment angle from edge detection
 * @param angle Pointer to store angle in degrees
 * @return true if angle detected, false otherwise
 */
bool omv_get_align_angle(float *angle);

/**
 * @brief Get frame center coordinates
 * @param cx Pointer to store center X coordinate
 * @param cy Pointer to store center Y coordinate
 * @return true if successful, false otherwise
 */
bool omv_get_frame_center(float *cx, float *cy);

/**
 * @brief Check for yellow danger detection
 * @param cx Pointer to store danger center X coordinate
 * @param cy Pointer to store danger center Y coordinate
 * @param area Pointer to store danger area
 * @return true if danger detected, false otherwise
 */
bool omv_check_danger(float *cx, float *cy, float *area);

// --- Convenience Functions for Specific Colors ---

/**
 * @brief Find the largest red blob
 * @param cx Pointer to store center X coordinate
 * @param cy Pointer to store center Y coordinate
 * @param area Pointer to store blob area
 * @return true if red blob found, false otherwise
 */
bool omv_find_red_blob(float *cx, float *cy, float *area);

/**
 * @brief Find the largest green blob
 * @param cx Pointer to store center X coordinate
 * @param cy Pointer to store center Y coordinate
 * @param area Pointer to store blob area
 * @return true if green blob found, false otherwise
 */
bool omv_find_green_blob(float *cx, float *cy, float *area);

/**
 * @brief Find the largest blue blob
 * @param cx Pointer to store center X coordinate
 * @param cy Pointer to store center Y coordinate
 * @param area Pointer to store blob area
 * @return true if blue blob found, false otherwise
 */
bool omv_find_blue_blob(float *cx, float *cy, float *area);

/**
 * @brief Detect yellow danger zones
 * @param cx Pointer to store danger center X coordinate
 * @param cy Pointer to store danger center Y coordinate
 * @param area Pointer to store danger area
 * @return true if yellow danger detected, false otherwise
 */
bool omv_detect_yellow_danger(float *cx, float *cy, float *area);

/**
 * @brief Clear new data flag
 */
void omv_clear_data_flag(void);

/**
 * @brief Feed received byte to RPC parser (for interrupt handling)
 * @param byte Received byte from UART
 */
void omv_rpc_feed_byte(uint8_t byte);

/**
 * @brief OpenMV UART receive callback (call from main UART callback)
 * @param huart UART handle
 */
void omv_uart_rx_callback(UART_HandleTypeDef *huart);

/**
 * @brief OpenMV UART transmit callback (call from main UART callback)
 * @param huart UART handle
 */
void omv_uart_tx_callback(UART_HandleTypeDef *huart);

/**
 * @brief OpenMV UART error callback (call from main UART callback)
 * @param huart UART handle
 */
void omv_uart_error_callback(UART_HandleTypeDef *huart);

// --- Test Functions ---

/**
 * @brief Test OpenMV connection status
 * @return true if connection is working, false otherwise
 */
bool omv_test_connection(void);

/**
 * @brief Test OpenMV blob detection functionality
 */
void omv_test_blob_detection(void);

/**
 * @brief Test OpenMV alignment detection functionality
 */
void omv_test_align_detection(void);

/**
 * @brief Comprehensive test of all OpenMV functions
 */
void omv_test_all_functions(void);

/**
 * @brief Simple UART communication test
 */
void omv_test_uart_basic(void);

#ifdef __cplusplus
}
#endif

#endif // __OPENMV_UART_H