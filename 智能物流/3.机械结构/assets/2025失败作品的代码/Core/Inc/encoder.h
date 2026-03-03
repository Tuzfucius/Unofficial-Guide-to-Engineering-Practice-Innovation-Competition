#ifndef __ENCODER_H
#define __ENCODER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h" // 根据您的MCU系列修改，例如 "stm32f1xx_hal.h"

/* ============================================================================ */
/* ===================       用户需要配置的宏定义       =================== */
/* ============================================================================ */

/**
 * @brief 编码器单圈总脉冲数 (4倍频后)
 * @note  这个值至关重要，必须正确设置！
 *        计算公式: 电机输出轴转一圈，编码器产生的总脉冲数
 *        = (编码器原始分辨率 PPR) * (电机减速比) * 4
 *        例如: 11PPR * 30减速比 * 4 = 1320
 */
#define ENCODER_RESOLUTION      1320

/**
 * @brief 调用 Encoder_Update() 函数的频率 (Hz)
 * @note  这个值决定了速度计算的采样时间。
 *        例如，如果您在10ms的定时器中断或systick回调中调用它，
 *        那么这个频率就是 1 / 0.01s = 100 Hz.
 */
#define CONTROL_FREQUENCY       100

/* ============================================================================ */

/* Public type definitions ---------------------------------------------------*/

/**
 * @brief 编码器数据结构体
 */
typedef struct
{
    TIM_HandleTypeDef *htim;          // 对应的定时器句柄
    int32_t           Last_counter;   // 上一次读取的硬件计数值
    int64_t           Total_counter;  // 累计的总脉冲数 (使用64位防止溢出)
    float             Velocity;       // 当前速度 (单位: RPM - 转/分钟)

} Encoder_t;


/* Extern variables ----------------------------------------------------------*/
// 声明所有编码器相关的TIM句柄
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;

// 声明四个电机的编码器结构体变量
extern Encoder_t MotorA_Encoder; // 右前轮
extern Encoder_t MotorB_Encoder; // 左前轮
extern Encoder_t MotorC_Encoder; // 左后轮
extern Encoder_t MotorD_Encoder; // 右后轮

/* Public function prototypes ------------------------------------------------*/

/**
 * @brief 初始化所有电机的编码器接口
 * @note  在主函数初始化部分调用一次
 */
void Encoders_All_Init(void);

/**
 * @brief 更新所有编码器的数据
 * @note  必须以固定的频率 (CONTROL_FREQUENCY) 周期性调用
 *        推荐在 Systick 中断回调函数 HAL_SYSTICK_Callback 中调用
 */
void Encoders_All_Update(void);

/**
 * @brief 通过串口打印所有编码器的信息
 * @note  用于调试，请确保您的printf已重定向到UART
 */
void Encoders_All_Print(void);


#ifdef __cplusplus
}
#endif

#endif /* __ENCODER_H */