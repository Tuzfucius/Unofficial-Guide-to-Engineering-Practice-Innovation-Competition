#ifndef __MOTOR_TEST_H
#define __MOTOR_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>

/* ============================================================================ */
/*                          简化速度环PID测试函数                             */
/* ============================================================================ */

/**
 * @brief 启动所有电机速度环PID测试 (3秒自动停止)
 * @param target_rpm 目标速度 (RPM)
 */
void Motor_Speed_Test_Start_All(float target_rpm);

void Motor_Test_Display_Action(const char *title, float target_rpm);

void Motor_Test_Display_Yaw_Status(float start_angle_deg, float target_relative_deg);

/**
 * @brief 更新屏幕显示 (在主循环中调用)
 */
void Motor_Speed_Test_Update_Display(void);

/**
 * @brief 记录速度数据 (在TIM6中断中调用)
 */
void Motor_Speed_Test_Log_Data(void);

/**
 * @brief 检查测试是否完成
 * @return 1: 测试完成, 0: 测试进行中
 */
uint8_t Motor_Speed_Test_Is_Complete(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_TEST_H */
