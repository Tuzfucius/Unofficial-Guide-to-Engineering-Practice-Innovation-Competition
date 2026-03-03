#ifndef DATOU_PWM_H
#define DATOU_PWM_H

#include "main.h"

// --- 公共函数声明 ---

/**
 * @brief 初始化张大头电机PWM控制模块
 */
void Datou_PWM_Init(void);

/**
 * @brief 以指定的RPM转动指定的步数
 * @param dir: 方向 (0 或 1)
 * @param steps: 要移动的总步数
 * @param rpm: 目标转速 (转/分钟)
 */
void Datou_PWM_Move_Steps(uint8_t dir, uint32_t steps, float rpm);

/**
 * @brief 检查电机是否仍在运动
 * @retval 1: 正在运动; 0: 已停止
 */
uint8_t Datou_PWM_Is_Running(void);

void HAL_TIM_PeriodElapsedCallback_datou(TIM_HandleTypeDef *htim);

#endif /* DATOU_PWM_H */