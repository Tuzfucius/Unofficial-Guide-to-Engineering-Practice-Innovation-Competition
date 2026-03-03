#ifndef __MOTOR_H
#define __MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h" // 根据您的MCU系列修改，例如 "stm32f1xx_hal.h"

/* Public Defines ------------------------------------------------------------*/
// 定义电机的最大PWM值 (ARR的值)，请根据您定时器的配置进行修改
#define PWM_MAX_VALUE 999

/* Extern variables ----------------------------------------------------------*/
// 声明所有电机控制相关的TIM句柄
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim9;
extern TIM_HandleTypeDef htim10;
extern TIM_HandleTypeDef htim11;

/* Public function prototypes ------------------------------------------------*/

/**
 * @brief 初始化所有电机PWM通道
 * @note  必须在主函数的初始化部分调用一次
 */
void Motor_Init(void);

/**
 * @brief 控制单个电机的转动
 * @param speed 速度和方向 (-PWM_MAX_VALUE to +PWM_MAX_VALUE)
 *              - speed > 0: 正转
 *              - speed < 0: 反转
 *              - speed = 0: 停止 (自由滑动)
 */
void MotorA_SetSpeed(int speed); // 右前轮
void MotorB_SetSpeed(int speed); // 左前轮
void MotorC_SetSpeed(int speed); // 左后轮
void MotorD_SetSpeed(int speed); // 右后轮


/**
 * @brief 控制小车整体运动
 * @param speed_left  左侧轮速度 (-PWM_MAX_VALUE to +PWM_MAX_VALUE)
 * @param speed_right 右侧轮速度 (-PWM_MAX_VALUE to +PWM_MAX_VALUE)
 */
void Car_Move(int speed_left, int speed_right);

/**
 * @brief 小车前进
 * @param speed 速度 (0 to PWM_MAX_VALUE)
 */
void Car_Go(int speed);

/**
 * @brief 小车后退
 * @param speed 速度 (0 to PWM_MAX_VALUE)
 */
void Car_Back(int speed);

/**
 * @brief 小车停止 (自由滑动)
 */
void Car_Stop(void);

/**
 * @brief 小车刹车 (快速停止)
 * @note  通过给H桥两端相同电平实现
 */
void Car_Brake(void);

/**
 * @brief 小车原地左转
 * @param speed 速度 (0 to PWM_MAX_VALUE)
 */
void Car_TurnLeft(int speed);

/**
 * @brief 小车原地右转
 * @param speed 速度 (0 to PWM_MAX_VALUE)
 */
void Car_TurnRight(int speed);


#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H */