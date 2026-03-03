#include "Motor.h"
#include <stdlib.h> // 用于 abs() 函数

/**
 * @brief 初始化所有电机PWM通道
 * @note  必须在主函数的初始化部分调用一次
 */
void Motor_Init(void)
{
    // 启动电机A (右前) 的PWM通道
    HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);

    // 启动电机B (左前) 的PWM通道
    HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);

    // 启动电机C (左后) 的PWM通道
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

    // 启动电机D (右后) 的PWM通道
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
}

/**
 * @brief 控制电机A (右前轮) 的转动
 * @param speed 速度和方向 (-PWM_MAX_VALUE to +PWM_MAX_VALUE)
 */
void MotorA_SetSpeed(int speed)
{
    // 限幅
    if (speed > PWM_MAX_VALUE) speed = PWM_MAX_VALUE;
    if (speed < -PWM_MAX_VALUE) speed = -PWM_MAX_VALUE;

    if (speed >= 0) // 正转
    {
        __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, speed);
        __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, 0);
    }
    else // 反转
    {
        __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, -speed);
    }
}

/**
 * @brief 控制电机B (左前轮) 的转动
 * @param speed 速度和方向 (-PWM_MAX_VALUE to +PWM_MAX_VALUE)
 */
void MotorB_SetSpeed(int speed)
{
    // 限幅
    if (speed > PWM_MAX_VALUE) speed = PWM_MAX_VALUE;
    if (speed < -PWM_MAX_VALUE) speed = -PWM_MAX_VALUE;

    if (speed >= 0) // 正转
    {
        __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, speed);
        __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 0);
    }
    else // 反转
    {
        __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, -speed);
    }
}

/**
 * @brief 控制电机C (左后轮) 的转动
 * @param speed 速度和方向 (-PWM_MAX_VALUE to +PWM_MAX_VALUE)
 */
void MotorC_SetSpeed(int speed)
{
    // 限幅
    if (speed > PWM_MAX_VALUE) speed = PWM_MAX_VALUE;
    if (speed < -PWM_MAX_VALUE) speed = -PWM_MAX_VALUE;

    if (speed >= 0) // 正转
    {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, speed);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    }
    else // 反转
    {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, -speed);
    }
}

/**
 * @brief 控制电机D (右后轮) 的转动
 * @param speed 速度和方向 (-PWM_MAX_VALUE to +PWM_MAX_VALUE)
 */
void MotorD_SetSpeed(int speed)
{
    // 限幅
    if (speed > PWM_MAX_VALUE) speed = PWM_MAX_VALUE;
    if (speed < -PWM_MAX_VALUE) speed = -PWM_MAX_VALUE;

    if (speed >= 0) // 正转
    {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, speed);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
    }
    else // 反转
    {
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, -speed);
    }
}

/*******************************************************************************
*                                                                              *
*                           高层运动控制函数 (已修正)                          *
*                                                                              *
*******************************************************************************/
// 对于您的电机和驱动板组合来说，提供一个正的PWM值（正转）实际上让车轮向后转
// 而提供一个负的PWM值（反转）反而让车轮向前转
/**
 * @brief 控制小车整体运动的核心函数
 * @note  此函数为底层接口，直接映射左右轮速度。
 *        左侧轮组: 电机B (左前), 电机C (左后)
 *        右侧轮组: 电机A (右前), 电机D (右后)
 * @param speed_left  左侧轮速度
 * @param speed_right 右侧轮速度
 */
void Car_Move(int speed_left, int speed_right)
{
    MotorB_SetSpeed(speed_left);
    MotorC_SetSpeed(speed_left);
    MotorA_SetSpeed(speed_right);
    MotorD_SetSpeed(speed_right);
}

/**
 * @brief 小车前进
 * @note  前进需要给电机设置一个负的速度值
 * @param speed 速度 (0 to PWM_MAX_VALUE)
 */
void Car_Go(int speed)
{
    int spd = abs(speed);
    // 要前进，需传递负值给电机
    Car_Move(-spd, -spd);
}

/**
 * @brief 小车后退
 * @note  后退需要给电机设置一个正的速度值
 * @param speed 速度 (0 to PWM_MAX_VALUE)
 */
void Car_Back(int speed)
{
    int spd = abs(speed);
    // 要后退，需传递正值给电机
    Car_Move(spd, spd);
}

/**
 * @brief 小车停止 (自由滑动)
 */
void Car_Stop(void)
{
    Car_Move(0, 0);
}

/**
 * @brief 小车刹车 (快速停止)
 */
void Car_Brake(void)
{
    // 通过将H桥两端都设为高电平（或都设为低电平）实现刹车
    __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, PWM_MAX_VALUE);
    __HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, PWM_MAX_VALUE);

    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, PWM_MAX_VALUE);
    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, PWM_MAX_VALUE);

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, PWM_MAX_VALUE);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, PWM_MAX_VALUE);
    
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, PWM_MAX_VALUE);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, PWM_MAX_VALUE);
}

/**
 * @brief 小车原地左转
 * @note  左轮后退 (正速度), 右轮前进 (负速度)
 * @param speed 速度 (0 to PWM_MAX_VALUE)
 */
void Car_TurnLeft(int speed)
{
    int spd = abs(speed);
    Car_Move(spd, -spd);
}

/**
 * @brief 小车原地右转
 * @note  左轮前进 (负速度), 右轮后退 (正速度)
 * @param speed 速度 (0 to PWM_MAX_VALUE)
 */
void Car_TurnRight(int speed)
{
    int spd = abs(speed);
    Car_Move(-spd, spd);
}