#include "datou_pwm.h"

// --- 外部变量引用 ---
// 引用 main.c 中由CubeMX生成的定时器TIM12句柄
extern TIM_HandleTypeDef htim12;

// --- 内部私有变量 ---
// 使用 volatile 关键字，因为这个变量会在主循环和中断中同时被访问
volatile uint32_t steps_remaining = 0;

// 重要：请根据您的驱动器细分设置修改此值！
// 电机一圈所需脉冲数 = 200 * 细分值
// 示例: 1.8°电机，16细分 -> 200 * 16 = 3200
const uint32_t STEPS_PER_REVOLUTION = 3200;

// --- 内部私有函数 ---
static void Datou_PWM_Stop(void);

/**
 * @brief 初始化张大头电机PWM控制模块
 */
void Datou_PWM_Init(void)
{
    // 确保电机初始为失能状态，节省功耗且安全
    // 假设高电平失能。如果您的驱动器是低电平失能，请改为 GPIO_PIN_RESET
    HAL_GPIO_WritePin(MOTOR_EN_GPIO_Port, MOTOR_EN_Pin, GPIO_PIN_SET);
    steps_remaining = 0;
}

/**
 * @brief 内部函数：停止电机运动
 */
static void Datou_PWM_Stop(void)
{
    HAL_TIM_Base_Stop_IT(&htim12);            // 停止定时器更新中断
    HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_1); // 停止PWM脉冲输出

    // 运动结束后失能电机
    HAL_GPIO_WritePin(MOTOR_EN_GPIO_Port, MOTOR_EN_Pin, GPIO_PIN_SET);
}

/**
 * @brief 以指定的RPM转动指定的步数
 * @param dir: 方向 (0 或 1)1向上，0向下
 * @param steps: 要移动的总步数，3200一圈
 * @param rpm: 目标转速 (转/分钟)
 */
void Datou_PWM_Move_Steps(uint8_t dir, uint32_t steps, float rpm)
{
    if (steps == 0 || rpm <= 0)
    {
        return;
    }

    // 如果电机正在运动，则忽略新指令
    if (steps_remaining > 0)
    {
        return;
    }

    // 1. 设置方向 (PC8)
    HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, (GPIO_PinState)dir);

    // 2. 计算定时器参数
    const uint32_t timer_clock_freq = 1000000; // 1MHz 计数时钟 (由 PSC=83 得到)
    float steps_per_second = (rpm * STEPS_PER_REVOLUTION) / 60.0f;
    uint32_t arr_value = (uint32_t)(timer_clock_freq / steps_per_second);

    // 3. 设置定时器周期(ARR)和占空比(CCR)
    __HAL_TIM_SET_AUTORELOAD(&htim12, arr_value);
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, arr_value / 2); // 50%占空比

    // 4. 更新剩余步数
    steps_remaining = steps;

    // 5. 使能电机(PC9)并启动运动
    // 假设低电平使能
    HAL_GPIO_WritePin(MOTOR_EN_GPIO_Port, MOTOR_EN_Pin, GPIO_PIN_RESET);
    HAL_Delay(1); // 短暂延时，等待驱动器稳定

    __HAL_TIM_SET_COUNTER(&htim12, 0);         // 清零计数器，从头开始
    HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1); // 启动PWM输出 (PB14)
    HAL_TIM_Base_Start_IT(&htim12);            // 启动定时器更新中断
}

/**
 * @brief 检查电机是否仍在运动
 * @retval 1: 正在运动; 0: 已停止
 */
uint8_t Datou_PWM_Is_Running(void)
{
    return (steps_remaining > 0);
}

/**
 * @brief 定时器周期溢出回调函数
 * @note  这个函数需要在 main.c 或者 stm32f4xx_it.c 中被 HAL 库自动调用
 */
void HAL_TIM_PeriodElapsedCallback_datou(TIM_HandleTypeDef *htim)
{

    if (steps_remaining > 0)
    {
        steps_remaining--; // 剩余步数减1
    }

    if (steps_remaining == 0)
    {
        Datou_PWM_Stop(); // 如果步数已走完，则调用停止函数
    }

    // 如果您有其他定时器中断，可以在这里添加 else if (htim->Instance == TIMx)
}