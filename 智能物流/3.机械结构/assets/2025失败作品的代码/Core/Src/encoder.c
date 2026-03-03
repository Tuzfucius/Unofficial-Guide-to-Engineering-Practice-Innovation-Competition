#include "encoder.h"
#include <stdio.h>

/* Private variables ---------------------------------------------------------*/
// 定义四个电机的编码器结构体变量
Encoder_t MotorA_Encoder; // 右前轮, TIM2
Encoder_t MotorB_Encoder; // 左前轮, TIM3
Encoder_t MotorC_Encoder; // 左后轮, TIM4
Encoder_t MotorD_Encoder; // 右后轮, TIM5

// 定义速度计算的转换系数
// 1分钟有60秒, CONTROL_FREQUENCY是每秒的采样次数
static const float VELOCITY_COEFFICIENT = (60.0f * CONTROL_FREQUENCY) / ENCODER_RESOLUTION;

/* Private function prototypes -----------------------------------------------*/
static void Encoder_Init(Encoder_t *encoder, TIM_HandleTypeDef *htim);
static void Encoder_Update(Encoder_t *encoder);


/**
 * @brief 初始化单个编码器
 * @param encoder 指向编码器结构体的指针
 * @param htim    指向对应定时器句柄的指针
 */
static void Encoder_Init(Encoder_t *encoder, TIM_HandleTypeDef *htim)
{
    encoder->htim = htim;
    encoder->Last_counter = 0;
    encoder->Total_counter = 0;
    encoder->Velocity = 0.0f;

    HAL_TIM_Encoder_Start(htim, TIM_CHANNEL_ALL);
    __HAL_TIM_SET_COUNTER(htim, 0); // 将硬件计数器清零
}

/**
 * @brief 更新单个编码器的数据 (速度、累计脉冲等)
 * @param encoder 指向编码器结构体的指针
 */
static void Encoder_Update(Encoder_t *encoder)
{
    // 1. 读取当前硬件计数器的值
    int32_t current_counter = __HAL_TIM_GET_COUNTER(encoder->htim);

    // 2. 计算自上次更新以来的脉冲增量 (delta)
    int32_t delta = current_counter - encoder->Last_counter;

    // 3. 处理硬件计数器溢出
    // TIM3 和 TIM4 是16位定时器 (0-65535)
    if (encoder->htim->Instance == TIM3 || encoder->htim->Instance == TIM4)
    {
        if (delta > 32767)      // 发生下溢出 (e.g., from 10 to 65530)
        {
            delta -= 65536;
        }
        else if (delta < -32768) // 发生上溢出 (e.g., from 65530 to 10)
        {
            delta += 65536;
        }
    }
    // 注意: TIM2 和 TIM5 是32位定时器，在常规应用中几乎不会溢出，因此不作处理。
    
    // 如果是左轮，并且电机在前进（计数器递减，delta为负），我们将其转为正值，
    // 以匹配PID控制器“正速度=前进”的约定。
    if (encoder->htim->Instance == TIM3 || encoder->htim->Instance == TIM4)
    {
        delta = -delta; // 翻转脉冲增量的符号
    }
    // 4. 更新累计脉冲数
    encoder->Total_counter += delta;

    // 5. 更新上一次的计数值
    encoder->Last_counter = current_counter;

    // 6. 计算当前速度 (RPM)
    // 速度 = (增量脉冲数 / 每转脉冲数) * (采样频率 * 60秒)
    encoder->Velocity = (float)delta * VELOCITY_COEFFICIENT;
}

/**
 * @brief 初始化所有电机的编码器接口
 */
void Encoders_All_Init(void)
{
    Encoder_Init(&MotorA_Encoder, &htim2);
    Encoder_Init(&MotorB_Encoder, &htim3);
    Encoder_Init(&MotorC_Encoder, &htim4);
    Encoder_Init(&MotorD_Encoder, &htim5);
}

/**
 * @brief 更新所有编码器的数据
 */
void Encoders_All_Update(void)
{
    Encoder_Update(&MotorA_Encoder);
    Encoder_Update(&MotorB_Encoder);
    Encoder_Update(&MotorC_Encoder);
    Encoder_Update(&MotorD_Encoder);
}

/**
 * @brief 通过串口打印所有编码器的信息
 */
void Encoders_All_Print(void)
{
    // 使用 long long (%lld) 来打印64位的 Total_counter
    printf("A: %lld, %.1f RPM | B: %lld, %.1f RPM | C: %lld, %.1f RPM | D: %lld, %.1f RPM\r\n",
           (long long)MotorA_Encoder.Total_counter, MotorA_Encoder.Velocity,
           (long long)MotorB_Encoder.Total_counter, MotorB_Encoder.Velocity,
           (long long)MotorC_Encoder.Total_counter, MotorC_Encoder.Velocity,
           (long long)MotorD_Encoder.Total_counter, MotorD_Encoder.Velocity
          );
}
