#ifndef __MOTOR_PID_H
#define __MOTOR_PID_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "encoder.h" // 需要包含编码器头文件以获取反馈
#include <stdbool.h> // 需要bool类型
#include "hwt101.h"  // 需要包含陀螺仪头文件以获取角度反馈

/* ============================================================================ */
/* ===================        PID 控制器核心参数        =================== */
/* ============================================================================ */

/**
 * @brief 右侧电机动力补偿系数
 * @note  - 如果右轮动力比左轮弱，请将此值设为 > 1.0 (例如 1.05f, 1.1f)
 *        - 如果右轮动力比左轮强，请将此值设为 < 1.0 (例如 0.95f, 0.9f)
 *        - 初始值设为 1.0f (无补偿)
 */
#define RIGHT_MOTOR_POWER_FACTOR 1.0f

// <<< NEW: 平滑启动配置宏
#define RAMP_UP_DURATION_MS 500    // 速度爬升总时间 (毫秒)
#define RAMP_UPDATE_INTERVAL_MS 10 // 速度更新间隔 (毫秒)

// 您可以在 .h 文件中定义这个宏，用于调整刹车时间
#define RAMP_DOWN_DURATION_MS 200U // 平滑减速持续时间 (ms)

// <<< NEW: 斜率控制持续时间
#define SLEW_DURATION_MS 150U // 速度变化平滑过渡时间 (ms)

// PID死区，单位为脉冲 (Position) 或 RPM (Speed)
#define POSITION_PID_DEADBAND 25 // 位置环死区

// <<< NEW: 定义位置环PID的双死区 (单位: 编码器脉冲数)
// 第一阶段：进入平滑减速区
#define POSITION_PID_DEADBAND_SMOOTH 300.0f
// 第二阶段：最终保持/刹车区
#define POSITION_PID_DEADBAND_FINAL 50.0f

// <<< NEW: 轮径与编码器换算参数
#define WHEEL_DIAMETER_MM 100.0f
#define ENCODER_PULSES_PER_REV 330.0f

// <<< NEW: 位置模式下的速度上限 (RPM)
#define POSITION_MODE_MAX_SPEED_RPM 70.0f

// <<< NEW: 左平移位置模式速度上限 (RPM)
#define STRAFE_POSITION_MAX_SPEED_RPM 20.0f

// <<< NEW: 平滑减速阶段的速度上限 (RPM)
#define POSITION_SMOOTH_MAX_SPEED_RPM 30.0f

// <<< NEW: 最终位置保持阶段允许的最大目标速度 (RPM)
#define POSITION_HOLD_MAX_SPEED_RPM 20.0f

// <<< NEW: 最终位置保持阶段比例/阻尼系数 (相对当前Kp、速度反馈)
#define POSITION_HOLD_KP_FACTOR 0.25f
#define POSITION_HOLD_KD_FACTOR 0.10f

// <<< NEW: 死区保持时间阈值 (毫秒)
#define POSITION_DEADBAND_HOLD_MS 800U

/* ============================================================================ */
/*                           PID参数宏定义 - 根据测试数据优化                    */
/* ============================================================================ */

// 基准位置环PID参数
#define POS_PID_KP_BASE 8.0f
#define POS_PID_KI_BASE 0.0f
#define POS_PID_KD_BASE 0.0f
#define POS_PID_IMAX_BASE 10000.0f
#define POS_PID_OMAX_BASE 1000.0f

// 基准速度环PID参数 (V3: 针对RPM单位调整)
#define SPEED_PID_KP_BASE 57.0f
#define SPEED_PID_KI_BASE 0.0f
#define SPEED_PID_KD_BASE 20.0f
#define SPEED_PID_IMAX_BASE 10000.0f
#define SPEED_PID_OMAX_BASE PWM_MAX_VALUE

// L1电机参数调整(B)
#define L1_POS_PID_KP_FACTOR 1.0f
#define L1_POS_PID_KI_FACTOR 1.0f
#define L1_POS_PID_KD_FACTOR 1.0f
#define L1_POS_PID_OMAX_FACTOR 0.667f
#define L1_SPEED_PID_KP_FACTOR 0.995f // (4324/4379)*0.88 = 0.867 -> 0.87
#define L1_SPEED_PID_KI_FACTOR 1.0f // 设为0
#define L1_SPEED_PID_KD_FACTOR 1.0f // 设为0

// R1电机参数调整（A）
#define R1_POS_PID_KP_FACTOR 1.0f
#define R1_POS_PID_KI_FACTOR 1.0f
#define R1_POS_PID_KD_FACTOR 1.0f
#define R1_POS_PID_OMAX_FACTOR 1.0f
#define R1_SPEED_PID_KP_FACTOR 0.995f // 基准, (4324/4335)*1.15 = 1.147 -> 1.15
#define R1_SPEED_PID_KI_FACTOR 1.0f  // 设为0
#define R1_SPEED_PID_KD_FACTOR 1.05f  // 设为0

// L2电机参数调整 （C）
#define L2_POS_PID_KP_FACTOR 1.0f
#define L2_POS_PID_KI_FACTOR 1.0f
#define L2_POS_PID_KD_FACTOR 1.0f
#define L2_POS_PID_OMAX_FACTOR 100.0f
#define L2_SPEED_PID_KP_FACTOR 0.97f // (4324/4295)*1.12 = 1.127 -> 1.13
#define L2_SPEED_PID_KI_FACTOR 1.0f // 设为0
#define L2_SPEED_PID_KD_FACTOR 1.0f // 设为0

// R2电机参数调整 （D）
#define R2_POS_PID_KP_FACTOR 1.0f
#define R2_POS_PID_KI_FACTOR 1.0f
#define R2_POS_PID_KD_FACTOR 1.0f
#define R2_POS_PID_OMAX_FACTOR 1.0f // 30.0f / 30.0f = 1.0
#define R2_SPEED_PID_KP_FACTOR 0.98f // (4324/4286)*1.4 = 1.413 -> 1.42
#define R2_SPEED_PID_KI_FACTOR 1.0f // 设为0
#define R2_SPEED_PID_KD_FACTOR 1.0f // 设为0

// 轮子补偿因子 (根据最新测试数据精细调整)
#define L1_TRAVEL_FACTOR 1
#define L2_TRAVEL_FACTOR 1
#define R1_TRAVEL_FACTOR 1
#define R2_TRAVEL_FACTOR 1

    /* ============================================================================ */
    /*                             PID控制器类型定义                              */
    /* ============================================================================ */

    /**
     * @brief PID控制模式枚举
     */
    typedef enum
    {
        PID_MODE_DISABLE, // PID禁用状态
        PID_MODE_SPEED,   // 速度闭环模式
        PID_MODE_POSITION // 位置闭环模式
    } PID_Mode_t;

    /**
     * @brief PID控制器结构体
     */
    typedef struct
    {
        float Kp; // 比例 Proportional
        float Ki; // 积分 Integral
        float Kd; // 微分 Derivative

        float Target;   // 目标值
        float Measured; // 测量值 (反馈值)

        float Error;      // 当前误差 (Target - Measured)
        float Last_Error; // 上一次的误差
        float Integral;   // 积分累计值
        float Output;     // PID控制器计算输出

        float Integral_Max; // 积分项最大值 (用于积分限幅)
        float Output_Max;   // 控制器总输出最大值

    } PID_Controller_t;

    /**
     * @brief 单个电机的双环PID控制总结构体 (已重构)
     */
    typedef struct
    {
        PID_Controller_t Position_PID; // 位置环 (外环)
        PID_Controller_t Speed_PID;    // 速度环 (内环)
        Encoder_t *Encoder;            // 指向对应编码器
        PID_Mode_t Mode;               // 当前控制模式
    } Motor_PID_t;

    /* Extern variables ----------------------------------------------------------*/
    // 为四个电机声明独立的PID控制结构体
    extern Motor_PID_t MotorA_PID;
    extern Motor_PID_t MotorB_PID;
    extern Motor_PID_t MotorC_PID;
    extern Motor_PID_t MotorD_PID;

    // <<< NEW: 声明PID激活状态的全局标志位
    extern uint8_t g_is_pid_active;
    extern volatile bool g_request_stop;
    void Car_Stop_Request(void);
    void Car_Process_Stop_Request(void);

    // <<< NEW: 平滑启动 (速度爬升) 功能相关全局变量
    extern bool g_is_ramping;           // 是否正在进行速度爬升
    extern float g_final_target_rpm[4]; // 四个轮子的最终目标RPM
    extern float g_current_ramp_rpm[4]; // 四个轮子在爬升过程中的当前目标RPM
    extern uint32_t g_ramp_start_tick;  // 爬升开始的系统时间戳 (ms)

    /* Public function prototypes ------------------------------------------------*/

    /**
     * @brief 增量式PID控制器计算函数
     * @param pid 指向PID控制器结构体
     * @param target 目标值
     * @param measured 测量值
     * @return PID计算输出值
     */
    float PID_Calc(PID_Controller_t *pid, float target, float measured);

    /**
     * @brief 初始化所有电机的PID控制器
     * @note  在主函数初始化部分调用一次
     */
    void Motor_PID_Init_All(void);

    /**
     * @brief 更新所有电机的PID计算并施加控制
     * @note  必须在固定的周期性中断中调用 (例如 TIM6 中断)
     */
    void Motor_PID_Update_All(void);

    /**
     * @brief 为单个电机设置目标速度 (用于速度闭环模式)
     * @param motor_pid 指向要控制的电机PID结构体
     * @param target_rpm 目标速度，单位：RPM (转/分钟)
     */
    void Motor_PID_Set_Speed(Motor_PID_t *motor_pid, float target_rpm);
    void Motor_PID_Set_Position(Motor_PID_t *motor_pid, int64_t target_pos);

    /* ============================================================================ */
    /* ===================        高层整车运动控制函数        =================== */
    /* ============================================================================ */

    // 核心运动函数
    void Car_Mecanum_Move_PID(float rpm_a, float rpm_b, float rpm_c, float rpm_d);

    // 基础移动
    void Car_Go_PID(float rpm);
    void Car_Back_PID(float rpm);
    void Car_Stop_PID(void);
    void Car_Stop_PID_Enhanced(void); // 增强版急刹车函数
    void Car_Stop_Smooth_PID(void);
    void Motor_PID_Decel_Update(void);

    // 旋转与平移
    void Car_TurnLeft_PID(float rpm);
    void Car_TurnRight_PID(float rpm);
    void Car_Strafe_Left_PID(float rpm);
    void Car_Strafe_Right_PID(float rpm);
    void Car_Strafe_Left_Smooth(float rpm, uint32_t duration_ms);
    /* 新增：右平移平滑启动/停止函数 */
    void Car_Strafe_Right_Smooth(float rpm, uint32_t duration_ms);

    // 斜向运动
    void Car_Go_Left_Front_PID(float rpm);
    void Car_Go_Right_Front_PID(float rpm);
    void Car_Back_Left_Rear_PID(float rpm);
    void Car_Back_Right_Rear_PID(float rpm);

    // 位置控制 (基于差速模型)
    void Car_Move_To_Pos_PID(int64_t left_pos, int64_t right_pos);
    void Car_Go_Pulses_PID(uint32_t pulses);
    void Car_Back_Pulses_PID(uint32_t pulses);

    // 横向位置控制 (新增)
    void Car_Strafe_Left_Pulses_PID(uint32_t pulses);
    void Car_Strafe_Right_Pulses_PID(uint32_t pulses);

    /**
     * @brief [优化版] 精确直线行走位置控制
     * @param pulses 目标行走脉冲数（正数前进，负数后退）
     * @note  专为直线行走优化，根据实际测试数据对各轮进行补偿
     */
    void Car_Go_Straight_Pulses_PID(int32_t pulses);
    bool Car_Wait_For_Position_Stop(uint32_t timeout_ms);

    /**
     * @brief  复位所有电机和PID的状态，确保起步一致性
     * @note   在每次独立的运动序列开始前调用此函数，
     *         例如，在Car_Stop_PID()之后，下一次Car_Go_PID()之前。
     */
    void Car_Reset_And_Sync_State(void);

    /**
     * @brief [内部函数] 更新速度爬升状态
     * @note  应该在PID更新循环中被调用
     */
    void Motor_PID_Ramp_Update(void);

    /**
     * @brief [调试函数] 测试左右轮同步性能
     * @note  用于诊断"左轮和右轮起跑不一致"问题
     */
    void Car_Debug_Left_Right_Sync_Test(void);

    /**
     * @brief 检查小车是否已完全停止
     * @return true: 小车已停止, false: 小车仍在运动
     * @note 检查所有电机的目标速度和实际速度是否都为0
     */
    bool Car_Is_Stopped(void);

    /**
     * @brief 等待小车完全停止
     * @param timeout_ms 超时时间(毫秒), 0表示无超时
     * @return true: 停止成功, false: 超时
     */
    bool Car_Wait_Until_Stopped(uint32_t timeout_ms);

    /**
     * @brief [调试函数] 测试左平移速度环性能
     * @note  专门用于测试左平移时各轮子的速度响应和协调性
     */
    void Car_Debug_Left_Strafe_Test(void);

    void Car_LEFT90(void);
    void Car_FORWARD(float rpm, uint32_t delay);
    void Car_BACKWARD(float rpm, uint32_t delay);

    void Car_Debug_Left_Right_Sync_Test(void);
    void Car_Debug_Rotation_Test();

    void Car_LEFT(float time);

    float Motor_Encoder_PulsesToDistanceMM(int32_t pulses);

    /**
     * @brief 【新增】全向移动控制函数，融合前进、平移和旋转.
     * @param forward_speed 前进速度(RPM), >0 前进, <0 后退.
     * @param strafe_speed  平移速度(RPM), >0 右移, <0 左移.
     * @param turn_speed    旋转速度(RPM), >0 右转, <0 左转.
     */
    void Car_Mecanum_Move_Combined_PID(float forward_speed, float strafe_speed, float turn_speed);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_PID_H */