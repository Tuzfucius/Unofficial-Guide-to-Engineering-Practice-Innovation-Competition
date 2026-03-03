#ifndef ANGLE_PID_H
#define ANGLE_PID_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>

/* ============================================================================ */
/* ===================        双环航向PID参数定义        =================== */
/* ============================================================================ */

// 角度环PID参数 (外环) - 角度控制 -> 角速度目标
#define YAW_ANGLE_PID_KP         15.0f      // 角度环比例系数
#define YAW_ANGLE_PID_KI         0.0f      // 角度环积分系数
#define YAW_ANGLE_PID_KD         0.0f      // 角度环微分系数
#define YAW_ANGLE_PID_IMAX       50.0f     // 角度环积分限制
#define YAW_ANGLE_PID_OMAX       120.0f    // 角度环输出限制(度/秒)

// 角速度环PID参数 (内环) - 角速度控制 -> 电机转速
#define YAW_ANGVEL_PID_KP        0.70f      // 角速度环比例系数
#define YAW_ANGVEL_PID_KI        0.00f     // 角速度环积分系数
#define YAW_ANGVEL_PID_KD        0.00f      // 角速度环微分系数
#define YAW_ANGVEL_PID_IMAX      30.0f     // 角速度环积分限制
#define YAW_ANGVEL_PID_OMAX      100.0f    // 角速度环输出限制(RPM)

// 控制参数
#define YAW_ANGLE_DEADBAND       5.0f      // 角度死区(度)
#define YAW_ANGVEL_DEADBAND      8.0f      // 角速度死区(度/秒)

// 速度控制参数
#define YAW_SLOW_BAND_DEG        15.0f     // 慢速区域角度 (度)
#define YAW_FAST_BAND_DEG        45.0f     // 快速区域角度 (度)
#define YAW_SLOW_SPEED_LIMIT     25.0f     // 慢速限制 (RPM)
#define YAW_FAST_SPEED_LIMIT     80.0f     // 快速限制 (RPM)
#define YAW_MIN_SPEED_LIMIT      8.0f      // 最小速度限制 (RPM)

/* ============================================================================ */
/* ===================        双环PID结构体定义        =================== */
/* ============================================================================ */

/**
 * @brief 基础PID控制器结构体
 */
typedef struct {
    float Kp;           // 比例系数
    float Ki;           // 积分系数
    float Kd;           // 微分系数
    float Target;       // 目标值
    float Measured;     // 实际测量值
    float Output;       // PID输出
    float Integral;     // 积分累积
    float Last_Error;   // 上次误差
    float Integral_Max; // 积分限制
    float Output_Max;   // 输出限制
} Basic_PID_t;

/**
 * @brief 航向环双环PID控制结构体 (仿照Motor_PID结构)
 */
typedef struct {
    // 角度环 (外环) - 目标角度 -> 目标角速度
    Basic_PID_t Angle_PID;

    // 角速度环 (内环) - 目标角速度 -> 电机转速
    Basic_PID_t AngularVel_PID;

    // 控制状态变量
    float Target_Angle;          // 目标角度 (度)
    float Current_Angle;         // 当前角度 (度)
    float Target_AngularVel;     // 目标角速度 (度/秒)
    float Current_AngularVel;    // 当前角速度 (度/秒)
    float Final_Output;          // 最终输出到电机的值

    // 控制标志位
    bool Is_Active;              // PID是否激活
    bool Angle_Loop_Enable;      // 角度环使能
    bool AngularVel_Loop_Enable; // 角速度环使能

    // 时间控制
    uint32_t Start_Time;         // 开始时间
    uint32_t Timeout_ms;         // 超时时间
} Yaw_Dual_PID_t;

/* ============================================================================ */
/* ===================        全局变量声明        =================== */
/* ============================================================================ */

// 全局双环PID控制器实例
extern Yaw_Dual_PID_t g_yaw_dual_pid;

/* ============================================================================ */
/* ===================        函数声明        =================== */
/* ============================================================================ */

// 双环PID控制函数
void Yaw_PID_Init(void);
void Yaw_PID_Update(void);
void Yaw_PID_Reset_Static_Vars(void);
void Yaw_PID_Set_Current_As_Zero(void);
bool Yaw_PID_Is_Finished(void);

// 双环PID模式控制
void Yaw_PID_Set_Loop_Mode(bool angle_enable, bool angvel_enable);
void Yaw_PID_Set_Target_AngularVel(float target_angular_vel);

// 小车转向控制函数 (外部接口保持不变)
void Car_Turn_To_Angle_PID(float angle, uint32_t timeout_ms);
void Car_Turn_To_Absolute_Angle_PID(float target_angle, uint32_t timeout_ms);
void Car_Turn_With_AngularVel_PID(float angular_velocity, uint32_t duration_ms);

// 调试和测试函数
void Car_Debug_Yaw_PID_Test(void);
void Car_Demo_Dual_PID_Modes(void);
void Car_Debug_AngularVel_PID_Test(void);

// 实时显示函数
void Yaw_PID_Display_AngularVel_Status(void);
void Yaw_PID_Print_AngularVel_Status(void);
void Main_Display_Yaw_PID_Status(void);  // 主循环中的显示函数

void Car_debug_go(float rpm, uint32_t duration_ms);

#endif /* ANGLE_PID_H */
