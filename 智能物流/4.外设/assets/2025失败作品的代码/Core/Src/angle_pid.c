#include "Motor_PID.h"
#include "Motor.h"
#include <math.h>
#include <stdio.h>
#include <limits.h>
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include <All.h>
#include "angle_pid.h"

/* ============================================================================ */
/* ===================        双环航向PID系统实现        =================== */
/* ============================================================================ */

// 全局双环PID控制器实例
Yaw_Dual_PID_t g_yaw_dual_pid;

// 静态变量和控制参数
#ifndef PI
#define PI 3.1415926535f
#endif

#define YAW_SOFT_STOP_STEPS     24U
#define YAW_SOFT_STOP_MIN_SPEED 0.4f
#define YAW_STOP_HOLD_TICKS     30U
#define YAW_FINISH_HOLD_TICKS   6U
#define YAW_VEL_FILTER_ALPHA    0.2f
#define YAW_RAMP_DURATION_MS    300U
#define YAW_TURN_CMD_DEADBAND   1.0f
#define YAW_TURN_CMD_MAX_DEG    180.0f
#define YAW_SMALL_ANGLE_THRESHOLD 5.0f
#define YAW_SMALL_ANGLE_MAX_SPEED 16.0f
#define YAW_MIN_SPEED_RELEASE_BAND 1.0f
#define YAW_FINAL_FREEZE_ERROR   0.6f
#define YAW_FINAL_FREEZE_CMD     2.0f

// 静态控制变量
static uint8_t s_yaw_stop_counter = 0;
static uint32_t s_last_debug_time = 0;
static bool s_soft_stop_active = false;
static float s_soft_stop_initial_speed = 0.0f;
static uint8_t s_soft_stop_step = 0;
static float s_last_turn_command = 0.0f;
static uint8_t s_finish_hold_counter = 0;
static float s_filtered_angular_velocity = 0.0f;
static bool s_yaw_ramp_active = false;
static uint32_t s_yaw_ramp_start_tick = 0;

// 全局重置标志
bool g_reset_yaw_static = false;

/* ============================================================================ */
/* ===================        基础PID计算函数        =================== */
/* ============================================================================ */

/**
 * @brief 基础PID计算函数
 * @param pid PID控制器结构体指针
 * @param target 目标值
 * @param measured 测量值
 * @return PID输出值
 */
static float Basic_PID_Calc(Basic_PID_t *pid, float target, float measured)
{
    if (!pid) return 0.0f;

    pid->Target = target;
    pid->Measured = measured;

    // 计算误差
    float error = target - measured;

    // 积分计算 (带限制)
    pid->Integral += error;
    if (pid->Integral > pid->Integral_Max) {
        pid->Integral = pid->Integral_Max;
    } else if (pid->Integral < -pid->Integral_Max) {
        pid->Integral = -pid->Integral_Max;
    }

    // 微分计算
    float derivative = error - pid->Last_Error;

    // PID输出计算
    pid->Output = pid->Kp * error +
                  pid->Ki * pid->Integral +
                  pid->Kd * derivative;

    // 输出限制
    if (pid->Output > pid->Output_Max) {
        pid->Output = pid->Output_Max;
    } else if (pid->Output < -pid->Output_Max) {
        pid->Output = -pid->Output_Max;
    }

    pid->Last_Error = error;
    return pid->Output;
}

/**
 * @brief 重置PID控制器
 */
static void Basic_PID_Reset(Basic_PID_t *pid)
{
    if (!pid) return;

    pid->Target = 0.0f;
    pid->Measured = 0.0f;
    pid->Output = 0.0f;
    pid->Integral = 0.0f;
    pid->Last_Error = 0.0f;
}

/* ============================================================================ */
/* ===================        角度计算和辅助函数        =================== */
/* ============================================================================ */

/**
 * @brief 角度差值计算 (处理角度回绕问题)
 */
static float Yaw_Angle_Diff(float target, float current)
{
    float diff = target - current;

    while (diff > 180.0f) diff -= 360.0f;
    while (diff < -180.0f) diff += 360.0f;

    return diff;
}

/**
 * @brief 输出调整和限制
 */
static float Yaw_PID_Shape_Command(float command, float angle_error, uint32_t now_tick)
{
    float abs_command = fabsf(command);
    float abs_error = fabsf(angle_error);

    // 分段速度控制
    float max_speed = YAW_FAST_SPEED_LIMIT;

    if (abs_error <= YAW_SLOW_BAND_DEG) {
        max_speed = YAW_SLOW_SPEED_LIMIT;
    } else if (abs_error <= YAW_FAST_BAND_DEG) {
        float span = YAW_FAST_BAND_DEG - YAW_SLOW_BAND_DEG;
        if (span < 1.0f) span = 1.0f;
        float ratio = (abs_error - YAW_SLOW_BAND_DEG) / span;
        max_speed = YAW_SLOW_SPEED_LIMIT + ratio * (YAW_FAST_SPEED_LIMIT - YAW_SLOW_SPEED_LIMIT);
    }

    if (abs_error <= YAW_SMALL_ANGLE_THRESHOLD && max_speed > YAW_SMALL_ANGLE_MAX_SPEED) {
        max_speed = YAW_SMALL_ANGLE_MAX_SPEED;
    }

    // 速度限制
    if (abs_command > max_speed) {
        command = (command > 0.0f) ? max_speed : -max_speed;
        abs_command = max_speed;
    }

    // 最小速度限制
    bool allow_low_speed = (abs_error <= YAW_MIN_SPEED_RELEASE_BAND);
    if (!allow_low_speed && abs_command < YAW_MIN_SPEED_LIMIT && abs_command > 0.1f) {
        command = (command > 0.0f) ? YAW_MIN_SPEED_LIMIT : -YAW_MIN_SPEED_LIMIT;
        abs_command = YAW_MIN_SPEED_LIMIT;
    } else if (allow_low_speed && abs_command < 0.3f) {
        command = 0.0f;
        abs_command = 0.0f;
    }

    // 最终冻结：误差已经极小且命令幅值不大，直接置零防止继续摆动
    if (fabsf(angle_error) <= YAW_FINAL_FREEZE_ERROR && abs_command <= YAW_FINAL_FREEZE_CMD) {
        command = 0.0f;
    }

    // 启动斜坡控制
    if (s_yaw_ramp_active) {
        uint32_t elapsed = now_tick - s_yaw_ramp_start_tick;
        if (elapsed >= YAW_RAMP_DURATION_MS) {
            s_yaw_ramp_active = false;
        } else {
            float progress = (float)elapsed / (float)YAW_RAMP_DURATION_MS;
            if (progress < 0.0f) progress = 0.0f;
            if (progress > 1.0f) progress = 1.0f;
            float scale = 0.5f * (1.0f - cosf(progress * PI));
            command *= scale;
        }
    }

    return command;
}

/**
 * @brief 开始启动斜坡
 */
static void Yaw_PID_Start_Ramp(void)
{
    s_yaw_ramp_active = true;
    s_yaw_ramp_start_tick = HAL_GetTick();
}

/**
 * @brief 开始软停止
 */
static void Yaw_PID_Begin_Soft_Stop(float start_speed)
{
    if (!(start_speed == start_speed) || start_speed > 1000.0f || start_speed < -1000.0f) {
        start_speed = 0.0f;
    }

    s_soft_stop_initial_speed = start_speed;
    s_soft_stop_step = 0;
    s_soft_stop_active = (fabsf(start_speed) > YAW_SOFT_STOP_MIN_SPEED);
    s_yaw_ramp_active = false;
}

/**
 * @brief 应用转向到电机
 */
static void Yaw_PID_Apply_Turn(float command)
{
    s_last_turn_command = command;

    if (command > 0.0f) {
        Car_Mecanum_Move_PID(command, -command, -command, command);
    } else if (command < 0.0f) {
        float abs_speed = fabsf(command);
        Car_Mecanum_Move_PID(-abs_speed, abs_speed, abs_speed, -abs_speed);
    } else {
        Car_Stop_Request();
    }
}

/* ============================================================================ */
/* ===================        双环PID主要控制函数        =================== */
/* ============================================================================ */

/**
 * @brief 初始化双环航向PID控制器
 */
void Yaw_PID_Init(void)
{
    // ========== 角度环PID参数初始化 (外环) ==========
    g_yaw_dual_pid.Angle_PID.Kp = YAW_ANGLE_PID_KP;
    g_yaw_dual_pid.Angle_PID.Ki = YAW_ANGLE_PID_KI;
    g_yaw_dual_pid.Angle_PID.Kd = YAW_ANGLE_PID_KD;
    g_yaw_dual_pid.Angle_PID.Integral_Max = YAW_ANGLE_PID_IMAX;
    g_yaw_dual_pid.Angle_PID.Output_Max = YAW_ANGLE_PID_OMAX;
    Basic_PID_Reset(&g_yaw_dual_pid.Angle_PID);

    // ========== 角速度环PID参数初始化 (内环) ==========
    g_yaw_dual_pid.AngularVel_PID.Kp = YAW_ANGVEL_PID_KP;
    g_yaw_dual_pid.AngularVel_PID.Ki = YAW_ANGVEL_PID_KI;
    g_yaw_dual_pid.AngularVel_PID.Kd = YAW_ANGVEL_PID_KD;
    g_yaw_dual_pid.AngularVel_PID.Integral_Max = YAW_ANGVEL_PID_IMAX;
    g_yaw_dual_pid.AngularVel_PID.Output_Max = YAW_ANGVEL_PID_OMAX;
    Basic_PID_Reset(&g_yaw_dual_pid.AngularVel_PID);

    // ========== 控制状态初始化 ==========
    g_yaw_dual_pid.Target_Angle = 0.0f;
    g_yaw_dual_pid.Current_Angle = 0.0f;
    g_yaw_dual_pid.Target_AngularVel = 0.0f;
    g_yaw_dual_pid.Current_AngularVel = 0.0f;
    g_yaw_dual_pid.Final_Output = 0.0f;

    // 控制标志位初始化
    g_yaw_dual_pid.Is_Active = false;
    g_yaw_dual_pid.Angle_Loop_Enable = true;
    g_yaw_dual_pid.AngularVel_Loop_Enable = true;

    // 时间控制初始化
    g_yaw_dual_pid.Start_Time = 0;
    g_yaw_dual_pid.Timeout_ms = 0;

    printf("Yaw Dual PID initialized: Angle Loop + AngularVel Loop\r\n");
}

/**
 * @brief 重置静态变量
 */
void Yaw_PID_Reset_Static_Vars(void)
{
    g_reset_yaw_static = true;
    s_yaw_stop_counter = 0;
    s_last_debug_time = 0;
    s_soft_stop_active = false;
    s_soft_stop_initial_speed = 0.0f;
    s_soft_stop_step = 0;
    s_last_turn_command = 0.0f;
    s_finish_hold_counter = 0;
    s_filtered_angular_velocity = 0.0f;
    s_yaw_ramp_active = false;
}

/**
 * @brief 将当前角度设为零点
 */
void Yaw_PID_Set_Current_As_Zero(void)
{
    float current_angle = g_hwt101_data.Angle_Z;
    float current_angular_vel = g_hwt101_data.AngVel_Z;

    g_yaw_dual_pid.Is_Active = false;
    g_yaw_dual_pid.Target_Angle = current_angle;
    g_yaw_dual_pid.Current_Angle = current_angle;
    g_yaw_dual_pid.Target_AngularVel = 0.0f;
    g_yaw_dual_pid.Current_AngularVel = current_angular_vel;
    g_yaw_dual_pid.Final_Output = 0.0f;

    g_yaw_dual_pid.Timeout_ms = 0;
    g_yaw_dual_pid.Start_Time = HAL_GetTick();

    Basic_PID_Reset(&g_yaw_dual_pid.Angle_PID);
    Basic_PID_Reset(&g_yaw_dual_pid.AngularVel_PID);

    Car_Stop_PID();
    Yaw_PID_Reset_Static_Vars();

    printf("Yaw Dual PID reset to zero: Angle=%.1f°\r\n", current_angle);
}

/**
 * @brief 双环PID更新函数 - 核心控制逻辑
 */
void Yaw_PID_Update(void)
{
    // 重置静态变量标志处理
    if (g_reset_yaw_static) {
        s_yaw_stop_counter = 0;
        s_last_debug_time = HAL_GetTick();
        s_soft_stop_active = false;
        s_soft_stop_initial_speed = 0.0f;
        s_soft_stop_step = 0;
        s_last_turn_command = 0.0f;
        s_finish_hold_counter = 0;
        s_filtered_angular_velocity = g_hwt101_data.AngVel_Z;
        s_yaw_ramp_active = false;
        g_reset_yaw_static = false;
    }

    // 检查双环PID是否激活
    if (!g_yaw_dual_pid.Is_Active) {
        // 软停止处理
        if (s_soft_stop_active) {
            if (s_soft_stop_step < YAW_SOFT_STOP_STEPS) {
                float denom = (float)(YAW_SOFT_STOP_STEPS - 1U);
                if (denom <= 0.0f) {
                    denom = 1.0f;
                }

                float progress = (float)s_soft_stop_step / denom;
                if (progress < 0.0f) progress = 0.0f;
                if (progress > 1.0f) progress = 1.0f;

                // Cosine ease-out：起始保持原速度，逐步平滑降到0，避免刹停突兀
                float smooth_scale = 0.5f * (1.0f + cosf(progress * PI));
                float command = s_soft_stop_initial_speed * smooth_scale;

                if (fabsf(command) <= YAW_SOFT_STOP_MIN_SPEED) {
                    command = 0.0f;
                }

                if (command != 0.0f) {
                    Yaw_PID_Apply_Turn(command);
                } else {
                    Car_Stop_Request();
                }

                s_soft_stop_step++;

                if (command == 0.0f || s_soft_stop_step >= YAW_SOFT_STOP_STEPS) {
                    s_soft_stop_active = false;
                    s_yaw_stop_counter = 0;
                }
                return;
            }
            s_soft_stop_active = false;
        }

        // 强制停止电机
        if (s_yaw_stop_counter < YAW_STOP_HOLD_TICKS) {
            Car_Stop_Request();
            s_yaw_stop_counter++;
        }
        s_finish_hold_counter = 0;
        return;
    }

    // ========== 更新传感器数据 ==========
    g_yaw_dual_pid.Current_Angle = g_hwt101_data.Angle_Z;
    float raw_angular_velocity = g_hwt101_data.AngVel_Z;

    // 角速度滤波
    s_filtered_angular_velocity += YAW_VEL_FILTER_ALPHA * (raw_angular_velocity - s_filtered_angular_velocity);
    g_yaw_dual_pid.Current_AngularVel = s_filtered_angular_velocity;

    // ========== 超时检查 ==========
    uint32_t current_tick = HAL_GetTick();
    uint32_t elapsed = current_tick - g_yaw_dual_pid.Start_Time;

    if (g_yaw_dual_pid.Timeout_ms > 0 && elapsed >= g_yaw_dual_pid.Timeout_ms) {
        g_yaw_dual_pid.Is_Active = false;
        Yaw_PID_Begin_Soft_Stop(s_last_turn_command);
        return;
    }

    if (elapsed >= 15000) { // 15秒安全超时
        g_yaw_dual_pid.Is_Active = false;
        Yaw_PID_Begin_Soft_Stop(s_last_turn_command);
        return;
    }

    // ========== 双环PID控制逻辑 ==========

    // Step 1: 角度环控制 (外环)
    float angle_error = Yaw_Angle_Diff(g_yaw_dual_pid.Target_Angle, g_yaw_dual_pid.Current_Angle);

    if (g_yaw_dual_pid.Angle_Loop_Enable) {
        // 角度环PID计算: 角度误差 -> 目标角速度
        g_yaw_dual_pid.Target_AngularVel = Basic_PID_Calc(&g_yaw_dual_pid.Angle_PID, 0.0f, angle_error);
    }

    // Step 2: 角速度环控制 (内环)
    if (g_yaw_dual_pid.AngularVel_Loop_Enable) {
        // 角速度环PID计算: 角速度误差 -> 电机输出
        g_yaw_dual_pid.Final_Output = Basic_PID_Calc(&g_yaw_dual_pid.AngularVel_PID,
                                                     g_yaw_dual_pid.Target_AngularVel,
                                                     g_yaw_dual_pid.Current_AngularVel);
    } else {
        // 如果角速度环禁用，直接使用角度环输出
        g_yaw_dual_pid.Final_Output = g_yaw_dual_pid.Target_AngularVel;
    }

    // Step 3: 输出调整
    float final_output = Yaw_PID_Shape_Command(g_yaw_dual_pid.Final_Output, angle_error, current_tick);

    // Step 4: 到达判断
    if (fabsf(angle_error) <= YAW_ANGLE_DEADBAND &&
        fabsf(g_yaw_dual_pid.Current_AngularVel) <= YAW_ANGVEL_DEADBAND) {

        if (s_finish_hold_counter < UINT8_MAX) {
            s_finish_hold_counter++;
        }

        if (s_finish_hold_counter >= YAW_FINISH_HOLD_TICKS) {
            bool timeout_active = (g_yaw_dual_pid.Timeout_ms > 0U);
            bool hold_time_remaining = timeout_active && (elapsed < g_yaw_dual_pid.Timeout_ms);

            if (hold_time_remaining) {
                // 目标已到，但仍在超时时间内，维持PID激活用于姿态保持
                g_yaw_dual_pid.Target_AngularVel = 0.0f;
                g_yaw_dual_pid.Final_Output = 0.0f;
                final_output = 0.0f;
                s_last_turn_command = 0.0f;
            } else {
                g_yaw_dual_pid.Is_Active = false;
                Basic_PID_Reset(&g_yaw_dual_pid.Angle_PID);
                Basic_PID_Reset(&g_yaw_dual_pid.AngularVel_PID);
                s_finish_hold_counter = 0;

                Car_Stop_PID();

                if (fabsf(s_last_turn_command) > YAW_SOFT_STOP_MIN_SPEED) {
                    Yaw_PID_Begin_Soft_Stop(s_last_turn_command);
                } else {
                    s_soft_stop_active = false;
                    s_soft_stop_initial_speed = 0.0f;
                    s_soft_stop_step = 0;
                    s_last_turn_command = 0.0f;
                }
                return;
            }
        }
    } else {
        s_finish_hold_counter = 0;
    }

    // Step 5: 应用最终输出到电机
    Yaw_PID_Apply_Turn(final_output);
}

/* ============================================================================ */
/* ===================        双环PID控制接口        =================== */
/* ============================================================================ */

/**
 * @brief 检查转向是否完成
 */
bool Yaw_PID_Is_Finished(void)
{
    return !g_yaw_dual_pid.Is_Active;
}

/**
 * @brief 设置双环PID模式
 */
void Yaw_PID_Set_Loop_Mode(bool angle_enable, bool angvel_enable)
{
    g_yaw_dual_pid.Angle_Loop_Enable = angle_enable;
    g_yaw_dual_pid.AngularVel_Loop_Enable = angvel_enable;

    printf("Yaw PID Mode: Angle=%s, AngVel=%s\r\n",
           angle_enable ? "ON" : "OFF",
           angvel_enable ? "ON" : "OFF");
}

/**
 * @brief 直接设置目标角速度
 */
void Yaw_PID_Set_Target_AngularVel(float target_angular_vel)
{
    g_yaw_dual_pid.Target_AngularVel = target_angular_vel;
}

/* ============================================================================ */
/* ===================        外部接口函数        =================== */
/* ============================================================================ */

/**
 * @brief 控制小车转向到相对角度
 */
void Car_Turn_To_Angle_PID(float angle, uint32_t timeout_ms)
{
    printf("Starting dual PID turn to relative angle: %.1f°\r\n", angle);

    float abs_angle = fabsf(angle);
    if (abs_angle < YAW_TURN_CMD_DEADBAND) {
        printf("Turn command within deadband (|%.2f°| < %.2f°), skipping\r\n", abs_angle, YAW_TURN_CMD_DEADBAND);
        return;
    }

    if (abs_angle > YAW_TURN_CMD_MAX_DEG) {
        angle = (angle > 0.0f) ? YAW_TURN_CMD_MAX_DEG : -YAW_TURN_CMD_MAX_DEG;
        abs_angle = YAW_TURN_CMD_MAX_DEG;
        printf("Turn command limited to %.1f° due to max constraint\r\n", angle);
    }
    if (abs_angle <= YAW_SMALL_ANGLE_THRESHOLD) {
        printf("Applying small-angle speed limit (<= %.1f° -> %.1f RPM max)\r\n",
               YAW_SMALL_ANGLE_THRESHOLD, YAW_SMALL_ANGLE_MAX_SPEED);
    }

    float current_angle = g_hwt101_data.Angle_Z;
    float current_angular_vel = g_hwt101_data.AngVel_Z;

    // 计算绝对目标角度
    float absolute_target = current_angle + angle;
    while (absolute_target > 180.0f) absolute_target -= 360.0f;
    while (absolute_target < -180.0f) absolute_target += 360.0f;

    // 设置双环PID参数
    g_yaw_dual_pid.Target_Angle = absolute_target;
    g_yaw_dual_pid.Current_Angle = current_angle;
    g_yaw_dual_pid.Target_AngularVel = 0.0f;
    g_yaw_dual_pid.Current_AngularVel = current_angular_vel;
    g_yaw_dual_pid.Final_Output = 0.0f;

    g_yaw_dual_pid.Timeout_ms = timeout_ms;
    g_yaw_dual_pid.Start_Time = HAL_GetTick();

    // 重置PID状态
    Basic_PID_Reset(&g_yaw_dual_pid.Angle_PID);
    Basic_PID_Reset(&g_yaw_dual_pid.AngularVel_PID);

    // 确保双环启用
    g_yaw_dual_pid.Angle_Loop_Enable = true;
    g_yaw_dual_pid.AngularVel_Loop_Enable = true;

    // 激活控制
    g_yaw_dual_pid.Is_Active = true;
    Yaw_PID_Start_Ramp();

    printf("Yaw Dual PID started: %.1f° -> %.1f° (delta: %.1f°)\r\n",
           current_angle, absolute_target, angle);

    // 等待完成
    while (!Yaw_PID_Is_Finished()) {
        HAL_Delay(10);

        uint32_t now = HAL_GetTick();
        if (timeout_ms > 0 && (now - g_yaw_dual_pid.Start_Time) >= timeout_ms) {
            g_yaw_dual_pid.Is_Active = false;
            Car_Stop_PID();
            break;
        }
    }

    uint32_t actual_time = HAL_GetTick() - g_yaw_dual_pid.Start_Time;
    printf("Turn completed in %ums\r\n", actual_time);
}

/**
 * @brief 控制小车转向到绝对角度
 */
void Car_Turn_To_Absolute_Angle_PID(float target_angle, uint32_t timeout_ms)
{
    float current_angle = g_hwt101_data.Angle_Z;
    float relative_angle = Yaw_Angle_Diff(target_angle, current_angle);

    printf("Starting dual PID turn to absolute angle: %.1f° (current: %.1f°, delta: %.1f°)\r\n",
           target_angle, current_angle, relative_angle);

    Car_Turn_To_Angle_PID(relative_angle, timeout_ms);
}

/**
 * @brief 控制小车以指定角速度转动指定时间
 * @param angular_velocity 目标角速度 (度/秒，正值顺时针，负值逆时针)
 * @param duration_ms 转动时间 (毫秒)
 */
void Car_Turn_With_AngularVel_PID(float angular_velocity, uint32_t duration_ms)
{
    printf("Starting angular velocity control: %.1f°/s for %ums\r\n", angular_velocity, duration_ms);

    float current_angle = g_hwt101_data.Angle_Z;
    float current_angular_vel = g_hwt101_data.AngVel_Z;

    // 设置为仅角速度环模式
    g_yaw_dual_pid.Angle_Loop_Enable = false;
    g_yaw_dual_pid.AngularVel_Loop_Enable = true;

    // 设置双环PID参数
    g_yaw_dual_pid.Target_Angle = current_angle;  // 角度环不使用，设为当前角度
    g_yaw_dual_pid.Current_Angle = current_angle;
    g_yaw_dual_pid.Target_AngularVel = angular_velocity;
    g_yaw_dual_pid.Current_AngularVel = current_angular_vel;
    g_yaw_dual_pid.Final_Output = 0.0f;

    g_yaw_dual_pid.Timeout_ms = duration_ms;
    g_yaw_dual_pid.Start_Time = HAL_GetTick();

    // 重置PID状态
    Basic_PID_Reset(&g_yaw_dual_pid.Angle_PID);
    Basic_PID_Reset(&g_yaw_dual_pid.AngularVel_PID);

    // 激活控制
    g_yaw_dual_pid.Is_Active = true;
    Yaw_PID_Start_Ramp();

    printf("Angular velocity PID started: %.1f°/s for %ums\r\n", angular_velocity, duration_ms);

    // 等待完成 - 带实时显示
    while (!Yaw_PID_Is_Finished()) {
        // 实时显示角速度环状态
        Yaw_PID_Display_AngularVel_Status();
        Yaw_PID_Print_AngularVel_Status();

        HAL_Delay(50);  // 50ms刷新率

        uint32_t now = HAL_GetTick();
        if ((now - g_yaw_dual_pid.Start_Time) >= duration_ms) {
            g_yaw_dual_pid.Is_Active = false;
            Car_Stop_PID();
            break;
        }
    }

    uint32_t actual_time = HAL_GetTick() - g_yaw_dual_pid.Start_Time;
    float final_angle = g_hwt101_data.Angle_Z;
    float actual_rotation = final_angle - current_angle;

    // 处理角度回绕
    while (actual_rotation > 180.0f) actual_rotation -= 360.0f;
    while (actual_rotation < -180.0f) actual_rotation += 360.0f;

    // 恢复标准双环模式
    g_yaw_dual_pid.Angle_Loop_Enable = true;
    g_yaw_dual_pid.AngularVel_Loop_Enable = true;

    printf("Angular velocity control completed: %.1f°/s for %ums, rotated %.1f°\r\n",
           angular_velocity, actual_time, actual_rotation);
}

/* ============================================================================ */
/* ===================        调试和测试函数        =================== */
/* ============================================================================ */

/**
 * @brief 双环PID测试函数
 */
void Car_Debug_Yaw_PID_Test(void)
{
    extern const SSD1306_Font_t Font_7x10;
    char buffer[64];

    float start_angle = g_hwt101_data.Angle_Z;
    float relative_angle = 90.0f;

    ssd1306_Fill(0);
    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString("Dual PID Test", Font_7x10, 1);
    ssd1306_SetCursor(0, 12);
    snprintf(buffer, sizeof(buffer), "Start: %.1f°", start_angle);
    ssd1306_WriteString(buffer, Font_7x10, 1);
    ssd1306_SetCursor(0, 24);
    snprintf(buffer, sizeof(buffer), "Turn: %.1f°", relative_angle);
    ssd1306_WriteString(buffer, Font_7x10, 1);
    ssd1306_UpdateScreen();

    HAL_Delay(1000);

    uint32_t test_start = HAL_GetTick();
    Car_Turn_To_Angle_PID(relative_angle, 5000);
    uint32_t test_end = HAL_GetTick();

    uint32_t total_time = test_end - test_start;
    float final_angle = g_hwt101_data.Angle_Z;
    float actual_turn = final_angle - start_angle;

    while (actual_turn > 180.0f) actual_turn -= 360.0f;
    while (actual_turn < -180.0f) actual_turn += 360.0f;

    float error = actual_turn - relative_angle;

    printf("Yaw Dual PID Test Results:\r\n");
    printf("Target: %.1f° | Actual: %.1f° | Error: %.1f° | Time: %ums\r\n",
           relative_angle, actual_turn, error, total_time);
    printf("Angle Loop: Kp=%.2f Ki=%.2f Kd=%.2f\r\n",
           g_yaw_dual_pid.Angle_PID.Kp, g_yaw_dual_pid.Angle_PID.Ki, g_yaw_dual_pid.Angle_PID.Kd);
    printf("AngVel Loop: Kp=%.2f Ki=%.2f Kd=%.2f\r\n",
           g_yaw_dual_pid.AngularVel_PID.Kp, g_yaw_dual_pid.AngularVel_PID.Ki, g_yaw_dual_pid.AngularVel_PID.Kd);
}

/**
 * @brief 双环PID模式演示
 */
void Car_Demo_Dual_PID_Modes(void)
{
    extern const SSD1306_Font_t Font_7x10;

    printf("=== Yaw Dual PID Modes Demo ===\r\n");

    // 模式1: 标准双环控制
    ssd1306_Fill(0);
    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString("Mode 1:", Font_7x10, 1);
    ssd1306_SetCursor(0, 12);
    ssd1306_WriteString("Dual Loop", Font_7x10, 1);
    ssd1306_UpdateScreen();

    Yaw_PID_Set_Loop_Mode(true, true);
    Car_Turn_To_Angle_PID(90.0f, 3000);
    HAL_Delay(1000);

    // 模式2: 仅角速度环控制
    ssd1306_Fill(0);
    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString("Mode 2:", Font_7x10, 1);
    ssd1306_SetCursor(0, 12);
    ssd1306_WriteString("AngVel Only", Font_7x10, 1);
    ssd1306_UpdateScreen();

    Yaw_PID_Set_Loop_Mode(false, true);
    g_yaw_dual_pid.Is_Active = true;
    g_yaw_dual_pid.Start_Time = HAL_GetTick();
    g_yaw_dual_pid.Timeout_ms = 2000;

    Yaw_PID_Set_Target_AngularVel(60.0f);
    HAL_Delay(1000);
    Yaw_PID_Set_Target_AngularVel(0.0f);
    HAL_Delay(500);

    // 恢复标准模式
    Yaw_PID_Set_Loop_Mode(true, true);

    ssd1306_Fill(0);
    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString("Demo Complete", Font_7x10, 1);
    ssd1306_UpdateScreen();

    printf("Dual PID modes demo completed\r\n");
}

/**
 * @brief 实时显示角速度环PID状态 - OLED显示
 */
void Yaw_PID_Display_AngularVel_Status(void)
{
    extern const SSD1306_Font_t Font_7x10;
    char buffer[32];

    // 清屏
    ssd1306_Fill(0);

    // 第一行：标题
    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString("AngVel PID Status", Font_7x10, 1);

    // 第二行：目标角速度 vs 实际角速度
    ssd1306_SetCursor(0, 12);
    snprintf(buffer, sizeof(buffer), "T:%.1f A:%.1f",
             g_yaw_dual_pid.Target_AngularVel,
             g_yaw_dual_pid.Current_AngularVel);
    ssd1306_WriteString(buffer, Font_7x10, 1);

    // 第三行：PID输出
    ssd1306_SetCursor(0, 24);
    snprintf(buffer, sizeof(buffer), "Out: %.1f",
             g_yaw_dual_pid.AngularVel_PID.Output);
    ssd1306_WriteString(buffer, Font_7x10, 1);

    // 第四行：误差
    float angular_vel_error = g_yaw_dual_pid.Target_AngularVel - g_yaw_dual_pid.Current_AngularVel;
    ssd1306_SetCursor(0, 36);
    snprintf(buffer, sizeof(buffer), "Err: %.1f", angular_vel_error);
    ssd1306_WriteString(buffer, Font_7x10, 1);

    // 第五行：状态指示
    ssd1306_SetCursor(0, 48);
    if (g_yaw_dual_pid.Is_Active) {
        bool is_reached = (fabsf(angular_vel_error) <= YAW_ANGVEL_DEADBAND);
        ssd1306_WriteString(is_reached ? "REACHED" : "RUNNING", Font_7x10, 1);
    } else {
        ssd1306_WriteString("STOPPED", Font_7x10, 1);
    }

    ssd1306_UpdateScreen();
}

/**
 * @brief 实时串口打印角速度环PID状态
 */
void Yaw_PID_Print_AngularVel_Status(void)
{
    static uint32_t last_print_time = 0;
    uint32_t current_time = HAL_GetTick();

    // 每200ms打印一次
    if (current_time - last_print_time >= 200) {
        last_print_time = current_time;

        float angular_vel_error = g_yaw_dual_pid.Target_AngularVel - g_yaw_dual_pid.Current_AngularVel;
        bool is_reached = (fabsf(angular_vel_error) <= YAW_ANGVEL_DEADBAND);

        printf("AngVel PID | Target: %6.1f°/s | Current: %6.1f°/s | Error: %6.1f°/s | Output: %6.1f | %s\r\n",
               g_yaw_dual_pid.Target_AngularVel,
               g_yaw_dual_pid.Current_AngularVel,
               angular_vel_error,
               g_yaw_dual_pid.AngularVel_PID.Output,
               is_reached ? "REACHED" : "TRACKING");
    }
}

/**
 * @brief 角速度环PID调试测试函数 - 带实时显示
 */
void Car_Debug_AngularVel_PID_Test(void)
{
    extern const SSD1306_Font_t Font_7x10;

    printf("=== Angular Velocity PID Test ===\r\n");

    // 显示测试开始信息
    ssd1306_Fill(0);
    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString("AngVel Test", Font_7x10, 1);
    ssd1306_SetCursor(0, 12);
    ssd1306_WriteString("Starting...", Font_7x10, 1);
    ssd1306_UpdateScreen();
    HAL_Delay(1000);

    // 设置为仅角速度环模式
    Yaw_PID_Set_Loop_Mode(false, true);

    // 激活控制
    g_yaw_dual_pid.Is_Active = true;
    g_yaw_dual_pid.Start_Time = HAL_GetTick();
    g_yaw_dual_pid.Timeout_ms = 0;  // 无超时，手动控制

    // 测试序列：不同角速度
    float test_speeds[] = {30.0f, -45.0f, 60.0f, -30.0f, 0.0f};
    uint32_t test_durations[] = {2000, 2000, 1500, 1500, 1000};
    int test_count = sizeof(test_speeds) / sizeof(test_speeds[0]);

    for (int i = 0; i < test_count; i++) {
        printf("\n--- Test %d: %.1f°/s for %ums ---\r\n", i+1, test_speeds[i], test_durations[i]);

        // 设置目标角速度
        Yaw_PID_Set_Target_AngularVel(test_speeds[i]);

        uint32_t test_start = HAL_GetTick();
        uint32_t last_display_time = 0;

        // 运行测试并实时显示
        while ((HAL_GetTick() - test_start) < test_durations[i]) {
            // 实时显示状态
            Yaw_PID_Display_AngularVel_Status();
            Yaw_PID_Print_AngularVel_Status();

            HAL_Delay(50);  // 50ms刷新率
        }

        printf("Test %d completed\r\n", i+1);
        HAL_Delay(500);  // 测试间隔
    }

    // 停止控制
    g_yaw_dual_pid.Is_Active = false;
    Car_Stop_PID();

    // 恢复标准双环模式
    Yaw_PID_Set_Loop_Mode(true, true);

    // 显示测试完成
    ssd1306_Fill(0);
    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString("AngVel Test", Font_7x10, 1);
    ssd1306_SetCursor(0, 12);
    ssd1306_WriteString("Completed!", Font_7x10, 1);
    ssd1306_UpdateScreen();

    printf("=== Angular Velocity PID Test Completed ===\r\n");
}

/**
 * @brief 主循环中的航向PID状态显示函数 - 适合放在main.c中调用
 *
 * 功能说明：
 * - OLED显示：航向角、角速度、PID状态
 * - 串口打印：双环PID详细数据（每500ms打印一次）
 * - 自动检测控制模式和状态
 *
 * 使用方法：
 * 在main.c的主循环中调用：Main_Display_Yaw_PID_Status();
 */
void Main_Display_Yaw_PID_Status(void)
{
    extern const SSD1306_Font_t Font_7x10;
    static uint32_t last_print_time = 0;
    uint32_t current_time = HAL_GetTick();
    char buffer[32];

    // 更新传感器数据到PID结构体
    g_yaw_dual_pid.Current_Angle = g_hwt101_data.Angle_Z;
    g_yaw_dual_pid.Current_AngularVel = g_hwt101_data.AngVel_Z;

    // ========== OLED实时显示航向角和角速度 ==========
    ssd1306_Fill(0);

    // 第一行：航向角标题和数值
    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString("Heading Angle:", Font_7x10, 1);

    // 第二行：航向角数值（大字体显示）
    ssd1306_SetCursor(0, 12);
    snprintf(buffer, sizeof(buffer), "%.2f°", g_yaw_dual_pid.Current_Angle);
    ssd1306_WriteString(buffer, Font_7x10, 1);

    // 第三行：角速度标题和数值
    ssd1306_SetCursor(0, 24);
    ssd1306_WriteString("Angular Vel:", Font_7x10, 1);

    // 第四行：角速度数值
    ssd1306_SetCursor(0, 36);
    snprintf(buffer, sizeof(buffer), "%.2f°/s", g_yaw_dual_pid.Current_AngularVel);
    ssd1306_WriteString(buffer, Font_7x10, 1);

    // 第五行：PID输出数值
    ssd1306_SetCursor(0, 48);
    if (g_yaw_dual_pid.Is_Active) {
        snprintf(buffer, sizeof(buffer), "PID: %.2f", g_yaw_dual_pid.Final_Output);
        ssd1306_WriteString(buffer, Font_7x10, 1);
    } else {
        ssd1306_WriteString("PID: 0.00", Font_7x10, 1);
    }

    ssd1306_UpdateScreen();

    // ========== 串口实时打印航向角和角速度（每200ms一次） ==========
    if (current_time - last_print_time >= 200) {
        last_print_time = current_time;

        // 显示航向角、角速度和PID输出数值
        printf("Heading: %7.2f° | AngVel: %6.1f°/s | PID_Out: %6.2f\r\n",
               g_yaw_dual_pid.Current_Angle,
               g_yaw_dual_pid.Current_AngularVel,
               g_yaw_dual_pid.Is_Active ? g_yaw_dual_pid.Final_Output : 0.0f);
    }
}

