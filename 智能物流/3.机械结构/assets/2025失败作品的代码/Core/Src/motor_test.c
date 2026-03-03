/**
 * @file motor_test.c
 * @brief 简化速度环PID测试模块 - 直接使用现有PID控制器
 * @date 2025-10-19
 */

#include "motor_test.h"
#include "Motor_PID.h"    // 直接使用现有的PID控制器
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include "main.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "angle_pid.h"
#include "hwt101.h"

// 声明字体
extern const SSD1306_Font_t Font_7x10;

/* ============================================================================ */
/*                              简化测试变量                                  */
/* ============================================================================ */

// 测试状态
static volatile uint8_t g_test_active = 0;
static volatile uint8_t g_last_test_state = 0; // 新增：用于检测状态变化
static volatile float g_test_target_rpm = 0.0f;
static volatile uint32_t g_test_start_time = 0;

// 各电机达到目标速度的时间记录
static volatile uint32_t g_motor_settle_time[4] = {0, 0, 0, 0};
static volatile uint8_t g_motor_settled[4] = {0, 0, 0, 0};
static volatile uint32_t g_all_settled_time = 0; // 新增：所有电机都稳定的时刻

// 测试常量
#define TEST_DURATION_MS 3000          // 总测试时间5秒 (安全超时)
#define RUN_AFTER_SETTLED_MS 5000      // 稳定后继续运行的时间
#define SETTLE_THRESHOLD 5.0f         // 达到目标速度的误差阈值 (RPM)

// PID结构体指针数组
static Motor_PID_t* g_motor_pids[4] = {
    &MotorA_PID,  // 电机A
    &MotorB_PID,  // 电机B
    &MotorC_PID,  // 电机C
    &MotorD_PID   // 电机D
};

// 电机名称
static const char* g_motor_names[4] = {"A", "B", "C", "D"};

/**
 * @brief  简易封装的动作提示显示
 * @note   在主控调用运动函数前展示标题/目标速度/当前航向角
 */
static float Motor_Test_Normalize_Angle(float angle_deg)
{
    while (angle_deg > 180.0f)
    {
        angle_deg -= 360.0f;
    }
    while (angle_deg < -180.0f)
    {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

void Motor_Test_Display_Action(const char *title, float target_rpm)
{
    if (!title)
    {
        title = "PID Action";
    }

    char buffer[32];
    char title_buffer[32];

    size_t title_len = strlen(title);
    if (title_len >= sizeof(title_buffer))
    {
        title_len = sizeof(title_buffer) - 1U;
    }
    memcpy(title_buffer, title, title_len);
    title_buffer[title_len] = '\0';

    ssd1306_Fill(0);
    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString(title_buffer, Font_7x10, 1);

    ssd1306_SetCursor(0, 12);
    snprintf(buffer, sizeof(buffer), "Target:%5.1fRPM", target_rpm);
    ssd1306_WriteString(buffer, Font_7x10, 1);

    ssd1306_SetCursor(0, 24);
    snprintf(buffer, sizeof(buffer), "Yaw:%6.1fdeg", g_hwt101_data.Angle_Z);
    ssd1306_WriteString(buffer, Font_7x10, 1);

    ssd1306_UpdateScreen();
}

void Motor_Test_Display_Yaw_Status(float current_angle_input_deg, float target_relative_deg)
{
    static uint8_t s_initialized = 0;
    static float s_start_angle_deg = 0.0f;
    static float s_target_relative_deg = 0.0f;
    static uint32_t s_last_refresh_tick = 0;

    uint32_t now = HAL_GetTick();
    if ((now - s_last_refresh_tick) < 100U && s_last_refresh_tick != 0U)
    {
        return;
    }
    s_last_refresh_tick = now;

    if (!s_initialized || fabsf(target_relative_deg - s_target_relative_deg) > 0.5f)
    {
        s_start_angle_deg = current_angle_input_deg;
        s_target_relative_deg = target_relative_deg;
        s_initialized = 1;
    }

    float current_angle_deg = g_hwt101_data.Angle_Z;
    if (!isfinite(current_angle_deg))
    {
        current_angle_deg = current_angle_input_deg;
    }

    float target_absolute_deg = Motor_Test_Normalize_Angle(s_start_angle_deg + s_target_relative_deg);
    float actual_turn_deg = Motor_Test_Normalize_Angle(current_angle_deg - s_start_angle_deg);
    float error_deg = Motor_Test_Normalize_Angle(target_absolute_deg - current_angle_deg);

    char buffer[32];

    ssd1306_Fill(0);
    ssd1306_SetCursor(0, 0);
    ssd1306_WriteString("Yaw Monitor", Font_7x10, 1);

    ssd1306_SetCursor(0, 12);
    snprintf(buffer, sizeof(buffer), "Start:%6.1f", s_start_angle_deg);
    ssd1306_WriteString(buffer, Font_7x10, 1);

    ssd1306_SetCursor(0, 24);
    snprintf(buffer, sizeof(buffer), "Target:%+5.1f", s_target_relative_deg);
    ssd1306_WriteString(buffer, Font_7x10, 1);

    ssd1306_SetCursor(0, 36);
    snprintf(buffer, sizeof(buffer), "Turn: %6.1f", actual_turn_deg);
    ssd1306_WriteString(buffer, Font_7x10, 1);

    ssd1306_SetCursor(0, 48);
    snprintf(buffer, sizeof(buffer), "Err:  %6.1f", error_deg);
    ssd1306_WriteString(buffer, Font_7x10, 1);

    ssd1306_UpdateScreen();
}

/* ============================================================================ */
/*                               公共函数                                     */
/* ============================================================================ */

/**
 * @brief 启动所有电机速度环PID测试 (2.秒自动停止)
 * @param target_rpm 目标速度 (RPM)
 */
void Motor_Speed_Test_Start_All(float target_rpm)
{
    // V6: 在测试开始前，重置所有电机和PID状态
    Car_Reset_And_Sync_State();

    // 重置所有状态
    g_test_target_rpm = target_rpm;
    g_test_start_time = HAL_GetTick();
    g_test_active = 1;
    g_all_settled_time = 0; // 重置

    for (uint8_t i = 0; i < 4; i++) {
        g_motor_settle_time[i] = 0;
        g_motor_settled[i] = 0;
    }

    // 激活PID系统
    g_is_pid_active = 1;

    // 设置所有电机目标速度
    Motor_PID_Set_Speed(&MotorA_PID, target_rpm);
    Motor_PID_Set_Speed(&MotorB_PID, target_rpm);
    Motor_PID_Set_Speed(&MotorC_PID, target_rpm);
    Motor_PID_Set_Speed(&MotorD_PID, target_rpm);
}

/**
 * @brief 记录速度数据 (在TIM6中断中调用)
 */
void Motor_Speed_Test_Log_Data(void)
{
    if (!g_test_active) return;

    uint32_t current_time = HAL_GetTick();
    uint32_t elapsed = current_time - g_test_start_time;

    // 1. 安全超时：如果总时间超过最大测试时间，强制结束
    if (elapsed >= TEST_DURATION_MS) {
        g_test_active = 0;
        g_is_pid_active = 0;
        return;
    }

    // 2. 检查是否已经进入“稳定后运行”阶段
    if (g_all_settled_time > 0) {
        // 如果是，检查是否已运行足够长的时间
        if (current_time - g_all_settled_time >= RUN_AFTER_SETTLED_MS) {
            g_test_active = 0; // 结束测试
            g_is_pid_active = 0;
        }
        return; // 在此阶段，无需再检查下面的稳定状态
    }

    // 3. 如果尚未稳定，则继续检查各电机状态
    uint8_t all_settled = 1;
    for (uint8_t i = 0; i < 4; i++) {
        if (!g_motor_settled[i]) {
            float current_speed = g_motor_pids[i]->Speed_PID.Measured;
            float error = fabsf(g_test_target_rpm - current_speed);

            if (error <= SETTLE_THRESHOLD) {
                g_motor_settled[i] = 1;
                g_motor_settle_time[i] = elapsed;
            } else {
                all_settled = 0; // 只要有一个电机没稳定，就标记为未全部稳定
            }
        }
    }

    // 4. 如果刚刚所有电机都达到了稳定状态
    if (all_settled) {
        g_all_settled_time = current_time; // 记录下这个时刻
        // 注意：此时不结束测试，等待1秒
    }
}

/**
 * @brief 更新屏幕显示 (在主循环中调用)
 */
void Motor_Speed_Test_Update_Display(void)
{
    // 检测测试是否刚刚结束
    if (g_last_test_state == 1 && g_test_active == 0) {
        // 测试刚刚结束，执行停止操作
        Car_Stop_PID();
        ssd1306_Fill(0); // 清屏以显示最终结果
    }
    g_last_test_state = g_test_active;

    if (g_test_active) {
        // 测试进行中 - 显示实时速度
        uint32_t elapsed = HAL_GetTick() - g_test_start_time;

        // 标题
        ssd1306_SetCursor(0, 0);
        ssd1306_WriteString("Speed Test", Font_7x10, 1);

        // 目标速度和剩余时间
        char buffer[32];
        ssd1306_SetCursor(0, 12);
        snprintf(buffer, sizeof(buffer), "Target:%.0fRPM %ds",
                g_test_target_rpm, (TEST_DURATION_MS - elapsed) / 1000);
        ssd1306_WriteString(buffer, Font_7x10, 1);

        // 显示各电机实时速度和PID输出
        for (uint8_t i = 0; i < 2; i++) { // 一行显示两个电机
            // 左侧电机
            ssd1306_SetCursor(0, 22 + i * 20);
            snprintf(buffer, sizeof(buffer), "%s:%.0f %s",
                    g_motor_names[i*2],
                    g_motor_pids[i*2]->Speed_PID.Measured,
                    g_motor_settled[i*2] ? "OK" : "");
            ssd1306_WriteString(buffer, Font_7x10, 1);
            
            ssd1306_SetCursor(0, 32 + i * 20);
            snprintf(buffer, sizeof(buffer), " P:%.0f", g_motor_pids[i*2]->Speed_PID.Output);
            ssd1306_WriteString(buffer, Font_7x10, 1);

            // 右侧电机
            ssd1306_SetCursor(64, 22 + i * 20);
            snprintf(buffer, sizeof(buffer), "%s:%.0f %s",
                    g_motor_names[i*2+1],
                    g_motor_pids[i*2+1]->Speed_PID.Measured,
                    g_motor_settled[i*2+1] ? "OK" : "");
            ssd1306_WriteString(buffer, Font_7x10, 1);

            ssd1306_SetCursor(64, 32 + i * 20);
            snprintf(buffer, sizeof(buffer), " P:%.0f", g_motor_pids[i*2+1]->Speed_PID.Output);
            ssd1306_WriteString(buffer, Font_7x10, 1);
        }

         // 通过 printf 打印 PID 输出值到串口
         printf("PID Out -> A:%.0f, B:%.0f, C:%.0f, D:%.0f\r\n",
             g_motor_pids[0]->Speed_PID.Output,
             g_motor_pids[1]->Speed_PID.Output,
             g_motor_pids[2]->Speed_PID.Output,
             g_motor_pids[3]->Speed_PID.Output);

         // 最底部显示当前航向角，便于观察姿态
         ssd1306_SetCursor(0, 54);
         snprintf(buffer, sizeof(buffer), "Yaw:%6.1fdeg", g_hwt101_data.Angle_Z);
         ssd1306_WriteString(buffer, Font_7x10, 1);

    } else {
        // 测试完成 - 显示结果
        ssd1306_SetCursor(0, 0);
        ssd1306_WriteString("Test Complete", Font_7x10, 1);

        // 显示各电机达到目标速度的时间和总脉冲
        char buffer[32];
        for (uint8_t i = 0; i < 4; i++) {
            ssd1306_SetCursor(0, 12 + i * 12);
            if (g_motor_settled[i]) {
                snprintf(buffer, sizeof(buffer), "%s:%.2fs %lldp",
                        g_motor_names[i], 
                        g_motor_settle_time[i] / 1000.0f,
                        g_motor_pids[i]->Encoder->Total_counter);
            } else {
                snprintf(buffer, sizeof(buffer), "%s:Fail %lldp", 
                        g_motor_names[i],
                        g_motor_pids[i]->Encoder->Total_counter);
            }
            ssd1306_WriteString(buffer, Font_7x10, 1);
        }

        ssd1306_SetCursor(0, 54);
        snprintf(buffer, sizeof(buffer), "Yaw:%6.1fdeg", g_hwt101_data.Angle_Z);
        ssd1306_WriteString(buffer, Font_7x10, 1);
    }

    ssd1306_UpdateScreen();
}

/**
 * @brief 检查测试是否完成
 * @return 1: 测试完成, 0: 测试进行中
 */
uint8_t Motor_Speed_Test_Is_Complete(void)
{
    return !g_test_active;
}
