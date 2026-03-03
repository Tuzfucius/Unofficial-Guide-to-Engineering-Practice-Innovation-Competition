#include "visual_control.h"
#include "all2.h"
#include <stdio.h>
#include <stdlib.h> // 用于 abs() 函数

// 引用在 main.c 中定义的 parse_openmv_data 函数
void parse_openmv_data(void);

// 定义对准过程中的状态
typedef enum
{
    ALIGN_STATE_ROTATING,   // 阶段一：旋转对准角度
    ALIGN_STATE_TRANSLATING // 阶段二：平移对准中心
} AlignState;

/**
 * @brief  【内部辅助函数】根据误差计算并执行小车平移 (此函数保持不变)
 * @retval bool: true 表示已在死区内, false 表示仍在移动或未找到目标.
 */
static bool process_and_move(float move_speed_rpm, int dead_zone_pixels)
{
    // ... 您的 process_and_move 函数代码无需任何修改 ...
    if (target_x == -1)
    {
        Car_Strafe_Right_PID(move_speed_rpm);

        // Car_Stop_PID();
        // return false;
    }
    int error_x = target_x - center_x;
    int error_y = target_y - center_y;
    if (abs(error_x) <= dead_zone_pixels && abs(error_y) <= dead_zone_pixels)
    {
        Car_Stop_PID();
        return true;
    }
    if (abs(error_x) > abs(error_y))
    {
        if (error_x > dead_zone_pixels)
        {
            Car_Back_PID(move_speed_rpm);
            // Car_Back_Pulses_PID(40);
            // Car_Move_Back_Auto(40);
        }
        else
        {
            Car_Go_PID(move_speed_rpm);
            //Car_Go_Straight_Pulses_PID(40);
            // Car_Move_Forward_Auto(40);
        }
    }
    else
    {
        if (error_y > dead_zone_pixels)
        {
            Car_Strafe_Left_PID(move_speed_rpm);
        }
        else
        {
            Car_Strafe_Right_PID(move_speed_rpm);
        }
    }
    return false;
}

/**
 * @brief  【“一键式”阻塞型】视觉对准函数实现 (V2 - 先旋转后平移).
 * @param  target_angle_deg   目标角度 (例如 90)
 * @param  angle_dead_zone    角度死区 (例如 5 度, 表示 85-95 度都算对准)
 * @param  rotate_speed_rpm   旋转时的电机速度
 * @param  move_speed_rpm     平移时的电机速度
 * @param  dead_zone_pixels   平移时的中心点死区 (像素)
 * @param  timeout_ms         总超时时间 (毫秒)
 * @retval bool: true 表示对准成功, false 表示超时或失败.
 */
bool visual_align_blocking_V2(int angle_dead_zone, float rotate_speed_rpm,
                              float move_speed_rpm, int dead_zone_pixels, uint32_t timeout_ms)
{
    int target_angle_deg = 90;
    uint32_t start_time = HAL_GetTick();
    AlignState current_state = ALIGN_STATE_TRANSLATING; // ALIGN_STATE_ROTATING; // 初始状态为旋转

    while (1)
    {
        // 1. 检查是否超时 (对整个过程有效)
        if (timeout_ms > 0 && (HAL_GetTick() - start_time > timeout_ms))
        {
            Car_Stop_PID();
            printf("Error: Visual alignment timed out after %lu ms.\r\n", timeout_ms);
            return false;
        }

        // 2. 统一的数据解析入口
        if (openmv_data_ready)
        {
            openmv_data_ready = false;
            parse_openmv_data();
        }

        // 3. 根据当前状态执行不同逻辑
        if (current_state == ALIGN_STATE_ROTATING)
        {
            // --- 旋转阶段 ---
            if (target_angle == -1 && target_x != -1) // 如果在旋转时丢失了直线目标
            {
                Car_Strafe_Left_PID(move_speed_rpm + 2);
                // Car_Stop_PID_Enhanced(); // 停车等待目标重新出现
                // printf("Warning: Line target lost during rotation. Waiting...\r\n");
            }
            else
            {
                int angle_error = target_angle_deg - target_angle;

                if (abs(angle_error) <= angle_dead_zone)
                {
                    // 角度已对准！
                    Car_Stop_PID_Enhanced();
                    current_state = ALIGN_STATE_TRANSLATING; // 切换到平移状态
                    printf("Phase 1 Complete: Angle aligned (%d deg). Moving to translation.\r\n", target_angle);
                }
                else
                {
                    // 根据误差方向进行旋转
                    // 已知：右转角度减小
                    if (angle_error > 0)
                    {
                        Car_TurnLeft_PID(rotate_speed_rpm);
                    }
                    else
                    {
                        Car_TurnRight_PID(rotate_speed_rpm);
                    }
                }
            }
        }
        else // current_state == ALIGN_STATE_TRANSLATING
        {
            // --- 平移阶段 ---
            if (process_and_move(move_speed_rpm, dead_zone_pixels))
            {
                // 如果 process_and_move 返回 true, 说明平移也完成了
                printf("Success: Phase 2 Complete. Alignment finished in %lu ms.\r\n", HAL_GetTick() - start_time);
                return true; // 整个对准过程成功！
            }
            // 如果返回 false, 会在下一次循环中继续尝试平移
        }

        // 给予适当延时, 避免CPU空转
        HAL_Delay(5);
    }
}
void visual_align_blocking_111(void)
{
    visual_align_blocking_V2(1, 10, 10, 2, 5000);
}

#include "visual_control.h"
#include <stdio.h>
#include <stdlib.h> // 用于 abs()
#include <math.h>   // 用于 fabsf()

/**
 * @brief  【优化版】专用于角度对准的高精度视觉对准函数
 * @note   移除平移功能，专注于角度对准，提高响应精度和速度
 * @param  target_angle_deg   目标角度 (例如 90)
 * @param  angle_dead_zone    角度死区 (例如 1 度)
 * @param  rotate_speed_rpm   旋转速度
 * @param  timeout_ms         超时时间 (毫秒)
 * @retval bool: true 表示对准成功, false 表示超时或失败
 */
bool visual_align_blocking_V3(int target_angle_deg, int angle_dead_zone, float rotate_speed_rpm, uint32_t timeout_ms)
{
    uint32_t start_time = HAL_GetTick();
    uint32_t last_control_time = start_time;
    const uint32_t CONTROL_PERIOD_MS = 2; // 控制周期2ms，提高响应速度

    while (1)
    {
        uint32_t current_time = HAL_GetTick();

        // 1. 检查超时
        if (timeout_ms > 0 && (current_time - start_time > timeout_ms))
        {
            Car_Stop_PID_Enhanced();
            return false;
        }

        // 2. 更新视觉数据（每次循环都检查）
        if (openmv_data_ready)
        {
            openmv_data_ready = false;
            parse_openmv_data();
        }

        // 3. 按固定控制周期执行控制逻辑（提高精度）
        if (current_time - last_control_time >= CONTROL_PERIOD_MS)
        {
            last_control_time = current_time;

            if (target_angle == -1)
            {
                // 丢失目标，停车等待
                Car_Stop_PID_Enhanced();
            }
            else
            {
                int angle_error = target_angle - target_angle_deg;

                if (abs(angle_error) <= angle_dead_zone)
                {
                    // 角度对准完成！
                    Car_Stop_PID_Enhanced();
                    return true;
                }
                else
                {
                    // 根据误差方向旋转
                    if (angle_error > 0)
                        Car_TurnRight_PID(rotate_speed_rpm);
                    else
                        Car_TurnLeft_PID(rotate_speed_rpm);
                }
            }
        }

        // 4. 短暂延时，避免CPU占用过高，但保持高响应性
        HAL_Delay(1);
    }
}

void visual_align_blocking_V3_111(void)
{
    parse_openmv_data();
    visual_align_blocking_V3(
        90,    // 目标角度
        1,     // 角度死区
        15.0f, // 旋转速度
        1600   // 超时时间(ms)
    );
    HAL_Delay(100);
    visual_align_blocking_V2(1, 13, 14, 2, 6000);
    HAL_Delay(100);
    // visual_align_blocking_V3(
    //     90,    // 目标角度
    //     1,     // 角度死区
    //     15.0f, // 旋转速度
    //     1,     // 位置死区 (像素)
    //     14.0f, // 最大平移速度
    //     8.0f,  // 最小平移速度
    //     0.4f,  // 平移比例Kp
    //     0.8f,  // 陀螺仪锁定Kp
    //     1600   // 超时20秒
    // );
    // HAL_Delay(80);
    // visual_align_blocking_V2(1, 13, 13, 1, 6000);
}
void visual_align_blocking_V3_111_2(void)
{
    parse_openmv_data();
    visual_align_blocking_V3(
        90,    // 目标角度
        1,     // 角度死区
        15.0f, // 旋转速度
        3200   // 超时时间(ms)
    );
    HAL_Delay(100);
    visual_align_blocking_V2(1, 13, 14, 2, 6000);
    HAL_Delay(100);
}
void final_destination_blocking(void)
{
    // Car_Strafe_Left_Smooth(42, 250);
    // visual_align_blocking_V3(
    //     90,    // 目标角度
    //     1,     // 角度死区
    //     13.0f, // 旋转速度
    //     1,     // 位置死区 (像素)
    //     14.0f, // 最大平移速度
    //     8.0f,  // 最小平移速度
    //     0.4f,  // 平移比例Kp
    //     0.8f,  // 陀螺仪锁定Kp
    //     1600   // 超时20秒
    // );
    HAL_Delay(100);
    // toggle_openmv_mode();
    Car_FORWARD(15, 150);
    HAL_Delay(100);
    // visual_align_blocking_V2(1, 13, 13, 9, 6000);
    // HAL_Delay(100);
    Car_FORWARD(50, 300);
    HAL_Delay(100);
    Car_Strafe_Right_Smooth(50, 640);
}