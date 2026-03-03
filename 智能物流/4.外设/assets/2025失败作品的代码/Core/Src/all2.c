#include "visual_control.h"
#include "ft_servo.h"
#include "Motor_PID.h"
#include "angle_pid.h"
#include "All.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include "all2.h"
#include <string.h>
#include "all2.h"

void parse_openmv_data(void);
extern const unsigned char tongse[];
extern const unsigned char yise[];

static float forward_speed = 85; // 默认的直行速度（RPM），供多个 Task 共享
extern bool color_matched;
extern int center_x;
extern int center_y; // OpenMV图像中心点
extern char target_color;
extern bool g_is_decelerating;

int color[3] = {0, 0, 0}; // 0表示空 1蓝色 2绿色 3红色
void get_color()
{
    visual_align_blocking_V3_111();
    HAL_Delay(80);
    parse_openmv_data();
    HAL_Delay(50);
    if (target_color == 'b')
    {
        color[0] = 1;
    }
    else if (target_color == 'g')
    {
        color[0] = 2;
    }
    else if (target_color == 'r')
    {
        color[0] = 3;
    }
    Car_FORWARD(80, 253);
    Car_Reset_And_Sync_State();
    HAL_Delay(100);
    parse_openmv_data();
    HAL_Delay(50);
    if (target_color == 'b')
    {
        color[1] = 1;
    }
    else if (target_color == 'g')
    {
        color[1] = 2;
    }
    else if (target_color == 'r')
    {
        color[1] = 3;
    }
    Car_FORWARD(80, 253);
    Car_Reset_And_Sync_State();
    HAL_Delay(100);
    parse_openmv_data();
    HAL_Delay(50);
    if (target_color == 'b')
    {
        color[2] = 1;
    }
    else if (target_color == 'g')
    {
        color[2] = 2;
    }
    else if (target_color == 'r')
    {
        color[2] = 3;
    }
    Car_BACKWARD(80, 520);
    if (color[0] == color[1] && color[0] == color[2] && color[0] == ' ')
    {
        color[0] = 1;
        color[1] = 2;
        color[2] = 3;
    }
}

// 低处放置78 44
// 高处看 78 28
// 二维码之后
void task1()
{
    Car_Reset_And_Sync_State();
    HAL_Delay(150);
    set_openmv_light(1);
    HAL_Delay(20);

    Car_Move_Forward_Auto(500); // 走出来一点
    HAL_Delay(70);
    Car_Reset_And_Sync_State();
    HAL_Delay(1000);
    Car_Move_Left_Auto(215);

//************************************************** */
    HAL_Delay(70);
    Car_Reset_And_Sync_State();
    HAL_Delay(500);
    Car_Stop_PID();
    HAL_Delay(500);
    Car_Reset_And_Sync_State();
    HAL_Delay(70);
//************************************************** */

    HAL_Delay(70);
    Car_Reset_And_Sync_State();
    HAL_Delay(70);
    Car_Move_Forward_Auto(890);// 走到二维码位置

    // Car_Move_Forward_Auto(1390); // 走到二维码位置
    
    //Car_Move_Left_Auto(177); // 远离二维码
    HAL_Delay(70);
    Car_Reset_And_Sync_State();
    HAL_Delay(70);
}
void task2()
{
    Car_Move_Right_Auto(67);
    HAL_Delay(70);
    Car_Reset_And_Sync_State();
    HAL_Delay(70);

    Car_Move_Forward_Auto(1110);// 走到抓取位置
    HAL_Delay(70);
    Car_Reset_And_Sync_State();
    HAL_Delay(70);
    visual_align_blocking_V3_111();
    HAL_Delay(70);
    near_catch();
    HAL_Delay(70);
    visual_align_blocking_V3_111();
    HAL_Delay(70);

//************************************************** */
    Car_Move_Left_Auto(200);
    HAL_Delay(70);
    Car_Reset_And_Sync_State();
    HAL_Delay(70);
//************************************************** */
//************************************************** */


    Car_Move_Forward_Auto(580); // 走到第一个角

    HAL_Delay(70);
    Car_Reset_And_Sync_State();
    HAL_Delay(70);

    Car_Turn_To_Angle_PID(-90, 2200U);// 第一个转角,转90度

//************************************************** */
    HAL_Delay(70);
    Car_Reset_And_Sync_State();
    HAL_Delay(500);
    Car_Stop_PID();
    HAL_Delay(500);
    Car_Reset_And_Sync_State();
    HAL_Delay(70);
//************************************************** */
//************************************************** */


    Car_Move_Forward_Auto(1250); //走到凸台第一个中心
    HAL_Delay(300);
    Car_Reset_And_Sync_State();
    HAL_Delay(70);

//************************************************** *右移一点
    Car_Move_Right_Auto(200);
    HAL_Delay(70);
    Car_Reset_And_Sync_State();
    HAL_Delay(70);
//************************************************** */
//************************************************** */
}
void task3()
{
    // 视觉对齐
    visual_align_blocking_V3_111_2();
    
    HAL_Delay(70);
    near_place_all();
    HAL_Delay(70);
    near_place_all_catchpart();
    HAL_Delay(70);

    Car_Move_Forward_Auto(2110); // 离开凸台区
    HAL_Delay(70);
    Car_Reset_And_Sync_State();
    HAL_Delay(70);

    // Car_LEFT(355); // 第2个转角,转90度
    Car_Turn_To_Angle_PID(-90, 2200U);
    HAL_Delay(50);
    Car_Reset_And_Sync_State();
    HAL_Delay(70);
    Car_Reset_And_Sync_State();
    HAL_Delay(70);

//************************************************** */
    HAL_Delay(70);
    Car_Reset_And_Sync_State();
    HAL_Delay(500);
    Car_Stop_PID();
    HAL_Delay(500);
    Car_Reset_And_Sync_State();
    HAL_Delay(70);
//************************************************** */
}
void task4()
{
    Car_Reset_And_Sync_State();
    HAL_Delay(70);
    Car_Move_Forward_Auto(1170); // 走到放置区
    HAL_Delay(70);
    Car_Reset_And_Sync_State();
    HAL_Delay(70);

//************************************************** *右移一点
    Car_Move_Right_Auto(200);
    HAL_Delay(70);
    Car_Reset_And_Sync_State();
    HAL_Delay(70);
//************************************************** */
//************************************************** */

    near_place_all_low();

    Car_Move_Back_Auto(120);
    HAL_Delay(70);
    Car_Reset_And_Sync_State();
    HAL_Delay(70);

    Car_Turn_To_Angle_PID(-90, 2200U);
    HAL_Delay(50);
    Car_Reset_And_Sync_State();
    HAL_Delay(70);
    Car_Reset_And_Sync_State();
    HAL_Delay(200);
//************************************************** */
    HAL_Delay(70);
    Car_Reset_And_Sync_State();
    HAL_Delay(500);
    Car_Stop_PID();
    HAL_Delay(500);
    Car_Reset_And_Sync_State();
    HAL_Delay(70);
//************************************************** */
}
void task5()
{
    Car_Move_Back_Auto(100);//****************************** */
    HAL_Delay(70);
    Car_Reset_And_Sync_State();
    HAL_Delay(200);
    Car_Move_Forward_Auto(3640); // 去物料台位置
    HAL_Delay(200);
    Car_Reset_And_Sync_State();
    HAL_Delay(70);
    Car_Turn_To_Angle_PID(-90, 2200U);
    HAL_Delay(50);
    Car_Reset_And_Sync_State();
    HAL_Delay(100);
    // Car_FORWARD(60, 190);
    // HAL_Delay(70);
    Car_Reset_And_Sync_State();
    HAL_Delay(70);

    // Car_Move_Forward_Auto(100);
    // HAL_Delay(70);
    // Car_Reset_And_Sync_State();
    // HAL_Delay(70);

    
    // center_y = 16; // 这是看高处的
    // far_catch_all();
//************************************************** *右移一点
    Car_Move_Right_Auto(150);
    HAL_Delay(70);
    Car_Reset_And_Sync_State();
    HAL_Delay(70);
//************************************************** */
//************************************************** */

    visual_align_blocking_V3_111();
    HAL_Delay(70);
    far_catch_all_seenear(); // 近处定位

    HAL_Delay(70);
    Car_Reset_And_Sync_State();
    HAL_Delay(70);
    Car_Move_Forward_Auto(800);/************第二圈转前 */
    HAL_Delay(70);
    Car_Turn_To_Angle_PID(-90, 2200U);

//************************************************** */
    HAL_Delay(70);
    Car_Reset_And_Sync_State();
    HAL_Delay(500);
    Car_Stop_PID();
    HAL_Delay(500);
    Car_Reset_And_Sync_State();
    HAL_Delay(70);
//************************************************** */

    HAL_Delay(50);
    Car_Reset_And_Sync_State();
    HAL_Delay(70);
    Car_Reset_And_Sync_State();
    HAL_Delay(50);
    // Car_Process_Stop_Request();
    // HAL_Delay(50);
    // Car_Process_Stop_Request();
    // HAL_Delay(50);
    Car_Move_Forward_Auto(1270); // 走到凸台第一个中心？？？无法执行
    HAL_Delay(300);
    Car_Reset_And_Sync_State();
    HAL_Delay(70);
}

// 第二圈
void task6() // 重复了，使用task7
{
    Car_Reset_And_Sync_State();
    HAL_Delay(70);
    visual_align_blocking_V3_111();
    HAL_Delay(70);

    // far_catch_all(); 写错了
    near_place_all_2(); // 凸台放置
    HAL_Delay(70);
    Car_Move_Forward_Auto(1300); // 走到第一个角
    HAL_Delay(70);
    Car_Reset_And_Sync_State();
    HAL_Delay(70);
    //  Car_LEFT(355); // 第一个转角,转90度
    Car_Turn_To_Angle_PID(-90, 2200U);
    HAL_Delay(50);
    //************************************************** */
    HAL_Delay(70);
    Car_Reset_And_Sync_State();
    HAL_Delay(500);
    Car_Stop_PID();
    HAL_Delay(500);
    Car_Reset_And_Sync_State();
    HAL_Delay(70);
//************************************************** */
    Car_Reset_And_Sync_State();
    HAL_Delay(50);
    Car_Reset_And_Sync_State();
    HAL_Delay(20);
    Car_Reset_And_Sync_State();
    HAL_Delay(200);
    Car_Move_Forward_Auto(1250); // 走到凸台第一个中心
    HAL_Delay(300);
    Car_Reset_And_Sync_State();
    HAL_Delay(70);
}

void task7() 
{

    
    visual_align_blocking_V3_111_2();
    HAL_Delay(70);

//************************************************** *右移一点
    Car_Move_Right_Auto(100);
    HAL_Delay(70);
    Car_Reset_And_Sync_State();
    HAL_Delay(70);
//************************************************** */
//************************************************** */

    near_place_all_2(); // 凸台放置
    HAL_Delay(70);
    near_place_all_catchpart(); // ？
    HAL_Delay(70);

    HAL_Delay(70);
    Car_Reset_And_Sync_State();
    HAL_Delay(70);

    Car_Move_Forward_Auto(2550); // 离开凸台区
    HAL_Delay(70);
    Car_Reset_And_Sync_State();
    HAL_Delay(70);

    // Car_LEFT(355); // 第2个转角,转90度
    Car_Turn_To_Angle_PID(-90, 2200U);
    HAL_Delay(30);
    Car_Reset_And_Sync_State();
    HAL_Delay(30);
    Car_Reset_And_Sync_State();
    HAL_Delay(70);
    //************************************************** */
    HAL_Delay(70);
    Car_Reset_And_Sync_State();
    HAL_Delay(500);
    Car_Stop_PID();
    HAL_Delay(500);
    Car_Reset_And_Sync_State();
    HAL_Delay(70);
//************************************************** */
}

void task8()
{
    Car_Reset_And_Sync_State();
    HAL_Delay(70);
    Car_Move_Forward_Auto(1260); // 走到放置区
    HAL_Delay(70);
    Car_Reset_And_Sync_State();
    HAL_Delay(70);

//************************************************** *右移一点
    Car_Move_Right_Auto(100);
    HAL_Delay(70);
    Car_Reset_And_Sync_State();
    HAL_Delay(70);
//************************************************** */
//************************************************** */

    near_place_all_low_2();
    HAL_Delay(70);
    // 抓一组
    catch_group();
}

void task()
{
    Car_Reset_And_Sync_State();
    HAL_Delay(50);
    Car_Move_Forward_Auto(1250);

    HAL_Delay(70);
    Car_Reset_And_Sync_State();
    HAL_Delay(70);

    // Car_LEFT(355); // 
    Car_Turn_To_Angle_PID(-90, 2200U);
    HAL_Delay(30);
    Car_Reset_And_Sync_State();
    HAL_Delay(30);
    Car_Reset_And_Sync_State();
    HAL_Delay(70);
    //************************************************** */
    HAL_Delay(70);
    Car_Reset_And_Sync_State();
    HAL_Delay(500);
    Car_Stop_PID();
    HAL_Delay(500);
    Car_Reset_And_Sync_State();
    HAL_Delay(70);
//************************************************** */

    HAL_Delay(70);
    Car_Reset_And_Sync_State();
    HAL_Delay(70);
    Car_Move_Forward_Auto(3700);
    HAL_Delay(70);
    final_destination_blocking();

    // Car_Turn_To_Angle_PID(90, 2200U);
    // HAL_Delay(70);
    // Car_Reset_And_Sync_State();
    // HAL_Delay(70);
    // Car_Move_Forward_Auto(100);
}

bool Car_QR_Wait_And_Display(bool *out_color_matched, uint32_t timeout_ms)
{
    const uint8_t target_qr_content[] = {0xD2, 0xEC, 0xC9, 0xAB, 0xB4, 0xED, 0xC5, 0xE4};
    const size_t target_qr_len = sizeof(target_qr_content);
    const uint32_t start_tick = HAL_GetTick();
    uint32_t last_update_tick = 0U;
    bool has_valid_frame = false;
    bool latest_color_matched = false;

    if (out_color_matched != NULL)
    {
        *out_color_matched = false;
    }

    while (1)
    {
        uint32_t now = HAL_GetTick();

        if (timeout_ms > 0U && (now - start_tick) >= timeout_ms)
        {
            if (has_valid_frame)
            {
                if (out_color_matched != NULL)
                {
                    *out_color_matched = latest_color_matched;
                }
                return true;
            }
            return false;
        }

        if (qr_has_new_data())
        {
            uint8_t qr_data[256];
            uint16_t len = qr_get_data(qr_data);

            if (len == 0U)
            {
                continue;
            }

            ssd1306_Fill(0);
            ssd1306_UpdateScreen();
            HAL_Delay(500);

            if (len >= target_qr_len && memcmp(qr_data, target_qr_content, target_qr_len) == 0)
            {
                latest_color_matched = false;
                ssd1306_DrawBitmap(yise, 0, 0, 128, 64);
            }
            else
            {
                latest_color_matched = true;
                ssd1306_DrawBitmap(tongse, 0, 0, 128, 64);
            }

            ssd1306_UpdateScreen();
            has_valid_frame = true;
            last_update_tick = HAL_GetTick();
            continue;
        }

        if (has_valid_frame)
        {
            if ((now - last_update_tick) >= 1000U)
            {
                if (out_color_matched != NULL)
                {
                    *out_color_matched = latest_color_matched;
                }
                return true;
            }
        }

        HAL_Delay(5);
    }
}

// ========== Motor PID 封装函数实现 ==========
// 将原有的运动控制+等待逻辑封装成一个函数调用

/**
 * @brief 位置模式运动辅助函数——带平滑起步与刹车
 * @param pulses 位置模式脉冲数，正数前进，负数后退
 * @param timeout_ms 等待到位的最大时长
 * @retval bool true 表示到位并完成减速，false 表示超时失败
 */
bool Car_Move_Position_Smooth(int32_t pulses, uint32_t timeout_ms)
{
    Car_Go_Straight_Pulses_PID(pulses);

    if (!Car_Wait_For_Position_Stop(timeout_ms))
    {
        Car_Stop_PID();
        return false;
    }

    Car_Stop_Smooth_PID();

    uint32_t decel_start_tick = HAL_GetTick();
    while (g_is_decelerating && (HAL_GetTick() - decel_start_tick) < 600U)
    {
        HAL_Delay(5);
    }

    Car_Stop_PID();
    return true;
}

/**
 * @brief 小车向前运动并自动等待到位（带超时保护）
 * @param pulses 运动的脉冲数，正数向前，负数向后
 */
void Car_Move_Forward_Auto(int32_t pulses)
{
    Car_Go_Straight_Pulses_PID(pulses);
    if (!Car_Wait_For_Position_Stop(5000U))
    {
        Car_Stop_PID();
    }
    // (void)Car_Move_Position_Smooth(pulses, 5000U);
}

/**
 * @brief 小车向左平移并自动等待到位（带超时保护）
 * @param pulses 运动的脉冲数
 * @retval bool 成功到达目标位置返回true，超时返回false
 */
void Car_Move_Left_Auto(uint32_t pulses)
{
    Car_Strafe_Left_Pulses_PID(pulses);

    if (!Car_Wait_For_Position_Stop(5000U))
    {
        Car_Stop_PID();
    }
}

/**
 * @brief 小车向右平移并自动等待到位（带超时保护）
 * @param pulses 运动的脉冲数
 * @retval bool 成功到达目标位置返回true，超时返回false
 */
void Car_Move_Right_Auto(uint32_t pulses)
{
    Car_Strafe_Right_Pulses_PID(pulses);

    if (!Car_Wait_For_Position_Stop(5000U))
    {
        Car_Stop_PID();
    }
}

/**
 * @brief 小车后退并自动等待到位（带超时保护）
 * @param pulses 运动的脉冲数
 */
void Car_Move_Back_Auto(uint32_t pulses)
{
    Car_Back_Pulses_PID(pulses);
    if (!Car_Wait_For_Position_Stop(5000U))
    {
        Car_Stop_PID();
    }
    // (void)Car_Move_Position_Smooth(-(int32_t)pulses, 5000U);
}
