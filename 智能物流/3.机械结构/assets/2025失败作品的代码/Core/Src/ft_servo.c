#include <main.h>
#include <ft_servo.h>
#include "datou_pwm.h"
#include "Motor_PID.h"
#include "angle_pid.h"
#include "visual_control.h"
#include "all2.h"

// 运动封装函数声明，避免隐式声明编译告警
void Car_Move_Forward_Auto(int32_t pulses);
void Car_Move_Left_Auto(uint32_t pulses);
void Car_Move_Right_Auto(uint32_t pulses);
void Car_Move_Back_Auto(uint32_t pulses);

// 4095 一圈
// 1 物料盘 蓝色827
// 2 主旋转轴  正对1200 回头2610    抓取右侧1770 左侧675
// 3 前后 最远5200 最近393 物料1440   抓取右侧3540 左侧2770
// 4 夹爪 闭合2550 张开2170  后面抓2450
// Datou_PWM_Move_Steps(0, 6800, 140);
// 步进电机最低6800  0下降 1上升
// 4350放置高度

extern UART_HandleTypeDef huart5;

// 初始位置
void servo_zero()
{
    servo_blue();
    servo_open();
    servo_main_front();
    servo_move_back();
}

// 物料盘
void servo_blue()
{
    Servo_SendPositionAccelSpeedCommand(&huart5, 1, 2400, 1807, 3400);
    HAL_Delay(200);
}
void servo_green()
{
    Servo_SendPositionAccelSpeedCommand(&huart5, 1, 2400, 1807 - 1364, 3400);
    HAL_Delay(200);
}
void servo_red()
{
    Servo_SendPositionAccelSpeedCommand(&huart5, 1, 2400, 1807 - 1364 - 1364, 3400);
    HAL_Delay(200);
}

// 夹爪
void servo_close()
{
    Servo_SendPositionAccelSpeedCommand(&huart5, 4, 2500, 2400-470, 2100);
    HAL_Delay(200);
}
void servo_open()
{
    Servo_SendPositionAccelSpeedCommand(&huart5, 4, 2500, 1913-470, 2100);
    HAL_Delay(200);
}
void servo_half_open()
{
    Servo_SendPositionAccelSpeedCommand(&huart5, 4, 2500, 2150-470, 2100);
    HAL_Delay(200);
}
// 主旋转轴

void servo_main_front()
{
    Servo_SendPositionAccelSpeedCommand(&huart5, 2, 900, 1200, 1500);
    HAL_Delay(200);
}
void servo_main_back()
{
    Servo_SendPositionAccelSpeedCommand(&huart5, 2, 900, 2580, 1500);
    HAL_Delay(200);
}
void servo_main_left()
{
    Servo_SendPositionAccelSpeedCommand(&huart5, 2, 900, 790, 1500);
    HAL_Delay(200);
}
void servo_main_right()
{
    Servo_SendPositionAccelSpeedCommand(&huart5, 2, 900, 1560, 1500);
    HAL_Delay(200);
}

// 前后移动
void servo_move_front() // 最远
{
    Servo_SendPositionAccelSpeedCommand(&huart5, 3, 1500, 5150 + 1126 + 526 - 4095, 3000);
    HAL_Delay(200);
}
void servo_move_back() // 最近
{
    Servo_SendPositionAccelSpeedCommand(&huart5, 3, 1500, 520 + 1126 + 526 - 4095, 3000);
    HAL_Delay(200);
}
void servo_move_place()
{
    // Servo_SendPositionAccelSpeedCommand(&huart5, 3, 1500, 1440 - 4095, 3000);
    Servo_SendPositionAccelSpeedCommand(&huart5, 3, 1500, 520 + 1126 + 526 - 4095, 3000);
    HAL_Delay(200);
}
void servo_move_left()
{
    Servo_SendPositionAccelSpeedCommand(&huart5, 3, 1500, 3050 + 1126 + 526 - 4095, 3000);
    HAL_Delay(200);
}
void servo_move_right()
{
    Servo_SendPositionAccelSpeedCommand(&huart5, 3, 1500, 2656 + 1126 + 526 - 4095, 3000);
    HAL_Delay(200);
}
// ------------------------------------------------------------------------
// 高级封装

// 抓一个中间
void near_catch_one()
{
    Datou_PWM_Move_Steps(0, 6800, 160);
    HAL_Delay(1000);
    servo_close();
    HAL_Delay(200);
    Datou_PWM_Move_Steps(1, 6800, 160);
    HAL_Delay(1000);
    // servo_green();
    HAL_Delay(800);
    servo_main_back();
    HAL_Delay(900);
    servo_move_place();
    HAL_Delay(500);
    Datou_PWM_Move_Steps(0, 4350, 160);
    HAL_Delay(800);
    servo_half_open();
    HAL_Delay(500);
    Datou_PWM_Move_Steps(1, 4350, 160);
    HAL_Delay(800);
    servo_move_back();
    HAL_Delay(300);
    servo_main_front();
    HAL_Delay(300);
    servo_open();
    HAL_Delay(200);
}
void near_catch()
{
    center_y = 20; // 高处看
    // 中间绿色
    Datou_PWM_Move_Steps(0, 6800, 160);
    HAL_Delay(1000);
    servo_close();
    HAL_Delay(200);
    Datou_PWM_Move_Steps(1, 6800, 160);
    HAL_Delay(1000);
    servo_green();
    HAL_Delay(800);
    servo_main_back();
    HAL_Delay(900);
    servo_move_place();
    HAL_Delay(500);
    Datou_PWM_Move_Steps(0, 4350, 160);
    HAL_Delay(800);
    // servo_open();
    servo_half_open();
    HAL_Delay(300);
    Datou_PWM_Move_Steps(1, 4350, 160);
    HAL_Delay(800);
    servo_open();
    HAL_Delay(100);
    // servo_move_back();
    // HAL_Delay(300);
    // servo_main_front();
    // HAL_Delay(1000);

    // 右侧蓝色
    servo_main_right();
    HAL_Delay(500);
    servo_move_right();
    HAL_Delay(500);
    Datou_PWM_Move_Steps(0, 6400, 160);
    HAL_Delay(1000);
    servo_close();
    HAL_Delay(200);
    Datou_PWM_Move_Steps(1, 6400, 160);
    HAL_Delay(1000);
    servo_blue();
    HAL_Delay(200);
    servo_move_place();
    HAL_Delay(500);
    servo_main_back();
    HAL_Delay(900);
    Datou_PWM_Move_Steps(0, 4350, 160);
    HAL_Delay(800);
    // servo_open();
    servo_half_open();
    HAL_Delay(300);
    Datou_PWM_Move_Steps(1, 4350, 160);
    HAL_Delay(800);
    servo_open();
    HAL_Delay(100);
    // servo_move_back();
    // HAL_Delay(300);
    // servo_main_front();
    // HAL_Delay(1000);

    // 左侧红色
    servo_main_left();
    HAL_Delay(500);
    servo_move_left();
    HAL_Delay(500);
    Datou_PWM_Move_Steps(0, 6400, 160);
    HAL_Delay(1000);
    servo_close();
    HAL_Delay(200);
    Datou_PWM_Move_Steps(1, 6400, 160);
    HAL_Delay(1000);
    servo_red();
    HAL_Delay(200);
    servo_move_place();
    HAL_Delay(500);
    servo_main_back();
    HAL_Delay(900);
    Datou_PWM_Move_Steps(0, 4350, 160);
    HAL_Delay(800);
    // servo_open();
    servo_half_open();
    HAL_Delay(300);
    Datou_PWM_Move_Steps(1, 4350, 160);
    HAL_Delay(800);
    servo_move_back();
    HAL_Delay(500);
    servo_main_front();
    HAL_Delay(200);
    servo_open();
    HAL_Delay(100);
}

// 这个只能抓一个
void far_catch()
{
    // // center_y = 65; // 低处看
    // servo_open();
    // HAL_Delay(200);
    // servo_main_front();
    // HAL_Delay(1000);
    // servo_move_front();
    // HAL_Delay(2700);
    // Datou_PWM_Move_Steps(0, 1400, 160);
    // HAL_Delay(800);
    // servo_close();
    // HAL_Delay(300);
    // Datou_PWM_Move_Steps(1, 1400, 160);
    // HAL_Delay(800);
    // servo_move_place();
    // HAL_Delay(500);
    // servo_main_back();
    // HAL_Delay(1200);
    // Datou_PWM_Move_Steps(0, 3500, 160);
    // HAL_Delay(800);
    // // servo_open();
    // servo_half_open();
    // HAL_Delay(350);
    // Datou_PWM_Move_Steps(1, 3500, 160);
    // HAL_Delay(800);
    // servo_main_front();
    // HAL_Delay(800);
    // servo_open();
    // HAL_Delay(100);

    servo_close();
    HAL_Delay(90);
    servo_move_place();
    HAL_Delay(500);
    servo_main_back();
    HAL_Delay(800);
    Datou_PWM_Move_Steps(0, 3800, 160);
    HAL_Delay(500);
    servo_half_open();
    HAL_Delay(60);
    Datou_PWM_Move_Steps(1, 3800, 160);
    HAL_Delay(500);
    servo_open();
    HAL_Delay(80);
    servo_main_front();
    HAL_Delay(900);
}
extern int color[3];
void far_catch_all()
{
    servo_zero();
    HAL_Delay(100);
    servo_move_front();
    center_y = 16;
    // 需要伸长到最前面进行定位
    // 抓取先收缩再旋转

    servo_blue();
    HAL_Delay(80);
    visual_align_blocking_V3_111();
    HAL_Delay(500);
    far_catch();
    Car_FORWARD(80, 273);
    HAL_Delay(100);

    servo_move_front();
    HAL_Delay(1000);
    servo_green();
    HAL_Delay(80);
    visual_align_blocking_V3_111();
    HAL_Delay(500);
    far_catch();
    Car_FORWARD(80, 273);
    HAL_Delay(100);

    servo_move_front();
    HAL_Delay(1000);
    servo_red();
    HAL_Delay(80);
    visual_align_blocking_V3_111();
    HAL_Delay(500);
    far_catch();
}
void far_catch_all_seenear() // 看到近处进行定位
{
    servo_zero();
    HAL_Delay(100);
    // servo_move_front();
    // center_y = 16;
    // 需要伸长到最前面进行定位
    // 抓取先收缩再旋转

    servo_blue();
    HAL_Delay(80);
    visual_align_blocking_V3_111();
    HAL_Delay(170);
    servo_move_front();
    HAL_Delay(1000);
    far_catch();
    Car_FORWARD(80, 273);
    HAL_Delay(100);

    // HAL_Delay(1000);
    servo_green();
    HAL_Delay(80);
    visual_align_blocking_V3_111();
    HAL_Delay(170);
    servo_move_front();
    HAL_Delay(1000);
    far_catch();
    Car_FORWARD(80, 273);
    HAL_Delay(100);

    // servo_move_front();
    // HAL_Delay(1000);
    servo_red();
    HAL_Delay(80);
    visual_align_blocking_V3_111();
    HAL_Delay(170);
    servo_move_front();
    HAL_Delay(1000);
    far_catch();
}
void near_place() // 这是放在低处
{
    // center_y = 44;
    // 放置第一个
    HAL_Delay(500);
    visual_align_blocking_V3_111();
    HAL_Delay(100);
    // servo_blue();
    // HAL_Delay(500);
    servo_move_place();
    HAL_Delay(500);
    servo_half_open();
    HAL_Delay(100);
    servo_main_back();
    HAL_Delay(700);
    Datou_PWM_Move_Steps(0, 4350, 160);
    HAL_Delay(1000);
    servo_close();
    HAL_Delay(500);
    Datou_PWM_Move_Steps(1, 4350, 160);
    HAL_Delay(800);
    servo_main_front();
    HAL_Delay(1000);
    servo_move_back();
    HAL_Delay(500);
    Datou_PWM_Move_Steps(0, 6800, 140);
    HAL_Delay(1000);
    servo_open();
    HAL_Delay(500);
    Datou_PWM_Move_Steps(1, 6800, 140);
    HAL_Delay(600);
}
void near_place_high()
{
    //   center_y = 22;
    //   center_x = 76;
    // 放置第一个
    HAL_Delay(500);
    visual_align_blocking_V3_111();
    HAL_Delay(100);
    // servo_blue();
    // HAL_Delay(500);
    servo_move_place();
    HAL_Delay(500);
    servo_half_open();
    HAL_Delay(100);
    servo_main_back();
    HAL_Delay(700);
    Datou_PWM_Move_Steps(0, 4350, 160);
    HAL_Delay(1000);
    servo_close();
    HAL_Delay(500);
    Datou_PWM_Move_Steps(1, 4350, 160);
    HAL_Delay(800);
    servo_main_front();
    HAL_Delay(1000);
    servo_move_back();
    HAL_Delay(500);
    Datou_PWM_Move_Steps(0, 2162, 140); // 需要测试
    HAL_Delay(1000);
    servo_open();
    HAL_Delay(500);
    Datou_PWM_Move_Steps(1, 2162, 140);
    HAL_Delay(600);
}
void near_place_high_2() // 二层放在凸台
{
    //   center_y = 22;
    //   center_x = 76;
    // 放置第一个
    HAL_Delay(300);
    visual_align_blocking_V3_111();
    HAL_Delay(100);
    // servo_blue();
    // HAL_Delay(500);
    servo_move_place();
    HAL_Delay(500);
    servo_half_open();
    HAL_Delay(100);
    servo_main_back();
    HAL_Delay(700);
    Datou_PWM_Move_Steps(0, 3700, 160);
    HAL_Delay(1000);
    servo_close();
    HAL_Delay(500);
    Datou_PWM_Move_Steps(1, 3700, 160);
    HAL_Delay(800);
    servo_main_front();
    HAL_Delay(1000);
    servo_move_back();
    HAL_Delay(500);
    Datou_PWM_Move_Steps(0, 2212, 140); // 需要测试
    HAL_Delay(1000);
    servo_open();
    HAL_Delay(500);
    Datou_PWM_Move_Steps(1, 2212, 140);
    HAL_Delay(600);
}
void near_place_all_low()
{
    servo_zero();
    HAL_Delay(70);
    Car_Reset_And_Sync_State();
    HAL_Delay(70);

    HAL_Delay(200);
    servo_blue();
    HAL_Delay(500);
    near_place();
    Car_FORWARD(80, 273);
    // Car_Move_Forward_Auto(285);
    HAL_Delay(70);
    Car_Reset_And_Sync_State();
    HAL_Delay(70);

    HAL_Delay(200);
    servo_green();
    HAL_Delay(500);
    near_place();
    Car_FORWARD(80, 273);
    // Car_Move_Forward_Auto(285);
    HAL_Delay(70);
    Car_Reset_And_Sync_State();
    HAL_Delay(70);

    HAL_Delay(200);
    servo_red();
    HAL_Delay(500);
    near_place();
    // servo_green();
    HAL_Delay(200);
    // near_place();
}
extern bool color_matched;
void near_place_all_low_2() // 二层放在一层物料上方
{
    center_x = 78;
    center_y = 20;
    servo_zero();
    HAL_Delay(70);
    Car_Reset_And_Sync_State();
    HAL_Delay(70);

    if (color_matched == true)
    {
        // HAL_Delay(200);
        // if (color[0] == 1)
        // {
        //     servo_blue();
        // }
        // else if (color[0] == 2)
        // {
        //     servo_green();
        // }
        // else if (color[0] == 3)
        // {
        //     servo_red();
        // }
        servo_blue();
        HAL_Delay(500);
        near_place_high();
        Car_FORWARD(80, 273);
        // Car_Move_Forward_Auto(285);
        HAL_Delay(70);
        Car_Reset_And_Sync_State();
        HAL_Delay(70);

        HAL_Delay(200);
        // if (color[1] == 1)
        // {
        //     servo_blue();
        // }
        // else if (color[1] == 2)
        // {
        //     servo_green();
        // }
        // else if (color[1] == 3)
        // {
        //     servo_red();
        // }
        servo_green();
        HAL_Delay(500);
        near_place_high();
        Car_FORWARD(80, 273);
        // Car_Move_Forward_Auto(285);
        HAL_Delay(70);
        Car_Reset_And_Sync_State();
        HAL_Delay(70);

        HAL_Delay(200);
        // if (color[2] == 1)
        // {
        //     servo_blue();
        // }
        // else if (color[2] == 2)
        // {
        //     servo_green();
        // }
        // else if (color[2] == 3)
        // {
        //     servo_red();
        // }
        servo_red();
        HAL_Delay(500);
        near_place_high();
        // servo_green();
        HAL_Delay(200);
        // near_place();
    }
    else
    {
        HAL_Delay(200);
        servo_green();
        HAL_Delay(500);
        near_place_high();
        Car_FORWARD(80, 273);
        // Car_Move_Forward_Auto(285);
        HAL_Delay(70);
        Car_Reset_And_Sync_State();
        HAL_Delay(70);

        HAL_Delay(200);
        servo_red();
        HAL_Delay(500);
        near_place_high();
        Car_FORWARD(80, 273);
        // Car_Move_Forward_Auto(285);
        HAL_Delay(70);
        Car_Reset_And_Sync_State();
        HAL_Delay(70);

        HAL_Delay(200);
        servo_blue();
        HAL_Delay(500);
        near_place_high();
        // servo_green();
        HAL_Delay(200);
        // near_place();
    }
}
void near_place_all() //???
{
    servo_zero();
    HAL_Delay(70);
    Car_Reset_And_Sync_State();
    HAL_Delay(70);
    // HAL_Delay(200);
    // servo_blue();
    // HAL_Delay(200);
    // near_place();

    // Car_FORWARD(80, 213);
    // servo_green();
    // HAL_Delay(200);
    // near_place();

    // Car_FORWARD(80, 213);
    // servo_red();
    // HAL_Delay(200);
    // near_place();

    // servo_zero();
    // HAL_Delay(500);
    HAL_Delay(200);
    servo_blue();
    HAL_Delay(500);
    near_place_high();
    Car_FORWARD(80, 273);
    // Car_Move_Forward_Auto(285);
    HAL_Delay(70);
    Car_Reset_And_Sync_State();
    HAL_Delay(70);

    HAL_Delay(200);
    servo_green();
    HAL_Delay(500);
    near_place_high();
    Car_FORWARD(80, 273);
    // Car_Move_Forward_Auto(285);
    HAL_Delay(70);
    Car_Reset_And_Sync_State();
    HAL_Delay(70);

    HAL_Delay(200);
    servo_red();
    HAL_Delay(500);
    near_place_high();
    // servo_green();
    HAL_Delay(200);
    // near_place();
}
void near_place_all_2()
{
    center_y = 44;
    center_x = 75;

    servo_zero();
    HAL_Delay(70);
    Car_Reset_And_Sync_State();
    HAL_Delay(70);
    // HAL_Delay(200);
    // servo_blue();
    // HAL_Delay(200);
    // near_place();

    // Car_FORWARD(80, 213);
    // servo_green();
    // HAL_Delay(200);
    // near_place();

    // Car_FORWARD(80, 213);
    // servo_red();
    // HAL_Delay(200);
    // near_place();

    // servo_zero();
    // HAL_Delay(500);
    HAL_Delay(200);
    servo_blue();
    HAL_Delay(500);
    near_place_high_2();
    Car_FORWARD(80, 273);
    // Car_Move_Forward_Auto(285);
    HAL_Delay(70);
    Car_Reset_And_Sync_State();
    HAL_Delay(70);

    HAL_Delay(200);
    servo_green();
    HAL_Delay(500);
    near_place_high_2();
    Car_FORWARD(80, 273);
    // Car_Move_Forward_Auto(285);
    HAL_Delay(70);
    Car_Reset_And_Sync_State();
    HAL_Delay(70);

    HAL_Delay(200);
    servo_red();
    HAL_Delay(500);
    near_place_high_2();
    // servo_green();
    HAL_Delay(200);
    // near_place();
}
void near_place_all_catchpart()
{
    HAL_Delay(70);
    Car_Reset_And_Sync_State();
    HAL_Delay(70);
    servo_zero();
    HAL_Delay(200);
    servo_red();
    Datou_PWM_Move_Steps(0, 2100, 160);
    HAL_Delay(700);
    servo_close();
    HAL_Delay(200);
    Datou_PWM_Move_Steps(1, 2100, 160);
    servo_main_back();
    HAL_Delay(1000);
    Datou_PWM_Move_Steps(0, 4000, 160);
    HAL_Delay(800);
    servo_half_open();
    HAL_Delay(500);
    Datou_PWM_Move_Steps(1, 4000, 160);
    HAL_Delay(800);
    servo_open();
    servo_main_front();
    HAL_Delay(500);
    Car_BACKWARD(80, 223);
    // Car_Move_Back_Auto(285);
    HAL_Delay(70);
    Car_Reset_And_Sync_State();
    HAL_Delay(500);

    servo_green();
    Datou_PWM_Move_Steps(0, 2100, 160);
    HAL_Delay(700);
    servo_close();
    HAL_Delay(200);
    Datou_PWM_Move_Steps(1, 2100, 160);
    servo_main_back();
    HAL_Delay(1000);
    Datou_PWM_Move_Steps(0, 4000, 160);
    HAL_Delay(800);
    servo_half_open();
    HAL_Delay(500);
    Datou_PWM_Move_Steps(1, 4000, 160);
    HAL_Delay(800);
    servo_open();
    servo_main_front();
    HAL_Delay(500);
    Car_BACKWARD(80, 233);
    // Car_Move_Back_Auto(285);
    HAL_Delay(70);
    Car_Reset_And_Sync_State();
    HAL_Delay(500);

    servo_blue();
    Datou_PWM_Move_Steps(0, 2100, 160);
    HAL_Delay(700);
    servo_close();
    HAL_Delay(200);
    Datou_PWM_Move_Steps(1, 2100, 160);
    servo_main_back();
    HAL_Delay(1000);
    Datou_PWM_Move_Steps(0, 4000, 160);
    HAL_Delay(800);
    servo_half_open();
    HAL_Delay(500);
    Datou_PWM_Move_Steps(1, 4000, 160);
    HAL_Delay(800);
    servo_open();
    servo_main_front();
    HAL_Delay(500);
    // Car_BACKWARD(80,213);
    // HAL_Delay(500);
    visual_align_blocking_V3_111();
    HAL_Delay(70);
}
void near_place_catch()
{
    // 放置第一个
    HAL_Delay(500);
    visual_align_blocking_V3_111();
    HAL_Delay(100);
    // servo_blue();
    // HAL_Delay(500);
    servo_move_place();
    HAL_Delay(500);
    servo_main_back();
    HAL_Delay(700);
    Datou_PWM_Move_Steps(0, 4350, 160);
    HAL_Delay(800);
    servo_close();
    HAL_Delay(500);
    Datou_PWM_Move_Steps(1, 4350, 160);
    HAL_Delay(800);
    servo_main_front();
    HAL_Delay(1000);
    servo_move_back();
    HAL_Delay(500);
    Datou_PWM_Move_Steps(0, 2062, 140); // 需要测试，高凸台
    HAL_Delay(1000);
    servo_half_open();
    HAL_Delay(1300);
    servo_close();
    HAL_Delay(400);
    Datou_PWM_Move_Steps(1, 2062, 140);
    HAL_Delay(600);

    // servo_red();
    // HAL_Delay(200);
    servo_move_place();
    HAL_Delay(400);
    servo_main_back();
    HAL_Delay(900);
    Datou_PWM_Move_Steps(0, 4350, 160);
    HAL_Delay(800);
    servo_half_open();
    HAL_Delay(500);
    Datou_PWM_Move_Steps(1, 4350, 160);
    HAL_Delay(800);
    servo_move_back();
    HAL_Delay(500);
    servo_main_front();
    HAL_Delay(200);
}
void near_place_catch_all()
{
    // servo_zero();
    HAL_Delay(200);
    servo_blue();
    HAL_Delay(200);
    near_place_catch();

    // Car_Move_Forward_Auto(285);
    Car_FORWARD(80, 253);
    HAL_Delay(70);
    Car_Reset_And_Sync_State();
    HAL_Delay(70);
    // Car_FORWARD(80, 213);
    servo_green();
    HAL_Delay(70);
    near_place_catch();

    // Car_Move_Forward_Auto(285);
    Car_FORWARD(80, 253);
    HAL_Delay(70);
    Car_Reset_And_Sync_State();
    HAL_Delay(70);
    // Car_FORWARD(80, 213);
    servo_red();
    HAL_Delay(70);
    near_place_catch();

    HAL_Delay(500);
}
void catch_group()
{
    servo_zero();
    HAL_Delay(500);
    Datou_PWM_Move_Steps(0, 3102, 140); // 需要测试
    HAL_Delay(1000);
    servo_close();
    HAL_Delay(500);
    Datou_PWM_Move_Steps(1, 3102, 140);
    HAL_Delay(600);
    servo_main_back();
    HAL_Delay(800);
    Datou_PWM_Move_Steps(0, 4350, 160);
    HAL_Delay(400);
    // Datou_PWM_Move_Steps(1, 4350, 160);
}