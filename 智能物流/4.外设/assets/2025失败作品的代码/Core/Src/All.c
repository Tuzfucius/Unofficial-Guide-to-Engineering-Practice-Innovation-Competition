#include "All.h"

// #define TEST_MODE 0

// // 全局共享变量：OLED 字体、通用输出缓冲区和默认直行速度
// extern const SSD1306_Font_t Font_7x10;
// static char buffer[64];
// static float forward_speed = 75; // 默认的直行速度（RPM），供多个 Task 共享
// extern const unsigned char yise[];
// extern const unsigned char tongse[];

// // 原料区---------
// #define STEP1_TIME 5352 // 去原料区的时长
// #define STEP2_TIME 2000 // 离开原料区的时长

// /**
//  * @brief Task1
//  * @note  第一次去原料区
//  */
// void Task1(void)
// {
//   // 使用文件作用域的 Font_7x10、buffer 和 forward_speed

//   // 原料区 - 第一步：直行8秒测试

//   // 重置并同步状态
//   Car_Reset_And_Sync_State();
//   HAL_Delay(200);

//   // 仅在测试模式下显示OLED信息
//   if (TEST_MODE)
//   {
//     ssd1306_Fill(0);
//     ssd1306_SetCursor(0, 0);
//     ssd1306_WriteString("Step 1: Forward", Font_7x10, 1);
//     ssd1306_SetCursor(0, 12);
//     snprintf(buffer, sizeof(buffer), "Speed: %.1f RPM", forward_speed);
//     ssd1306_WriteString(buffer, Font_7x10, 1);
//     ssd1306_SetCursor(0, 24);
//     ssd1306_WriteString("Running 8s...", Font_7x10, 1);
//     ssd1306_UpdateScreen();
//   }

//   // Servo_ForwardBackward(&FSUS_Usart, FORWARD_BACKWARD_MIN, 1000, 0);
//   // HAL_Delay(100);
//   set0position();
//   HAL_Delay(20);

//   // 起步
//   Car_FORWARD(forward_speed, 460); // 二维码
//   HAL_Delay(700);
//   set_openmv_light(1);
//   HAL_Delay(20);

//   // 显示图像
//   ssd1306_Fill(0);
//   ssd1306_UpdateScreen();
//   HAL_Delay(500);
//   // 3. 调用绘图函数，将图像数据加载到缓冲区
//   ssd1306_DrawBitmap(tongse, 0, 0, 128, 64);
//   // 4. 刷新屏幕，将缓冲区的内容显示出来 (关键一步!)
//   ssd1306_UpdateScreen();
//   HAL_Delay(70);
//   Car_Strafe_Left_Smooth(forward_speed - 20, 200);
//   HAL_Delay(70);
//   Car_Reset_And_Sync_State();
//   HAL_Delay(70);
//   Car_FORWARD(forward_speed, 625); // 抓取物料区域
//   // Car_FORWARD(forward_speed, 570);
//   HAL_Delay(70);

//   // 老版一次抓三个
//   visual_align_blocking_V3_111(); // 视觉对齐---------------------------------------------
//   HAL_Delay(70);
//   catchall();
//   HAL_Delay(70);

//   // 新版一次抓一个
//   // catch_middle_three();


//   Car_Strafe_Left_Smooth(forward_speed - 20, 270);
//   HAL_Delay(70);
//   Car_FORWARD(forward_speed, 470);
//   // Car_FORWARD(forward_speed, 455);
//   HAL_Delay(70);
//   Car_LEFT(415); // 转90度
//   HAL_Delay(70);
// }

// void Task2(void)
// {
//   // 去凸台区域
//   Car_FORWARD(forward_speed, 660);
//   HAL_Delay(70);

//   // 视觉对齐
//   visual_align_blocking_V3_111(); // 视觉对齐
//   HAL_Delay(70);
//   placeall_same();
//   HAL_Delay(70);
//   catchall();
//   HAL_Delay(70);

//   // 离开凸台区
//   Car_Strafe_Left_Smooth(forward_speed - 20, 230);
//   HAL_Delay(70);
//   Car_FORWARD(forward_speed, 680);
//   HAL_Delay(70);

//   Car_LEFT(413);
//   HAL_Delay(70);

//   // 从中间去物料台

//   Car_FORWARD(forward_speed, 629); // 运行到中间
//   HAL_Delay(70);
//   visual_align_blocking_V3_111();
//   // placeall_same();
//   place_middle_three();
//   HAL_Delay(70);
//   Car_Strafe_Left_Smooth(forward_speed - 20, 280);
//   HAL_Delay(70);
//   Car_BACKWARD(forward_speed, 260);
//   HAL_Delay(100);
//   Car_LEFT(422);
//   HAL_Delay(70);

//   // 去物料台子

//   Car_FORWARD(forward_speed, 1355);
//   HAL_Delay(100);
//   set0position();
//   HAL_Delay(100);
//   Servo_RotateMiddle(&FSUS_Usart);
//   HAL_Delay(100);
//   Car_LEFT(418);
//   Car_FORWARD(forward_speed, 207);
// }

// void Task3() // 第二圈
// {
//   Car_FORWARD(forward_speed, 385);
//   HAL_Delay(70);
//   Car_LEFT(408); // 转90度
//   HAL_Delay(70);

//   // 去凸台区
//   Car_FORWARD(forward_speed, 700);
//   HAL_Delay(70);

//   // 视觉对齐
//   visual_align_blocking_V3_111(); // 视觉对齐
//   HAL_Delay(70);
//   // placeall_same();
//   //-----------------------------------------------------------------------------
//   set0position();
//   HAL_Delay(100);
//   Servo_ForwardBackward(&FSUS_Usart, FORWARD_BACKWARD_MIN, 800, 0);
//   HAL_Delay(300);

//   // placegreen_same();
//   // 绿色
//   Servo_PlaceGreen(&FSUS_Usart);
//   HAL_Delay(200);
//   Servo_RotateBehind(&FSUS_Usart);
//   HAL_Delay(1500);
//   Datou_PWM_Move_Steps(0, 3540, 140);
//   HAL_Delay(1000);
//   Servo_Close(&FSUS_Usart);
//   HAL_Delay(600);
//   Datou_PWM_Move_Steps(1, 3540, 140);
//   HAL_Delay(1000);
//   Servo_RotateMiddle(&FSUS_Usart);
//   HAL_Delay(1200);
//   Datou_PWM_Move_Steps(0, 8350, 140); // 下方台阶
//   HAL_Delay(1300);
//   Servo_Open(&FSUS_Usart);
//   HAL_Delay(400);
//   Servo_Open(&FSUS_Usart);
//   HAL_Delay(400);
//   Datou_PWM_Move_Steps(1, 8320, 140); // 上升归位
//   HAL_Delay(800);
//   HAL_Delay(50);
//   // placeblue_same();
//   // 蓝色
//   Servo_PlaceBlue(&FSUS_Usart);
//   HAL_Delay(500);
//   Servo_RotateBehind(&FSUS_Usart);
//   HAL_Delay(1500);
//   Datou_PWM_Move_Steps(0, 3540, 140);
//   HAL_Delay(1000);
//   Servo_Close(&FSUS_Usart);
//   HAL_Delay(600);
//   Datou_PWM_Move_Steps(1, 3540, 140);
//   HAL_Delay(1000);
//   Servo_RotateRight(&FSUS_Usart);
//   HAL_Delay(1200);
//   Servo_ForwardBackward(&FSUS_Usart, FORWARD_BACKWARD_CATCH - 20, 800, 0);
//   HAL_Delay(1500);
//   Datou_PWM_Move_Steps(0, 8350, 140); // 下方台阶
//   HAL_Delay(1000);
//   Servo_Open(&FSUS_Usart);
//   HAL_Delay(1000);
//   Servo_Open(&FSUS_Usart);
//   HAL_Delay(400);
//   Servo_Open(&FSUS_Usart);
//   HAL_Delay(400);
//   Servo_ForwardBackward(&FSUS_Usart, FORWARD_BACKWARD_MIN, 800, 0);
//   HAL_Delay(100);
//   Datou_PWM_Move_Steps(1, 8350, 140); // 上升归位
//   HAL_Delay(1000);

//   HAL_Delay(120);
//   // placered_same();
//   // 红色
//   Servo_PlaceRed(&FSUS_Usart);
//   HAL_Delay(600);
//   Servo_RotateBehind(&FSUS_Usart);
//   HAL_Delay(500);
//   Servo_RotateBehind(&FSUS_Usart);
//   HAL_Delay(1200);
//   Datou_PWM_Move_Steps(0, 3540, 140);
//   HAL_Delay(1000);
//   Servo_Close(&FSUS_Usart);
//   HAL_Delay(600);
//   Datou_PWM_Move_Steps(1, 3540, 140);
//   HAL_Delay(1000);
//   Servo_RotateLeft(&FSUS_Usart);
//   HAL_Delay(1200);
//   Servo_ForwardBackward(&FSUS_Usart, FORWARD_BACKWARD_CATCH - 20, 800, 0);
//   HAL_Delay(1500);
//   Datou_PWM_Move_Steps(0, 8350, 140); // 下方台阶
//   HAL_Delay(1000);
//   Servo_Open(&FSUS_Usart);
//   HAL_Delay(1000);
//   Servo_Open(&FSUS_Usart);
//   HAL_Delay(900);
//   Servo_ForwardBackward(&FSUS_Usart, FORWARD_BACKWARD_MIN, 800, 0);
//   HAL_Delay(100);
//   Datou_PWM_Move_Steps(1, 8350, 140); // 上升归位
//   HAL_Delay(800);

//   //-----------------------------------------------------------------------------

//   HAL_Delay(70);
//   // 抓取
//   set0position();
//   HAL_Delay(100);
//   Servo_RotateMiddle(&FSUS_Usart);
//   HAL_Delay(100);

//   // 绿色
//   Servo_PlaceGreen(&FSUS_Usart);
//   HAL_Delay(100);
//   Servo_ForwardBackward(&FSUS_Usart, FORWARD_BACKWARD_CATCH_MIN, 1000, 0);
//   HAL_Delay(400);
//   // Servo_RotateMiddle(&FSUS_Usart); // 因为前一步就是居中，所以这里就不用旋转了
//   // HAL_Delay(1500);
//   Datou_PWM_Move_Steps(0, 8720, 160); // 下方台阶
//   HAL_Delay(800);
//   HAL_Delay(500);
//   Servo_Close(&FSUS_Usart);
//   HAL_Delay(800);
//   Datou_PWM_Move_Steps(1, 8720, 160); // 上升归位
//   HAL_Delay(100);
//   Servo_ForwardBackward(&FSUS_Usart, FORWARD_BACKWARD_MIN, 1000, 0);
//   HAL_Delay(1000);
//   Servo_RotateBehind(&FSUS_Usart);
//   HAL_Delay(2000);
//   Datou_PWM_Move_Steps(0, 4200, 160);
//   HAL_Delay(1000);
//   Servo_Open(&FSUS_Usart);
//   HAL_Delay(600);
//   Datou_PWM_Move_Steps(1, 4200, 160);
//   HAL_Delay(500);
//   Servo_RotateRight(&FSUS_Usart);
//   HAL_Delay(1500);

//   // 蓝色（夹爪无法向右转啊啊啊啊）
//   // 已经修正向右转
//   Servo_PlaceBlue(&FSUS_Usart);
//   HAL_Delay(100);
//   Servo_ForwardBackward(&FSUS_Usart, FORWARD_BACKWARD_CATCH, 800, 0);
//   HAL_Delay(800);
//   Datou_PWM_Move_Steps(0, 8490, 160); // 下方台阶
//   HAL_Delay(1300);
//   Servo_Close(&FSUS_Usart);
//   HAL_Delay(800);
//   Datou_PWM_Move_Steps(1, 8490, 160); // 上升归位
//   HAL_Delay(100);
//   Servo_ForwardBackward(&FSUS_Usart, FORWARD_BACKWARD_MIN, 800, 0);
//   HAL_Delay(1000);
//   Servo_RotateBehind(&FSUS_Usart);
//   HAL_Delay(1500);
//   Datou_PWM_Move_Steps(0, 4200, 160);
//   HAL_Delay(1000);
//   Servo_Open(&FSUS_Usart);
//   HAL_Delay(500);
//   Datou_PWM_Move_Steps(1, 4200, 160);
//   HAL_Delay(500);
//   Servo_RotateLeft(&FSUS_Usart);
//   HAL_Delay(800);

//   // 红色
//   Servo_PlaceRed(&FSUS_Usart);
//   HAL_Delay(100);
//   Servo_ForwardBackward(&FSUS_Usart, FORWARD_BACKWARD_CATCH, 800, 0);
//   HAL_Delay(100);
//   Servo_ForwardBackward(&FSUS_Usart, FORWARD_BACKWARD_CATCH, 800, 0);
//   HAL_Delay(800);
//   Datou_PWM_Move_Steps(0, 8710, 160); // 下方台阶
//   HAL_Delay(800);
//   Servo_Close(&FSUS_Usart);
//   HAL_Delay(500);
//   Servo_Close(&FSUS_Usart);
//   HAL_Delay(800);
//   Datou_PWM_Move_Steps(1, 8710, 160); // 上升归位
//   HAL_Delay(100);
//   Servo_ForwardBackward(&FSUS_Usart, FORWARD_BACKWARD_MIN, 800, 0);
//   HAL_Delay(1000);
//   Servo_RotateBehind(&FSUS_Usart);
//   HAL_Delay(1500);
//   Datou_PWM_Move_Steps(0, 4200, 160);
//   HAL_Delay(1000);
//   Servo_Open(&FSUS_Usart);
//   HAL_Delay(500);
//   Datou_PWM_Move_Steps(1, 4200, 160);
//   HAL_Delay(500);
//   Servo_RotateMiddle(&FSUS_Usart);
//   HAL_Delay(100);
//   Servo_ForwardBackward(&FSUS_Usart, FORWARD_BACKWARD_MIN, 800, 0);
//   HAL_Delay(500);


//   Servo_ForwardBackward(&FSUS_Usart, FORWARD_BACKWARD_CATCH_MIN, 800, 0);
//   HAL_Delay(100);
  

//   // 离开凸台区
//   Car_Strafe_Left_Smooth(forward_speed - 20, 280);
//   HAL_Delay(70);
//   Car_FORWARD(forward_speed, 690);
//   HAL_Delay(70);
//   Car_LEFT(408); // 转90度
//   HAL_Delay(70);

//   Car_FORWARD(forward_speed, 675);
//   HAL_Delay(70);

//   // 视觉对齐
//   visual_align_blocking_V3_111(); // 视觉对齐
//   HAL_Delay(70);
// }
// void Task4()
// {
//   Car_Strafe_Left_Smooth(forward_speed - 20, 250);
//   HAL_Delay(70);
//   Car_FORWARD(forward_speed, 690);
//   HAL_Delay(70);
//   Car_LEFT(420); // 转90度
//   HAL_Delay(70);
//   Car_FORWARD(forward_speed, 1500);
//   HAL_Delay(70);
//   HAL_Delay(500);

//   Servo_Open(&FSUS_Usart);
//   HAL_Delay(500);
//   Datou_PWM_Move_Steps(1, 4100, 160);
//   HAL_Delay(500);
//   Servo_RotateMiddle(&FSUS_Usart);
//   HAL_Delay(500);

//   // visual_align_blocking_V3_111();
//   // HAL_Delay(70);

//   // Car_FORWARD(forward_speed, 200);
//   // HAL_Delay(70);

//   // Car_Strafe_Right_Smooth(forward_speed - 20, 375);
// }
// // HAL_Delay(500);
// // visual_align_blocking_V2(1, 13, 13, 1, 5000); // 视觉对齐
// // HAL_Delay(70);
// // Car_LEFT90();
// // HAL_Delay(70);

// // // 第二圈
// // // 起步
// // Car_FORWARD(forward_speed - 20, 400);
// // HAL_Delay(500);

// // 给舵机等设定初始位置
// void set0position(void)
// {
//   Servo_PlaceBlue(&FSUS_Usart);
//   HAL_Delay(100);
//   Servo_RotateMiddle(&FSUS_Usart);
//   HAL_Delay(100);
//   Servo_Open(&FSUS_Usart);
//   HAL_Delay(100);
//   Servo_Open(&FSUS_Usart);
//   HAL_Delay(100);
//   Servo_ForwardBackward(&FSUS_Usart, FORWARD_BACKWARD_CATCH_MIN, 1000, 0);
//   HAL_Delay(1002);
// }

// void catch_green(void)
// {
//   // 绿色
//   Servo_PlaceGreen(&FSUS_Usart);
//   HAL_Delay(100);
//   Servo_ForwardBackward(&FSUS_Usart, FORWARD_BACKWARD_CATCH_MIN, 1000, 0);
//   HAL_Delay(400);
//   // Servo_RotateMiddle(&FSUS_Usart); // 因为前一步就是居中，所以这里就不用旋转了
//   // HAL_Delay(1500);
//   Datou_PWM_Move_Steps(0, 8720, 160); // 下方台阶
//   HAL_Delay(800);
//   HAL_Delay(500);
//   Servo_Close(&FSUS_Usart);
//   HAL_Delay(800);
//   Datou_PWM_Move_Steps(1, 8720, 160); // 上升归位
//   HAL_Delay(100);
//   Servo_ForwardBackward(&FSUS_Usart, FORWARD_BACKWARD_MIN, 1000, 0);
//   HAL_Delay(1000);
//   Servo_RotateBehind(&FSUS_Usart);
//   HAL_Delay(2000);
//   Datou_PWM_Move_Steps(0, 4200, 160);
//   HAL_Delay(1000);
//   Servo_Open(&FSUS_Usart);
//   HAL_Delay(600);
//   Datou_PWM_Move_Steps(1, 4200, 160);
//   HAL_Delay(500);
//   Servo_RotateRight(&FSUS_Usart);
//   HAL_Delay(1500);
// }
// void catch_blue(void)
// {
//   // 蓝色（夹爪无法向右转啊啊啊啊）
//   // 已经修正向右转
//   Servo_PlaceBlue(&FSUS_Usart);
//   HAL_Delay(100);
//   Servo_ForwardBackward(&FSUS_Usart, FORWARD_BACKWARD_CATCH, 800, 0);
//   HAL_Delay(800);
//   Datou_PWM_Move_Steps(0, 8300, 160); // 下方台阶
//   HAL_Delay(1300);
//   Servo_Close(&FSUS_Usart);
//   HAL_Delay(800);
//   Datou_PWM_Move_Steps(1, 8300, 160); // 上升归位
//   HAL_Delay(100);

//   Servo_ForwardBackward(&FSUS_Usart, FORWARD_BACKWARD_MIN, 800, 0);
//   HAL_Delay(1000);
//   Servo_RotateBehind(&FSUS_Usart);
//   HAL_Delay(1500);
//   Datou_PWM_Move_Steps(0, 4200, 160);
//   HAL_Delay(1000);
//   Servo_Open(&FSUS_Usart);
//   HAL_Delay(500);
//   Datou_PWM_Move_Steps(1, 4200, 160);
//   HAL_Delay(500);
//   Servo_RotateLeft(&FSUS_Usart);
//   HAL_Delay(800);
// }
// void catch_red(void)
// {
//   // 红色
//   Servo_PlaceRed(&FSUS_Usart);
//   HAL_Delay(100);
//   Servo_ForwardBackward(&FSUS_Usart, FORWARD_BACKWARD_CATCH, 800, 0);
//   HAL_Delay(100);
//   Servo_ForwardBackward(&FSUS_Usart, FORWARD_BACKWARD_CATCH, 800, 0);

  
//   HAL_Delay(800);
//   Datou_PWM_Move_Steps(0, 8400, 160); // 下方台阶
//   HAL_Delay(800);
//   Servo_Close(&FSUS_Usart);
//   HAL_Delay(500);
//   Servo_Close(&FSUS_Usart);
//   HAL_Delay(800);
//   Datou_PWM_Move_Steps(1, 8400, 160); // 上升归位
//   HAL_Delay(100);
//   Servo_ForwardBackward(&FSUS_Usart, FORWARD_BACKWARD_MIN, 800, 0);
//   HAL_Delay(1000);
//   Servo_RotateBehind(&FSUS_Usart);
//   HAL_Delay(1500);
//   Datou_PWM_Move_Steps(0, 4200, 160);
//   HAL_Delay(1000);
//   Servo_Open(&FSUS_Usart);
//   HAL_Delay(500);
//   Datou_PWM_Move_Steps(1, 4200, 160);
//   HAL_Delay(500);
//   Servo_RotateMiddle(&FSUS_Usart);
//   HAL_Delay(100);
//   Servo_ForwardBackward(&FSUS_Usart, FORWARD_BACKWARD_MIN, 800, 0);
//   HAL_Delay(500);
// }
// void catchall(void) // 第一次抓取
// {
//   set0position();
//   HAL_Delay(100);
//   Servo_RotateMiddle(&FSUS_Usart);
//   HAL_Delay(100);

//   catch_green();
//   HAL_Delay(100);
//   catch_blue();
//   HAL_Delay(100);
//   catch_red();
//   HAL_Delay(100);

//   Servo_ForwardBackward(&FSUS_Usart, FORWARD_BACKWARD_CATCH_MIN, 800, 0);
//   HAL_Delay(100);
// }

// void place_middle(void)
// {
//   // Servo_PlaceGreen(&FSUS_Usart); //指定颜色
//   HAL_Delay(100);
//   Servo_RotateBehind(&FSUS_Usart);
//   HAL_Delay(100);
//   Servo_RotateBehind(&FSUS_Usart);
//   HAL_Delay(1500);
//   Datou_PWM_Move_Steps(0, 4100, 160);
//   HAL_Delay(1000);
//   Servo_Close(&FSUS_Usart);
//   HAL_Delay(600);
//   Datou_PWM_Move_Steps(1, 4100, 160);
//   HAL_Delay(500);
//   Servo_RotateMiddle(&FSUS_Usart);
//   HAL_Delay(900);
//   Datou_PWM_Move_Steps(0, 8450, 160); // 下方台阶
//   HAL_Delay(1300);
//   Servo_Open(&FSUS_Usart);
//   HAL_Delay(400);
//   Servo_Open(&FSUS_Usart);
//   HAL_Delay(400);
//   Datou_PWM_Move_Steps(1, 8450, 160); // 上升归位
//   HAL_Delay(800);
// }

// void catch_middle_three(void)
// {
//   visual_align_blocking_V3_111();
//   HAL_Delay(100);

//   Servo_PlaceBlue(&FSUS_Usart);
//   HAL_Delay(100);
//   Servo_ForwardBackward(&FSUS_Usart, FORWARD_BACKWARD_CATCH_MIN, 1000, 0);
//   HAL_Delay(400);
//   // Servo_RotateMiddle(&FSUS_Usart); // 因为前一步就是居中，所以这里就不用旋转了
//   // HAL_Delay(1500);
//   Datou_PWM_Move_Steps(0, 8720, 160); // 下方台阶
//   HAL_Delay(800);
//   HAL_Delay(500);
//   Servo_Close(&FSUS_Usart);
//   HAL_Delay(800);
//   Datou_PWM_Move_Steps(1, 8720, 160); // 上升归位
//   HAL_Delay(100);
//   Servo_ForwardBackward(&FSUS_Usart, FORWARD_BACKWARD_MIN, 1000, 0);
//   HAL_Delay(1000);
//   Servo_RotateBehind(&FSUS_Usart);
//   HAL_Delay(2000);
//   Datou_PWM_Move_Steps(0, 4200, 160);
//   HAL_Delay(1000);
//   Servo_Open(&FSUS_Usart);
//   HAL_Delay(600);
//   Datou_PWM_Move_Steps(1, 4200, 160);
//   HAL_Delay(500);
//   Servo_RotateMiddle(&FSUS_Usart);
//   HAL_Delay(1200);
//   Car_FORWARD(forward_speed, 240);
//   HAL_Delay(100);
//   visual_align_blocking_V3_111();
//   HAL_Delay(100);

//   Servo_PlaceGreen(&FSUS_Usart);
//   HAL_Delay(100);
//   Servo_ForwardBackward(&FSUS_Usart, FORWARD_BACKWARD_CATCH_MIN, 1000, 0);
//   HAL_Delay(400);
//   // Servo_RotateMiddle(&FSUS_Usart); // 因为前一步就是居中，所以这里就不用旋转了
//   // HAL_Delay(1500);
//   Datou_PWM_Move_Steps(0, 8720, 160); // 下方台阶
//   HAL_Delay(800);
//   HAL_Delay(500);
//   Servo_Close(&FSUS_Usart);
//   HAL_Delay(800);
//   Datou_PWM_Move_Steps(1, 8720, 160); // 上升归位
//   HAL_Delay(100);
//   Servo_ForwardBackward(&FSUS_Usart, FORWARD_BACKWARD_MIN, 1000, 0);
//   HAL_Delay(1000);
//   Servo_RotateBehind(&FSUS_Usart);
//   HAL_Delay(2000);
//   Datou_PWM_Move_Steps(0, 4200, 160);
//   HAL_Delay(1000);
//   Servo_Open(&FSUS_Usart);
//   HAL_Delay(600);
//   Datou_PWM_Move_Steps(1, 4200, 160);
//   HAL_Delay(500);
//   Servo_RotateMiddle(&FSUS_Usart);
//   HAL_Delay(1200);
//   Car_FORWARD(forward_speed, 240);
//   HAL_Delay(100);
//   visual_align_blocking_V3_111();
//   HAL_Delay(100);

//   Servo_PlaceRed(&FSUS_Usart);
//   HAL_Delay(100);
//   Servo_ForwardBackward(&FSUS_Usart, FORWARD_BACKWARD_CATCH_MIN, 1000, 0);
//   HAL_Delay(400);
//   Datou_PWM_Move_Steps(0, 8720, 160); // 下方台阶
//   HAL_Delay(800);
//   HAL_Delay(500);
//   Servo_Close(&FSUS_Usart);
//   HAL_Delay(800);
//   Datou_PWM_Move_Steps(1, 8720, 160); // 上升归位
//   HAL_Delay(100);
//   Servo_ForwardBackward(&FSUS_Usart, FORWARD_BACKWARD_MIN, 1000, 0);
//   HAL_Delay(1000);
//   Servo_RotateBehind(&FSUS_Usart);
//   HAL_Delay(2000);
//   Datou_PWM_Move_Steps(0, 4200, 160);
//   HAL_Delay(1000);
//   Servo_Open(&FSUS_Usart);
//   HAL_Delay(600);
//   Datou_PWM_Move_Steps(1, 4200, 160);
//   HAL_Delay(500);
//   Servo_RotateMiddle(&FSUS_Usart);
//   HAL_Delay(1200);

  
// }
// void place_middle_three(void)
// {
//   // visual_align_blocking_V3_111();
//   // HAL_Delay(100);

//   Servo_ForwardBackward(&FSUS_Usart, FORWARD_BACKWARD_MIN, 1000, 0);
//   HAL_Delay(350);

//   Servo_PlaceBlue(&FSUS_Usart);
//   HAL_Delay(100);
//   place_middle();
//   HAL_Delay(100);
//   Car_FORWARD(forward_speed, 240);
//   HAL_Delay(100);
//   visual_align_blocking_V3_111();
//   HAL_Delay(100);

//   Servo_PlaceGreen(&FSUS_Usart);
//   HAL_Delay(100);
//   place_middle();
//   HAL_Delay(100);
//   Car_FORWARD(forward_speed, 240);
//   HAL_Delay(100);
//   visual_align_blocking_V3_111();
//   HAL_Delay(100);

//   Servo_PlaceRed(&FSUS_Usart);
//   HAL_Delay(100);
//   place_middle();
//   HAL_Delay(100);
// }
// void placegreen_same(void)
// {

//   // 绿色
//   Servo_PlaceGreen(&FSUS_Usart);
//   HAL_Delay(100);
//   Servo_RotateBehind(&FSUS_Usart);
//   HAL_Delay(100);
//   Servo_RotateBehind(&FSUS_Usart);
//   HAL_Delay(1500);
//   Datou_PWM_Move_Steps(0, 4100, 160);
//   HAL_Delay(1000);
//   Servo_Close(&FSUS_Usart);
//   HAL_Delay(600);
//   Datou_PWM_Move_Steps(1, 4100, 160);
//   HAL_Delay(500);
//   Servo_RotateMiddle(&FSUS_Usart);
//   HAL_Delay(900);
//   Servo_ForwardBackward(&FSUS_Usart, FORWARD_BACKWARD_MIN + 25 , 800, 0);
//   HAL_Delay(500);
//   Datou_PWM_Move_Steps(0, 8450, 160); // 下方台阶
//   HAL_Delay(1300);
//   Servo_Open(&FSUS_Usart);
//   HAL_Delay(400);
//   Servo_Open(&FSUS_Usart);
//   HAL_Delay(400);
//   Servo_ForwardBackward(&FSUS_Usart, FORWARD_BACKWARD_MIN, 800, 0);
//   HAL_Delay(100);
//   Datou_PWM_Move_Steps(1, 8450, 160); // 上升归位
//   HAL_Delay(1200);
// }

// void placeblue_same(void)
// {

//   // 蓝色
//   Servo_PlaceBlue(&FSUS_Usart);
//   HAL_Delay(100);
//   Servo_RotateBehind(&FSUS_Usart);
//   HAL_Delay(1500);
//   Datou_PWM_Move_Steps(0, 4100, 160);
//   HAL_Delay(1000);
//   Servo_Close(&FSUS_Usart);
//   HAL_Delay(600);
//   Datou_PWM_Move_Steps(1, 4100, 160);
//   HAL_Delay(500);
//   Servo_RotateRight(&FSUS_Usart);
//   HAL_Delay(900);
//   Servo_ForwardBackward(&FSUS_Usart, FORWARD_BACKWARD_CATCH + 10, 800, 0);
//   HAL_Delay(1500);
//   Datou_PWM_Move_Steps(0, 8450, 160); // 下方台阶
//   HAL_Delay(1000);
//   Servo_Open(&FSUS_Usart);
//   HAL_Delay(1000);
//   Servo_Open(&FSUS_Usart);
//   HAL_Delay(400);
//   Servo_Open(&FSUS_Usart);
//   HAL_Delay(400);
//   Servo_ForwardBackward(&FSUS_Usart, FORWARD_BACKWARD_MIN, 800, 0);
//   HAL_Delay(100);
//   Datou_PWM_Move_Steps(1, 8450, 160); // 上升归位
//   HAL_Delay(1200);
// }

// void placered_same(void)
// {
//   Servo_ForwardBackward(&FSUS_Usart, FORWARD_BACKWARD_MIN, 800, 0);
//   HAL_Delay(100);
//   // 红色
//   Servo_PlaceRed(&FSUS_Usart);
//   HAL_Delay(600);
//   Servo_RotateBehind(&FSUS_Usart);
//   HAL_Delay_us(200);
//   HAL_Delay(500);
//   Servo_RotateBehind(&FSUS_Usart);
//   HAL_Delay(1500);
//   Datou_PWM_Move_Steps(0, 4100, 160);
//   HAL_Delay(1000);
//   Servo_Close(&FSUS_Usart);
//   HAL_Delay(600);
//   Datou_PWM_Move_Steps(1, 4100, 160);
//   HAL_Delay(500);
//   Servo_RotateLeft(&FSUS_Usart);
//   HAL_Delay(900);
//   Servo_ForwardBackward(&FSUS_Usart, FORWARD_BACKWARD_CATCH +5, 800, 0);
//   HAL_Delay(1500);
//   Datou_PWM_Move_Steps(0, 8450, 160); // 下方台阶
//   HAL_Delay(1000);
//   Servo_Open(&FSUS_Usart);
//   HAL_Delay(1000);
//   Servo_Open(&FSUS_Usart);
//   HAL_Delay(900);
//   Servo_ForwardBackward(&FSUS_Usart, FORWARD_BACKWARD_MIN, 800, 0);
//   HAL_Delay(100);
//   Datou_PWM_Move_Steps(1, 8450, 160); // 上升归位
//   HAL_Delay(1200);
// }

// void placeall_same(void)
// {
//   // placeblue();
//   // HAL_Delay(500);
//   // Car_FORWARD(75 - 20, 299); // 走一小步
//   // HAL_Delay(500);
//   // // visual_align_blocking_V2(1,13,13,1,5000);
//   // HAL_Delay(1000);

//   // placegreen();
//   // HAL_Delay(500);
//   // Car_FORWARD(75 - 20, 297); // 走一小步
//   // HAL_Delay(500);
//   // // visual_align_blocking_V2(1,13,13,1,5000);
//   // HAL_Delay(1000);

//   // placered();
//   // HAL_Delay(500);

//   set0position();
//   HAL_Delay(100);
//   Servo_ForwardBackward(&FSUS_Usart, FORWARD_BACKWARD_MIN, 800, 0);
//   HAL_Delay(120);

//   placegreen_same();
//   HAL_Delay(120);
//   placeblue_same();
//   HAL_Delay(120);
//   placered_same();
//   HAL_Delay(120);
// }

// void upcatchbule(void)
// {
//   // Servo_TurntableRotate(&FSUS_Usart, 67 - 20, 900, 0);
//   Servo_PlaceBlue(&FSUS_Usart);
//   HAL_Delay(100);
//   Servo_Open(&FSUS_Usart);
//   HAL_Delay(500);
//   Servo_ForwardBackward(&FSUS_Usart, FORWARD_BACKWARD_FULL, 800, 0);
//   HAL_Delay(100);
//   Servo_ForwardBackward(&FSUS_Usart, FORWARD_BACKWARD_FULL, 800, 0);
//   HAL_Delay(700);
//   Datou_PWM_Move_Steps(0, 1800, 140); // 下
//   HAL_Delay(500);
//   Servo_Close(&FSUS_Usart);
//   HAL_Delay(1000);
//   Datou_PWM_Move_Steps(1, 1800, 140); // 上
//   HAL_Delay(500);
//   Servo_ForwardBackward(&FSUS_Usart, FORWARD_BACKWARD_MIN + 10, 800, 0);
//   HAL_Delay(700);
//   Servo_RotateBehind(&FSUS_Usart);
//   HAL_Delay(2000);
//   Datou_PWM_Move_Steps(0, 3965, 140);
//   HAL_Delay(1000);
//   Servo_Open(&FSUS_Usart);
//   HAL_Delay(500);
//   Datou_PWM_Move_Steps(1, 3965, 140);
//   HAL_Delay(500);
//   Servo_RotateMiddle(&FSUS_Usart);
//   HAL_Delay(500);
// }
// void upcatchgreen(void)
// {
//   // Servo_TurntableRotate(&FSUS_Usart, 67 - 20 + 120, 900, 0);
//   Servo_PlaceGreen(&FSUS_Usart);
//   HAL_Delay(100);
//   Servo_Open(&FSUS_Usart);
//   HAL_Delay(500);
//   Servo_ForwardBackward(&FSUS_Usart, FORWARD_BACKWARD_FULL, 800, 0);
//   HAL_Delay(800);
//   Datou_PWM_Move_Steps(0, 1800, 140); // 下
//   HAL_Delay(500);
//   Servo_Close(&FSUS_Usart);
//   HAL_Delay(1000);
//   Datou_PWM_Move_Steps(1, 1800, 140); // 上
//   HAL_Delay(500);
//   Servo_ForwardBackward(&FSUS_Usart, FORWARD_BACKWARD_MIN + 10, 800, 0);
//   HAL_Delay(700);
//   Servo_RotateBehind(&FSUS_Usart);
//   HAL_Delay(2000);
//   Datou_PWM_Move_Steps(0, 3965, 140);
//   HAL_Delay(1000);
//   Servo_Open(&FSUS_Usart);
//   HAL_Delay(500);
//   Datou_PWM_Move_Steps(1, 3965, 140);
//   HAL_Delay(700);
//   Servo_RotateMiddle(&FSUS_Usart);
//   HAL_Delay(500);
// }
// void upcatchred(void)
// {
//   // Servo_TurntableRotate(&FSUS_Usart, 67 - 20 + 240, 900, 0);
//   Servo_PlaceRed(&FSUS_Usart);
//   HAL_Delay(100);
//   Servo_Open(&FSUS_Usart);
//   HAL_Delay(500);
//   Servo_ForwardBackward(&FSUS_Usart, FORWARD_BACKWARD_FULL, 800, 0);
//   HAL_Delay(100);
//   Servo_ForwardBackward(&FSUS_Usart, FORWARD_BACKWARD_FULL, 800, 0);
//   HAL_Delay(700);
//   Datou_PWM_Move_Steps(0, 1800, 140); // 下
//   HAL_Delay(500);
//   Servo_Close(&FSUS_Usart);
//   HAL_Delay(1000);
//   Datou_PWM_Move_Steps(1, 1800, 140); // 上
//   HAL_Delay(500);
//   Servo_ForwardBackward(&FSUS_Usart, FORWARD_BACKWARD_MIN + 10, 800, 0);
//   HAL_Delay(700);
//   Servo_RotateBehind(&FSUS_Usart);
//   HAL_Delay(2000);
//   Datou_PWM_Move_Steps(0, 3965, 140);
//   HAL_Delay(1000);
//   Servo_Open(&FSUS_Usart);
//   HAL_Delay(500);
//   Datou_PWM_Move_Steps(1, 3965, 140);
//   HAL_Delay(500);
//   Servo_RotateMiddle(&FSUS_Usart);
//   HAL_Delay(500);
// }

// void catch_diff(void)
// {
//   visual_align_blocking_V3_111();
//   HAL_Delay(100);

//   upcatchbule();
//   HAL_Delay(100);
//   Car_FORWARD(forward_speed, 240);
//   HAL_Delay(100);
//   visual_align_blocking_V3_111();
//   HAL_Delay(100);

//   upcatchgreen();
//   HAL_Delay(100);
//   Car_FORWARD(forward_speed, 240);
//   HAL_Delay(70);
//   visual_align_blocking_V3_111();
//   HAL_Delay(100);

//   upcatchred();
//   HAL_Delay(50);
// }

// void upplace_same(void)
// {
//   set0position();
//   HAL_Delay(50);
//   Servo_ForwardBackward(&FSUS_Usart, FORWARD_BACKWARD_MIN, 800, 0);
//   HAL_Delay(120);

//   // 蓝色
//   Servo_PlaceBlue(&FSUS_Usart);
//   HAL_Delay(50);
//   Servo_RotateBehind(&FSUS_Usart);
//   HAL_Delay(1500);
//   Datou_PWM_Move_Steps(0, 3350, 140);
//   HAL_Delay(1000);
//   Servo_Close(&FSUS_Usart);
//   HAL_Delay(600);
//   Datou_PWM_Move_Steps(1, 3350, 140);
//   HAL_Delay(500);
//   Servo_RotateRight(&FSUS_Usart);
//   HAL_Delay(900);
//   Servo_ForwardBackward(&FSUS_Usart, FORWARD_BACKWARD_CATCH, 800, 0);
//   HAL_Delay(1500);
//   Datou_PWM_Move_Steps(0, 2070, 140); // 下方台阶
//   HAL_Delay(1000);
//   Servo_Open(&FSUS_Usart);
//   HAL_Delay(1000);
//   Servo_Open(&FSUS_Usart);
//   HAL_Delay(400);
//   Servo_Open(&FSUS_Usart);
//   HAL_Delay(400);
//   Servo_ForwardBackward(&FSUS_Usart, FORWARD_BACKWARD_MIN, 800, 0);
//   HAL_Delay(50);
//   Datou_PWM_Move_Steps(1, 2070, 140); // 上升归位
//   HAL_Delay(800);

//   // 绿色
//   Servo_PlaceGreen(&FSUS_Usart);
//   HAL_Delay(50);
//   Servo_RotateBehind(&FSUS_Usart);
//   HAL_Delay(1500);
//   Datou_PWM_Move_Steps(0, 3350, 140);
//   HAL_Delay(1000);
//   Servo_Close(&FSUS_Usart);
//   HAL_Delay(600);
//   Datou_PWM_Move_Steps(1, 3350, 140);
//   HAL_Delay(500);
//   Servo_RotateMiddle(&FSUS_Usart);
//   HAL_Delay(1200);
//   Datou_PWM_Move_Steps(0, 2070, 140); // 下方台阶
//   HAL_Delay(1300);
//   Servo_Open(&FSUS_Usart);
//   HAL_Delay(400);
//   Servo_Open(&FSUS_Usart);
//   HAL_Delay(400);
//   Datou_PWM_Move_Steps(1, 2070, 140); // 上升归位
//   HAL_Delay(800);

//   // 红色
//   Servo_PlaceRed(&FSUS_Usart);
//   HAL_Delay(50);
//   Servo_RotateBehind(&FSUS_Usart);
//   HAL_Delay(1500);
//   Datou_PWM_Move_Steps(0, 3350, 140);
//   HAL_Delay(1000);
//   Servo_Close(&FSUS_Usart);
//   HAL_Delay(600);
//   Datou_PWM_Move_Steps(1, 3350, 140);
//   HAL_Delay(500);
//   Servo_RotateLeft(&FSUS_Usart);
//   HAL_Delay(900);
//   Servo_ForwardBackward(&FSUS_Usart, FORWARD_BACKWARD_CATCH - 30, 800, 0);
//   HAL_Delay(1500);
//   Datou_PWM_Move_Steps(0, 2170, 140); // 下方台阶
//   HAL_Delay(1000);
//   Servo_Open(&FSUS_Usart);
//   HAL_Delay(1000);
//   Servo_Open(&FSUS_Usart);
//   HAL_Delay(800);
//   // Servo_ForwardBackward(&FSUS_Usart, FORWARD_BACKWARD_MIN, 800, 0);
//   // HAL_Delay(50);
//   // Datou_PWM_Move_Steps(1, 2070, 140); // 上升归位
//   // HAL_Delay(800);

//   // 抓取随便一个
//   Servo_Close(&FSUS_Usart);
//   HAL_Delay(500);
//   Servo_ForwardBackward(&FSUS_Usart, FORWARD_BACKWARD_MIN, 800, 0);
//   HAL_Delay(100);
//   Datou_PWM_Move_Steps(1, 2170, 140); // 上升归位
//   HAL_Delay(800);
//   Servo_RotateBehind(&FSUS_Usart);
//   HAL_Delay(1500);
//   Datou_PWM_Move_Steps(0, 4080, 140);
//   HAL_Delay(700);
// }