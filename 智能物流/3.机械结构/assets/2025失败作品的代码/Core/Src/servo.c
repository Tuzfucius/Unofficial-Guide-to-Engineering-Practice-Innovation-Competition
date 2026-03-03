#include "fashion_star_uart_servo.h"
#include "All.h"

// ========== 新增的物料转换函数 ==========

/**
 * @brief servo_1函数 - 转换n个物料
 * @param usart UART实例指针
 * @param item_count 物料个数 (1-6个)
 * @param time_ms 执行时间(毫秒)
 */
void servo_1(Usart_DataTypeDef *usart, uint8_t item_count, uint32_t time_ms)
{
  float angle;
  FSUS_STATUS status;

  // 计算旋转角度：每个物料60度
  angle = item_count * 60.0f;

  printf("[Material] Convert %d items (rotate %.1f deg), time %dms\r\n", item_count, angle, time_ms);

  // 使用异步命令发送，功率设置为0（无限制）
  status = FSUS_SetServoAngle(usart, 1, angle, time_ms, 0);

  if (status == FSUS_STATUS_SUCCESS)
  {
    printf("[Material] Command sent successfully, converting %d items\r\n", item_count);
  }
  else
  {
    printf("[Material] Command failed, status code: %d\r\n", status);
  }
}

// ========== 4个舵机初始位置设置函数 ==========

/**
 * @brief 物料转盘初始位置设置函数 (设置到-9度起始位置)
 * @param usart UART实例指针
 * @param time_ms 执行时间(毫秒)
 */
void Servo_TurntableSetInitPosition(Usart_DataTypeDef *usart, uint16_t time_ms)
{
  float init_angle;
  FSUS_STATUS status;

  init_angle = -9.0f; // 初始角度为-9度

  printf("[Material Turntable] Set initial position to %.1f deg, time %dms\r\n", init_angle, time_ms);

  // 使用异步命令发送，不等待响应
  status = FSUS_SetServoAngle(usart, 1, init_angle, time_ms, 0);

  if (status == FSUS_STATUS_SUCCESS)
  {
    printf("[Material Turntable] Initial position set successfully\r\n");
  }
  else
  {
    printf("[Material Turntable] Initial position set failed, status code: %d\r\n", status);
  }
}
void Servo_MainSetInitPosition(Usart_DataTypeDef *usart, uint16_t time_ms)
{
  float init_angle;
  FSUS_STATUS status;

  init_angle = -9.0f; // 初始角度为-9度

  printf("[Material Turntable] Set initial position to %.1f deg, time %dms\r\n", init_angle, time_ms);

  // 使用异步命令发送，不等待响应
  status = FSUS_SetServoAngle(usart, 1, init_angle, time_ms, 0);

  if (status == FSUS_STATUS_SUCCESS)
  {
    printf("[Material Turntable] Initial position set successfully\r\n");
  }
  else
  {
    printf("[Material Turntable] Initial position set failed, status code: %d\r\n", status);
  }
}
void Servo_ForwardBackwardSetInitPosition(Usart_DataTypeDef *usart, uint16_t time_ms)
{
  float init_angle;
  FSUS_STATUS status;

  init_angle = -9.0f; // 初始角度为-9度

  printf("[Material Turntable] Set initial position to %.1f deg, time %dms\r\n", init_angle, time_ms);

  // 使用异步命令发送，不等待响应
  status = FSUS_SetServoAngle(usart, 1, init_angle, time_ms, 0);

  if (status == FSUS_STATUS_SUCCESS)
  {
    printf("[Material Turntable] Initial position set successfully\r\n");
  }
  else
  {
    printf("[Material Turntable] Initial position set failed, status code: %d\r\n", status);
  }
}
void Servo_UpDownSetInitPosition(Usart_DataTypeDef *usart, uint16_t time_ms)
{
  float init_angle;
  FSUS_STATUS status;

  init_angle = -9.0f; // 初始角度为-9度

  printf("[Material Turntable] Set initial position to %.1f deg, time %dms\r\n", init_angle, time_ms);

  // 使用异步命令发送，不等待响应
  status = FSUS_SetServoAngle(usart, 1, init_angle, time_ms, 0);

  if (status == FSUS_STATUS_SUCCESS)
  {
    printf("[Material Turntable] Initial position set successfully\r\n");
  }
  else
  {
    printf("[Material Turntable] Initial position set failed, status code: %d\r\n", status);
  }
}

// ========== 4个舵机独立控制函数 ==========

// /**
//  * @brief ID1: 物料转盘旋转 (多圈模式)
//  * @param usart UART实例指针
//  * @param angle 目标角度
//  * @param time_ms 执行时间(毫秒)
//  * @param acc 加速时间(毫秒)
//  * @param dec 减速时间(毫秒)
//  */
// void Servo_TurntableRotate(Usart_DataTypeDef *usart, float angle, uint16_t time_ms, uint16_t acc, uint16_t dec)
// {
//   FSUS_sync_servo servo;
//   FSUS_STATUS status;

//   printf("[Turntable] Rotate to %.1f deg, time %dms (acc %dms, dec %dms)\r\n", angle, time_ms, acc, dec);

//   servo.id = 1;
//   servo.angle = angle;
//   servo.interval_single = time_ms;
//   servo.t_acc = acc;
//   servo.t_dec = dec;
//   servo.power = 0;

//   status = FSUS_SyncCommand(usart, 1, MODE_SET_SERVO_ANGLE_BY_INTERVAL, &servo, NULL);

//   if (status == FSUS_STATUS_SUCCESS)
//   {
//     printf("[Turntable] Command sent successfully\r\n");
//   }
//   else
//   {
//     printf("[Turntable] Command failed, status code: %d\r\n", status);
//   }
// }

// /**
//  * @brief ID2: 主旋转盘 (单圈模式)
//  * @param usart UART实例指针
//  * @param angle 目标角度
//  * @param time_ms 执行时间(毫秒)
//  * @param acc 加速时间(毫秒)
//  * @param dec 减速时间(毫秒)
//  */
// void Servo_MainRotate(Usart_DataTypeDef *usart, float angle, uint16_t time_ms, uint16_t acc, uint16_t dec)
// {
//   FSUS_sync_servo servo;
//   FSUS_STATUS status;

//   printf("[Main Rotation] Rotate to %.1f deg, time %dms (acc %dms, dec %dms)\r\n", angle, time_ms, acc, dec);

//   servo.id = 2;
//   servo.angle = angle;
//   servo.interval_single = time_ms;
//   servo.t_acc = acc;
//   servo.t_dec = dec;
//   servo.power = 0;

//   status = FSUS_SyncCommand(usart, 1, MODE_SET_SERVO_ANGLE_BY_INTERVAL, &servo, NULL);

//   if (status == FSUS_STATUS_SUCCESS)
//   {
//     printf("[Main Rotation] Command sent successfully\r\n");
//   }
//   else
//   {
//     printf("[Main Rotation] Command failed, status code: %d\r\n", status);
//   }
// }

// /**
//  * @brief ID3: 前后运动 (多圈模式)
//  * @param usart UART实例指针
//  * @param angle 目标角度
//  * @param time_ms 执行时间(毫秒)
//  * @param power 舵机功率 (0-1000)
//  */
// void Servo_ForwardBackward(Usart_DataTypeDef *usart, float angle, uint32_t time_ms, uint16_t power)
// {
//   FSUS_STATUS status;

//   printf("[Forward/Backward] Rotate to %.1f deg, time %ldms, power %dmW (multi-turn mode)\r\n", angle, time_ms, power);

//   status = FSUS_SetServoAngleMTurn(usart, 3, angle, time_ms, power);

//   if (status == FSUS_STATUS_SUCCESS)
//   {
//     printf("[Forward/Backward] Command sent successfully\r\n");
//   }
//   else
//   {
//     printf("[Forward/Backward] Command failed, status code: %d\r\n", status);
//   }
// }

// /**
//  * @brief ID4: 夹抓张开 - 舵机张开到指定角度
//  * @param usart UART实例指针
//  * @param time_ms 执行时间(毫秒)
//  * @param acc 加速时间(毫秒)
//  * @param dec 减速时间(毫秒)
//  */
// void Servo_GripperOpen(Usart_DataTypeDef *usart, uint16_t time_ms, uint16_t acc, uint16_t dec)
// {
//   float angle;
//   FSUS_sync_servo servo;
//   FSUS_STATUS status;

//   angle = 120.0f; // 固定张开角度

//   printf("[Gripper Open] Angle %.1f deg, time %dms (acc %dms, dec %dms)\r\n", angle, time_ms, acc, dec);

//   servo.id = 4;
//   servo.angle = angle;
//   servo.interval_single = time_ms;
//   servo.t_acc = acc;
//   servo.t_dec = dec;
//   servo.power = 0;

//   status = FSUS_SyncCommand(usart, 1, MODE_SET_SERVO_ANGLE_BY_INTERVAL, &servo, NULL);

//   if (status == FSUS_STATUS_SUCCESS)
//   {
//     printf("[Gripper Open] Command sent successfully\r\n");
//   }
//   else
//   {
//     printf("[Gripper Open] Command failed, status code: %d\r\n", status);
//   }
// }

// /**
//  * @brief ID4: 夹抓闭合 - 舵机闭合到指定角度
//  * @param usart UART实例指针
//  * @param time_ms 执行时间(毫秒)
//  * @param acc 加速时间(毫秒)
//  * @param dec 减速时间(毫秒)
//  */
// void Servo_GripperClose(Usart_DataTypeDef *usart, uint16_t time_ms, uint16_t acc, uint16_t dec)
// {
//   float angle;
//   FSUS_sync_servo servo;
//   FSUS_STATUS status;

//   angle = 30.0f; // 固定闭合角度

//   printf("[Gripper Close] Angle %.1f deg, time %dms (acc %dms, dec %dms)\r\n", angle, time_ms, acc, dec);

//   servo.id = 4;
//   servo.angle = angle;
//   servo.interval_single = time_ms;
//   servo.t_acc = acc;
//   servo.t_dec = dec;
//   servo.power = 0;

//   status = FSUS_SyncCommand(usart, 1, MODE_SET_SERVO_ANGLE_BY_INTERVAL, &servo, NULL);

//   if (status == FSUS_STATUS_SUCCESS)
//   {
//     printf("[Gripper Close] Command sent successfully\r\n");
//   }
//   else
//   {
//     printf("[Gripper Close] Command failed, status code: %d\r\n", status);
//   }
// }

// /**
//  * @brief ID6: 上下运动 (多圈模式)
//  * @param usart UART实例指针
//  * @param angle 目标角度 (可以是多圈的角度, e.g., 720.0 for 2 turns)
//  * @param time_ms 执行时间(毫秒)
//  * @param power 舵机功率 (0-1000, 0为不限制)
//  */
// void Servo_UpDown(Usart_DataTypeDef *usart, float angle, uint32_t time_ms, uint16_t power)
// {
//   FSUS_STATUS status;

//   // 打印日志，方便调试
//   printf("[Up/Down] Rotate to %.1f deg, time %lums, power %dmW (multi-turn mode)\r\n", angle, time_ms, power);

//   // 调用多圈角度控制指令
//   // 参数: usart实例, 舵机ID, 目标角度, 执行时间, 功率
//   status = FSUS_SetServoAngleMTurn(usart, 6, angle, time_ms, power);

//   // 检查指令发送状态
//   if (status == FSUS_STATUS_SUCCESS)
//   {
//     printf("[Up/Down] Command sent successfully\r\n");
//   }
//   else
//   {
//     printf("[Up/Down] Command failed, status code: %d\r\n", status);
//   }
// }
/**
 * @brief ID1: 物料转盘旋转 (多圈模式)
 */
void Servo_TurntableRotate(Usart_DataTypeDef *usart, float angle, uint32_t time_ms, uint16_t power)
{
  FSUS_SetServoAngleMTurn(usart, 1, angle, time_ms, power);
}

/**
 * @brief ID2: 主旋转盘 (多圈模式)
 */
void Servo_MainRotate(Usart_DataTypeDef *usart, float angle, uint32_t time_ms, uint16_t power)
{
  FSUS_SetServoAngleMTurn(usart, 2, angle, time_ms, power);
}

/**
 * @brief ID3: 前后运动 (多圈模式)
 */
void Servo_ForwardBackward(Usart_DataTypeDef *usart, float angle, uint32_t time_ms, uint16_t power)
{
  FSUS_SetServoAngleMTurn(usart, 3, angle, time_ms, power);
}

// Servo_GripperOpen 和 Servo_GripperClose 保持不变...

/**
 * @brief ID6: 上下运动 (多圈模式)
 */
void Servo_UpDown(Usart_DataTypeDef *usart, float angle, uint32_t time_ms, uint16_t power)
{
  FSUS_SetServoAngleMTurn(usart, 6, angle, time_ms, power);
}

/**
 * @brief ID4: 夹抓张开 - 从当前位置反转指定行程角度 (MTurn版)
 */
void Servo_GripperOpen(Usart_DataTypeDef *usart, uint32_t time_ms, uint16_t power)
{
  float current_angle = 0.0f;
  float target_angle = 0.0f;

  // 1. 查询当前角度
  FSUS_QueryServoAngleMTurn(usart, 4, &current_angle);

  // 2. 计算目标角度 (当前角度 - 行程)
  target_angle = current_angle + 20;

  // 3. 发送 MTurn 指令
  FSUS_SetServoAngleMTurn(usart, 4, target_angle, time_ms, power);
}

/**
 * @brief ID4: 夹抓闭合 - 从当前位置正转指定行程角度 (MTurn版)
 */
void Servo_GripperClose(Usart_DataTypeDef *usart, uint32_t time_ms, uint16_t power)
{
  float current_angle = 0.0f;
  float target_angle = 0.0f;

  // 1. 查询当前角度
  FSUS_QueryServoAngleMTurn(usart, 4, &current_angle);

  // 2. 计算目标角度 (当前角度 + 行程)
  target_angle = current_angle + 50 + 25;

  // 3. 发送 MTurn 指令
  FSUS_SetServoAngleMTurn(usart, 4, target_angle, time_ms, power);
}

// ========== 状态查询函数 ==========

/**
 * @brief 4舵机状态查询函数
 * @param usart UART实例指针
 */
void Servo_QueryStatus(Usart_DataTypeDef *usart)
{
  ServoData servo_data[4];
  FSUS_sync_servo servo_sync[4];
  FSUS_STATUS status;
  int i;

  printf("[Query] 4-servo status query:\r\n");

  // 初始化同步舵机结构体数组
  for (i = 0; i < 4; i++)
  {
    servo_sync[i].id = i + 1; // 舵机ID: 1, 2, 3, 4
    servo_sync[i].angle = 0;
    servo_sync[i].velocity = 0;
    servo_sync[i].interval_single = 0;
    servo_sync[i].interval_multi = 0;
    servo_sync[i].t_acc = 0;
    servo_sync[i].t_dec = 0;
    servo_sync[i].power = 0;
  }

  // 使用同步监控命令获取舵机状态
  status = FSUS_SyncCommand(usart, 4, MODE_Query_SERVO_Monitor, servo_sync, servo_data);

  if (status == FSUS_STATUS_SUCCESS)
  {
    printf("Command sent successfully and received all responses\r\n");

    for (i = 0; i < 4; i++)
    {
      printf("  Servo%d: angle=%.1f deg, voltage=%dmV, current=%dmA, temp=%.1f C, power=%dmW\r\n",
             servo_data[i].id,
             servo_data[i].angle,
             servo_data[i].voltage,
             servo_data[i].current,
             servo_data[i].temperature,
             servo_data[i].power);
    }
  }
  else
  {
    printf("[Query Failed] Status code: %d\r\n", status);
  }
}

/**
 * @brief 单个舵机角度查询函数
 * @param usart UART实例指针
 * @param servo_id 舵机ID (1-4)
 * @param angle 返回的角度值指针
 * @return FSUS_STATUS 查询状态
 */
FSUS_STATUS Servo_QuerySingleAngle(Usart_DataTypeDef *usart, uint8_t servo_id, float *angle)
{
  FSUS_STATUS status;

  printf("[Query] Servo%d angle query\r\n", servo_id);

  // 查询单个舵机角度
  status = FSUS_QueryServoAngle(usart, servo_id, angle);

  if (status == FSUS_STATUS_SUCCESS)
  {
    printf("[Query] Servo%d angle: %.1f deg\r\n", servo_id, *angle);
  }
  else
  {
    printf("[Query Failed] Servo%d query failed, status code: %d\r\n", servo_id, status);
  }

  return status;
} // ========== 示例函数 ==========

// /**
//  * @brief 舵机控制示例函数
//  * @param usart UART实例指针
//  */
// void Servo_Example(Usart_DataTypeDef *usart)
// {
//   printf("=== Servo Independent Control Example ===\r\n");

//   printf("\nExample 0: Set material turntable initial position (-9 deg)\r\n");
//   Servo_TurntableSetInitPosition(usart, 2000);
//   HAL_Delay(3000);

//   printf("\nExample 1: Convert 1 item (60 deg)\r\n");
//   servo_1(usart, 1, 2500);
//   HAL_Delay(3000);

//   printf("\nExample 1a: Convert 3 items (180 deg)\r\n");
//   servo_1(usart, 3, 7500);
//   HAL_Delay(8000);

//   printf("\nExample 2: Main rotation to 90 deg\r\n");
//   Servo_MainRotate(usart, 90.0f, 2000, 100);
//   HAL_Delay(2000);

//   printf("\nExample 3: Forward/Backward motion 720 deg (2 turns)\r\n");
//   Servo_ForwardBackward(usart, 720.0f, 3000, 0);
//   HAL_Delay(3500);

//   printf("\nExample 4: Gripper open and close\r\n");
//   Servo_GripperOpen(usart, 1000, 100, 100);
//   HAL_Delay(1500);
//   Servo_GripperClose(usart, 1000, 100, 100);
//   HAL_Delay(1500);

//   printf("\nExample 5: Query all servo status\r\n");
//   Servo_QueryStatus(usart);
//   HAL_Delay(1000);

//   printf("\n=== Example finished ===\r\n");
// }

// ========== 组合动作函数 ==========

#include "ssd1306.h"
extern const SSD1306_Font_t Font_7x10;
void display(char *str)
{
  ssd1306_Fill(0);
  ssd1306_SetCursor(0, 0);
  ssd1306_WriteString(str, Font_7x10, 1);
  ssd1306_UpdateScreen();
}

// /**
//  * @brief 执行从外部抓取物料并放置到物料盘的全过程 (最终版)
//  * @note  本版本为每个动作手动设定了独立的执行时间, 并通过绝对归位消除累积误差。
//  * @param usart UART实例指针
//  */
// void Servo_catch(Usart_DataTypeDef *usart)
// {
//     float current_angle = 0.0f;
//     float target_angle = 0.0f;

//     // 在动作开始前, 记录初始位置
//     float initial_angle_id2 = 0.0f;
//     float initial_angle_id6 = 0.0f;

//     FSUS_QueryServoAngleMTurn(usart, 2, &initial_angle_id2);
//     FSUS_QueryServoAngleMTurn(usart, 6, &initial_angle_id6);

//     HAL_Delay(50); // 短暂延时确保查询指令完成

//     // Servo_UpDown(usart, initial_angle_id6, 1000, 0);
//     // HAL_Delay(500);

//     // 步骤 1: id6 下降到获取外部物料位置
//     FSUS_QueryServoAngleMTurn(usart, 6, &current_angle);
//     HAL_Delay(50);
//     target_angle = current_angle ;
//     Servo_UpDown(usart, target_angle, 2500, 0); // 手动设定下降时间为 2000ms
//     display("1:id6 down");
//     HAL_Delay(3600); // 动作时间 + 500ms 缓冲

//     // 步骤 2: 调用id4夹取
//     Servo_GripperClose(usart, 1300, 0); // 手动设定夹取时间为 800ms
//     display("2:id4 catch");
//     HAL_Delay(1300);

//     // 步骤 3: id6 抬升
//     FSUS_QueryServoAngleMTurn(usart, 6, &current_angle);
//     HAL_Delay(50);
//     target_angle = current_angle +200; //ARM_LIFT_AFTER_CATCH;
//     Servo_UpDown(usart, target_angle, 2500, 0); // 手动设定抬升时间为 1800ms
//     display("3:id6 up");
//     HAL_Delay(3200);

//     // 步骤 4: id2 转身
//     FSUS_QueryServoAngleMTurn(usart, 2, &current_angle);
//     HAL_Delay(50);
//     target_angle = current_angle + MAIN_BODY_TURN_AROUND;
//     Servo_MainRotate(usart, target_angle, 2000, 0); // 手动设定转身时间为 1500ms
//     display("4:id2 turn");
//     HAL_Delay(2000);

//     // 步骤 5: id6 下降到物料盘高度
//     FSUS_QueryServoAngleMTurn(usart, 6, &current_angle);
//     HAL_Delay(50);
//     target_angle = current_angle - ARM_DESCEND_TO_TABLE;
//     Servo_UpDown(usart, target_angle, 1200, 0); // 手动设定下降时间为 1200ms
//     display("5:id6 down");
//     HAL_Delay(2400);

//     // 步骤 6: 调用id4放下物料到物料盘
//     Servo_GripperOpen(usart, 800, 0); // 手动设定张开时间为 800ms
//     display("6:id4 release");
//     HAL_Delay(1300);

//     // 步骤 7: id6 返回最高点 (直接回到初始位置)
//     // FSUS_QueryServoAngleMTurn(usart, 6, &current_angle);
//     // target_angle = current_angle + ARM_DESCEND_TO_TABLE - ARM_LIFT_AFTER_CATCH + ARM_DESCEND_FOR_CATCH;
//     Servo_UpDown(usart, initial_angle_id6, 1200, 0); // 手动设定归位时间为 1200ms
//     display("7:id6 up");
//     HAL_Delay(2700);

//     // 步骤 8: id2 转身回到初始位置 (直接回到初始位置)
//     FSUS_QueryServoAngleMTurn(usart, 2, &current_angle);
//     HAL_Delay(50);
//     target_angle = current_angle - MAIN_BODY_TURN_AROUND;
//     Servo_MainRotate(usart, target_angle, 1500, 0); // 手动设定归位时间为 1500ms
//     display("8:id2 turn");
//     HAL_Delay(2000);

//     // 步骤 9: id1 转到下一个物料槽
//     FSUS_QueryServoAngleMTurn(usart, 1, &current_angle);
//     HAL_Delay(50);
//     target_angle = current_angle + TURNTABLE_NEXT_SLOT;
//     Servo_TurntableRotate(usart, target_angle, 1000, 0); // 手动设定转盘时间为 1000ms
//     display("9:id1 turn");
//     HAL_Delay(1000);
// }

/**
 * @brief 将关键舵机的当前物理位置设置为新的零点 (静默版)
 * @note  本函数会依次失锁舵机并写入新零点，无任何日志输出和状态检查。
 *        执行前，必须手动将机械臂摆放到理想的零姿态。
 *        执行过程中，舵机将失锁，执行完毕后需要重启设备。
 * @param usart UART实例指针
 */
void Servo_SetCurrentAsOrigin(Usart_DataTypeDef *usart) // 无效
{
  // --- 为主旋转盘(ID2)设置零点 ---
  FSUS_StopOnControlMode(usart, 2, 0, 0); // Mode 0 = 停止后卸力(失锁)
  HAL_Delay(50);                          // 等待舵机失锁
  FSUS_SetOriginPoint(usart, 2, 1);       // 参数'1'表示执行设置
  HAL_Delay(100);                         // 等待EEPROM写入完成

  // --- 为上下臂(ID6)设置零点 ---
  FSUS_StopOnControlMode(usart, 6, 0, 0);
  HAL_Delay(50);
  FSUS_SetOriginPoint(usart, 6, 1);
  HAL_Delay(100);

  // --- 为物料盘(ID1)设置零点 ---
  FSUS_StopOnControlMode(usart, 1, 0, 0);
  HAL_Delay(50);
  FSUS_SetOriginPoint(usart, 1, 1);
  HAL_Delay(100);

  // --- 为夹爪(ID4)设置零点 ---
  FSUS_StopOnControlMode(usart, 4, 0, 0);
  HAL_Delay(50);
  FSUS_SetOriginPoint(usart, 4, 1);
  HAL_Delay(100);
}

// 夹爪动作
void Servo_Close(Usart_DataTypeDef *usart)
{
  FSUS_SetServoAngleMTurn(usart, 4, 34, 400, 0);
}
void Servo_Open(Usart_DataTypeDef *usart)
{
  FSUS_SetServoAngleMTurn(usart, 4, 12, 400, 0);
}
void Servo_Open_big(Usart_DataTypeDef *usart)
{
  FSUS_SetServoAngleMTurn(usart, 4, 5, 800, 0);
}
// 放置区域动作
void Servo_PlaceBlue(Usart_DataTypeDef *usart)
{
  Servo_TurntableRotate(usart, -160, 900, 0);
}
void Servo_PlaceGreen(Usart_DataTypeDef *usart)
{
  Servo_TurntableRotate(usart, -160 + 120, 900, 0);
}
void Servo_PlaceRed(Usart_DataTypeDef *usart)
{
  Servo_TurntableRotate(usart,  -160 + 120 + 120 + 2, 900, 0);
}
// 主旋转轴动作
void Servo_RotateBehind(Usart_DataTypeDef *usart)
{
  Servo_MainRotate(usart, -139, 1500,0);
}
void Servo_RotateMiddle(Usart_DataTypeDef *usart)
{
  Servo_MainRotate(usart, 29, 1200,0);
}
void Servo_RotateRight(Usart_DataTypeDef *usart)
{
  Servo_MainRotate(usart, 74-3, 1200,0);
}
void Servo_RotateLeft(Usart_DataTypeDef *usart)
{
  Servo_MainRotate(usart, -17, 1200,0);
}


// 中央，左侧，右侧
// 抓取高度15cm
// id3前后  初始20，伸长极限600，抓取两侧200
// id2正对物料 +135 左侧90 右侧180

/**
 * @brief 执行抓取序列 (无状态版)
 * @note  本版本在每次调用时，都将当前的舵机位置动态设为该次动作的“零点”，
 *        并在结束时精确返回此零点，从而无需全局变量即可保证归位精度。
 *        默认执行前，机械臂应处于静止的初始姿态。
 * @param usart UART实例指针
 */
void Servo_catch(Usart_DataTypeDef *usart)
{
  // Servo_SetCurrentAsOrigin(usart); // 动态设定当前为零点

  float target_angle = 0.0f;

  // --- 步骤 0: 动态校准 ---
  // 在动作开始前，获取 ID2 和 ID6 的当前绝对位置，
  // 将其作为本次独立任务的“动态零点”。
  float dynamic_origin_id2 = 0.0f;
  float dynamic_origin_id6 = 0.0f;

  FSUS_ServoAngleReset(usart, 2); // 重置 ID2 的多圈计数
  FSUS_ServoAngleReset(usart, 1);
  FSUS_ServoAngleReset(usart, 3);
  FSUS_ServoAngleReset(usart, 6);
  FSUS_ServoAngleReset(usart, 4);

  FSUS_QueryServoAngleMTurn(usart, 2, &dynamic_origin_id2);
  HAL_Delay(100);
  FSUS_QueryServoAngleMTurn(usart, 6, &dynamic_origin_id6);
  HAL_Delay(100);

  char buffer[20];
  sprintf(buffer, "0:dynamic_origin_id6: %.2f", dynamic_origin_id6);
  display(buffer);
  HAL_Delay(2000);

  Servo_TurntableRotate(usart, -100 - 240, 500, 0);
  HAL_Delay(1000);

  Servo_ForwardBackward(usart, 20, 2000, 0);
  HAL_Delay(1000);

  // --- 步骤 1: id6 下降 ---
  // 基于动态零点进行计算
  target_angle = dynamic_origin_id6 - ARM_DESCEND_FOR_CATCH * 2;
  Servo_UpDown(usart, target_angle, 1800, 0);
  display("1:id6 down");
  HAL_Delay(3200);

  // --- 步骤 2: 夹取 ---
  Servo_GripperClose(usart, 800, 0);
  display("2:id4 catch");
  HAL_Delay(1800);

  // --- 步骤 3: id6 抬升 ---
  // 此时需要查询当前位置，因为下降后位置已变
  float current_angle_id6;
  FSUS_QueryServoAngleMTurn(usart, 6, &current_angle_id6);
  HAL_Delay(100);
  target_angle = current_angle_id6 + ARM_LIFT_AFTER_CATCH - 330;
  Servo_UpDown(usart, target_angle, 2200, 0);
  display("3:id6 up");
  HAL_Delay(3000);

  // --- 步骤 4: id2 转身 ---
  // 基于动态零点进行计算3
  FSUS_ServoAngleReset(usart, 2);
  target_angle = dynamic_origin_id2 - 52;
  Servo_MainRotate(usart, target_angle, 2000, 0);
  display("4:id2 turn");
  HAL_Delay(2200);

  // --- 步骤 5: id6 下降到物料盘 ---
  float current_angle_id6_step5;
  FSUS_QueryServoAngleMTurn(usart, 6, &current_angle_id6_step5);
  HAL_Delay(100);
  target_angle = current_angle_id6_step5 - ARM_DESCEND_TO_TABLE;
  Servo_UpDown(usart, target_angle, 1200, 0);
  display("5:id6 down");
  HAL_Delay(2000);

  // --- 步骤 6: 放下物料 ---
  Servo_GripperOpen(usart, 800, 0);
  display("6:id4 release");
  HAL_Delay(1500);

  // --- 步骤 7 & 8: 强制归位到本次任务开始时的“动态零点” ---
  FSUS_QueryServoAngleMTurn(usart, 2, &target_angle);
  HAL_Delay(100);
  Servo_UpDown(usart, target_angle + ARM_DESCEND_TO_TABLE, 1200, 0);
  display("7:id6 up");
  HAL_Delay(2200);

  Servo_MainRotate(usart, dynamic_origin_id2 + 90, 2000, 0); //+135改为90
  display("8:id2 turn");
  HAL_Delay(3000);

  Servo_ForwardBackward(usart, 210, 2000, 0); // 拿侧边
  HAL_Delay(1000);

  // --- 步骤 9: id1 转盘 (此舵机逻辑保持不变) ---
  float current_angle_id1 = 0;
  FSUS_QueryServoAngleMTurn(usart, 1, &current_angle_id1);
  HAL_Delay(100);
  target_angle = current_angle_id1 - 100; // 绝对位置  -100 (-120)两次
  Servo_TurntableRotate(usart, target_angle, 500, 0);
  display("9:id1 turn");
  HAL_Delay(2000);

  // --------------------------
  sprintf(buffer, "0:dynamic_origin_id6: %.2f", dynamic_origin_id6);
  display(buffer);
  HAL_Delay(2000);

  // --- 步骤 1: id6 下降 ---
  // 基于动态零点进行计算
  target_angle = dynamic_origin_id6 - ARM_DESCEND_FOR_CATCH * 2;
  Servo_UpDown(usart, target_angle, 1800, 0);
  display("1:id6 down");
  HAL_Delay(3200);

  // --- 步骤 2: 夹取 ---
  Servo_GripperClose(usart, 800, 0);
  display("2:id4 catch");
  HAL_Delay(1800);

  // --- 步骤 3: id6 抬升 ---
  // 此时需要查询当前位置，因为下降后位置已变
  current_angle_id6 = 0;
  FSUS_QueryServoAngleMTurn(usart, 6, &current_angle_id6);
  HAL_Delay(100);
  target_angle = current_angle_id6 + ARM_LIFT_AFTER_CATCH - 330;
  Servo_UpDown(usart, target_angle, 2200, 0);
  display("3:id6 up");
  HAL_Delay(3000);

  Servo_ForwardBackward(usart, 20, 2000, 0); // 收回
  HAL_Delay(1000);

  // --- 步骤 4: id2 转身 ---
  // 基于动态零点进行计算3
  FSUS_ServoAngleReset(usart, 2);
  target_angle = dynamic_origin_id2 - 52;
  Servo_MainRotate(usart, target_angle, 2000, 0);
  display("4:id2 turn");
  HAL_Delay(2200);

  // --- 步骤 5: id6 下降到物料盘 ---
  current_angle_id6_step5 = 0;
  FSUS_QueryServoAngleMTurn(usart, 6, &current_angle_id6_step5);
  HAL_Delay(100);
  target_angle = current_angle_id6_step5 - ARM_DESCEND_TO_TABLE;
  Servo_UpDown(usart, target_angle, 1200, 0);
  display("5:id6 down");
  HAL_Delay(2000);

  // --- 步骤 6: 放下物料 ---
  Servo_GripperOpen(usart, 800, 0);
  display("6:id4 release");
  HAL_Delay(1500);

  // --- 步骤 7 & 8: 强制归位到本次任务开始时的“动态零点” ---
  FSUS_QueryServoAngleMTurn(usart, 2, &target_angle);
  HAL_Delay(100);
  Servo_UpDown(usart, target_angle + ARM_DESCEND_TO_TABLE, 1200, 0);
  display("7:id6 up");
  HAL_Delay(2200);

  Servo_MainRotate(usart, dynamic_origin_id2 + 180, 2000, 0); //+135改为180
  display("8:id2 turn");
  HAL_Delay(3000);

  // --- 步骤 9: id1 转盘 (此舵机逻辑保持不变) ---
  current_angle_id1 = 0;
  FSUS_QueryServoAngleMTurn(usart, 1, &current_angle_id1);
  HAL_Delay(100);
  target_angle = current_angle_id1 - 100 - 120; // 绝对位置  -100 (-120)两次
  Servo_TurntableRotate(usart, target_angle, 500, 0);
  display("9:id1 turn");
  HAL_Delay(2000);

  Servo_ForwardBackward(usart, 210, 2000, 0); // 拿侧边
  HAL_Delay(1000);

  //----------------------------------------
  sprintf(buffer, "0:dynamic_origin_id6: %.2f", dynamic_origin_id6);
  display(buffer);
  HAL_Delay(2000);

  // --- 步骤 1: id6 下降 ---
  // 基于动态零点进行计算
  target_angle = dynamic_origin_id6 - ARM_DESCEND_FOR_CATCH * 2;
  Servo_UpDown(usart, target_angle, 1800, 0);
  display("1:id6 down");
  HAL_Delay(3200);

  // --- 步骤 2: 夹取 ---
  Servo_GripperClose(usart, 800, 0);
  display("2:id4 catch");
  HAL_Delay(1800);

  // --- 步骤 3: id6 抬升 ---
  // 此时需要查询当前位置，因为下降后位置已变
  current_angle_id6 = 0;
  FSUS_QueryServoAngleMTurn(usart, 6, &current_angle_id6);
  HAL_Delay(100);
  target_angle = current_angle_id6 + ARM_LIFT_AFTER_CATCH - 330;
  Servo_UpDown(usart, target_angle, 2200, 0);
  display("3:id6 up");
  HAL_Delay(3000);

  Servo_ForwardBackward(usart, 20, 2000, 0); // 收回
  HAL_Delay(1000);

  // --- 步骤 4: id2 转身 ---
  // 基于动态零点进行计算3
  FSUS_ServoAngleReset(usart, 2);
  target_angle = dynamic_origin_id2 - 52;
  Servo_MainRotate(usart, target_angle, 2000, 0);
  display("4:id2 turn");
  HAL_Delay(2200);

  // --- 步骤 5: id6 下降到物料盘 ---
  current_angle_id6_step5 = 0;
  FSUS_QueryServoAngleMTurn(usart, 6, &current_angle_id6_step5);
  HAL_Delay(100);
  target_angle = current_angle_id6_step5 - ARM_DESCEND_TO_TABLE;
  Servo_UpDown(usart, target_angle, 1200, 0);
  display("5:id6 down");
  HAL_Delay(2000);

  // --- 步骤 6: 放下物料 ---
  Servo_GripperOpen(usart, 800, 0);
  display("6:id4 release");
  HAL_Delay(1500);

  // --- 步骤 7 & 8: 强制归位到本次任务开始时的“动态零点” ---
  FSUS_QueryServoAngleMTurn(usart, 2, &target_angle);
  HAL_Delay(100);
  Servo_UpDown(usart, target_angle + ARM_DESCEND_TO_TABLE, 1200, 0);
  display("7:id6 up");
  HAL_Delay(2200);

  Servo_MainRotate(usart, dynamic_origin_id2 + 135, 2000, 0); // 回正
  display("8:id2 turn");
  HAL_Delay(3000);

  // --- 步骤 9: id1 转盘 (此舵机逻辑保持不变) ---
  current_angle_id1 = 0;
  FSUS_QueryServoAngleMTurn(usart, 1, &current_angle_id1);
  HAL_Delay(100);
  target_angle = current_angle_id1 - 100 - 240; // 绝对位置  -100 (-120)两次
  Servo_TurntableRotate(usart, target_angle, 500, 0);
  display("9:id1 turn");
  HAL_Delay(2000);

  //----------------------------------------
}

// 中央，左侧，右侧
// 抓取高度15cm
// id3前后  初始20，伸长极限600，抓取两侧200
// id2正对物料 +135 左侧90 右侧180
/**
 * @brief [新增] 控制舵机从物料盘抓取物料并放置到外部
 * @note  此函数是 Servo_catch 的逆过程。
 *        它假设起始状态是机械臂在最高点，夹爪张开，车身朝前。
 * @param usart 串口总线指针
 */
void Servo_place(Usart_DataTypeDef *usart)
{
  // Servo_SetCurrentAsOrigin(usart); // 动态设定当前为零点

  float target_angle = 0.0f;

  // --- 步骤 0: 动态校准 ---
  // 在动作开始前，获取 ID2 和 ID6 的当前绝对位置，
  // 将其作为本次独立任务的“动态零点”。
  float dynamic_origin_id2 = 0.0f;
  float dynamic_origin_id6 = 0.0f;

  FSUS_ServoAngleReset(usart, 2); // 重置 ID2 的多圈计数
  FSUS_ServoAngleReset(usart, 1);
  FSUS_ServoAngleReset(usart, 3);
  FSUS_ServoAngleReset(usart, 6);
  FSUS_ServoAngleReset(usart, 4);

  FSUS_QueryServoAngleMTurn(usart, 2, &dynamic_origin_id2);
  HAL_Delay(100);
  FSUS_QueryServoAngleMTurn(usart, 6, &dynamic_origin_id6);
  HAL_Delay(100);

  Servo_TurntableRotate(usart, -100 - 240, 500, 0);
  HAL_Delay(1000);

  Servo_ForwardBackward(usart, 20, 2000, 0);
  HAL_Delay(1000);

  Servo_GripperOpen(usart, 800, 0);
  HAL_Delay(1000);

  // 转身
  FSUS_ServoAngleReset(usart, 2);
  target_angle = dynamic_origin_id2 - 52;
  Servo_MainRotate(usart, target_angle, 2000, 0);
  HAL_Delay(2200);

  // --- 步骤 2: id6 下降到物料盘 ---
  float current_angle_id6_step5;
  FSUS_QueryServoAngleMTurn(usart, 6, &current_angle_id6_step5);
  HAL_Delay(100);
  target_angle = current_angle_id6_step5 - ARM_DESCEND_TO_TABLE - 402;
  Servo_UpDown(usart, target_angle, 1200, 0);
  display("5:id6 down");
  HAL_Delay(2000);

  // 抓取
  Servo_GripperClose(usart, 800, 0);
  HAL_Delay(1400);

  // 抬升回正
  FSUS_QueryServoAngleMTurn(usart, 2, &target_angle);
  HAL_Delay(100);
  Servo_UpDown(usart, target_angle + ARM_DESCEND_TO_TABLE, 1200, 0);
  display("7:id6 up");
  HAL_Delay(2200);

  // // 抬升回正
  // float current_angle_id6 = 0;
  // FSUS_QueryServoAngleMTurn(usart, 6, &current_angle_id6);
  // HAL_Delay(100);
  // target_angle = current_angle_id6 + ARM_LIFT_AFTER_CATCH - 100;
  // Servo_UpDown(usart, target_angle, 2200, 0);
  // display("3:id6 up");
  // HAL_Delay(3000);

  Servo_MainRotate(usart, dynamic_origin_id2 + 135, 2000, 0);
  display("8:id2 turn");
  HAL_Delay(3000);

  // 下降
  target_angle = dynamic_origin_id6 - ARM_DESCEND_FOR_CATCH * 2;
  Servo_UpDown(usart, target_angle, 1800, 0);
  display("1:id6 down");
  HAL_Delay(3500);

  // 放下
  Servo_GripperOpen(usart, 800, 0);
  HAL_Delay(1000);

  // 抬升回正
  // FSUS_QueryServoAngleMTurn(usart, 2, &target_angle);
  // HAL_Delay(100);
  // Servo_UpDown(usart, target_angle + ARM_DESCEND_TO_TABLE, 1200, 0);
  // display("7:id6 up");
  // HAL_Delay(2200);
  float current_angle_id6 = 0;
  FSUS_QueryServoAngleMTurn(usart, 6, &current_angle_id6);
  HAL_Delay(100);
  target_angle = current_angle_id6 + ARM_LIFT_AFTER_CATCH - 400;
  Servo_UpDown(usart, target_angle, 2200, 0);
  display("3:id6 up");
  HAL_Delay(3000);

  // 物料盘旋转
  Servo_TurntableRotate(usart, -100, 500, 0);
  HAL_Delay(1000);

  //-------------------------------------

  // 转身
  FSUS_ServoAngleReset(usart, 2);
  target_angle = dynamic_origin_id2 - 52;
  Servo_MainRotate(usart, target_angle, 2000, 0);
  HAL_Delay(2200);

  // --- 步骤 2: id6 下降到物料盘 ---
  current_angle_id6_step5 = 0;
  FSUS_QueryServoAngleMTurn(usart, 6, &current_angle_id6_step5);
  HAL_Delay(100);
  target_angle = current_angle_id6_step5 - ARM_DESCEND_TO_TABLE - 40;
  Servo_UpDown(usart, target_angle, 1200, 0);
  display("5:id6 down");
  HAL_Delay(2000);

  // 抓取
  Servo_GripperClose(usart, 800, 0);
  HAL_Delay(1000);

  // 抬升回正
  FSUS_QueryServoAngleMTurn(usart, 2, &target_angle);
  HAL_Delay(100);
  Servo_UpDown(usart, target_angle + ARM_DESCEND_TO_TABLE + 180 + 180, 1200, 0);
  display("7:id6 up");
  HAL_Delay(2200);

  Servo_MainRotate(usart, dynamic_origin_id2 + 90, 2000, 0); // 改方向
  display("8:id2 turn");
  HAL_Delay(3000);

  // 伸长
  Servo_ForwardBackward(usart, 210, 2000, 0); // 收回
  HAL_Delay(1000);

  // 下降
  target_angle = dynamic_origin_id6 - ARM_DESCEND_FOR_CATCH * 2;
  Servo_UpDown(usart, target_angle, 1800, 0);
  display("1:id6 down");
  HAL_Delay(3200);

  // 放下
  Servo_GripperOpen(usart, 800, 0);
  HAL_Delay(1000);

  // 抬升回正
  FSUS_QueryServoAngleMTurn(usart, 2, &target_angle);
  HAL_Delay(100);
  Servo_UpDown(usart, target_angle + ARM_DESCEND_TO_TABLE + 180, 1200, 0);
  display("7:id6 up");
  HAL_Delay(2200);

  // 收回
  Servo_ForwardBackward(usart, 20, 2000, 0);
  HAL_Delay(1000);

  // 物料盘旋转
  Servo_TurntableRotate(usart, -100 - 120, 500, 0);
  HAL_Delay(1000);

  //-------------------------------------

  // 转身
  FSUS_ServoAngleReset(usart, 2);
  target_angle = dynamic_origin_id2 - 52;
  Servo_MainRotate(usart, target_angle, 2000, 0);
  HAL_Delay(2200);

  // --- 步骤 2: id6 下降到物料盘 ---
  current_angle_id6_step5 = 0;
  FSUS_QueryServoAngleMTurn(usart, 6, &current_angle_id6_step5);
  HAL_Delay(100);
  target_angle = current_angle_id6_step5 - ARM_DESCEND_TO_TABLE - 40;
  Servo_UpDown(usart, target_angle, 1200, 0);
  display("5:id6 down");
  HAL_Delay(2000);

  // 抓取
  Servo_GripperClose(usart, 800, 0);
  HAL_Delay(1000);

  // 抬升回正
  FSUS_QueryServoAngleMTurn(usart, 2, &target_angle);
  HAL_Delay(100);
  Servo_UpDown(usart, target_angle + ARM_DESCEND_TO_TABLE + 180, 1200, 0);
  display("7:id6 up");
  HAL_Delay(2200);

  Servo_MainRotate(usart, dynamic_origin_id2 + 180, 2000, 0); // 改方向
  display("8:id2 turn");
  HAL_Delay(3000);

  // 伸长
  Servo_ForwardBackward(usart, 210, 2000, 0);
  HAL_Delay(1000);

  // 下降
  target_angle = dynamic_origin_id6 - ARM_DESCEND_FOR_CATCH * 2;
  Servo_UpDown(usart, target_angle, 1800, 0);
  display("1:id6 down");
  HAL_Delay(3200);

  // 放下
  Servo_GripperOpen(usart, 800, 0);
  HAL_Delay(1000);

  // 抬升回正
  FSUS_QueryServoAngleMTurn(usart, 2, &target_angle);
  HAL_Delay(100);
  Servo_UpDown(usart, target_angle + ARM_DESCEND_TO_TABLE + 180, 1200, 0);
  display("7:id6 up");
  HAL_Delay(2200);

  // 收回
  Servo_ForwardBackward(usart, 20, 2000, 0);
  HAL_Delay(1000);

  //----------------------------------

  // 夹爪回正
  Servo_MainRotate(usart, dynamic_origin_id2 + 135, 2000, 0);
  display("8:id2 turn");
  HAL_Delay(3000);

  // 物料盘旋转
  Servo_TurntableRotate(usart, -100 - 240, 500, 0);
  HAL_Delay(1000);
}