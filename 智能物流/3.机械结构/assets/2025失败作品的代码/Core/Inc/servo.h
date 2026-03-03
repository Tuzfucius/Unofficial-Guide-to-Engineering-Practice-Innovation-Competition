#ifndef SERVO_H
#define SERVO_H

#include "fashion_star_uart_servo.h"
#include "usart.h"

// ========== 组合动作的相对运动宏定义 (新) ==========
// --- ID6 (机械臂上下) ---
#define ARM_DESCEND_FOR_CATCH 400.0f // 从最高点下降以抓取外部物料的角度
#define ARM_LIFT_AFTER_CATCH 400.0f  // 抓取到物料后抬升的角度
#define ARM_DESCEND_TO_TABLE 230.0f  // 为放置在物料盘上而下降的角度
#define ARM_ASCEND_TO_HOME 230.0f    // 从物料盘放好后返回最高点的角度

// --- ID2 (主旋转盘) ---
#define MAIN_BODY_TURN_AROUND 100.0f // 主旋转盘转身180度的角度

// --- ID1 (物料盘) ---
#define TURNTABLE_NEXT_SLOT 120.0f // 物料盘转到下一个槽位的角度

// --- ID3 (前后伸缩) ---
#define LINEAR_EXTEND_FULL 600.0f // 从最内侧完全伸出到最外侧的角度

// --- ID4 (夹爪) ---
#define GRIPPER_TRAVEL_ANGLE 50.0f // 夹爪从张开到闭合的行程角度

// ========== 4个舵机独立控制函数声明 ==========

// ID1: 物料转盘旋转 (多圈模式)
void Servo_TurntableRotate(Usart_DataTypeDef *usart, float angle, uint32_t time_ms, uint16_t power);

// ID2: 主旋转盘 (多圈模式)
void Servo_MainRotate(Usart_DataTypeDef *usart, float angle, uint32_t time_ms, uint16_t power);

// ID3: 前后运动 (多圈模式)
// 最远角度150(505) 最近-280(75)
#define FORWARD_BACKWARD_FULL 150.0f+437
#define FORWARD_BACKWARD_MIN -280.0f+437
#define FORWARD_BACKWARD_CATCH_MIN -250.0f+437
#define FORWARD_BACKWARD_CATCH -55.0f+437
void Servo_ForwardBackward(Usart_DataTypeDef *usart, float angle, uint32_t time_ms, uint16_t power);

// ID4: 夾爪控制 (多圈模式 - 更新函数签名)
void Servo_GripperOpen(Usart_DataTypeDef *usart, uint32_t time_ms, uint16_t power);
void Servo_GripperClose(Usart_DataTypeDef *usart, uint32_t time_ms, uint16_t power);

// ID6: 上下运动 (多圈模式) 废弃了
void Servo_UpDown(Usart_DataTypeDef *usart, float angle, uint32_t time_ms, uint16_t power);

// // ID1: 物料转盘旋转 (单圈模式)
// void Servo_TurntableRotate(Usart_DataTypeDef *usart, float angle, uint16_t time_ms, uint16_t acc, uint16_t dec);

// // ID2: 主旋转盘 (单圈模式)
// void Servo_MainRotate(Usart_DataTypeDef *usart, float angle, uint16_t time_ms, uint16_t acc, uint16_t dec);

// // ID3: 前后运动 (多圈模式)
// void Servo_ForwardBackward(Usart_DataTypeDef *usart, float angle, uint32_t time_ms, uint16_t power);

// // ID4: 夹抓控制 (单圈模式)
// void Servo_GripperOpen(Usart_DataTypeDef *usart, uint16_t time_ms, uint16_t acc, uint16_t dec);
// void Servo_GripperClose(Usart_DataTypeDef *usart, uint16_t time_ms, uint16_t acc, uint16_t dec);

// // ID6: 上下运动 (多圈模式)
// void Servo_UpDown(Usart_DataTypeDef *usart, float angle, uint32_t time_ms, uint16_t power);

// // 查询函数
// void Servo_QueryAll(Usart_DataTypeDef *usart);

// // 示例函数
// void Servo_Examples(Usart_DataTypeDef *usart);

void test(Usart_DataTypeDef *usart, uint16_t time_ms);

// 物料盘动作
void Servo_PlaceBlue(Usart_DataTypeDef *usart);
void Servo_PlaceGreen(Usart_DataTypeDef *usart);
void Servo_PlaceRed(Usart_DataTypeDef *usart);

// 主旋转轴动作
void Servo_RotateBehind(Usart_DataTypeDef *usart);
void Servo_RotateMiddle(Usart_DataTypeDef *usart);
void Servo_RotateRight(Usart_DataTypeDef *usart);
void Servo_RotateLeft(Usart_DataTypeDef *usart);

// 抓取动作
void Servo_Close(Usart_DataTypeDef *usart);
void Servo_Open(Usart_DataTypeDef *usart);
void Servo_Open_big(Usart_DataTypeDef *usart);


// ========== 组合动作函数 ==========
/**
 * @brief 执行一套完整的抓取、放置、归位动作
 * @note  所有角度均为相对运动。函数会先查询当前角度再执行。
 * @param usart UART实例指针
 */
void Servo_catch(Usart_DataTypeDef *usart);

/**
 * @brief 执行一套完整的从转盘取物、转身、放置的动作
 * @note  所有角度均为相对运动。
 * @param usart UART实例指针
 */
void Servo_place(Usart_DataTypeDef *usart);

// ========== 校准函数 ==========
void Servo_SetCurrentAsOrigin(Usart_DataTypeDef *usart);

void display(char *str);

#endif /* SERVO_H */