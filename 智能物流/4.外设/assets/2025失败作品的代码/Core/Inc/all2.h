#include <stdbool.h>

extern bool color_matched;

void get_color();
void task1();
void task2();
void task3();
void task4();
void task5(void);
void task6(void);
void task7(void);
void task8(void);
void task(void);
bool Car_QR_Wait_And_Display(bool *out_color_matched, uint32_t timeout_ms);

// ========== Motor PID 封装函数声明 ==========
// 这些函数将原有的运动控制+等待逻辑封装成一个函数调用
// 超时时间为运动时间+1秒，超时时自动停止电机

/**
 * @brief 小车向前运动并自动等待到位（带超时保护）
 * @param pulses 运动的脉冲数，正数向前，负数向后
 * @retval bool 成功到达目标位置返回true，超时返回false
 */
void Car_Move_Forward_Auto(int32_t pulses);

/**
 * @brief 小车向左平移并自动等待到位（带超时保护）
 * @param pulses 运动的脉冲数
 * @retval bool 成功到达目标位置返回true，超时返回false
 */
void Car_Move_Left_Auto(uint32_t pulses);

/**
 * @brief 小车向右平移并自动等待到位（带超时保护）
 * @param pulses 运动的脉冲数
 * @retval bool 成功到达目标位置返回true，超时返回false
 */
void Car_Move_Right_Auto(uint32_t pulses);

/**
 * @brief 小车后退并自动等待到位（带超时保护）
 * @param pulses 运动的脉冲数
 */
void Car_Move_Back_Auto(uint32_t pulses);

bool Car_Move_Position_Smooth(int32_t pulses, uint32_t timeout_ms);
