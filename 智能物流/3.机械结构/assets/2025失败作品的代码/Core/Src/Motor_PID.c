#include "Motor_PID.h"
#include "Motor.h"     // 包含电机底层驱动
#include "angle_pid.h" // 包含角度PID控制
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include <All.h>
#include <math.h>  // 用于 fabsf()
#include <stdio.h> // 用于 snprintf()
#include <math.h>  // 确保已包含

#ifndef PI
#define PI 3.1415926535f
#endif

// ... 其他代码 ...

/* ============================================================================
 */
/*            宏定义 和 私有变量 (Speed Loop Only Implementation)             */
/* ============================================================================
 */

// 为四个电机定义独立的PID控制结构体
Motor_PID_t MotorA_PID;
Motor_PID_t MotorB_PID;
Motor_PID_t MotorC_PID;
Motor_PID_t MotorD_PID;

// <<< NEW: 定义并初始化PID激活状态的全局标志位
// 0: Inactive (PID运算暂停), 1: Active (PID运算运行)
uint8_t g_is_pid_active = 0;

// <<< NEW: 平滑启动 (速度爬升) 功能相关全局变量定义
bool g_is_ramping = false;
float g_final_target_rpm[4] = {0.0f, 0.0f, 0.0f, 0.0f};
float g_current_ramp_rpm[4] = {0.0f, 0.0f, 0.0f, 0.0f};
uint32_t g_ramp_start_tick = 0;

// <<< NEW: 斜率控制 (Slew Rate Control) 相关全局变量
bool g_is_slewing = false;                                    // 是否正在进行斜率控制
float g_slew_initial_rpm[4] = {0.0f, 0.0f, 0.0f, 0.0f};      // 斜率控制开始时的RPM
float g_slew_final_target_rpm[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // 斜率控制的最终目标RPM
uint32_t g_slew_start_tick = 0;                               // 斜率控制开始的时间戳

bool g_is_decelerating = false;
float g_start_decel_rpm[4] = {0.0f, 0.0f, 0.0f, 0.0f};
uint32_t g_decel_start_tick = 0;

// <<< NEW: 位置模式平滑启动控制变量
bool g_is_position_ramping = false;
float g_position_final_speed_limit[4] = {0.0f, 0.0f, 0.0f, 0.0f};
float g_position_current_speed_limit[4] = {0.0f, 0.0f, 0.0f, 0.0f};
uint32_t g_position_ramp_start_tick = 0;

// <<< NEW: 位置模式结束时是否需要自动平滑刹车
bool g_position_auto_stop_enabled = false;

// 全局死区进入次数计数器，用于显示
uint8_t g_deadband_entry_count = 0;

static bool g_position_deadband_timing = false;
static uint32_t g_position_deadband_entry_tick = 0;
static bool g_position_stop_pending = false;
volatile bool g_request_stop = false; // 异步请求急停，由主循环处理

void Car_Stop_Request(void)
{
  g_request_stop = true;
}

static bool Motor_PID_All_Position_Errors_In_Deadband(float threshold)
{
  const Motor_PID_t *motors[] = {&MotorA_PID, &MotorB_PID, &MotorC_PID, &MotorD_PID};

  for (int i = 0; i < 4; ++i)
  {
    if (motors[i]->Mode == PID_MODE_POSITION)
    {
      if (fabsf(motors[i]->Position_PID.Error) >= threshold)
      {
        return false;
      }
    }
  }

  return true;
}

static bool Motor_PID_Has_Position_Control(void)
{
  const Motor_PID_t *motors[] = {&MotorA_PID, &MotorB_PID, &MotorC_PID, &MotorD_PID};

  for (int i = 0; i < 4; ++i)
  {
    if (motors[i]->Mode == PID_MODE_POSITION)
    {
      return true;
    }
  }

  return false;
}

bool Car_Wait_For_Position_Stop(uint32_t timeout_ms)
{
  const uint32_t poll_interval_ms = 10U;
  uint32_t start_tick = HAL_GetTick();
  bool deadband_tracking = false;
  uint32_t deadband_start_tick = 0U;

  while (1)
  {
    Car_Process_Stop_Request();
    if (!g_is_pid_active)
    {
      return true;
    }

    bool has_position = false;
    const Motor_PID_t *motors[] = {&MotorA_PID, &MotorB_PID, &MotorC_PID, &MotorD_PID};

    for (int i = 0; i < 4; ++i)
    {
      if (motors[i]->Mode == PID_MODE_POSITION)
      {
        has_position = true;
        break;
      }
    }

    if (!has_position)
    {
      return true;
    }

    bool all_in_final_deadband =
        Motor_PID_All_Position_Errors_In_Deadband(POSITION_PID_DEADBAND_FINAL);

    if (all_in_final_deadband)
    {
      if (!deadband_tracking)
      {
        deadband_tracking = true;
        deadband_start_tick = HAL_GetTick();
      }
      else if ((HAL_GetTick() - deadband_start_tick) >= POSITION_DEADBAND_HOLD_MS)
      {
        return true;
      }
    }
    else
    {
      deadband_tracking = false;
    }

    if (timeout_ms > 0U)
    {
      uint32_t elapsed = HAL_GetTick() - start_tick;
      if (elapsed >= timeout_ms)
      {
        return false;
      }
    }

    HAL_Delay(poll_interval_ms);
  }
}

/* ============================================================================
 */
/*                             私有函数 (核心算法) */
/* ============================================================================
 */

/**
 * @brief  【修正】位置式PID控制器计算函数 计算PID输出
 */
float PID_Calc(PID_Controller_t *pid, float target, float measured)
{
  pid->Target = target;
  pid->Measured = measured;
  pid->Error = pid->Target - pid->Measured;

  // 积分项累加，并进行积分限幅
  pid->Integral += pid->Error;
  if (pid->Integral > pid->Integral_Max)
  {
    pid->Integral = pid->Integral_Max;
  }
  else if (pid->Integral < -pid->Integral_Max)
  {
    pid->Integral = -pid->Integral_Max;
  }

  // PID核心计算公式 (位置式)
  pid->Output = pid->Kp * pid->Error + pid->Ki * pid->Integral +
                pid->Kd * (pid->Error - pid->Last_Error);

  // 更新上一次误差
  pid->Last_Error = pid->Error;

  // 输出限幅
  if (pid->Output > pid->Output_Max)
  {
    pid->Output = pid->Output_Max;
  }
  else if (pid->Output < -pid->Output_Max)
  {
    pid->Output = -pid->Output_Max;
  }

  return pid->Output;
}

/**
 * @brief  更新单个电机的双环PID控制 (已重构)
 */
static void Motor_PID_Update_Single(Motor_PID_t *motor_pid,
                                    void (*motor_set_speed_func)(int))
{
  float speed_target = 0.0f;
  int motor_index = 0;

  if (motor_pid == &MotorA_PID)
  {
    motor_index = 0;
  }
  else if (motor_pid == &MotorB_PID)
  {
    motor_index = 1;
  }
  else if (motor_pid == &MotorC_PID)
  {
    motor_index = 2;
  }
  else
  {
    motor_index = 3;
  }

  // --- 位置环 (外环) ---
  if (motor_pid->Mode == PID_MODE_POSITION)
  {
    // 1. 获取当前位置，计算位置误差（编码器）
    motor_pid->Position_PID.Measured = motor_pid->Encoder->Total_counter;
    motor_pid->Position_PID.Error =
      motor_pid->Position_PID.Target - motor_pid->Position_PID.Measured;

    float abs_error = fabsf(motor_pid->Position_PID.Error);

    // 2. 应用双死区逻辑
    if (abs_error < POSITION_PID_DEADBAND_FINAL)
    {
      // --- 在死区内 ---
      // 认为已经到达目标. 若仍在平滑刹车中，则保持当前目标；否则提供小幅保持力矩
      if (g_is_decelerating)
      {
        speed_target = motor_pid->Speed_PID.Target;
      }
      else
      {
        float hold_kp = motor_pid->Position_PID.Kp * POSITION_HOLD_KP_FACTOR;
        float hold_kd = POSITION_HOLD_KD_FACTOR;
        float hold_speed = hold_kp * motor_pid->Position_PID.Error -
                           hold_kd * motor_pid->Speed_PID.Measured;

        if (hold_speed > POSITION_HOLD_MAX_SPEED_RPM)
        {
          hold_speed = POSITION_HOLD_MAX_SPEED_RPM;
        }
        else if (hold_speed < -POSITION_HOLD_MAX_SPEED_RPM)
        {
          hold_speed = -POSITION_HOLD_MAX_SPEED_RPM;
        }

        speed_target = hold_speed;
      }

      // 【修正】关键：只重置积分项以防止过冲和累积，但【不要】禁用PID模式。
      // 让PID控制器继续保持对位置的“锁定”，以提供保持力矩。
      motor_pid->Position_PID.Integral = 0.0f;
      motor_pid->Speed_PID.Integral = 0.0f; // 同时重置速度环积分，准备刹车
      // 【已删除】 motor_pid->Mode = PID_MODE_DISABLE; // <-- 这是导致问题的根源
    }
    else
    {
      // --- 在死区外 ---
      // 正常执行位置环PID计算，其输出是速度环的目标
      speed_target =
          PID_Calc(&motor_pid->Position_PID, motor_pid->Position_PID.Target,
                   motor_pid->Position_PID.Measured);

      // 限制位置模式输出的速度峰值，避免瞬间冲到极限
      float position_speed_limit = POSITION_MODE_MAX_SPEED_RPM;
      float final_limit = g_position_final_speed_limit[motor_index];

      if (final_limit > 0.0f && final_limit < position_speed_limit)
      {
        position_speed_limit = final_limit;
      }

      if (g_position_auto_stop_enabled && abs_error < POSITION_PID_DEADBAND_SMOOTH)
      {
        float smooth_limit = POSITION_SMOOTH_MAX_SPEED_RPM;

        if (smooth_limit < position_speed_limit)
        {
          position_speed_limit = smooth_limit;
        }
      }

      if (g_is_position_ramping)
      {
        position_speed_limit = g_position_current_speed_limit[motor_index];
      }

      if (speed_target > position_speed_limit)
      {
        speed_target = position_speed_limit;
      }
      else if (speed_target < -position_speed_limit)
      {
        speed_target = -position_speed_limit;
      }
    }
  }

  else if (motor_pid->Mode == PID_MODE_SPEED)
  {
    // 速度模式：直接使用速度环设定的目标
    // 注意：这个值可能在 Motor_PID_Ramp_Update() 中被平滑地更新
    speed_target = motor_pid->Speed_PID.Target;
  }

  // --- 速度环 (内环) - 无论哪种模式都必须执行 ---
  // 位置模式：speed_target来自位置环PID输出
  // 速度模式：speed_target来自外部设定值 (可能经过爬升处理)
  motor_pid->Speed_PID.Target = speed_target;

  // V2: 编码器速度已经是RPM单位，直接使用
  motor_pid->Speed_PID.Measured = motor_pid->Encoder->Velocity;
  int pwm_output =
      (int)PID_Calc(&motor_pid->Speed_PID, motor_pid->Speed_PID.Target,
                    motor_pid->Speed_PID.Measured);

  // 应用控制输出到电机驱动
  motor_set_speed_func(-pwm_output);
}
/* ============================================================================
 */
/*                             公共函数 (API) */
/* ============================================================================
 */

/**
 * @brief [内部函数] 更新速度爬升状态
 * @note  应该在PID更新循环中被调用. 已升级为S型曲线加速。
 */
void Motor_PID_Ramp_Update(void)
{
  if (!g_is_ramping)
  {
    return; // 如果没有在爬升，直接返回
  }

  // 1. 计算从爬升开始到现在所经过的时间
  uint32_t elapsed_time = HAL_GetTick() - g_ramp_start_tick;

  // 2. 判断爬升是否已经完成
  if (elapsed_time >= RAMP_UP_DURATION_MS)
  {
    // 爬升结束
    g_is_ramping = false;
    // 将目标速度设置为最终值，确保精确
    MotorA_PID.Speed_PID.Target = g_final_target_rpm[0];
    MotorB_PID.Speed_PID.Target = g_final_target_rpm[1];
    MotorC_PID.Speed_PID.Target = g_final_target_rpm[2];
    MotorD_PID.Speed_PID.Target = g_final_target_rpm[3];
  }
  else
  {
    // 爬升正在进行中
    // 3. 【核心修正】计算S型曲线进度，而不是线性进度

    // 3a. 首先计算线性进度 (0.0 到 1.0)
    float linear_progress = (float)elapsed_time / (float)RAMP_UP_DURATION_MS;

    // 3b. 将线性进度通过 "Ease-In-Out Sine" 函数映射为S型曲线进度
    float curve_progress = 0.5f * (1.0f - cosf(linear_progress * PI));

    // 4. 根据【S型曲线进度】线性计算每个轮子当前的瞬时目标速度
    //    这里的 g_final_target_rpm 相当于曲线的“高度”
    g_current_ramp_rpm[0] = g_final_target_rpm[0] * curve_progress-5;
    g_current_ramp_rpm[1] = g_final_target_rpm[1] * curve_progress-5;
    g_current_ramp_rpm[2] = g_final_target_rpm[2] * curve_progress-5;
    g_current_ramp_rpm[3] = g_final_target_rpm[3] * curve_progress-5;

    // 5. 将计算出的瞬时目标速度应用到PID控制器
    MotorA_PID.Speed_PID.Target = g_current_ramp_rpm[0];
    MotorB_PID.Speed_PID.Target = g_current_ramp_rpm[1];
    MotorC_PID.Speed_PID.Target = g_current_ramp_rpm[2];
    MotorD_PID.Speed_PID.Target = g_current_ramp_rpm[3];
  }
}

/**
 * @brief [内部函数] 更新斜率控制状态
 * @note  应该在PID更新循环中被调用，用于平滑地改变运行中的目标速度。
 */
static void Motor_PID_Slew_Update(void)
{
  if (!g_is_slewing)
  {
    return;
  }

  uint32_t elapsed_time = HAL_GetTick() - g_slew_start_tick;

  if (elapsed_time >= SLEW_DURATION_MS)
  {
    // 斜率控制完成
    g_is_slewing = false;
    Motor_PID_Set_Speed(&MotorA_PID, g_slew_final_target_rpm[0]);
    Motor_PID_Set_Speed(&MotorB_PID, g_slew_final_target_rpm[1]);
    Motor_PID_Set_Speed(&MotorC_PID, g_slew_final_target_rpm[2]);
    Motor_PID_Set_Speed(&MotorD_PID, g_slew_final_target_rpm[3]);
  }
  else
  {
    // 正在进行斜率控制，计算当前时刻的目标速度
    float progress = (float)elapsed_time / SLEW_DURATION_MS;
    float slew_rpm[4];
    for (int i = 0; i < 4; i++)
    {
      slew_rpm[i] = g_slew_initial_rpm[i] + (g_slew_final_target_rpm[i] - g_slew_initial_rpm[i]) * progress;
    }
    // 直接更新PID目标值，绕过会触发新斜率控制的Set_Speed
    MotorA_PID.Speed_PID.Target = slew_rpm[0];
    MotorB_PID.Speed_PID.Target = slew_rpm[1];
    MotorC_PID.Speed_PID.Target = slew_rpm[2];
    MotorD_PID.Speed_PID.Target = slew_rpm[3];
  }
}

static void Motor_PID_Position_Ramp_Update(void)
{
  if (!g_is_position_ramping)
  {
    return;
  }

  uint32_t elapsed_time = HAL_GetTick() - g_position_ramp_start_tick;

  if (elapsed_time >= RAMP_UP_DURATION_MS)
  {
    g_is_position_ramping = false;

    for (int i = 0; i < 4; ++i)
    {
      g_position_current_speed_limit[i] = g_position_final_speed_limit[i];
    }
  }
  else
  {
    float linear_progress = (float)elapsed_time / (float)RAMP_UP_DURATION_MS;
    float curve_progress = 0.5f * (1.0f - cosf(linear_progress * PI));

    for (int i = 0; i < 4; ++i)
    {
      g_position_current_speed_limit[i] =
          g_position_final_speed_limit[i] * curve_progress;
    }
  }
}
/**
 * @brief 初始化所有电机的PID控制器 (已重构)
 */
void Motor_PID_Init_All(void)
{
  // --- 基准PID参数 (使用宏定义) ---
  PID_Controller_t pos_pid_base = {.Kp = POS_PID_KP_BASE,
                                   .Ki = POS_PID_KI_BASE,
                                   .Kd = POS_PID_KD_BASE,
                                   .Integral_Max = POS_PID_IMAX_BASE,
                                   .Output_Max = POS_PID_OMAX_BASE};

  PID_Controller_t speed_pid_base = {.Kp = SPEED_PID_KP_BASE,
                                     .Ki = SPEED_PID_KI_BASE,
                                     .Kd = SPEED_PID_KD_BASE,
                                     .Integral_Max = SPEED_PID_IMAX_BASE,
                                     .Output_Max = SPEED_PID_OMAX_BASE};

  // --- R1 (MotorA): 跑得最少，需要大幅增强响应 ---
  PID_Controller_t pos_pid_R1 = pos_pid_base;
  pos_pid_R1.Kp = POS_PID_KP_BASE * R1_POS_PID_KP_FACTOR;
  pos_pid_R1.Ki = POS_PID_KI_BASE * R1_POS_PID_KI_FACTOR;
  pos_pid_R1.Kd = POS_PID_KD_BASE * R1_POS_PID_KD_FACTOR;
  pos_pid_R1.Output_Max = POS_PID_OMAX_BASE * R1_POS_PID_OMAX_FACTOR;

  PID_Controller_t speed_pid_R1 = speed_pid_base;
  speed_pid_R1.Kp = SPEED_PID_KP_BASE * R1_SPEED_PID_KP_FACTOR;
  speed_pid_R1.Ki = SPEED_PID_KI_BASE * R1_SPEED_PID_KI_FACTOR;
  speed_pid_R1.Kd = SPEED_PID_KD_BASE * R1_SPEED_PID_KD_FACTOR;

  MotorA_PID = (Motor_PID_t){.Encoder = &MotorA_Encoder,
                             .Position_PID = pos_pid_R1,
                             .Speed_PID = speed_pid_R1,
                             .Mode = PID_MODE_DISABLE};

  // --- L1 (MotorB): 跑得太多，降低响应 ---
  PID_Controller_t pos_pid_L1 = pos_pid_base;
  pos_pid_L1.Kp = POS_PID_KP_BASE * L1_POS_PID_KP_FACTOR;
  pos_pid_L1.Ki = POS_PID_KI_BASE * L1_POS_PID_KI_FACTOR;
  pos_pid_L1.Kd = POS_PID_KD_BASE * L1_POS_PID_KD_FACTOR;
  pos_pid_L1.Output_Max = POS_PID_OMAX_BASE * L1_POS_PID_OMAX_FACTOR;

  PID_Controller_t speed_pid_L1 = speed_pid_base;
  speed_pid_L1.Kp = SPEED_PID_KP_BASE * L1_SPEED_PID_KP_FACTOR;
  speed_pid_L1.Ki = SPEED_PID_KI_BASE * L1_SPEED_PID_KI_FACTOR;
  speed_pid_L1.Kd = SPEED_PID_KD_BASE * L1_SPEED_PID_KD_FACTOR;

  MotorB_PID = (Motor_PID_t){.Encoder = &MotorB_Encoder,
                             .Position_PID = pos_pid_L1,
                             .Speed_PID = speed_pid_L1,
                             .Mode = PID_MODE_DISABLE};

  // --- L2 (MotorC): 稍微偏少，适当增强响应 ---
  PID_Controller_t pos_pid_L2 = pos_pid_base;
  pos_pid_L2.Kp = POS_PID_KP_BASE * L2_POS_PID_KP_FACTOR;
  pos_pid_L2.Ki = POS_PID_KI_BASE * L2_POS_PID_KI_FACTOR;
  pos_pid_L2.Kd = POS_PID_KD_BASE * L2_POS_PID_KD_FACTOR;
  pos_pid_L2.Output_Max = POS_PID_OMAX_BASE * L2_POS_PID_OMAX_FACTOR;

  PID_Controller_t speed_pid_L2 = speed_pid_base;
  speed_pid_L2.Kp = SPEED_PID_KP_BASE * L2_SPEED_PID_KP_FACTOR;
  speed_pid_L2.Ki = SPEED_PID_KI_BASE * L2_SPEED_PID_KI_FACTOR;
  speed_pid_L2.Kd = SPEED_PID_KD_BASE * L2_SPEED_PID_KD_FACTOR;

  MotorC_PID = (Motor_PID_t){.Encoder = &MotorC_Encoder,
                             .Position_PID = pos_pid_L2,
                             .Speed_PID = speed_pid_L2,
                             .Mode = PID_MODE_DISABLE};

  // --- R2 (MotorD): 跑得最少且不稳定，增强响应 ---
  PID_Controller_t pos_pid_R2 = pos_pid_base;
  pos_pid_R2.Kp = POS_PID_KP_BASE * R2_POS_PID_KP_FACTOR;
  pos_pid_R2.Ki = POS_PID_KI_BASE * R2_POS_PID_KI_FACTOR;
  pos_pid_R2.Kd = POS_PID_KD_BASE * R2_POS_PID_KD_FACTOR;
  pos_pid_R2.Output_Max = POS_PID_OMAX_BASE * R2_POS_PID_OMAX_FACTOR;

  PID_Controller_t speed_pid_R2 = speed_pid_base;
  speed_pid_R2.Kp = SPEED_PID_KP_BASE * R2_SPEED_PID_KP_FACTOR;
  speed_pid_R2.Ki = SPEED_PID_KI_BASE * R2_SPEED_PID_KI_FACTOR;
  speed_pid_R2.Kd = SPEED_PID_KD_BASE * R2_SPEED_PID_KD_FACTOR;

  MotorD_PID = (Motor_PID_t){.Encoder = &MotorD_Encoder,
                             .Position_PID = pos_pid_R2,
                             .Speed_PID = speed_pid_R2,
                             .Mode = PID_MODE_DISABLE};

  g_is_pid_active = 0;
  g_is_position_ramping = false;
  for (int i = 0; i < 4; ++i)
  {
    g_position_current_speed_limit[i] = 0.0f;
    g_position_final_speed_limit[i] = 0.0f;
  }
}

/**
 * @brief 更新所有电机的PID计算并施加控制
 */
void Motor_PID_Update_All(void)
{
  // 这是核心的“门卫”，如果PID未激活，则直接返回，不执行任何计算
  if (!g_is_pid_active)
  {
    return;
  }

  // 优先处理状态更新：加速、斜率控制或减速
  Motor_PID_Ramp_Update();
  Motor_PID_Slew_Update(); // <<< NEW: 更新斜率控制
  Motor_PID_Position_Ramp_Update();
  Motor_PID_Decel_Update(); // 更新减速状态

  // 【核心修正】无论是否在减速，只要PID激活，就必须执行PID计算
  // 减速过程本身就是通过不断更新Target来实现的，需要PID持续工作
  Motor_PID_Update_Single(&MotorA_PID, MotorA_SetSpeed);
  Motor_PID_Update_Single(&MotorB_PID, MotorB_SetSpeed);
  Motor_PID_Update_Single(&MotorC_PID, MotorC_SetSpeed);
  Motor_PID_Update_Single(&MotorD_PID, MotorD_SetSpeed);

  // printf("A:T%.0f,M%.0f,P%.0f|B:T%.0f,M%.0f,P%.0f|C:T%.0f,M%.0f,P%.0f|D:T%.0f,M%.0f,P%.0f\r\n",
  //        MotorA_PID.Speed_PID.Target, MotorA_PID.Speed_PID.Measured, MotorA_PID.Speed_PID.Output,
  //        MotorB_PID.Speed_PID.Target, MotorB_PID.Speed_PID.Measured, MotorB_PID.Speed_PID.Output,
  //        MotorC_PID.Speed_PID.Target, MotorC_PID.Speed_PID.Measured, MotorC_PID.Speed_PID.Output,
  //        MotorD_PID.Speed_PID.Target, MotorD_PID.Speed_PID.Measured, MotorD_PID.Speed_PID.Output);

  if (g_position_auto_stop_enabled && !g_is_decelerating)
  {
    if (Motor_PID_All_Position_Errors_In_Deadband(POSITION_PID_DEADBAND_SMOOTH))
    {
      Car_Stop_Smooth_PID();
      g_position_auto_stop_enabled = false;
    }
  }

  bool has_position = Motor_PID_Has_Position_Control();

  if (!g_is_decelerating && has_position && g_is_pid_active)
  {
    if (Motor_PID_All_Position_Errors_In_Deadband(POSITION_PID_DEADBAND_FINAL))
    {
      if (!g_position_deadband_timing)
      {
        g_position_deadband_timing = true;
        g_position_deadband_entry_tick = HAL_GetTick();
      }
      else if ((HAL_GetTick() - g_position_deadband_entry_tick) >= POSITION_DEADBAND_HOLD_MS)
      {
        g_position_stop_pending = true;
      }
    }
    else
    {
      g_position_deadband_timing = false;
    }
  }
  else
  {
    g_position_deadband_timing = false;
  }

  if (g_position_stop_pending)
  {
    g_position_stop_pending = false;
    g_position_deadband_timing = false;
    Car_Stop_Request();
    return;
  }
}

/**
 * @brief 为单个电机设置目标速度
 */
void Motor_PID_Set_Speed(Motor_PID_t *motor_pid, float target_rpm)
{
  motor_pid->Mode = PID_MODE_SPEED;
  motor_pid->Speed_PID.Target = target_rpm;
}
void Motor_PID_Set_Position(Motor_PID_t *motor_pid, int64_t target_pos)
{
  motor_pid->Mode = PID_MODE_POSITION;
  motor_pid->Position_PID.Target = target_pos;
  // 重置积分项，防止旧的积分影响新的位置控制
  motor_pid->Position_PID.Integral = 0;
  motor_pid->Speed_PID.Integral = 0;
}

/* ============================================================================
 */
/* ===============================  停车控制  ===============================
 */
/* ============================================================================
 */

/**
 * @brief 急停函数 - 通过立即停止PWM输出来防止滑行和漂移
 * @note  该函数直接操作底层电机驱动，确保所有电机同时停止，避免不均衡的制动力。
 */
void Car_Stop_PID(void)
{
  // 1. 立即停止PID循环，防止控制器发出任何指令
  g_is_pid_active = 0;
  g_is_ramping = false; // <<< NEW: 紧急停止时，必须终止任何正在进行的爬升
  g_is_position_ramping = false;
  g_position_auto_stop_enabled = false;
  for (int i = 0; i < 4; ++i)
  {
    g_position_current_speed_limit[i] = 0.0f;
    g_position_final_speed_limit[i] = 0.0f;
  }

  // 2. 强制所有电机停转 (直接操作底层，确保立即生效)
  MotorA_SetSpeed(0);
  MotorB_SetSpeed(0);
  MotorC_SetSpeed(0);
  MotorD_SetSpeed(0);

  // 3. 给予一个短暂的延时，确保电机物理上已经停止转动
  HAL_Delay(50);

  // 4. 再次确保电机停转，作为安全措施
  MotorA_SetSpeed(0);
  MotorB_SetSpeed(0);
  MotorC_SetSpeed(0);
  MotorD_SetSpeed(0);

  // 5. 清零所有PID控制器的内部状态，为下一次运动做准备
  MotorA_PID.Speed_PID.Integral = 0;
  MotorB_PID.Speed_PID.Integral = 0;
  MotorC_PID.Speed_PID.Integral = 0;
  MotorD_PID.Speed_PID.Integral = 0;

  MotorA_PID.Position_PID.Integral = 0;
  MotorB_PID.Position_PID.Integral = 0;
  MotorC_PID.Position_PID.Integral = 0;
  MotorD_PID.Position_PID.Integral = 0;
}

void Car_Process_Stop_Request(void)
{
  if (!g_request_stop)
  {
    return;
  }

  g_request_stop = false;
  Car_Stop_PID();
}

/**
 * @brief [新增核心函数] 平滑刹车函数 - 通过PID主动控制减速至停止
 * @note  这是推荐的日常停车方式，可以防止漂移。
 */
void Car_Stop_Smooth_PID(void)
{
  // 如果PID未激活或已经在刹车，则不执行任何操作
  if (!g_is_pid_active || g_is_decelerating)
  {
    return;
  }

  // 1. 立即停止任何正在进行的“加速”过程
  g_is_ramping = false;
  g_is_position_ramping = false;
  g_position_auto_stop_enabled = false;

  // 2. 记录调用刹车时，每个轮子当前的目标速度作为减速的【起点】
  float target_a = MotorA_PID.Speed_PID.Target;
  float target_b = MotorB_PID.Speed_PID.Target;
  float target_c = MotorC_PID.Speed_PID.Target;
  float target_d = MotorD_PID.Speed_PID.Target;

  if (fabsf(target_a) < 1.0f)
  {
    target_a = MotorA_PID.Speed_PID.Measured;
  }
  if (fabsf(target_b) < 1.0f)
  {
    target_b = MotorB_PID.Speed_PID.Measured;
  }
  if (fabsf(target_c) < 1.0f)
  {
    target_c = MotorC_PID.Speed_PID.Measured;
  }
  if (fabsf(target_d) < 1.0f)
  {
    target_d = MotorD_PID.Speed_PID.Measured;
  }

  g_start_decel_rpm[0] = target_a;
  g_start_decel_rpm[1] = target_b;
  g_start_decel_rpm[2] = target_c;
  g_start_decel_rpm[3] = target_d;

  // 3. 记录减速开始的时间
  g_decel_start_tick = HAL_GetTick();

  // 4. 激活减速标志位，PID循环将接管刹车过程
  g_is_decelerating = true;

  // 【注意】这里绝对不能关闭 g_is_pid_active，要让PID继续工作！
}
/**
 * @brief [新增内部函数] 更新速度平滑减速状态
 * @note  应该在PID更新循环中被调用，实现S型曲线刹车。
 */
void Motor_PID_Decel_Update(void)
{
  if (!g_is_decelerating)
  {
    return; // 如果没有在减速，直接返回
  }

  // 1. 计算从减速开始到现在所经过的时间
  uint32_t elapsed_time = HAL_GetTick() - g_decel_start_tick;

  // 2. 判断减速是否已经完成
  if (elapsed_time >= RAMP_DOWN_DURATION_MS)
  {
    // 减速结束
    g_is_decelerating = false;

    // 确保最终速度目标归零，便于保持姿态
    MotorA_PID.Speed_PID.Target = 0.0f;
    MotorB_PID.Speed_PID.Target = 0.0f;
    MotorC_PID.Speed_PID.Target = 0.0f;
    MotorD_PID.Speed_PID.Target = 0.0f;

    // 【关键】位置模式需要继续保持锁定，不可直接关闭PID
    if (Motor_PID_Has_Position_Control())
    {
      g_is_pid_active = 1;
    }
    else
    {
      g_is_pid_active = 0;
    }

    // 为确保彻底停止，可以调用原始的急停函数来清零PWM和积分
    // Car_Stop_PID();
  }
  else
  {
    // 减速正在进行中
    // 3. 计算S型曲线进度
    float linear_progress = (float)elapsed_time / (float)RAMP_DOWN_DURATION_MS;

    float curve_progress = 0.5f * (1.0f - cosf(linear_progress * PI));

    // 4. 根据S型曲线进度，计算从【起始速度】到【0】的瞬时目标速度
    //    公式为: 起始速度 * (1.0 - 曲线进度)
    g_current_ramp_rpm[0] = g_start_decel_rpm[0] * (1.0f - curve_progress);
    g_current_ramp_rpm[1] = g_start_decel_rpm[1] * (1.0f - curve_progress);
    g_current_ramp_rpm[2] = g_start_decel_rpm[2] * (1.0f - curve_progress);
    g_current_ramp_rpm[3] = g_start_decel_rpm[3] * (1.0f - curve_progress);

    // 5. 将计算出的瞬时目标速度应用到PID控制器
    MotorA_PID.Speed_PID.Target = g_current_ramp_rpm[0];
    MotorB_PID.Speed_PID.Target = g_current_ramp_rpm[1];
    MotorC_PID.Speed_PID.Target = g_current_ramp_rpm[2];
    MotorD_PID.Speed_PID.Target = g_current_ramp_rpm[3];
  }
}

/**
 * @brief 增强版急刹车函数 - 专门用于后续步骤
 * @note  比普通Car_Stop_PID提供更强的制动力，确保稳定停车
 */
void Car_Stop_PID_Enhanced(void)
{
  // 0. 紧急停止时，必须终止任何正在进行的爬升
  g_is_ramping = false;
  g_position_auto_stop_enabled = false;

  // 1. 先判断当前运动方向，保存当前速度信息
  float current_speed_A = MotorA_PID.Speed_PID.Target;
  float current_speed_B = MotorB_PID.Speed_PID.Target;
  float current_speed_C = MotorC_PID.Speed_PID.Target;
  float current_speed_D = MotorD_PID.Speed_PID.Target;

  // 2. 如果PID处于活动状态，执行增强主动制动
  if (g_is_pid_active)
  {
    // 增强版制动强度，提供更强的急刹车效果
    float brake_R1 = 0.4f; // R1增强制动力，提升稳定性
    float brake_L1 = 0.5f; // L1增强制动力，更强的急刹效果
    float brake_L2 = 0.7f; // L2进一步增强，主要制动轮
    float brake_R2 = 0.5f; // R2增强制动力，与L1保持一致

    // 反向制动，每个轮子独立调整
    Motor_PID_Set_Speed(&MotorA_PID,
                        -current_speed_A * brake_R1 * RIGHT_MOTOR_POWER_FACTOR);
    Motor_PID_Set_Speed(&MotorB_PID, -current_speed_B * brake_L1);
    Motor_PID_Set_Speed(&MotorC_PID, -current_speed_C * brake_L2);
    Motor_PID_Set_Speed(&MotorD_PID,
                        -current_speed_D * brake_R2 * RIGHT_MOTOR_POWER_FACTOR);

    // 增强版制动时间
    HAL_Delay(150);
  }

  // 3. 立即停止PID循环
  g_is_pid_active = 0;

  // 4. 安全停止：直接PWM归零，避免短路风险
  MotorA_SetSpeed(0);
  MotorB_SetSpeed(0);
  MotorC_SetSpeed(0);
  MotorD_SetSpeed(0);

  // 5. 增强版稳定时间
  HAL_Delay(80);

  // 6. 最终停止：PWM归零
  MotorA_SetSpeed(0);
  MotorB_SetSpeed(0);
  MotorC_SetSpeed(0);
  MotorD_SetSpeed(0);

  // 7. 清零PID状态
  MotorA_PID.Speed_PID.Integral = 0;
  MotorB_PID.Speed_PID.Integral = 0;
  MotorC_PID.Speed_PID.Integral = 0;
  MotorD_PID.Speed_PID.Integral = 0;
}

/**
 * @brief [新增核心函数] PID控制麦克纳姆轮整车运动
 * @note  这是所有麦克纳姆轮运动控制的基础函数。
 * @param rpm_a 电机A (右前) 的目标速度 (RPM)
 * @param rpm_b 电机B (左前) 的目标速度 (RPM)
 * @param rpm_c 电机C (左后) 的目标速度 (RPM)
 * @param rpm_d 电机D (右后) 的目标速度 (RPM)
 */
void Car_Mecanum_Move_PID(float rpm_a, float rpm_b, float rpm_c, float rpm_d)
{
  // 关键判断：只有在小车当前是停止状态时，才启用平滑启动
  if (!g_is_pid_active && !g_is_ramping)
  {
    // --- 平滑启动流程 ---
    // 1. 保存最终的目标速度
    g_final_target_rpm[0] = rpm_a * 1.01f; // 在此处对R1(A)应用补偿
    g_final_target_rpm[1] = rpm_b;
    g_final_target_rpm[2] = rpm_c;
    g_final_target_rpm[3] = rpm_d;

    // 2. 记录爬升开始时间
    g_ramp_start_tick = HAL_GetTick();

    // 3. 设置初始速度为0
    Motor_PID_Set_Speed(&MotorA_PID, 0.0f);
    Motor_PID_Set_Speed(&MotorB_PID, 0.0f);
    Motor_PID_Set_Speed(&MotorC_PID, 0.0f);
    Motor_PID_Set_Speed(&MotorD_PID, 0.0f);

    // 4. 激活爬升标志位
    g_is_ramping = true;
  }
  else
  {
    // --- 非平滑启动（已经在运动中，直接改变目标速度）---
    g_is_ramping = false; // 如果之前在爬升，立即停止
    Motor_PID_Set_Speed(&MotorA_PID, rpm_a);
    Motor_PID_Set_Speed(&MotorB_PID, rpm_b);
    Motor_PID_Set_Speed(&MotorC_PID, rpm_c);
    Motor_PID_Set_Speed(&MotorD_PID, rpm_d);
  }

  // 激活PID循环 (无论哪种情况都需要)
  g_is_pid_active = 1;
  g_is_decelerating = false;
}

/* ============================================================================
 */
/* ===============================  清0  ===============================
 */
/* ============================================================================
 */

/**
 * @brief  复位所有电机和PID的状态，确保起步一致性
 * @note   此函数是解决电机起步不统一问题的关键。
 *         它清除了所有可能影响下一次运动的残留状态。
 */
void Car_Reset_And_Sync_State(void)
{
  // 1. 暂停PID控制循环，防止在复位过程中发出任何指令
  g_is_pid_active = 0;
  g_position_auto_stop_enabled = false;
  g_request_stop = false; // 清除未处理的停车请求，避免影响后续运动

  // 2. 强制所有电机停转 (直接操作底层，确保立即生效)
  MotorA_SetSpeed(0);
  MotorB_SetSpeed(0);
  MotorC_SetSpeed(0);
  MotorD_SetSpeed(0);

  // 3. 清零所有编码器的累计脉冲数和速度
  //    这是统一"起跑线"的关键
  MotorA_Encoder.Total_counter = 0;
  MotorB_Encoder.Total_counter = 0;
  MotorC_Encoder.Total_counter = 0;
  MotorD_Encoder.Total_counter = 0;

  MotorA_Encoder.Velocity = 0;
  MotorB_Encoder.Velocity = 0;
  MotorC_Encoder.Velocity = 0;
  MotorD_Encoder.Velocity = 0;

  // 4. 重置所有PID控制器的内部状态，特别是积分项和上次误差
  //    这是防止"积分累积"导致起步冲击的核心步骤
  PID_Controller_t *pids[] = {&MotorA_PID.Speed_PID, &MotorA_PID.Position_PID,
                              &MotorB_PID.Speed_PID, &MotorB_PID.Position_PID,
                              &MotorC_PID.Speed_PID, &MotorC_PID.Position_PID,
                              &MotorD_PID.Speed_PID, &MotorD_PID.Position_PID};

  for (int i = 0; i < 8; i++)
  {
    pids[i]->Integral = 0.0f;
    pids[i]->Last_Error = 0.0f;
    pids[i]->Output = 0.0f;
  }

  // 5. 给予一个短暂的延时，确保所有硬件操作完成
  HAL_Delay(50);
}

/* ============================================================================
 */
/* ===============================  速度环控制  ===============================
 */
/* ============================================================================
 */

/**
 * @brief PID控制小车前进
 * @param rpm 目标速度 (RPM, 应为正值)
 */
void Car_Go_PID(float rpm)
{
  float speed = fabsf(rpm);
  Car_Mecanum_Move_PID(speed, speed, speed, speed);
}

/**
 * @brief PID控制小车后退
 * @param rpm 目标速度 (RPM, 应为正值)
 */
void Car_Back_PID(float rpm)
{
  float speed = -fabsf(rpm);
  Car_Mecanum_Move_PID(speed, speed, speed, speed);
}

/**
 * @brief PID控制小车原地左转
 * @param rpm 转动速度 (RPM, 应为正值)
 */
void Car_TurnLeft_PID(float rpm)
{
  float speed = fabsf(rpm);
  // 左轮后退，右轮前进
  Car_Mecanum_Move_PID(speed, -speed, -speed, speed);
}

/**
 * @brief PID控制小车原地右转
 * @param rpm 转动速度 (RPM, 应为正值)
 */
void Car_TurnRight_PID(float rpm)
{
  float speed = fabsf(rpm);
  // 左轮前进，右轮后退
  Car_Mecanum_Move_PID(-speed, speed, speed, -speed);
}

/**
 * @brief PID控制小车向左平移 (已优化)
 * @param rpm 目标速度 (RPM, 应为正值)
 */
void Car_Strafe_Left_PID(float rpm)
{
  float speed = fabsf(rpm);
  // 针对轮子差异的优化补偿
  float r1_speed = speed * 1.05f;  // R1补偿
  float l1_speed = -speed * 1.00f; // L1补偿
  float l2_speed = speed * 1.00f;  // L2补偿
  float r2_speed = -speed * 1.00f; // R2补偿

  Car_Mecanum_Move_PID(r1_speed, l1_speed, l2_speed, r2_speed);
}

/**
 * @brief PID控制小车向右平移 (已优化)
 * @param rpm 目标速度 (RPM, 应为正值)
 */
void Car_Strafe_Right_PID(float rpm)
{
  float speed = fabsf(rpm);
  // 针对轮子差异的优化补偿
  float r1_speed = -speed * 1.05f; // R1补偿
  float l1_speed = speed * 1.000f; // L1补偿
  float l2_speed = -speed * 1.00f; // L2补偿
  float r2_speed = speed * 1.00f;  // R2补偿

  Car_Mecanum_Move_PID(r1_speed, l1_speed, l2_speed, r2_speed);
}

/* ============================================================================
 */
/* ===============================  位置环控制  ===============================
 */
/* ============================================================================
 */

/**
 * @brief 精确直线行走位置控制
 * @param pulses 目标行走脉冲数（正数前进，负数后退）
 * @note  专为直线行走优化，考虑各轮差异补偿
 */
void Car_Go_Straight_Pulses_PID(int32_t pulses)
{
  bool should_use_ramp = (!g_is_pid_active && !g_is_position_ramping);
  int64_t target_R1, target_L1, target_L2, target_R2; // Declare targets here

  if (should_use_ramp)
  {
    g_is_position_ramping = true;
    g_position_ramp_start_tick = HAL_GetTick();

    g_position_final_speed_limit[0] = POSITION_MODE_MAX_SPEED_RPM;
    g_position_final_speed_limit[1] = POSITION_MODE_MAX_SPEED_RPM;
    g_position_final_speed_limit[2] = POSITION_MODE_MAX_SPEED_RPM;
    g_position_final_speed_limit[3] = POSITION_MODE_MAX_SPEED_RPM;

    for (int i = 0; i < 4; ++i)
    {
      g_position_current_speed_limit[i] = 0.0f;
    }
  }
  else
  {
    g_is_position_ramping = false;
  }

  // 1. 激活PID控制
  g_is_pid_active = 1;
  g_position_auto_stop_enabled = true;

  // 2. 记录当前各轮位置
  int64_t start_A = MotorA_Encoder.Total_counter; // R1 右前
  int64_t start_B = MotorB_Encoder.Total_counter; // L1 左前
  int64_t start_C = MotorC_Encoder.Total_counter; // L2 左后
  int64_t start_D = MotorD_Encoder.Total_counter; // R2 右后

  // 3. 根据补偿因子设定目标位置
  target_L1 = start_B + (int64_t)(pulses * L1_TRAVEL_FACTOR);
  target_L2 = start_C + (int64_t)(pulses * L2_TRAVEL_FACTOR);
  target_R1 = start_A + (int64_t)(pulses * R1_TRAVEL_FACTOR);
  target_R2 = start_D + (int64_t)(pulses * R2_TRAVEL_FACTOR);

  // 4. 设置各轮目标位置
  Motor_PID_Set_Position(&MotorA_PID, target_R1); // R1
  Motor_PID_Set_Position(&MotorB_PID, target_L1); // L1
  Motor_PID_Set_Position(&MotorC_PID, target_L2); // L2
  Motor_PID_Set_Position(&MotorD_PID, target_R2); // R2
}

/**
 * @brief [精确版] PID控制小车后退指定脉冲数
 * @param pulses 要后退的脉冲数 (应为正值)
 */
void Car_Back_Pulses_PID(uint32_t pulses)
{
  Car_Go_Straight_Pulses_PID(-(int32_t)pulses);
}

/**
 * @brief PID控制小车向左平移指定脉冲数
 * @param pulses 要平移的脉冲数 (应为正值)
 */
void Car_Strafe_Left_Pulses_PID(uint32_t pulses)
{
  const float strafe_max_rpm = STRAFE_POSITION_MAX_SPEED_RPM;
  bool should_use_ramp = (!g_is_pid_active && !g_is_position_ramping);

  if (should_use_ramp)
  {
    g_is_position_ramping = true;
    g_position_ramp_start_tick = HAL_GetTick();

    g_position_final_speed_limit[0] = strafe_max_rpm;
    g_position_final_speed_limit[1] = strafe_max_rpm;
    g_position_final_speed_limit[2] = strafe_max_rpm;
    g_position_final_speed_limit[3] = strafe_max_rpm;

    for (int i = 0; i < 4; ++i)
    {
      g_position_current_speed_limit[i] = 0.0f;
    }
  }
  else
  {
    g_is_position_ramping = false;

    g_position_final_speed_limit[0] = strafe_max_rpm;
    g_position_final_speed_limit[1] = strafe_max_rpm;
    g_position_final_speed_limit[2] = strafe_max_rpm;
    g_position_final_speed_limit[3] = strafe_max_rpm;
  }

  // 1. 激活PID控制
  g_is_pid_active = 1;
  g_position_auto_stop_enabled = true;

  // 2. 记录当前各轮位置
  int64_t start_R1 = MotorA_Encoder.Total_counter;
  int64_t start_L1 = MotorB_Encoder.Total_counter;
  int64_t start_L2 = MotorC_Encoder.Total_counter;
  int64_t start_R2 = MotorD_Encoder.Total_counter;

  // 3. 根据补偿因子设定目标位置 (左移: R1+, L1-, L2+, R2-)
  int64_t target_R1 = start_R1 + (int64_t)(pulses * R1_TRAVEL_FACTOR);
  int64_t target_L1 = start_L1 - (int64_t)(pulses * L1_TRAVEL_FACTOR);
  int64_t target_L2 = start_L2 + (int64_t)(pulses * L2_TRAVEL_FACTOR);
  int64_t target_R2 = start_R2 - (int64_t)(pulses * R2_TRAVEL_FACTOR);

  // 4. 设置各轮目标位置
  Motor_PID_Set_Position(&MotorA_PID, target_R1);
  Motor_PID_Set_Position(&MotorB_PID, target_L1);
  Motor_PID_Set_Position(&MotorC_PID, target_L2);
  Motor_PID_Set_Position(&MotorD_PID, target_R2);
}

/**
 * @brief PID控制小车向右平移指定脉冲数
 * @param pulses 要平移的脉冲数 (应为正值)
 */
void Car_Strafe_Right_Pulses_PID(uint32_t pulses)
{
  const float strafe_max_rpm = STRAFE_POSITION_MAX_SPEED_RPM;
  bool should_use_ramp = (!g_is_pid_active && !g_is_position_ramping);

  if (should_use_ramp)
  {
    g_is_position_ramping = true;
    g_position_ramp_start_tick = HAL_GetTick();

    g_position_final_speed_limit[0] = strafe_max_rpm;
    g_position_final_speed_limit[1] = strafe_max_rpm;
    g_position_final_speed_limit[2] = strafe_max_rpm;
    g_position_final_speed_limit[3] = strafe_max_rpm;

    for (int i = 0; i < 4; ++i)
    {
      g_position_current_speed_limit[i] = 0.0f;
    }
  }
  else
  {
    g_is_position_ramping = false;

    g_position_final_speed_limit[0] = strafe_max_rpm;
    g_position_final_speed_limit[1] = strafe_max_rpm;
    g_position_final_speed_limit[2] = strafe_max_rpm;
    g_position_final_speed_limit[3] = strafe_max_rpm;
  }

  // 1. 激活PID控制
  g_is_pid_active = 1;
  g_position_auto_stop_enabled = true;

  // 2. 记录当前各轮位置
  int64_t start_R1 = MotorA_Encoder.Total_counter;
  int64_t start_L1 = MotorB_Encoder.Total_counter;
  int64_t start_L2 = MotorC_Encoder.Total_counter;
  int64_t start_R2 = MotorD_Encoder.Total_counter;

  // 3. 根据补偿因子设定目标位置 (右移: R1-, L1+, L2-, R2+)
  int64_t target_R1 = start_R1 - (int64_t)(pulses * R1_TRAVEL_FACTOR);
  int64_t target_L1 = start_L1 + (int64_t)(pulses * L1_TRAVEL_FACTOR);
  int64_t target_L2 = start_L2 - (int64_t)(pulses * L2_TRAVEL_FACTOR);
  int64_t target_R2 = start_R2 + (int64_t)(pulses * R2_TRAVEL_FACTOR);

  // 4. 设置各轮目标位置
  Motor_PID_Set_Position(&MotorA_PID, target_R1);
  Motor_PID_Set_Position(&MotorB_PID, target_L1);
  Motor_PID_Set_Position(&MotorC_PID, target_L2);
  Motor_PID_Set_Position(&MotorD_PID, target_R2);
}

//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void Car_LEFT90(void)
{
  Car_Reset_And_Sync_State();
  Car_TurnLeft_PID(61.1f);
  HAL_Delay(401);
  Car_Stop_Smooth_PID();

  while (g_is_decelerating)
  {
    HAL_Delay(10);
  }
  Car_Stop_PID();
}
void Car_LEFT(float time)
{
  Car_Reset_And_Sync_State();
  Car_TurnLeft_PID(61.1f);
  HAL_Delay(time);
  Car_Stop_Smooth_PID();

  while (g_is_decelerating)
  {
    HAL_Delay(10);
  }
  Car_Stop_PID();
}
void Car_FORWARD(float rpm, uint32_t delay)
{
  Car_Reset_And_Sync_State();
  HAL_Delay(100);
  Car_Go_PID(rpm);
  HAL_Delay(delay);
  Car_Stop_Smooth_PID();

  while (g_is_decelerating)
  {
    HAL_Delay(10);
  }
  Car_Stop_PID();
}
void Car_BACKWARD(float rpm, uint32_t delay)
{
  Car_Reset_And_Sync_State();
  HAL_Delay(100);
  Car_Back_PID(rpm);
  HAL_Delay(delay);
  Car_Stop_Smooth_PID();

  while (g_is_decelerating)
  {
    HAL_Delay(10);
  }
  Car_Stop_PID();
}

/**
 * @brief [新增] 带平滑启动和停止的左平移函数
 * @param rpm       平移的目标速度 (RPM, 应为正值)
 * @param duration_ms 平移的持续时间 (毫秒)
 */
void Car_Strafe_Left_Smooth(float rpm, uint32_t duration_ms)
{
  // 1. 复位状态，确保干净的起步
  Car_Reset_And_Sync_State();
  HAL_Delay(100); // 短暂延时确保状态稳定

  // 2. 调用现有的左平移PID函数，它会自动处理平滑启动
  Car_Strafe_Left_PID(rpm);

  // 3. 持续运动指定时间
  HAL_Delay(duration_ms);

  // 4. 调用平滑刹车函数
  Car_Stop_Smooth_PID();

  // 5. 【新增】等待平滑减速完成
  while (g_is_decelerating)
  {
    // 必须在循环中加入短暂延时，让后台的PID任务有机会执行
    HAL_Delay(10);
  }
  Car_Stop_PID();
}

/**
 * @brief [新增] 带平滑启动和停止的右平移函数
 * @param rpm         平移的目标速度 (RPM, 应为正值)
 * @param duration_ms 平移的持续时间 (毫秒)
 */
void Car_Strafe_Right_Smooth(float rpm, uint32_t duration_ms)
{
  // 1. 复位状态，确保干净的起步
  Car_Reset_And_Sync_State();
  HAL_Delay(100); // 短暂延时确保状态稳定

  // 2. 调用现有的右平移PID函数，它会自动处理平滑启动
  Car_Strafe_Right_PID(rpm);

  // 3. 持续运动指定时间
  HAL_Delay(duration_ms);

  // 4. 调用平滑刹车函数
  Car_Stop_Smooth_PID();

  // 5. 等待平滑减速完成
  while (g_is_decelerating)
  {
    // 必须在循环中加入短暂延时，让后台的PID任务有机会执行
    HAL_Delay(10);
  }
  Car_Stop_PID();
}

/**
 * @brief [调试函数] 测试左右轮同步性能 - 单次运行
 * @note  用于诊断"左轮和右轮起跑不一致"问题
 */
void Car_Debug_Left_Right_Sync_Test(void)
{
  extern const SSD1306_Font_t Font_7x10;
  char buffer[32];

  // 清屏并显示测试开始信息
  ssd1306_Fill(0);
  ssd1306_SetCursor(0, 0);
  ssd1306_WriteString("Single Run Test", Font_7x10, 1);
  ssd1306_SetCursor(0, 12);
  ssd1306_WriteString("Starting...", Font_7x10, 1);
  ssd1306_UpdateScreen();
  HAL_Delay(2000);

  // --- 开始单次测试 ---

  // 显示重置信息
  ssd1306_Fill(0);
  ssd1306_SetCursor(0, 0);
  ssd1306_WriteString("Resetting...", Font_7x10, 1);
  ssd1306_UpdateScreen();

  // 1. 重置并同步状态
  Car_Reset_And_Sync_State();
  HAL_Delay(500);

  // 2. 记录初始编码器值
  int32_t start_A = MotorA_Encoder.Total_counter; // 右前 R1
  int32_t start_B = MotorB_Encoder.Total_counter; // 左前 L1
  int32_t start_C = MotorC_Encoder.Total_counter; // 左后 L2
  int32_t start_D = MotorD_Encoder.Total_counter; // 右后 R2

  // 3. 启动运动
  float test_speed = 75.0f; // RPM
  ssd1306_Fill(0);
  ssd1306_SetCursor(0, 0);
  ssd1306_WriteString("Running...", Font_7x10, 1);
  ssd1306_UpdateScreen();

  Car_Go_PID(test_speed);

  // 4. 运动1.2秒
  HAL_Delay(1200);

  // 5. 【核心修正】使用非阻塞方式等待平滑减速完成
  Car_Stop_Smooth_PID();

  // // 等待 g_is_decelerating 标志位被 Motor_PID_Decel_Update 清除
  // while(g_is_decelerating)
  // {
  //     // 必须在循环中加入短暂延时，让后台的PID任务有机会执行
  //     HAL_Delay(10);
  // }
  // // 当循环退出时，说明减速已完成

  HAL_Delay(50);

  // 6. 读取结束时的编码器值
  int32_t end_A = MotorA_Encoder.Total_counter;
  int32_t end_B = MotorB_Encoder.Total_counter;
  int32_t end_C = MotorC_Encoder.Total_counter;
  int32_t end_D = MotorD_Encoder.Total_counter;

  // 计算行程
  int32_t travel_R1 = end_A - start_A;
  int32_t travel_L1 = end_B - start_B;
  int32_t travel_L2 = end_C - start_C;
  int32_t travel_R2 = end_D - start_D;

  // --- 显示最终结果 ---
  ssd1306_Fill(0);
  ssd1306_SetCursor(0, 0);
  ssd1306_WriteString("Test Results:", Font_7x10, 1);

  ssd1306_SetCursor(0, 12);
  snprintf(buffer, sizeof(buffer), "R1:%ld L1:%ld", travel_R1, travel_L1);
  ssd1306_WriteString(buffer, Font_7x10, 1);

  ssd1306_SetCursor(0, 24);
  snprintf(buffer, sizeof(buffer), "L2:%ld R2:%ld", travel_L2, travel_R2);
  ssd1306_WriteString(buffer, Font_7x10, 1);

  // 计算左右轮组平均差异
  float left_avg = (travel_L1 + travel_L2) / 2.0f;
  float right_avg = (travel_R1 + travel_R2) / 2.0f;
  float lr_diff = left_avg - right_avg;

  ssd1306_SetCursor(0, 36);
  snprintf(buffer, sizeof(buffer), "L-R Diff:%.0f", lr_diff);
  ssd1306_WriteString(buffer, Font_7x10, 1);

  // 在末行显示最终结束标志
  ssd1306_SetCursor(0, 54);
  if (fabsf(lr_diff) > 50)
  {
    ssd1306_WriteString("NEED ADJUST-END", Font_7x10, 1);
  }
  else
  {
    ssd1306_WriteString("SYNC OK - END", Font_7x10, 1);
  }

  ssd1306_UpdateScreen();
  // 保持显示，不自动清除
}

/**
 * @brief [新增调试函数] 测试小车平滑旋转性能
 * @note  包含平滑启动和平滑停止，用于观察旋转的稳定性和精确性。
 */
void Car_Debug_Rotation_Test(void)
{
  const float test_rpm = 61.1f;         // 设置旋转速度 (RPM)
  const uint32_t run_duration_ms = 367; // 设置旋转持续时间 (毫秒)

  Car_Reset_And_Sync_State();
  Car_TurnLeft_PID(test_rpm);
  HAL_Delay(run_duration_ms);
  Car_Stop_Smooth_PID();
}

/**
 * @brief 【V3 - 已修正】全向移动控制函数，融合前进、平移和旋转.
 * @note  此版本已根据您的电机PID映射 (A=R1, B=L1, C=L2, D=R2) 进行了精确修正。
 */
void Car_Mecanum_Move_Combined_PID(float forward_speed, float strafe_speed, float turn_speed)
{
    // 麦克纳姆轮标准逆解算公式
    // L1 (左前), R1 (右前), L2 (左后), R2 (右后)
    // 注意，根据您的反馈，这里的运动方向定义如下：
    // strafe_speed: >0 右移, <0 左移
    // turn_speed:   >0 右转, <0 左转
    float rpm_l1 = forward_speed - strafe_speed - turn_speed;
    float rpm_r1 = forward_speed + strafe_speed + turn_speed;
    float rpm_l2 = forward_speed + strafe_speed - turn_speed;
    float rpm_r2 = forward_speed - strafe_speed + turn_speed;

    // 【核心修正】根据您指出的映射关系进行精确分配
    // MotorA_PID -> R1
    // MotorB_PID -> L1
    // MotorC_PID -> L2
    // MotorD_PID -> R2
    Motor_PID_Set_Speed(&MotorA_PID, rpm_r1);
    Motor_PID_Set_Speed(&MotorB_PID, rpm_l1);
    Motor_PID_Set_Speed(&MotorC_PID, rpm_l2);
    Motor_PID_Set_Speed(&MotorD_PID, rpm_r2);
}

  float Motor_Encoder_PulsesToDistanceMM(int32_t pulses)
  {
    const float circumference_mm = (float)PI * WHEEL_DIAMETER_MM;
    // 脉冲数 → 转过的圈数 → 沿轮胎周长对应的行驶距离
    return ((float)pulses / ENCODER_PULSES_PER_REV) * circumference_mm;
  }