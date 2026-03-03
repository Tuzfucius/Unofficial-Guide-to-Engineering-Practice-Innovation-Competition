/**
  ******************************************************************************
  * @file           : datou.h
  * @brief          : 张大头Emm_V5.0步进闭环电机驱动库 - 公共接口文件
  * @author         : ZHANGDATOU (原作), Gemini AI (封装和注释)
  * @version        : 1.1
  * @date           : 2025-10-02
  * @note           : 本文件定义了所有用户可调用的电机控制函数。
  *                   所有函数都通过串口发送指令，推荐使用DMA方式以提高效率。
  *                   淘宝店铺：https://zhangdatou.taobao.com
  *                   CSDN博客：https://blog.csdn.net/zhangdatou666
  ******************************************************************************
  */

#ifndef INC_DATOU_H_
#define INC_DATOU_H_

#include "main.h"
#include <stdbool.h> // 引入布尔类型支持

// 定义常量
#define DATOU_CHECKSUM 0x6B // 默认的固定校验字节

/**
 * @brief 系统参数读取类型枚举
 * @note  用于 Datou_Read_Sys_Params 函数，指定要读取哪一项参数。
 */
typedef enum
{
  S_VER,   // 固件版本和硬件版本
  S_RL,    // 相电阻和相电感
  S_PID,   // 位置环PID参数
  S_VBUS,  // 总线电压
  S_CPHA,  // 相电流
  S_ENCL,  // 经过线性化校准后的编码器值
  S_TPOS,  // 电机目标位置
  S_VEL,   // 电机实时转速
  S_CPOS,  // 电机实时位置
  S_PERR,  // 电机位置误差
  S_FLAG,  // 电机状态标志
  S_ORG,   // 回零状态标志
  S_Conf,  // 驱动配置参数
  S_State  // 系统状态参数
} Datou_SysParams_t;


/* ================================================================================= */
/* ============================= 核心运动控制函数 ============================= */
/* ================================================================================= */

/**
  * @brief  控制电机使能或失能。
  * @param  huart: 指向UART外设的句柄 (例如: &huart2)。
  * @param  addr:  目标电机的地址 (范围: 1-255)。
  * @param  state: 使能状态。`true` 为使能电机, `false` 为失能(关闭)电机。
  * @param  snF:   多机同步标志。`true` 为启用同步模式, `false` 为不启用。
  *                (启用后，此命令仅被预加载，需通过Datou_Synchronous_motion触发)。
  * @retval 无
  * @note   这是控制电机前通常需要发送的第一个指令。失能后电机将不响应运动指令。
  */
void Datou_En_Control(UART_HandleTypeDef *huart, uint8_t addr, bool state, bool snF);

/**
  * @brief  速度模式控制。让电机以指定的速度持续旋转。
  * @param  huart: 指向UART外设的句柄 (例如: &huart2)。
  * @param  addr:  目标电机的地址。
  * @param  dir:   旋转方向。`0` 为CW (顺时针), 非零值 (通常为`1`) 为CCW (逆时针)。
  * @param  vel:   目标速度。单位: RPM (转/分钟), 范围: 0 - 3000。
  * @param  acc:   加速度档位。范围: 0 - 255。
  *                **注意: 0 表示不使用曲线加减速, 电机将直接以目标速度启动**。
  * @param  snF:   多机同步标志。
  * @retval 无
  * @note   将速度 `vel` 设置为 0 即可使电机平滑停止。
  */
void Datou_Vel_Control(UART_HandleTypeDef *huart, uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF);

/**
  * @brief  位置模式控制。让电机精确转动指定的脉冲数。
  * @param  huart: 指向UART外设的句柄 (例如: &huart2)。
  * @param  addr:  目标电机的地址。
  * @param  dir:   旋转方向。`0` 为CW (顺时针), 非零值 (通常为`1`) 为CCW (逆时针)。
  * @param  vel:   运动过程中的最大速度。单位: RPM, 范围: 0 - 3000。
  * @param  acc:   加速度档位。范围: 0 - 255。
  * @param  clk:   目标脉冲数。电机将转动指定的脉冲数。
  *                (转动角度 = 脉冲数 / (200 * 细分) * 360°)。
  * @param  raF:   相对/绝对位置标志。`false` 为相对位置运动, `true` 为绝对位置运动。
  * @param  snF:   多机同步标志。
  * @retval 无
  */
void Datou_Pos_Control(UART_HandleTypeDef *huart, uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF);

/**
  * @brief  立即停止（紧急刹车）。
  * @param  huart: 指向UART外设的句柄 (例如: &huart2)。
  * @param  addr:  目标电机的地址。`0`为广播地址，可让所有电机同时停止。
  * @param  snF:   多机同步标志。
  * @retval 无
  * @note   此命令在任何模式下都有效，会使电机立即停止转动。
  */
void Datou_Stop_Now(UART_HandleTypeDef *huart, uint8_t addr, bool snF);

/**
  * @brief  触发多机同步运动。
  * @param  huart: 指向UART外设的句柄 (例如: &huart2)。
  * @param  addr:  必须使用广播地址 `0`。
  * @retval 无
  * @note   在发送此命令前，必须先将所有需要同步运动的电机的运动指令(速度或位置模式)
  *         的 `snF` 参数设置为 `true`。发送此命令后，所有电机将同时开始执行预加载的指令。
  */
void Datou_Synchronous_motion(UART_HandleTypeDef *huart, uint8_t addr);


/* ================================================================================= */
/* =========================== 参数配置与状态读取函数 =========================== */
/* ================================================================================= */

/**
  * @brief  将电机当前的位置清零。
  * @param  huart: 指向UART外设的句柄 (例如: &huart2)。
  * @param  addr:  目标电机的地址。
  * @retval 无
  */
void Datou_Reset_CurPos_To_Zero(UART_HandleTypeDef *huart, uint8_t addr);

/**
  * @brief  当电机发生堵转保护后，发送此命令以解除保护，使电机能重新响应指令。
  * @param  huart: 指向UART外设的句柄 (例如: &huart2)。
  * @param  addr:  目标电机的地址。
  * @retval 无
  */
void Datou_Reset_Clog_Pro(UART_HandleTypeDef *huart, uint8_t addr);

/**
  * @brief  读取电机的各种系统参数。
  * @param  huart: 指向UART外设的句柄 (例如: &huart2)。
  * @param  addr:  目标电机的地址。
  * @param  s:     要读取的参数类型，从 `Datou_SysParams_t` 枚举中选择一项。
  * @retval 无
  * @note   发送命令后，电机将通过串口返回数据，需要通过串口接收中断来处理返回的数据。
  */
void Datou_Read_Sys_Params(UART_HandleTypeDef *huart, uint8_t addr, Datou_SysParams_t s);

/**
  * @brief  修改开环/闭环控制模式。
  * @param  huart:     指向UART外设的句柄 (例如: &huart2)。
  * @param  addr:      目标电机的地址。
  * @param  svF:       是否存储标志。`true`表示设置会掉电保存, `false`表示不保存。
  * @param  ctrl_mode: 控制模式 (对应屏幕上的P_Pul菜单)。
  *                    - 0: 关闭脉冲输入引脚
  *                    - 1: 开环模式
  *                    - 2: 闭环模式 (FOC)
  *                    - 3: 复用引脚功能
  * @retval 无
  */
void Datou_Modify_Ctrl_Mode(UART_HandleTypeDef *huart, uint8_t addr, bool svF, uint8_t ctrl_mode);


/* ================================================================================= */
/* ============================== 回零(Homing)相关函数 ============================== */
/* ================================================================================= */

/**
  * @brief  将电机当前所在的位置设置为机械零点。
  * @param  huart: 指向UART外设的句柄 (例如: &huart2)。
  * @param  addr:  目标电机的地址。
  * @param  svF:   是否存储标志。`true`表示设置的零点位置会掉电保存。
  * @retval 无
  */
void Datou_Origin_Set_O(UART_HandleTypeDef *huart, uint8_t addr, bool svF);

/**
  * @brief  修改详细的回零参数。
  * @param  huart:  指向UART外设的句柄 (例如: &huart2)。
  * @param  addr:   目标电机的地址。
  * @param  svF:    是否存储标志。
  * @param  o_mode: 回零模式 (0:单圈就近, 1:单圈方向, 2:多圈碰撞, 3:多圈限位)。
  * @param  o_dir:  执行回零运动时的方向。`0`为CW, `1`为CCW。
  * @param  o_vel:  执行回零运动时的速度 (RPM)。
  * @param  o_tm:   回零超时时间 (ms)。
  * @param  sl_vel: (仅碰撞回零用) 碰撞检测的转速阈值 (RPM)。
  * @param  sl_ma:  (仅碰撞回零用) 碰撞检测的电流阈值 (mA)。
  * @param  sl_ms:  (仅碰撞回零用) 碰撞检测的持续时间阈值 (ms)。
  * @param  potF:   上电自动触发回零功能。`true`为使能, `false`为不使能。
  * @retval 无
  */
void Datou_Origin_Modify_Params(UART_HandleTypeDef *huart, uint8_t addr, bool svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF);

/**
  * @brief  发送命令，立即触发一次回零动作。
  * @param  huart:  指向UART外设的句柄 (例如: &huart2)。
  * @param  addr:   目标电机的地址。
  * @param  o_mode: 要执行的回零模式。
  * @param  snF:    多机同步标志。
  * @retval 无
  * @note   执行前回零参数需要预先设置好。
  */
void Datou_Origin_Trigger_Return(UART_HandleTypeDef *huart, uint8_t addr, uint8_t o_mode, bool snF);

/**
  * @brief  在电机正在执行回零的过程中，强制中断并退出回零。
  * @param  huart: 指向UART外设的句柄 (例如: &huart2)。
  * @param  addr:  目标电机的地址。
  * @retval 无
  */
void Datou_Origin_Interrupt(UART_HandleTypeDef *huart, uint8_t addr);


#endif /* INC_DATOU_H_ */