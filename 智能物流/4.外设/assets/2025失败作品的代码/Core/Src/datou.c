#include "datou.h"

/**
  * @brief    将当前位置清零
  * @param    huart ：UART句柄指针，例如 &huart2
  * @param    addr  ：电机地址
  * @retval   None
  */
void Datou_Reset_CurPos_To_Zero(UART_HandleTypeDef *huart, uint8_t addr)
{
  uint8_t cmd[4] = {0};
  
  cmd[0] = addr;
  cmd[1] = 0x0A;
  cmd[2] = 0x6D;
  cmd[3] = DATOU_CHECKSUM;
  
  HAL_UART_Transmit_DMA(huart, (uint8_t *)cmd, 4);
}

/**
  * @brief    解除堵转保护
  * @param    huart ：UART句柄指针
  * @param    addr  ：电机地址
  * @retval   None
  */
void Datou_Reset_Clog_Pro(UART_HandleTypeDef *huart, uint8_t addr)
{
  uint8_t cmd[4] = {0};
  
  cmd[0] = addr;
  cmd[1] = 0x0E;
  cmd[2] = 0x52;
  cmd[3] = DATOU_CHECKSUM;
  
  HAL_UART_Transmit_DMA(huart, (uint8_t *)cmd, 4);
}

/**
  * @brief    读取系统参数
  * @param    huart ：UART句柄指针
  * @param    addr  ：电机地址
  * @param    s     ：要读取的系统参数类型 (Datou_SysParams_t)
  * @retval   None
  */
void Datou_Read_Sys_Params(UART_HandleTypeDef *huart, uint8_t addr, Datou_SysParams_t s)
{
  uint8_t i = 0;
  uint8_t cmd[5] = {0}; // Max command length for this function is 4 bytes + checksum
  
  cmd[i++] = addr;

  switch(s)
  {
    case S_VER  : cmd[i++] = 0x1F; break;
    case S_RL   : cmd[i++] = 0x20; break;
    case S_PID  : cmd[i++] = 0x21; break;
    case S_VBUS : cmd[i++] = 0x24; break;
    case S_CPHA : cmd[i++] = 0x27; break;
    case S_ENCL : cmd[i++] = 0x31; break;
    case S_TPOS : cmd[i++] = 0x33; break;
    case S_VEL  : cmd[i++] = 0x35; break;
    case S_CPOS : cmd[i++] = 0x36; break;
    case S_PERR : cmd[i++] = 0x37; break;
    case S_FLAG : cmd[i++] = 0x3A; break;
    case S_ORG  : cmd[i++] = 0x3B; break;
    case S_Conf : cmd[i++] = 0x42; cmd[i++] = 0x6C; break;
    case S_State: cmd[i++] = 0x43; cmd[i++] = 0x7A; break;
    default: return; // Invalid parameter, do not send
  }

  cmd[i++] = DATOU_CHECKSUM;
  
  HAL_UART_Transmit_DMA(huart, (uint8_t *)cmd, i);
}

/**
  * @brief    修改开环/闭环控制模式
  * @param    huart     ：UART句柄指针
  * @param    addr      ：电机地址
  * @param    svF       ：是否存储标志，false为不存储，true为存储
  * @param    ctrl_mode ：控制模式
  * @retval   None
  */
void Datou_Modify_Ctrl_Mode(UART_HandleTypeDef *huart, uint8_t addr, bool svF, uint8_t ctrl_mode)
{
  uint8_t cmd[6] = {0};
  
  cmd[0] = addr;
  cmd[1] = 0x46;
  cmd[2] = 0x69;
  cmd[3] = svF;
  cmd[4] = ctrl_mode;
  cmd[5] = DATOU_CHECKSUM;
  
  HAL_UART_Transmit_DMA(huart, (uint8_t *)cmd, 6);
}

/**
  * @brief    使能信号控制
  * @param    huart ：UART句柄指针
  * @param    addr  ：电机地址
  * @param    state ：使能状态，true为使能，false为关闭
  * @param    snF   ：多机同步标志，false为不启用，true为启用
  * @retval   None
  */
void Datou_En_Control(UART_HandleTypeDef *huart, uint8_t addr, bool state, bool snF)
{
  uint8_t cmd[6] = {0};
  
  cmd[0] = addr;
  cmd[1] = 0xF3;
  cmd[2] = 0xAB;
  cmd[3] = (uint8_t)state;
  cmd[4] = snF;
  cmd[5] = DATOU_CHECKSUM;
  
  HAL_UART_Transmit_DMA(huart, (uint8_t *)cmd, 6);
}

/**
  * @brief    速度模式控制
  * @param    huart ：UART句柄指针
  * @param    addr  ：电机地址
  * @param    dir   ：方向，0为CW，其余值为CCW
  * @param    vel   ：速度，范围0 - 5000RPM
  * @param    acc   ：加速度，范围0 - 255，0是直接启动
  * @param    snF   ：多机同步标志
  * @retval   None
  */
void Datou_Vel_Control(UART_HandleTypeDef *huart, uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF)
{
  uint8_t cmd[8] = {0};

  cmd[0] = addr;
  cmd[1] = 0xF6;
  cmd[2] = dir;
  cmd[3] = (uint8_t)(vel >> 8);
  cmd[4] = (uint8_t)(vel >> 0);
  cmd[5] = acc;
  cmd[6] = snF;
  cmd[7] = DATOU_CHECKSUM;
  
  HAL_UART_Transmit_DMA(huart, (uint8_t *)cmd, 8);
}

/**
  * @brief    位置模式控制
  * @param    huart ：UART句柄指针
  * @param    addr  ：电机地址
  * @param    dir   ：方向，0为CW，其余值为CCW
  * @param    vel   ：速度(RPM)，范围0 - 5000
  * @param    acc   ：加速度，范围0 - 255
  * @param    clk   ：脉冲数
  * @param    raF   ：相对/绝对标志，false为相对，true为绝对
  * @param    snF   ：多机同步标志
  * @retval   None
  */
void Datou_Pos_Control(UART_HandleTypeDef *huart, uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF)
{
  uint8_t cmd[13] = {0};

  cmd[0]  = addr;
  cmd[1]  = 0xFD;
  cmd[2]  = dir;
  cmd[3]  = (uint8_t)(vel >> 8);
  cmd[4]  = (uint8_t)(vel >> 0); 
  cmd[5]  = acc;
  cmd[6]  = (uint8_t)(clk >> 24);
  cmd[7]  = (uint8_t)(clk >> 16);
  cmd[8]  = (uint8_t)(clk >> 8);
  cmd[9]  = (uint8_t)(clk >> 0);
  cmd[10] = raF;
  cmd[11] = snF;
  cmd[12] = DATOU_CHECKSUM;
  
  HAL_UART_Transmit_DMA(huart, (uint8_t *)cmd, 13);
}

/**
  * @brief    立即停止（所有控制模式都通用）
  * @param    huart ：UART句柄指针
  * @param    addr  ：电机地址
  * @param    snF   ：多机同步标志
  * @retval   None
  */
void Datou_Stop_Now(UART_HandleTypeDef *huart, uint8_t addr, bool snF)
{
  uint8_t cmd[5] = {0};
  
  cmd[0] = addr;
  cmd[1] = 0xFE;
  cmd[2] = 0x98;
  cmd[3] = snF;
  cmd[4] = DATOU_CHECKSUM;
  
  HAL_UART_Transmit_DMA(huart, (uint8_t *)cmd, 5);
}

/**
  * @brief    触发多机同步运动
  * @param    huart ：UART句柄指针
  * @param    addr  ：电机地址
  * @retval   None
  */
void Datou_Synchronous_motion(UART_HandleTypeDef *huart, uint8_t addr)
{
  uint8_t cmd[4] = {0};
  
  cmd[0] = addr;
  cmd[1] = 0xFF;
  cmd[2] = 0x66;
  cmd[3] = DATOU_CHECKSUM;
  
  HAL_UART_Transmit_DMA(huart, (uint8_t *)cmd, 4);
}

/**
  * @brief    设置单圈回零的零点位置
  * @param    huart ：UART句柄指针
  * @param    addr  ：电机地址
  * @param    svF   ：是否存储标志
  * @retval   None
  */
void Datou_Origin_Set_O(UART_HandleTypeDef *huart, uint8_t addr, bool svF)
{
   uint8_t cmd[5] = {0};
  
  cmd[0] = addr;
  cmd[1] = 0x93;
  cmd[2] = 0x88;
  cmd[3] = svF;
  cmd[4] = DATOU_CHECKSUM;
  
  HAL_UART_Transmit_DMA(huart, (uint8_t *)cmd, 5);
}

/**
  * @brief    修改回零参数
  * @param    huart  ：UART句柄指针
  * @param    addr   ：电机地址
  * @param    svF    ：是否存储标志
  * @param    o_mode ：回零模式
  * @param    o_dir  ：回零方向
  * @param    o_vel  ：回零速度
  * @param    o_tm   ：回零超时时间
  * @param    sl_vel ：无限位碰撞检测转速
  * @param    sl_ma  ：无限位碰撞检测电流
  * @param    sl_ms  ：无限位碰撞检测时间
  * @param    potF   ：上电自动触发回零
  * @retval   None
  */
void Datou_Origin_Modify_Params(UART_HandleTypeDef *huart, uint8_t addr, bool svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF)
{
  uint8_t cmd[20] = {0};
  
  cmd[0] =  addr;
  cmd[1] =  0x4C;
  cmd[2] =  0xAE;
  cmd[3] =  svF;
  cmd[4] =  o_mode;
  cmd[5] =  o_dir;
  cmd[6]  = (uint8_t)(o_vel >> 8);
  cmd[7]  = (uint8_t)(o_vel >> 0); 
  cmd[8]  = (uint8_t)(o_tm >> 24);
  cmd[9]  = (uint8_t)(o_tm >> 16);
  cmd[10] = (uint8_t)(o_tm >> 8);
  cmd[11] = (uint8_t)(o_tm >> 0);
  cmd[12] = (uint8_t)(sl_vel >> 8);
  cmd[13] = (uint8_t)(sl_vel >> 0); 
  cmd[14] = (uint8_t)(sl_ma >> 8);
  cmd[15] = (uint8_t)(sl_ma >> 0); 
  cmd[16] = (uint8_t)(sl_ms >> 8);
  cmd[17] = (uint8_t)(sl_ms >> 0);
  cmd[18] = potF;
  cmd[19] = DATOU_CHECKSUM;
  
  HAL_UART_Transmit_DMA(huart, (uint8_t *)cmd, 20);
}

/**
  * @brief    触发回零
  * @param    huart  ：UART句柄指针
  * @param    addr   ：电机地址
  * @param    o_mode ：回零模式
  * @param    snF    ：多机同步标志
  * @retval   None
  */
void Datou_Origin_Trigger_Return(UART_HandleTypeDef *huart, uint8_t addr, uint8_t o_mode, bool snF)
{
  uint8_t cmd[5] = {0};
  
  cmd[0] = addr;
  cmd[1] = 0x9A;
  cmd[2] = o_mode;
  cmd[3] = snF;
  cmd[4] = DATOU_CHECKSUM;
  
  HAL_UART_Transmit_DMA(huart, (uint8_t *)cmd, 5);
}

/**
  * @brief    强制中断并退出回零
  * @param    huart ：UART句柄指针
  * @param    addr  ：电机地址
  * @retval   None
  */
void Datou_Origin_Interrupt(UART_HandleTypeDef *huart, uint8_t addr)
{
  uint8_t cmd[4] = {0};
  
  cmd[0] = addr;
  cmd[1] = 0x9C;
  cmd[2] = 0x48;
  cmd[3] = DATOU_CHECKSUM;
  
  HAL_UART_Transmit_DMA(huart, (uint8_t *)cmd, 4);
}

void Datou_setpos(UART_HandleTypeDef *huart, uint8_t addr, int32_t pos)
{
  Datou_Pos_Control(huart, addr, 0, 0, 0, pos, true, false);
}

// 顺时针向上，逆时针向下
